using System;
using System.Collections.Generic;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    [DefaultExecutionOrder(100)]
    public sealed class RobotSafetyDistanceEvaluator : MonoBehaviour
    {
        [Header("Kinematic model")]
        [Tooltip("Loaded from JSON or assigned directly. Placeholder dimensions are intentionally labeled in the asset/JSON.")]
        public RobotKinematicParameters parameters;
        public float[] latestJointAnglesDegrees = { 0f, 0f, 0f, 0f, 0f };

        [Header("Kinematic frame")]
        [Tooltip("Keeps FK link segments located on the real scene robot instead of at the placeholder JSON origin.")]
        public bool useRobotControllerPositionAsBase = true;
        public Vector3 kinematicBaseOffsetMeters = Vector3.zero;

        [Header("Live robot source")]
        public bool readRobotControllerEveryFrame = true;
        public bool autoFindLiveSources = true;
        public global::ER9ProFullController robotController;

        [Header("Robot geometry source")]
        [Tooltip("Use the current Unity robot joint transforms for safety distance instead of the placeholder JSON FK chain.")]
        public bool useRobotControllerTransformsForDistance = true;
        [Tooltip("Treat the visible Unity robot transform chain as fresh robot geometry even when robot/state MQTT telemetry is unavailable.")]
        public bool visualRobotGeometryCountsAsFresh = true;
        [Tooltip("When multiple robot controllers exist, use the one with the largest renderer bounds as the visual robot.")]
        public bool preferLargestRenderedRobotController = true;
        public bool includeRobotControllerRootPoint = true;
        public bool includeRobotGripperPoint = true;

        [Header("Live human skeleton source")]
        public bool readPoseBoneDrivers = true;
        public bool autoRefreshPoseBoneDrivers = true;
        public global::PoseBoneDriver[] poseBoneDrivers;
        public Transform[] humanJointTransforms;
        public float staleAfterSeconds = 0.5f;

        [Header("Safety thresholds")]
        public float warningThresholdMeters = 0.45f;
        public float stopThresholdMeters = 0.25f;
        public float releaseThresholdMeters = 0.35f;

        [Header("Gesture state")]
        public bool readGestureReceiver = true;
        public global::StopGoReceiver gestureReceiver;
        public bool gestureStopRequested;
        public bool gestureGoRequested;

        [Header("Robot command output")]
        public bool publishSafetyCommands;
        public bool publishResumeOnRelease = true;
        [Tooltip("Seconds between repeated stop commands while Stop remains active. 0 disables repeats.")]
        public float repeatStopCommandSeconds;
        public string stopCommand = "stop";
        public string resumeCommand = "go";

        [Header("Debug drawing")]
        public bool drawDebug = true;
        public bool drawHumanJoints = true;
        public Color linkColor = Color.cyan;
        public Color robotJointColor = Color.yellow;
        public Color tcpColor = Color.magenta;
        public Color humanJointColor = Color.green;
        public Color closestDistanceColor = Color.red;
        public float debugSphereRadiusMeters = 0.025f;
        public Transform tcpMarker;
        public bool updateTcpMarkerFromFk;

        public SafetyDecision CurrentDecision { get; private set; } = SafetyDecision.Stop;
        public ForwardKinematicsResult CurrentKinematics { get; private set; }
        public HumanRobotDistanceResult CurrentDistance { get; private set; }
        public Vector3[] CurrentHumanJointPositions { get; private set; } = new Vector3[0];
        public float LastJointTelemetryTime { get; private set; } = float.NegativeInfinity;
        public float LastHumanPoseTime { get; private set; } = float.NegativeInfinity;
        public bool RobotDataFresh { get; private set; }
        public bool HumanDataFresh { get; private set; }
        public bool DataFresh => RobotDataFresh && HumanDataFresh;

        private float nextAutoFindTime;
        private float lastStopCommandTime = float.NegativeInfinity;
        private SafetyDecision lastPublishedDecision = SafetyDecision.Safe;
        private bool previousDataFresh;
        private bool currentRobotGeometryFromTransforms;

        public string EndEffectorDisplay =>
            CurrentKinematics.JointPositions == null
                ? "End effector: unavailable"
                : $"End effector: {CurrentKinematics.EndEffectorPosition:F3} m, phi={CurrentKinematics.EndEffectorPlanarAngleDegrees:F1} deg";

        public string MinimumDistanceDisplay =>
            !CurrentDistance.IsValid
                ? "Human-robot distance: unavailable"
                : $"Human-robot distance: {CurrentDistance.DistanceMeters:F3} m, human joint {CurrentDistance.HumanJointIndex}, robot link {CurrentDistance.RobotLinkIndex}";

        public void ApplyJointTelemetry(float[] jointAnglesDegrees, float timestampSeconds)
        {
            if (jointAnglesDegrees == null)
                throw new ArgumentNullException(nameof(jointAnglesDegrees));

            latestJointAnglesDegrees = (float[])jointAnglesDegrees.Clone();
            LastJointTelemetryTime = timestampSeconds;
        }

        public EvaluationSnapshot Evaluate(float nowSeconds)
        {
            if (TryBuildRobotControllerKinematics(out ForwardKinematicsResult visualKinematics))
            {
                CurrentKinematics = visualKinematics;
                currentRobotGeometryFromTransforms = true;
            }
            else
            {
                ApplyRuntimeKinematicFrame();
                CurrentKinematics = RobotKinematics.ForwardKinematics(parameters, latestJointAnglesDegrees);
                currentRobotGeometryFromTransforms = false;
            }

            CurrentHumanJointPositions = ReadHumanJointPositions(nowSeconds);
            CurrentDistance = RobotKinematics.MinimumDistance(CurrentHumanJointPositions, CurrentKinematics.LinkSegments);

            RobotDataFresh = currentRobotGeometryFromTransforms && visualRobotGeometryCountsAsFresh ||
                nowSeconds - LastJointTelemetryTime <= staleAfterSeconds;
            HumanDataFresh = CurrentHumanJointPositions.Length > 0 && nowSeconds - LastHumanPoseTime <= staleAfterSeconds;
            bool dataIsFresh = RobotDataFresh && HumanDataFresh;

            CurrentDecision = RobotKinematics.DecideSafety(
                dataIsFresh,
                gestureStopRequested,
                gestureGoRequested,
                CurrentDistance.DistanceMeters,
                warningThresholdMeters,
                stopThresholdMeters,
                releaseThresholdMeters,
                CurrentDecision);

            return new EvaluationSnapshot(CurrentDecision, dataIsFresh, CurrentKinematics, CurrentDistance);
        }

        private void ApplyRuntimeKinematicFrame()
        {
            if (!useRobotControllerPositionAsBase || parameters == null || robotController == null)
                return;

            parameters.basePositionMeters = robotController.transform.position + kinematicBaseOffsetMeters;
        }

        private bool HasRobotGeometryInput()
        {
            if (useRobotControllerTransformsForDistance && robotController != null && CountRobotControllerGeometryPoints() >= 2)
                return true;

            return parameters != null;
        }

        private int CountRobotControllerGeometryPoints()
        {
            if (robotController == null)
                return 0;

            int count = 0;
            if (includeRobotControllerRootPoint && IsFinite(robotController.transform.position)) count++;
            if (robotController.joint1 != null && IsFinite(robotController.joint1.position)) count++;
            if (robotController.joint2 != null && IsFinite(robotController.joint2.position)) count++;
            if (robotController.joint3 != null && IsFinite(robotController.joint3.position)) count++;
            if (robotController.joint4 != null && IsFinite(robotController.joint4.position)) count++;
            if (robotController.joint5 != null && IsFinite(robotController.joint5.position)) count++;
            if (includeRobotGripperPoint && robotController.gripper != null && IsFinite(robotController.gripper.position)) count++;
            return count;
        }

        private bool TryBuildRobotControllerKinematics(out ForwardKinematicsResult kinematics)
        {
            kinematics = default;
            if (!useRobotControllerTransformsForDistance || robotController == null)
                return false;

            List<Vector3> points = new List<Vector3>(7);
            List<string> names = new List<string>(7);

            if (includeRobotControllerRootPoint)
                AddRobotGeometryPoint(points, names, robotController.transform, "RobotRoot");
            AddRobotGeometryPoint(points, names, robotController.joint1, "Joint1");
            AddRobotGeometryPoint(points, names, robotController.joint2, "Joint2");
            AddRobotGeometryPoint(points, names, robotController.joint3, "Joint3");
            AddRobotGeometryPoint(points, names, robotController.joint4, "Joint4");
            AddRobotGeometryPoint(points, names, robotController.joint5, "Joint5");
            if (includeRobotGripperPoint)
                AddRobotGeometryPoint(points, names, robotController.gripper, "Gripper");

            if (points.Count < 2)
                return false;

            Vector3[] jointPositions = points.ToArray();
            RobotLinkSegment[] segments = new RobotLinkSegment[jointPositions.Length - 1];
            for (int i = 0; i < segments.Length; i++)
            {
                string segmentName = names[i] + "_to_" + names[i + 1];
                segments[i] = new RobotLinkSegment(segmentName, i, jointPositions[i], jointPositions[i + 1]);
            }

            kinematics = new ForwardKinematicsResult(jointPositions, segments, 0f);
            return true;
        }

        private static void AddRobotGeometryPoint(List<Vector3> points, List<string> names, Transform transform, string fallbackName)
        {
            if (transform == null)
                return;

            Vector3 point = transform.position;
            if (!IsFinite(point))
                return;

            if (points.Count > 0 && (points[points.Count - 1] - point).sqrMagnitude <= 0.000001f)
                return;

            points.Add(point);
            names.Add(string.IsNullOrWhiteSpace(transform.name) ? fallbackName : transform.name.Replace(",", ";"));
        }

        private void Update()
        {
            if (autoFindLiveSources)
                RefreshLiveSourcesIfNeeded();

            ApplyLiveRobotTelemetry();
            ApplyLiveGestureState();

            if (!HasRobotGeometryInput())
                return;

            EvaluationSnapshot snapshot = Evaluate(Time.time);
            if (updateTcpMarkerFromFk && tcpMarker != null && CurrentKinematics.JointPositions != null)
                tcpMarker.position = CurrentKinematics.EndEffectorPosition;

            PublishSafetyCommandIfNeeded(snapshot.Decision, snapshot.DataIsFresh);
        }

        private void RefreshLiveSourcesIfNeeded()
        {
            if (Time.time < nextAutoFindTime)
                return;

            nextAutoFindTime = Time.time + 1.0f;

            if (robotController == null || preferLargestRenderedRobotController)
            {
                global::ER9ProFullController bestRobotController = FindBestRenderedRobotController(robotController);
                if (bestRobotController != null)
                    robotController = bestRobotController;
            }

            if (gestureReceiver == null)
                gestureReceiver = FindObjectOfType<global::StopGoReceiver>();

            if (readPoseBoneDrivers && autoRefreshPoseBoneDrivers)
                poseBoneDrivers = FindScenePoseBoneDrivers();
            else if (readPoseBoneDrivers && (poseBoneDrivers == null || poseBoneDrivers.Length == 0))
                poseBoneDrivers = FindScenePoseBoneDrivers();
        }

        private static global::PoseBoneDriver[] FindScenePoseBoneDrivers()
        {
            global::PoseBoneDriver[] drivers = Resources.FindObjectsOfTypeAll<global::PoseBoneDriver>();
            List<global::PoseBoneDriver> sceneDrivers = new List<global::PoseBoneDriver>();
            foreach (global::PoseBoneDriver driver in drivers)
            {
                if (driver == null || !driver.gameObject.scene.IsValid())
                    continue;

                sceneDrivers.Add(driver);
            }

            return sceneDrivers.ToArray();
        }

        private static global::ER9ProFullController FindBestRenderedRobotController(global::ER9ProFullController fallback)
        {
            global::ER9ProFullController[] controllers = Resources.FindObjectsOfTypeAll<global::ER9ProFullController>();
            global::ER9ProFullController best = fallback;
            float bestScore = RendererBoundsScore(fallback);

            foreach (global::ER9ProFullController candidate in controllers)
            {
                if (candidate == null || !candidate.gameObject.scene.IsValid())
                    continue;

                float score = RendererBoundsScore(candidate);
                if (score > bestScore)
                {
                    best = candidate;
                    bestScore = score;
                }
            }

            return best;
        }

        private static float RendererBoundsScore(global::ER9ProFullController controller)
        {
            if (controller == null)
                return 0f;

            Renderer[] renderers = controller.GetComponentsInChildren<Renderer>(true);
            bool hasBounds = false;
            Bounds combined = new Bounds(controller.transform.position, Vector3.zero);

            foreach (Renderer renderer in renderers)
            {
                if (renderer == null)
                    continue;

                Bounds bounds = renderer.bounds;
                if (!IsFinite(bounds.center) || !IsFinite(bounds.size))
                    continue;

                if (!hasBounds)
                {
                    combined = bounds;
                    hasBounds = true;
                }
                else
                {
                    combined.Encapsulate(bounds);
                }
            }

            if (!hasBounds)
                return 0f;

            Vector3 size = combined.size;
            return Mathf.Max(0f, size.x) * Mathf.Max(0f, size.y) * Mathf.Max(0f, size.z);
        }

        private void ApplyLiveRobotTelemetry()
        {
            if (!readRobotControllerEveryFrame || robotController == null)
                return;

            // Journal integration point: MQTT robot/state enters ER9ProFullController,
            // then this evaluator snapshots the smoothed live joint angles for FK.
            ApplyJointTelemetry(robotController.GetCurrentJointAnglesDegrees(), robotController.LastRobotStateTelemetryTime);
        }

        private void ApplyLiveGestureState()
        {
            if (!readGestureReceiver || gestureReceiver == null)
                return;

            string command = gestureReceiver.currentCommand;
            gestureStopRequested = string.Equals(command, "stop", StringComparison.OrdinalIgnoreCase);
            gestureGoRequested = string.Equals(command, "go", StringComparison.OrdinalIgnoreCase);
        }

        private Vector3[] ReadHumanJointPositions(float nowSeconds)
        {
            List<Vector3> points = new List<Vector3>();
            float newestPoseTime = float.NegativeInfinity;

            if (readPoseBoneDrivers && poseBoneDrivers != null)
            {
                foreach (global::PoseBoneDriver driver in poseBoneDrivers)
                {
                    if (driver == null)
                        continue;

                    int before = points.Count;
                    driver.CopyActiveJointWorldPositions(points);
                    if (points.Count > before)
                        newestPoseTime = Mathf.Max(newestPoseTime, driver.LastPoseUpdateTime);
                }
            }

            if (humanJointTransforms != null)
            {
                foreach (Transform jointTransform in humanJointTransforms)
                {
                    if (jointTransform == null)
                        continue;

                    Vector3 point = jointTransform.position;
                    if (!IsFinite(point))
                        continue;

                    points.Add(point);
                }

                if (points.Count > 0 && float.IsNegativeInfinity(newestPoseTime))
                    newestPoseTime = nowSeconds;
            }

            if (!float.IsNegativeInfinity(newestPoseTime))
                LastHumanPoseTime = newestPoseTime;

            return points.ToArray();
        }

        private void PublishSafetyCommandIfNeeded(SafetyDecision decision, bool dataIsFresh)
        {
            if (!publishSafetyCommands)
            {
                lastPublishedDecision = decision;
                previousDataFresh = dataIsFresh;
                return;
            }

            bool isStop = decision == SafetyDecision.Stop;
            bool wasStop = lastPublishedDecision == SafetyDecision.Stop;

            if (isStop)
            {
                float effectiveRepeatSeconds = GetEffectiveStopRepeatSeconds();
                bool shouldRepeat = effectiveRepeatSeconds > 0f &&
                    Time.time - lastStopCommandTime >= effectiveRepeatSeconds;
                if (!wasStop || shouldRepeat)
                    PublishStopCommand();
            }
            else if (wasStop && publishResumeOnRelease)
            {
                PublishResumeCommand();
            }

            lastPublishedDecision = decision;
            previousDataFresh = dataIsFresh;
        }

        private float GetEffectiveStopRepeatSeconds()
        {
            return Mathf.Max(0f, repeatStopCommandSeconds);
        }

        private void PublishStopCommand()
        {
            lastStopCommandTime = Time.time;

            string distanceText = CurrentDistance.IsValid
                ? CurrentDistance.DistanceMeters.ToString("F3")
                : "unavailable";
            Debug.Log(
                "Safety stop publish: distance=" + distanceText +
                "m, robotFresh=" + RobotDataFresh +
                ", humanFresh=" + HumanDataFresh +
                ", stopCommand=" + stopCommand);

            if (robotController != null)
                robotController.StopAll();

            if (gestureReceiver != null)
                gestureReceiver.PublishExternalSafetyCommand(stopCommand);
        }

        private void PublishResumeCommand()
        {
            if (gestureReceiver != null)
            {
                gestureReceiver.PublishExternalSafetyCommand(resumeCommand);
                return;
            }

            if (robotController != null)
                robotController.PublishControlCommand(resumeCommand);
        }

        private void OnDrawGizmos()
        {
            if (!drawDebug)
                return;

            if (CurrentKinematics.JointPositions != null)
            {
                Gizmos.color = robotJointColor;
                foreach (Vector3 jointPosition in CurrentKinematics.JointPositions)
                    Gizmos.DrawSphere(jointPosition, debugSphereRadiusMeters);

                Gizmos.color = linkColor;
                foreach (RobotLinkSegment segment in CurrentKinematics.LinkSegments)
                    Gizmos.DrawLine(segment.Start, segment.End);

                Gizmos.color = tcpColor;
                Gizmos.DrawSphere(CurrentKinematics.EndEffectorPosition, debugSphereRadiusMeters * 1.4f);
            }

            if (drawHumanJoints && CurrentHumanJointPositions != null)
            {
                Gizmos.color = humanJointColor;
                foreach (Vector3 humanJoint in CurrentHumanJointPositions)
                    Gizmos.DrawSphere(humanJoint, debugSphereRadiusMeters * 0.85f);
            }

            if (CurrentDistance.IsValid)
            {
                Gizmos.color = closestDistanceColor;
                Gizmos.DrawLine(CurrentDistance.HumanJointPosition, CurrentDistance.ClosestPointOnRobotLink);
                Gizmos.DrawSphere(CurrentDistance.ClosestPointOnRobotLink, debugSphereRadiusMeters * 0.75f);
                Gizmos.DrawSphere(CurrentDistance.HumanJointPosition, debugSphereRadiusMeters * 1.1f);
            }

            if (tcpMarker != null && CurrentKinematics.JointPositions != null)
            {
                Gizmos.color = Color.white;
                Gizmos.DrawLine(CurrentKinematics.EndEffectorPosition, tcpMarker.position);
            }
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }

    public readonly struct EvaluationSnapshot
    {
        public readonly SafetyDecision Decision;
        public readonly bool DataIsFresh;
        public readonly ForwardKinematicsResult Kinematics;
        public readonly HumanRobotDistanceResult Distance;

        public EvaluationSnapshot(
            SafetyDecision decision,
            bool dataIsFresh,
            ForwardKinematicsResult kinematics,
            HumanRobotDistanceResult distance)
        {
            Decision = decision;
            DataIsFresh = dataIsFresh;
            Kinematics = kinematics;
            Distance = distance;
        }
    }
}
