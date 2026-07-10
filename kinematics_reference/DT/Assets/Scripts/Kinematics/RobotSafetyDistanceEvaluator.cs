using System;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotSafetyDistanceEvaluator : MonoBehaviour
    {
        [Header("Kinematic model")]
        public RobotKinematicParameters parameters;
        public float[] latestJointAnglesDegrees = { 0f, 0f, 0f };

        [Header("Human skeleton")]
        public Transform[] humanJointTransforms;
        public float staleAfterSeconds = 0.5f;

        [Header("Safety thresholds")]
        public float warningThresholdMeters = 0.45f;
        public float stopThresholdMeters = 0.25f;
        public float releaseThresholdMeters = 0.35f;

        [Header("Gesture state")]
        public bool gestureStopRequested;
        public bool gestureGoRequested;

        [Header("Debug drawing")]
        public bool drawDebug = true;
        public Color linkColor = Color.cyan;
        public Color jointColor = Color.yellow;
        public Color closestDistanceColor = Color.red;
        public float debugSphereRadiusMeters = 0.025f;

        public SafetyDecision CurrentDecision { get; private set; } = SafetyDecision.Stop;
        public ForwardKinematicsResult CurrentKinematics { get; private set; }
        public HumanRobotDistanceResult CurrentDistance { get; private set; }
        public float LastJointTelemetryTime { get; private set; } = float.NegativeInfinity;

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
            {
                throw new ArgumentNullException(nameof(jointAnglesDegrees));
            }

            latestJointAnglesDegrees = (float[])jointAnglesDegrees.Clone();
            LastJointTelemetryTime = timestampSeconds;
        }

        public EvaluationSnapshot Evaluate(float nowSeconds)
        {
            CurrentKinematics = RobotKinematics.ForwardKinematics(parameters, latestJointAnglesDegrees);
            Vector3[] humanJoints = ReadHumanJointPositions();
            CurrentDistance = RobotKinematics.MinimumDistance(humanJoints, CurrentKinematics.LinkSegments);

            bool dataIsFresh = nowSeconds - LastJointTelemetryTime <= staleAfterSeconds;
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

        private void Update()
        {
            if (parameters == null)
            {
                return;
            }

            Evaluate(Time.time);
        }

        private void OnDrawGizmos()
        {
            if (!drawDebug || CurrentKinematics.JointPositions == null)
            {
                return;
            }

            Gizmos.color = jointColor;
            foreach (Vector3 jointPosition in CurrentKinematics.JointPositions)
            {
                Gizmos.DrawSphere(jointPosition, debugSphereRadiusMeters);
            }

            Gizmos.color = linkColor;
            foreach (RobotLinkSegment segment in CurrentKinematics.LinkSegments)
            {
                Gizmos.DrawLine(segment.Start, segment.End);
            }

            if (CurrentDistance.IsValid)
            {
                Gizmos.color = closestDistanceColor;
                Gizmos.DrawLine(CurrentDistance.HumanJointPosition, CurrentDistance.ClosestPointOnRobotLink);
                Gizmos.DrawSphere(CurrentDistance.ClosestPointOnRobotLink, debugSphereRadiusMeters * 0.75f);
            }
        }

        private Vector3[] ReadHumanJointPositions()
        {
            if (humanJointTransforms == null)
            {
                return Array.Empty<Vector3>();
            }

            Vector3[] points = new Vector3[humanJointTransforms.Length];
            for (int i = 0; i < humanJointTransforms.Length; i++)
            {
                points[i] = humanJointTransforms[i] == null
                    ? new Vector3(float.NaN, float.NaN, float.NaN)
                    : humanJointTransforms[i].position;
            }

            return points;
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
