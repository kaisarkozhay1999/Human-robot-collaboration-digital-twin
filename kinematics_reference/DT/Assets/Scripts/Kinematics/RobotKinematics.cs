using System;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public enum SafetyDecision
    {
        Safe,
        Warning,
        Stop
    }

    public readonly struct RobotLinkSegment
    {
        public readonly string Name;
        public readonly int Index;
        public readonly Vector3 Start;
        public readonly Vector3 End;

        public RobotLinkSegment(string name, int index, Vector3 start, Vector3 end)
        {
            Name = string.IsNullOrWhiteSpace(name) ? $"Link{index + 1}" : name;
            Index = index;
            Start = start;
            End = end;
        }
    }

    public readonly struct ForwardKinematicsResult
    {
        public readonly Vector3[] JointPositions;
        public readonly RobotLinkSegment[] LinkSegments;
        public readonly Vector3 EndEffectorPosition;
        public readonly float EndEffectorPlanarAngleDegrees;

        public ForwardKinematicsResult(Vector3[] jointPositions, RobotLinkSegment[] linkSegments, float endEffectorPlanarAngleDegrees)
        {
            JointPositions = jointPositions;
            LinkSegments = linkSegments;
            EndEffectorPosition = jointPositions[jointPositions.Length - 1];
            EndEffectorPlanarAngleDegrees = endEffectorPlanarAngleDegrees;
        }
    }

    public readonly struct HumanRobotDistanceResult
    {
        public readonly bool IsValid;
        public readonly float DistanceMeters;
        public readonly int HumanJointIndex;
        public readonly int RobotLinkIndex;
        public readonly Vector3 HumanJointPosition;
        public readonly Vector3 ClosestPointOnRobotLink;

        public HumanRobotDistanceResult(
            bool isValid,
            float distanceMeters,
            int humanJointIndex,
            int robotLinkIndex,
            Vector3 humanJointPosition,
            Vector3 closestPointOnRobotLink)
        {
            IsValid = isValid;
            DistanceMeters = distanceMeters;
            HumanJointIndex = humanJointIndex;
            RobotLinkIndex = robotLinkIndex;
            HumanJointPosition = humanJointPosition;
            ClosestPointOnRobotLink = closestPointOnRobotLink;
        }
    }

    public readonly struct InverseKinematicsResult
    {
        public readonly bool Converged;
        public readonly float[] JointAnglesDegrees;
        public readonly int Iterations;
        public readonly float ResidualNorm;

        public InverseKinematicsResult(bool converged, float[] jointAnglesDegrees, int iterations, float residualNorm)
        {
            Converged = converged;
            JointAnglesDegrees = jointAnglesDegrees;
            Iterations = iterations;
            ResidualNorm = residualNorm;
        }
    }

    public static class RobotKinematics
    {
        public static ForwardKinematicsResult ForwardKinematics(RobotKinematicParameters parameters, float[] jointAnglesDegrees)
        {
            ValidateParameters(parameters, jointAnglesDegrees);

            Vector3 horizontal = parameters.horizontalAxis.normalized;
            Vector3 vertical = parameters.verticalAxis.normalized;
            Vector3[] joints = new Vector3[parameters.links.Length + 1];
            RobotLinkSegment[] segments = new RobotLinkSegment[parameters.links.Length];
            joints[0] = parameters.basePositionMeters;

            float cumulativeAngleDegrees = 0f;
            for (int i = 0; i < parameters.links.Length; i++)
            {
                RobotLinkParameter link = parameters.links[i];
                cumulativeAngleDegrees += jointAnglesDegrees[i] + link.jointOffsetDegrees;

                // The planar serial-chain model from the Mathematica notebook:
                // p_i = p_{i-1} + L_i [cos(sum theta), sin(sum theta)].
                // The configurable horizontal/vertical axes map that 2D plane into Unity world space.
                float angleRadians = cumulativeAngleDegrees * Mathf.Deg2Rad;
                Vector3 direction = horizontal * Mathf.Cos(angleRadians) + vertical * Mathf.Sin(angleRadians);
                joints[i + 1] = joints[i] + direction * link.linkLengthMeters;
                segments[i] = new RobotLinkSegment(link.name, i, joints[i], joints[i + 1]);
            }

            return new ForwardKinematicsResult(joints, segments, cumulativeAngleDegrees);
        }

        public static HumanRobotDistanceResult MinimumDistance(Vector3[] humanJoints, RobotLinkSegment[] robotSegments)
        {
            if (humanJoints == null || humanJoints.Length == 0 || robotSegments == null || robotSegments.Length == 0)
            {
                return new HumanRobotDistanceResult(false, float.PositiveInfinity, -1, -1, Vector3.zero, Vector3.zero);
            }

            float bestDistance = float.PositiveInfinity;
            int bestHumanJoint = -1;
            int bestRobotLink = -1;
            Vector3 bestHumanPoint = Vector3.zero;
            Vector3 bestRobotPoint = Vector3.zero;

            for (int humanIndex = 0; humanIndex < humanJoints.Length; humanIndex++)
            {
                Vector3 humanPoint = humanJoints[humanIndex];
                if (!IsFinite(humanPoint))
                {
                    continue;
                }

                for (int linkIndex = 0; linkIndex < robotSegments.Length; linkIndex++)
                {
                    float distance = PointToSegmentDistance(humanPoint, robotSegments[linkIndex].Start, robotSegments[linkIndex].End, out Vector3 closest);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        bestHumanJoint = humanIndex;
                        bestRobotLink = linkIndex;
                        bestHumanPoint = humanPoint;
                        bestRobotPoint = closest;
                    }
                }
            }

            return bestHumanJoint < 0
                ? new HumanRobotDistanceResult(false, float.PositiveInfinity, -1, -1, Vector3.zero, Vector3.zero)
                : new HumanRobotDistanceResult(true, bestDistance, bestHumanJoint, bestRobotLink, bestHumanPoint, bestRobotPoint);
        }

        public static float PointToSegmentDistance(Vector3 point, Vector3 segmentStart, Vector3 segmentEnd, out Vector3 closestPoint)
        {
            Vector3 segment = segmentEnd - segmentStart;
            float lengthSquared = segment.sqrMagnitude;
            if (lengthSquared <= Mathf.Epsilon)
            {
                closestPoint = segmentStart;
                return Vector3.Distance(point, segmentStart);
            }

            // Orthogonal projection parameter clamped to [0, 1] so the closest point lies on the finite link body.
            float t = Vector3.Dot(point - segmentStart, segment) / lengthSquared;
            t = Mathf.Clamp01(t);
            closestPoint = segmentStart + t * segment;
            return Vector3.Distance(point, closestPoint);
        }

        public static SafetyDecision DecideSafety(
            bool dataIsFresh,
            bool gestureStopRequested,
            bool gestureGoRequested,
            float distanceMeters,
            float warningThresholdMeters,
            float stopThresholdMeters,
            float releaseThresholdMeters,
            SafetyDecision previousDecision)
        {
            if (!dataIsFresh || gestureStopRequested)
            {
                return SafetyDecision.Stop;
            }

            if (!IsFinite(distanceMeters))
            {
                return SafetyDecision.Stop;
            }

            if (previousDecision == SafetyDecision.Stop && !gestureGoRequested && distanceMeters < releaseThresholdMeters)
            {
                return SafetyDecision.Stop;
            }

            if (distanceMeters <= stopThresholdMeters)
            {
                return SafetyDecision.Stop;
            }

            return distanceMeters <= warningThresholdMeters ? SafetyDecision.Warning : SafetyDecision.Safe;
        }

        public static InverseKinematicsResult SolvePlanarIk3(
            RobotKinematicParameters parameters,
            Vector2 targetMeters,
            float targetPlanarAngleDegrees,
            float[] initialGuessDegrees,
            int maxIterations = 40,
            float tolerance = 1e-4f)
        {
            if (parameters == null || parameters.links == null || parameters.links.Length != 3)
            {
                throw new ArgumentException("Newton-Raphson IK currently supports exactly the 3-link planar model from the notebook.");
            }

            if (initialGuessDegrees == null || initialGuessDegrees.Length != 3)
            {
                throw new ArgumentException("Initial guess must contain three joint angles.");
            }

            float[] theta = (float[])initialGuessDegrees.Clone();
            float residualNorm = float.PositiveInfinity;

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                Vector2 p = PlanarCoordinates(parameters, ForwardKinematics(parameters, theta).EndEffectorPosition);
                float phi = theta[0] + parameters.links[0].jointOffsetDegrees
                    + theta[1] + parameters.links[1].jointOffsetDegrees
                    + theta[2] + parameters.links[2].jointOffsetDegrees;

                float[] residual =
                {
                    p.x - targetMeters.x,
                    p.y - targetMeters.y,
                    Mathf.DeltaAngle(targetPlanarAngleDegrees, phi)
                };

                residualNorm = Mathf.Sqrt(residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2]);
                if (residualNorm <= tolerance)
                {
                    return new InverseKinematicsResult(true, theta, iteration, residualNorm);
                }

                float[,] jacobian = NumericalJacobian(parameters, theta);
                float[] correction = Solve3x3(jacobian, new[] { -residual[0], -residual[1], -residual[2] });
                for (int i = 0; i < 3; i++)
                {
                    theta[i] += correction[i];
                }
            }

            return new InverseKinematicsResult(false, theta, maxIterations, residualNorm);
        }

        private static Vector2 PlanarCoordinates(RobotKinematicParameters parameters, Vector3 worldPoint)
        {
            Vector3 relative = worldPoint - parameters.basePositionMeters;
            return new Vector2(
                Vector3.Dot(relative, parameters.horizontalAxis.normalized),
                Vector3.Dot(relative, parameters.verticalAxis.normalized));
        }

        private static float[,] NumericalJacobian(RobotKinematicParameters parameters, float[] theta)
        {
            const float h = 1e-3f;
            float[,] jacobian = new float[3, 3];
            Vector2 basePoint = PlanarCoordinates(parameters, ForwardKinematics(parameters, theta).EndEffectorPosition);
            float basePhi = theta[0] + theta[1] + theta[2]
                + parameters.links[0].jointOffsetDegrees
                + parameters.links[1].jointOffsetDegrees
                + parameters.links[2].jointOffsetDegrees;

            for (int column = 0; column < 3; column++)
            {
                float[] perturbed = (float[])theta.Clone();
                perturbed[column] += h;
                ForwardKinematicsResult fk = ForwardKinematics(parameters, perturbed);
                float phi = perturbed[0] + perturbed[1] + perturbed[2]
                    + parameters.links[0].jointOffsetDegrees
                    + parameters.links[1].jointOffsetDegrees
                    + parameters.links[2].jointOffsetDegrees;

                Vector2 planarPoint = PlanarCoordinates(parameters, fk.EndEffectorPosition);
                jacobian[0, column] = (planarPoint.x - basePoint.x) / h;
                jacobian[1, column] = (planarPoint.y - basePoint.y) / h;
                jacobian[2, column] = Mathf.DeltaAngle(basePhi, phi) / h;
            }

            return jacobian;
        }

        private static float[] Solve3x3(float[,] matrix, float[] rhs)
        {
            float[,] a = (float[,])matrix.Clone();
            float[] b = (float[])rhs.Clone();

            for (int pivot = 0; pivot < 3; pivot++)
            {
                int bestRow = pivot;
                float bestValue = Mathf.Abs(a[pivot, pivot]);
                for (int row = pivot + 1; row < 3; row++)
                {
                    float value = Mathf.Abs(a[row, pivot]);
                    if (value > bestValue)
                    {
                        bestValue = value;
                        bestRow = row;
                    }
                }

                if (bestValue <= 1e-8f)
                {
                    throw new InvalidOperationException("Singular IK Jacobian.");
                }

                if (bestRow != pivot)
                {
                    for (int column = pivot; column < 3; column++)
                    {
                        (a[pivot, column], a[bestRow, column]) = (a[bestRow, column], a[pivot, column]);
                    }

                    (b[pivot], b[bestRow]) = (b[bestRow], b[pivot]);
                }

                for (int row = pivot + 1; row < 3; row++)
                {
                    float factor = a[row, pivot] / a[pivot, pivot];
                    for (int column = pivot; column < 3; column++)
                    {
                        a[row, column] -= factor * a[pivot, column];
                    }

                    b[row] -= factor * b[pivot];
                }
            }

            float[] x = new float[3];
            for (int row = 2; row >= 0; row--)
            {
                float sum = b[row];
                for (int column = row + 1; column < 3; column++)
                {
                    sum -= a[row, column] * x[column];
                }

                x[row] = sum / a[row, row];
            }

            return x;
        }

        private static void ValidateParameters(RobotKinematicParameters parameters, float[] jointAnglesDegrees)
        {
            if (parameters == null)
            {
                throw new ArgumentNullException(nameof(parameters));
            }

            if (!parameters.IsValid(out string error))
            {
                throw new ArgumentException(error, nameof(parameters));
            }

            if (jointAnglesDegrees == null || jointAnglesDegrees.Length < parameters.links.Length)
            {
                throw new ArgumentException("Joint angle array must contain one angle per configured link.", nameof(jointAnglesDegrees));
            }
        }

        private static bool IsFinite(Vector3 v)
        {
            return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
