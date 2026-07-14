using System.Text;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotKinematicsValidationRunner : MonoBehaviour
    {
        public RobotKinematicParameters parameters;
        public Vector3 sampleHumanPointMeters = new Vector3(0.30f, 0.10f, 0f);
        public float[][] testJointAnglesDegrees =
        {
            new[] { 0f, 0f, 0f },
            new[] { 45f, -20f, 15f },
            new[] { 100f, 20f, 30f }
        };

        private void Start()
        {
            RunValidation();
        }

        [ContextMenu("Run Kinematics Validation")]
        public void RunValidation()
        {
            if (parameters == null)
            {
                Debug.LogWarning("RobotKinematicsValidationRunner requires RobotKinematicParameters.");
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.AppendLine($"Kinematic validation for {parameters.modelName}");

            foreach (float[] angles in testJointAnglesDegrees)
            {
                ForwardKinematicsResult fk = RobotKinematics.ForwardKinematics(parameters, angles);
                HumanRobotDistanceResult distance = RobotKinematics.MinimumDistance(
                    new[] { sampleHumanPointMeters },
                    fk.LinkSegments);

                builder.AppendLine($"Angles deg: [{string.Join(", ", angles)}]");
                for (int i = 0; i < fk.JointPositions.Length; i++)
                {
                    builder.AppendLine($"  joint[{i}] = {fk.JointPositions[i]:F4} m");
                }

                builder.AppendLine($"  end effector = {fk.EndEffectorPosition:F4} m, phi = {fk.EndEffectorPlanarAngleDegrees:F2} deg");
                builder.AppendLine($"  min distance to sample human point = {distance.DistanceMeters:F4} m on link {distance.RobotLinkIndex}");
            }

            Debug.Log(builder.ToString());
        }
    }
}
