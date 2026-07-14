using System.Text;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotKinematicsValidationRunner : MonoBehaviour
    {
        public RobotKinematicParameters parameters;
        public global::ER9ProFullController robotController;
        public bool useLiveRobotControllerAngles = true;

        [Tooltip("Manual joint angles in degrees. Used when no robot controller is assigned or live angles are disabled.")]
        public float[] manualJointAnglesDegrees = { 0f, 0f, 0f, 0f, 0f };

        [Tooltip("Optional visual TCP marker from the Unity robot model for FK-vs-Unity comparison.")]
        public Transform unityTcpMarker;

        public bool runOnStart;

        private void Start()
        {
            if (runOnStart)
                RunValidation();
        }

        [ContextMenu("Run Kinematics Validation")]
        public void RunValidation()
        {
            EnsureParameters();
            if (parameters == null)
            {
                Debug.LogWarning("RobotKinematicsValidationRunner requires RobotKinematicParameters.");
                return;
            }

            float[] angles = GetInputAngles();
            ForwardKinematicsResult fk = RobotKinematics.ForwardKinematics(parameters, angles);

            StringBuilder builder = new StringBuilder();
            builder.AppendLine($"Kinematic validation for {parameters.modelName}");
            builder.AppendLine($"Input joint angles deg: [{string.Join(", ", angles)}]");

            for (int i = 0; i < fk.JointPositions.Length; i++)
                builder.AppendLine($"FK joint[{i}] = {fk.JointPositions[i]:F4} m");

            builder.AppendLine($"FK TCP/end effector = {fk.EndEffectorPosition:F4} m, phi = {fk.EndEffectorPlanarAngleDegrees:F2} deg");

            if (unityTcpMarker != null)
            {
                float error = Vector3.Distance(fk.EndEffectorPosition, unityTcpMarker.position);
                builder.AppendLine($"Unity TCP_Marker = {unityTcpMarker.position:F4} m");
                builder.AppendLine($"FK-vs-Unity TCP error = {error:F4} m");
            }
            else
            {
                builder.AppendLine("Unity TCP_Marker = not assigned");
                builder.AppendLine("FK-vs-Unity TCP error = unavailable");
            }

            Debug.Log(builder.ToString());
        }

        private void EnsureParameters()
        {
            if (parameters != null)
                return;

            RobotKinematicJsonLoader loader = GetComponent<RobotKinematicJsonLoader>();
            if (loader != null && loader.LoadedParameters != null)
            {
                parameters = loader.LoadedParameters;
                return;
            }

            TextAsset json = Resources.Load<TextAsset>("Kinematics/robot_kinematics.placeholder");
            if (json != null)
                parameters = RobotKinematicJsonLoader.Load(json.text);
        }

        private float[] GetInputAngles()
        {
            if (useLiveRobotControllerAngles)
            {
                if (robotController == null)
                    robotController = FindObjectOfType<global::ER9ProFullController>();

                if (robotController != null)
                    return robotController.GetCurrentJointAnglesDegrees();
            }

            int count = parameters == null ? 0 : parameters.JointCount;
            if (manualJointAnglesDegrees == null)
                return new float[count];

            if (manualJointAnglesDegrees.Length >= count)
                return (float[])manualJointAnglesDegrees.Clone();

            float[] padded = new float[count];
            manualJointAnglesDegrees.CopyTo(padded, 0);
            return padded;
        }
    }
}
