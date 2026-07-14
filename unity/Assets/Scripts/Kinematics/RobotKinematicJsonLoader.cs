using System;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotKinematicJsonLoader : MonoBehaviour
    {
        [Tooltip("JSON file using the same schema as Config/robot_kinematics.placeholder.json.")]
        public TextAsset jsonConfig;

        [Tooltip("Optional Resources fallback without .json extension, used when jsonConfig is not assigned.")]
        public string resourcesFallbackPath = "Kinematics/robot_kinematics.placeholder";

        [Tooltip("Optional evaluator to receive the loaded parameter asset at startup.")]
        public RobotSafetyDistanceEvaluator targetEvaluator;

        public RobotKinematicParameters LoadedParameters { get; private set; }

        private void Awake()
        {
            if (targetEvaluator == null)
                targetEvaluator = GetComponent<RobotSafetyDistanceEvaluator>();

            TextAsset source = jsonConfig;
            if (source == null && !string.IsNullOrWhiteSpace(resourcesFallbackPath))
                source = Resources.Load<TextAsset>(resourcesFallbackPath);

            if (source == null)
            {
                return;
            }

            LoadedParameters = Load(source.text);
            if (targetEvaluator != null)
            {
                targetEvaluator.parameters = LoadedParameters;
            }
        }

        public static RobotKinematicParameters Load(string json)
        {
            if (string.IsNullOrWhiteSpace(json))
            {
                throw new ArgumentException("Robot kinematic JSON is empty.", nameof(json));
            }

            RobotKinematicJson parsed = JsonUtility.FromJson<RobotKinematicJson>(json);
            if (parsed == null || parsed.links == null || parsed.links.Length == 0)
            {
                throw new ArgumentException("Robot kinematic JSON must define a non-empty links array.", nameof(json));
            }

            RobotKinematicParameters parameters = ScriptableObject.CreateInstance<RobotKinematicParameters>();
            parameters.modelName = parsed.modelName;
            parameters.basePositionMeters = parsed.basePositionMeters;
            parameters.horizontalAxis = parsed.horizontalAxis;
            parameters.verticalAxis = parsed.verticalAxis;
            parameters.links = parsed.links;
            return parameters;
        }

        #pragma warning disable 0649
        [Serializable]
        private sealed class RobotKinematicJson
        {
            public string modelName;
            public Vector3 basePositionMeters;
            public Vector3 horizontalAxis = Vector3.right;
            public Vector3 verticalAxis = Vector3.up;
            public RobotLinkParameter[] links;
        }
        #pragma warning restore 0649
    }
}
