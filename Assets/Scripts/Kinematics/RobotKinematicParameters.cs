using System;
using UnityEngine;

namespace DigitalTwin.Kinematics
{
    [CreateAssetMenu(menuName = "Digital Twin/Robot Kinematic Parameters")]
    public sealed class RobotKinematicParameters : ScriptableObject
    {
        [Tooltip("Human-readable model identifier. Use this to distinguish placeholder and measured parameter sets.")]
        public string modelName = "ER9Pro_placeholder_5link_planar";

        [Tooltip("Base frame origin in Unity world meters.")]
        public Vector3 basePositionMeters = Vector3.zero;

        [Tooltip("Planar model local X direction in Unity world coordinates.")]
        public Vector3 horizontalAxis = Vector3.right;

        [Tooltip("Planar model local Y direction in Unity world coordinates.")]
        public Vector3 verticalAxis = Vector3.up;

        [Tooltip("Serial links ordered from base to end effector. Values are placeholders until replaced by measured robot dimensions.")]
        public RobotLinkParameter[] links =
        {
            new RobotLinkParameter("Base_to_shoulder_placeholder", 0.12f, 0f),
            new RobotLinkParameter("Shoulder_to_elbow_placeholder", 0.25f, 0f),
            new RobotLinkParameter("Elbow_to_wrist_pitch_placeholder", 0.20f, 0f),
            new RobotLinkParameter("Wrist_pitch_to_roll_placeholder", 0.08f, 0f),
            new RobotLinkParameter("Roll_to_tcp_placeholder", 0.06f, 0f)
        };

        public int JointCount => links == null ? 0 : links.Length;

        public bool IsValid(out string error)
        {
            if (links == null || links.Length == 0)
            {
                error = "Robot kinematic parameters must define at least one link.";
                return false;
            }

            for (int i = 0; i < links.Length; i++)
            {
                if (links[i].linkLengthMeters < 0f)
                {
                    error = $"Link {i} has a negative length.";
                    return false;
                }
            }

            if (horizontalAxis.sqrMagnitude <= Mathf.Epsilon || verticalAxis.sqrMagnitude <= Mathf.Epsilon)
            {
                error = "Horizontal and vertical axes must be non-zero vectors.";
                return false;
            }

            error = string.Empty;
            return true;
        }
    }

    [Serializable]
    public struct RobotLinkParameter
    {
        public string name;
        public float linkLengthMeters;
        public float jointOffsetDegrees;

        public RobotLinkParameter(string name, float linkLengthMeters, float jointOffsetDegrees)
        {
            this.name = name;
            this.linkLengthMeters = linkLengthMeters;
            this.jointOffsetDegrees = jointOffsetDegrees;
        }
    }
}
