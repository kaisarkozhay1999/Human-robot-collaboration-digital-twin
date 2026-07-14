using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotKinematicsHud : MonoBehaviour
    {
        public RobotSafetyDistanceEvaluator evaluator;
        public Vector2 screenPosition = new Vector2(12f, 12f);
        public int width = 560;
        public int height = 320;

        private void Awake()
        {
            if (evaluator == null)
                evaluator = GetComponent<RobotSafetyDistanceEvaluator>();
        }

        private void OnGUI()
        {
            if (evaluator == null)
            {
                return;
            }

            Rect rect = new Rect(screenPosition.x, screenPosition.y, width, height);
            GUILayout.BeginArea(rect, GUI.skin.box);
            GUILayout.Label($"Safety: {evaluator.CurrentDecision}");
            GUILayout.Label($"Fresh data: robot={evaluator.RobotDataFresh}, human={evaluator.HumanDataFresh}");
            GUILayout.Label($"Human joints: {evaluator.ActiveHumanJointCount}, pose age: {FormatAge(evaluator.HumanPoseAgeSeconds)}");
            GUILayout.Label($"Thresholds: warning={evaluator.warningThresholdMeters:F3} m, stop={evaluator.stopThresholdMeters:F3} m, release={evaluator.releaseThresholdMeters:F3} m");
            GUILayout.Label($"Human-robot distance: {FormatDistance(evaluator.CurrentDistance)}");
            GUILayout.Label(FormatGestureState(evaluator));
            GUILayout.Label(FormatMqttState(evaluator));
            GUILayout.Label(evaluator.EndEffectorDisplay);
            if (evaluator.CurrentDistance.IsValid)
            {
                GUILayout.Label($"Closest human joint: {evaluator.CurrentDistance.HumanJointIndex} at {evaluator.CurrentDistance.HumanJointPosition:F3}");
                GUILayout.Label($"Closest robot link: {evaluator.CurrentDistance.RobotLinkIndex} at {evaluator.CurrentDistance.ClosestPointOnRobotLink:F3}");
            }
            GUILayout.EndArea();
        }

        private static string FormatAge(float ageSeconds)
        {
            return float.IsInfinity(ageSeconds) ? "unavailable" : $"{ageSeconds:F3} s";
        }

        private static string FormatDistance(HumanRobotDistanceResult distance)
        {
            return distance.IsValid ? $"{distance.DistanceMeters:F3} m" : "unavailable";
        }

        private static string FormatGestureState(RobotSafetyDistanceEvaluator evaluator)
        {
            if (evaluator.gestureReceiver == null)
                return "Gesture command: unavailable";

            return $"Gesture command: {evaluator.gestureReceiver.currentCommand}, age={FormatAge(evaluator.gestureReceiver.CommandAgeSeconds)}, active stop={evaluator.gestureStopRequested}, go={evaluator.gestureGoRequested}";
        }

        private static string FormatMqttState(RobotSafetyDistanceEvaluator evaluator)
        {
            if (evaluator.gestureReceiver == null)
                return "MQTT command output: unavailable";

            return "MQTT command output: " + evaluator.gestureReceiver.MqttStatusText;
        }
    }
}
