using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotKinematicsHud : MonoBehaviour
    {
        public RobotSafetyDistanceEvaluator evaluator;
        public Vector2 screenPosition = new Vector2(12f, 12f);
        public int width = 560;
        public int height = 110;

        private void OnGUI()
        {
            if (evaluator == null)
            {
                return;
            }

            Rect rect = new Rect(screenPosition.x, screenPosition.y, width, height);
            GUILayout.BeginArea(rect, GUI.skin.box);
            GUILayout.Label($"Safety: {evaluator.CurrentDecision}");
            GUILayout.Label(evaluator.EndEffectorDisplay);
            GUILayout.Label(evaluator.MinimumDistanceDisplay);
            GUILayout.EndArea();
        }
    }
}
