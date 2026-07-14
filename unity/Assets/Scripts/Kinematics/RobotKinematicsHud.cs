using UnityEngine;

namespace DigitalTwin.Kinematics
{
    public sealed class RobotKinematicsHud : MonoBehaviour
    {
        public RobotSafetyDistanceEvaluator evaluator;
        public Vector2 screenPosition = new Vector2(12f, 12f);
        public int width = 560;
        public int height = 150;

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
            GUILayout.Label(evaluator.EndEffectorDisplay);
            GUILayout.Label(evaluator.MinimumDistanceDisplay);
            GUILayout.EndArea();
        }
    }
}
