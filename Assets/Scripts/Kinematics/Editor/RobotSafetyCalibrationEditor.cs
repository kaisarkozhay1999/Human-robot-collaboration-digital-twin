using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace DigitalTwin.Kinematics.Editor
{
    public static class RobotSafetyCalibrationEditor
    {
        [MenuItem("SmartLab/Safety/Reset Pose Scale To Original")]
        public static void ResetPoseScaleToOriginal()
        {
            PoseBoneDriver[] drivers = Resources.FindObjectsOfTypeAll<PoseBoneDriver>();
            int changed = 0;

            foreach (PoseBoneDriver driver in drivers)
            {
                if (driver == null || !driver.gameObject.scene.IsValid())
                    continue;

                Undo.RecordObject(driver, "Reset PoseBoneDriver scale to original");
                driver.poseScale = 0.1f;
                driver.singleAnchorScale = 0.1f;
                EditorUtility.SetDirty(driver);
                changed++;
            }

            if (changed > 0)
            {
                EditorSceneManager.MarkAllScenesDirty();
                Debug.Log($"Reset PoseBoneDriver poseScale and singleAnchorScale to 0.1 on {changed} scene object(s).");
            }
            else
            {
                Debug.LogWarning("No scene PoseBoneDriver objects found to reset.");
            }
        }

        [MenuItem("SmartLab/Safety/Set Trial Proximity Thresholds")]
        public static void SetTrialProximityThresholds()
        {
            RobotSafetyDistanceEvaluator[] evaluators = Resources.FindObjectsOfTypeAll<RobotSafetyDistanceEvaluator>();
            int changed = 0;

            foreach (RobotSafetyDistanceEvaluator evaluator in evaluators)
            {
                if (evaluator == null || !evaluator.gameObject.scene.IsValid())
                    continue;

                Undo.RecordObject(evaluator, "Set trial proximity thresholds");
                evaluator.warningThresholdMeters = 0.32f;
                evaluator.stopThresholdMeters = 0.25f;
                evaluator.releaseThresholdMeters = 0.30f;
                evaluator.gestureCommandFreshSeconds = 2.00f;
                evaluator.publishSafetyCommands = true;
                evaluator.publishResumeOnRelease = true;
                evaluator.repeatStopCommandSeconds = 0.00f;
                EditorUtility.SetDirty(evaluator);
                changed++;
            }

            if (changed > 0)
            {
                EditorSceneManager.MarkAllScenesDirty();
                Debug.Log($"Set RobotSafetyDistanceEvaluator thresholds to warning=0.32 m, stop=0.25 m, release=0.30 m, gesture freshness=2.00 s, and enabled transition-only Raspberry safety command publishing on {changed} scene object(s).");
            }
            else
            {
                Debug.LogWarning("No scene RobotSafetyDistanceEvaluator objects found to configure.");
            }
        }
    }
}
