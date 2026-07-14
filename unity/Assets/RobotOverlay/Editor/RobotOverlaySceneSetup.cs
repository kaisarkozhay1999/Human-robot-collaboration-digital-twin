#if UNITY_EDITOR
using Microsoft.MixedReality.OpenXR;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.ARFoundation;

public static class RobotOverlaySceneSetup
{
    [InitializeOnLoadMethod]
    static void AutoSetupRoboticArmScene()
    {
        EditorApplication.delayCall += () =>
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            Scene activeScene = SceneManager.GetActiveScene();
            if (!activeScene.IsValid() || activeScene.name != "Robotic arm")
                return;

            if (GameObject.Find("RobotOverlayAlignment") != null)
                return;

            SetupRoboticArmOverlay();
            EditorSceneManager.SaveScene(activeScene);
        };
    }

    [MenuItem("SmartLab/Setup Robot QR Overlay")]
    public static void SetupRoboticArmOverlay()
    {
        GameObject overlayRoot = GameObject.Find("Arm_Control");
        if (overlayRoot == null)
            overlayRoot = GameObject.Find("Arm");

        if (overlayRoot == null)
        {
            Debug.LogError("Could not find Arm_Control or Arm in the active scene.");
            return;
        }

        Transform markerMount = overlayRoot.transform.Find("MarkerMountPoint_QR");
        if (markerMount == null)
        {
            GameObject markerObject = new GameObject("MarkerMountPoint_QR");
            Undo.RegisterCreatedObjectUndo(markerObject, "Create robot marker mount point");
            markerMount = markerObject.transform;
            markerMount.SetParent(overlayRoot.transform, false);
            markerMount.localPosition = Vector3.zero;
            markerMount.localRotation = Quaternion.identity;
            markerMount.localScale = Vector3.one;
        }

        GameObject alignmentObject = GameObject.Find("RobotOverlayAlignment");
        if (alignmentObject == null)
        {
            alignmentObject = new GameObject("RobotOverlayAlignment");
            Undo.RegisterCreatedObjectUndo(alignmentObject, "Create robot overlay alignment");
        }

        RobotMarkerAligner aligner = alignmentObject.GetComponent<RobotMarkerAligner>();
        if (aligner == null)
            aligner = Undo.AddComponent<RobotMarkerAligner>(alignmentObject);

        aligner.overlayRoot = overlayRoot.transform;
        aligner.markerMountPoint = markerMount;
        aligner.continuousAlignment = true;
        aligner.snapFirstAlignment = true;
        aligner.smoothing = 12f;
        aligner.CaptureMarkerReference();
        EditorUtility.SetDirty(aligner);

        ARSession session = Object.FindObjectOfType<ARSession>();
        if (session == null)
        {
            GameObject sessionObject = new GameObject("AR Session");
            Undo.RegisterCreatedObjectUndo(sessionObject, "Create AR Session");
            session = Undo.AddComponent<ARSession>(sessionObject);
        }

        ARMarkerManager markerManager = Object.FindObjectOfType<ARMarkerManager>();
        if (markerManager == null)
        {
            GameObject xrOrigin = FindXrOriginObject();
            if (xrOrigin == null)
            {
                Debug.LogWarning("No XROrigin/ARSessionOrigin found. Add ARMarkerManager manually to the XR Origin used by the HoloLens rig.");
            }
            else
            {
                markerManager = Undo.AddComponent<ARMarkerManager>(xrOrigin);
            }
        }

        if (markerManager != null)
        {
            markerManager.enabledMarkerTypes = new[] { ARMarkerType.QRCode };
            markerManager.defaultTransformMode = TransformMode.Center;
            EditorUtility.SetDirty(markerManager);
        }

        HoloLensQRMarkerPoseProvider provider = alignmentObject.GetComponent<HoloLensQRMarkerPoseProvider>();
        if (provider == null)
            provider = Undo.AddComponent<HoloLensQRMarkerPoseProvider>(alignmentObject);

        provider.aligner = aligner;
        provider.markerManager = markerManager;
        provider.acceptAnyQRCode = false;
        provider.requiredDecodedText = "robot-overlay";
        provider.useCenterTransformMode = true;
        provider.maxMarkerAgeSeconds = 1f;
        EditorUtility.SetDirty(provider);

        Selection.activeTransform = markerMount;
        EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());

        Debug.Log(
            "Robot QR overlay setup complete. Move Arm_Control/MarkerMountPoint_QR to the exact marker center/orientation on the virtual robot, then use a QR code with text 'robot-overlay'.",
            markerMount);
    }

    static GameObject FindXrOriginObject()
    {
        Unity.XR.CoreUtils.XROrigin xrOrigin = Object.FindObjectOfType<Unity.XR.CoreUtils.XROrigin>();
        if (xrOrigin != null)
            return xrOrigin.gameObject;

#pragma warning disable 618
        ARSessionOrigin arSessionOrigin = Object.FindObjectOfType<ARSessionOrigin>();
        if (arSessionOrigin != null)
            return arSessionOrigin.gameObject;
#pragma warning restore 618

        GameObject mrtkRig = GameObject.Find("MRTK XR Rig");
        if (mrtkRig != null)
            return mrtkRig;

        Camera mainCamera = Camera.main;
        return mainCamera != null ? mainCamera.gameObject : null;
    }
}
#endif
