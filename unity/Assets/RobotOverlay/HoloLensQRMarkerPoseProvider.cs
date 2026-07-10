using Microsoft.MixedReality.OpenXR;
using UnityEngine;
using UnityEngine.XR.ARSubsystems;

[DisallowMultipleComponent]
public class HoloLensQRMarkerPoseProvider : MonoBehaviour
{
    [Header("References")]
    public ARMarkerManager markerManager;
    public RobotMarkerAligner aligner;

    [Header("QR Filter")]
    public bool acceptAnyQRCode = true;
    public string requiredDecodedText = "robot-overlay";
    public bool useCenterTransformMode = true;
    public float maxMarkerAgeSeconds = 1.0f;

    [Header("Diagnostics")]
    public bool logDetections = true;
    public string LastDecodedText { get; private set; }
    public float LastSeenTime { get; private set; }

    void Awake()
    {
        AutoAssignIfNeeded();
    }

    void OnEnable()
    {
        AutoAssignIfNeeded();

        if (markerManager != null)
        {
            markerManager.enabledMarkerTypes = new[] { ARMarkerType.QRCode };
            markerManager.defaultTransformMode = useCenterTransformMode
                ? TransformMode.Center
                : TransformMode.MostStable;
            markerManager.markersChanged += OnMarkersChanged;
        }
    }

    void OnDisable()
    {
        if (markerManager != null)
            markerManager.markersChanged -= OnMarkersChanged;
    }

    void AutoAssignIfNeeded()
    {
        if (markerManager == null)
            markerManager = FindObjectOfType<ARMarkerManager>();

        if (aligner == null)
            aligner = FindObjectOfType<RobotMarkerAligner>();
    }

    void OnMarkersChanged(ARMarkersChangedEventArgs args)
    {
        for (int i = 0; i < args.added.Count; i++)
            TryUseMarker(args.added[i]);

        for (int i = 0; i < args.updated.Count; i++)
            TryUseMarker(args.updated[i]);
    }

    void TryUseMarker(ARMarker marker)
    {
        if (marker == null || aligner == null)
            return;

        if (marker.trackingState != TrackingState.Tracking)
            return;

        if (Time.realtimeSinceStartup - marker.lastSeenTime > maxMarkerAgeSeconds)
            return;

        if (useCenterTransformMode && marker.transformMode != TransformMode.Center)
            marker.transformMode = TransformMode.Center;

        string decodedText = marker.GetDecodedString();
        if (!acceptAnyQRCode && decodedText != requiredDecodedText)
            return;

        LastDecodedText = decodedText;
        LastSeenTime = marker.lastSeenTime;

        if (aligner.TryAlignToMarkerPose(marker.transform.position, marker.transform.rotation, decodedText) &&
            logDetections)
        {
            Debug.Log(
                $"QR marker aligned robot. Text='{decodedText}', size={marker.size}, position={marker.transform.position}.",
                this);
        }
    }
}
