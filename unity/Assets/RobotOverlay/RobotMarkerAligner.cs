using UnityEngine;

public class RobotMarkerAligner : MonoBehaviour
{
    [Header("Robot")]
    public Transform overlayRoot;
    public Transform markerMountPoint;

    [Header("Detected Marker Adjustment")]
    public Vector3 markerPositionOffset;
    public Vector3 markerRotationOffsetEuler;

    [Header("Alignment")]
    public bool continuousAlignment = true;
    public bool snapFirstAlignment = true;
    public float smoothing = 12f;
    public bool logAlignment;

    public bool IsAligned { get; private set; }
    public string LastMarkerId { get; private set; }
    public float LastAlignmentTime { get; private set; }
    public Pose LastMarkerPose { get; private set; }

    Matrix4x4 rootToMarkerLocal = Matrix4x4.identity;
    bool hasMarkerReference;

    void Awake()
    {
        AutoAssignIfNeeded();
        CaptureMarkerReference();
    }

    void OnValidate()
    {
        smoothing = Mathf.Max(0f, smoothing);
    }

    public void CaptureMarkerReference()
    {
        hasMarkerReference = false;

        if (overlayRoot == null || markerMountPoint == null)
            return;

        rootToMarkerLocal = overlayRoot.worldToLocalMatrix * markerMountPoint.localToWorldMatrix;
        hasMarkerReference = true;
    }

    public bool TryAlignToMarkerPose(Pose detectedMarkerPose, string markerId = null)
    {
        return TryAlignToMarkerPose(detectedMarkerPose.position, detectedMarkerPose.rotation, markerId);
    }

    public bool TryAlignToMarkerPose(Vector3 detectedMarkerPosition, Quaternion detectedMarkerRotation, string markerId = null)
    {
        AutoAssignIfNeeded();

        if (!hasMarkerReference)
            CaptureMarkerReference();

        if (overlayRoot == null || markerMountPoint == null || !hasMarkerReference)
        {
            Debug.LogWarning("Robot marker alignment skipped: overlayRoot or markerMountPoint is not assigned.", this);
            return false;
        }

        if (IsAligned && !continuousAlignment)
            return true;

        Quaternion adjustedMarkerRotation = detectedMarkerRotation * Quaternion.Euler(markerRotationOffsetEuler);
        Vector3 adjustedMarkerPosition = detectedMarkerPosition + detectedMarkerRotation * markerPositionOffset;

        Matrix4x4 markerWorld = Matrix4x4.TRS(adjustedMarkerPosition, adjustedMarkerRotation, Vector3.one);
        Matrix4x4 rootWorld = markerWorld * rootToMarkerLocal.inverse;

        Vector3 targetPosition = rootWorld.GetColumn(3);
        Quaternion targetRotation = ExtractRotation(rootWorld);

        bool snap = !IsAligned && snapFirstAlignment || smoothing <= 0f;
        if (snap)
        {
            overlayRoot.SetPositionAndRotation(targetPosition, targetRotation);
        }
        else
        {
            float t = 1f - Mathf.Exp(-smoothing * Time.deltaTime);
            overlayRoot.position = Vector3.Lerp(overlayRoot.position, targetPosition, t);
            overlayRoot.rotation = Quaternion.Slerp(overlayRoot.rotation, targetRotation, t);
        }

        IsAligned = true;
        LastMarkerId = markerId;
        LastAlignmentTime = Time.time;
        LastMarkerPose = new Pose(detectedMarkerPosition, detectedMarkerRotation);

        if (logAlignment)
        {
            Debug.Log(
                $"Aligned '{overlayRoot.name}' to marker '{markerId ?? "<any>"}' at {detectedMarkerPosition}.",
                this);
        }

        return true;
    }

    public void ResetAlignmentState()
    {
        IsAligned = false;
        LastMarkerId = null;
        LastAlignmentTime = 0f;
    }

    void AutoAssignIfNeeded()
    {
        if (overlayRoot == null)
        {
            GameObject rootObject = GameObject.Find("Arm_Control");
            if (rootObject == null)
                rootObject = GameObject.Find("Arm");
            if (rootObject != null)
                overlayRoot = rootObject.transform;
        }

        if (markerMountPoint == null && overlayRoot != null)
        {
            Transform existing = overlayRoot.Find("MarkerMountPoint_QR");
            if (existing == null)
                existing = overlayRoot.Find("MarkerMountPoint");
            markerMountPoint = existing;
        }
    }

    static Quaternion ExtractRotation(Matrix4x4 matrix)
    {
        Vector3 forward = matrix.GetColumn(2);
        Vector3 up = matrix.GetColumn(1);

        if (forward.sqrMagnitude < 0.000001f)
            forward = Vector3.forward;
        if (up.sqrMagnitude < 0.000001f)
            up = Vector3.up;

        return Quaternion.LookRotation(forward.normalized, up.normalized);
    }

    void OnDrawGizmosSelected()
    {
        if (markerMountPoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(markerMountPoint.position, Vector3.one * 0.04f);
            Gizmos.DrawRay(markerMountPoint.position, markerMountPoint.forward * 0.08f);
            Gizmos.DrawRay(markerMountPoint.position, markerMountPoint.up * 0.05f);
        }

        if (IsAligned)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(LastMarkerPose.position, 0.04f);
            Gizmos.DrawRay(LastMarkerPose.position, LastMarkerPose.rotation * Vector3.forward * 0.08f);
        }
    }
}
