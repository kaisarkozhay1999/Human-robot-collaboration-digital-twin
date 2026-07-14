# Robot Overlay Alignment

This folder adds marker-based alignment for the HoloLens robot overlay.

## HoloLens path

The installed Microsoft Mixed Reality OpenXR package supports QR marker tracking on HoloLens 2. It does not support ArUco directly. For direct on-device tracking, print a QR code whose decoded text is:

```text
robot-overlay
```

## Scene setup

Open the `Robotic arm` scene in Unity, then run:

```text
SmartLab > Setup Robot QR Overlay
```

The setup creates:

- `RobotOverlayAlignment`
- `Arm_Control/MarkerMountPoint_QR`
- an `AR Session` if missing
- an `ARMarkerManager` on the XR origin if Unity can find it

Move `Arm_Control/MarkerMountPoint_QR` to the exact center and orientation of the physical marker on the virtual robot. The marker point is the calibration reference.

## Runtime behavior

When HoloLens detects the QR code, `HoloLensQRMarkerPoseProvider` passes the QR pose to `RobotMarkerAligner`. The aligner moves the virtual robot so the Unity marker point matches the real marker pose:

```text
World_Robot = World_DetectedMarker * inverse(Robot_LocalMarker)
```

After alignment, the existing robot controller can continue updating the virtual joints from MQTT.

## ArUco path

ArUco is still possible, but it needs an OpenCV/OpenCVForUnity detector or an external detector that sends a full marker pose. The same `RobotMarkerAligner.TryAlignToMarkerPose` API can be reused for that provider.
