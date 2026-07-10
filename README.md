# Human-Robot Collaboration Digital Twin

This repository contains the robot-human collaboration part of the SmartLab digital twin. It combines a Unity mixed-reality robot scene with Python stereo camera, YOLO pose, robot detection, calibration, UDP streaming, MQTT control, runtime metrics, and human-robot safety distance evaluation.

## Repository Layout

```text
unity/                  Unity 2022.3 project subset for the robot-human digital twin
  Assets/               Robot scene, prefabs, scripts, materials, TextMeshPro assets, M2MQTT, XR settings
  Packages/             Unity package manifest plus local MRTK/OpenXR package archives
  ProjectSettings/      Unity project settings copied from the source project

python/
  requirements.txt      Python dependencies for the live camera and safety pipelines
  yolo_pose_test/       Pose, stereo, robot detection, calibration, metrics, and Unity UDP scripts

kinematics_reference/
  DT/                   Standalone/reference kinematics layer, tests, docs, and config

examples/
  videos/               Demonstration videos of the Unity and HoloLens workflows
```

Generated Unity caches, builds, Python virtual environments, captured datasets, large metrics logs, and camera image dumps are intentionally excluded.

## Main Runtime Flow

```text
RTSP stereo cameras
  -> Python YOLO pose and stereo triangulation
  -> UDP pose packets to Unity
  -> PoseBoneDriver updates the human skeleton overlay
  -> ER9ProFullController reads robot joint telemetry over MQTT
  -> RobotSafetyDistanceEvaluator computes human-robot minimum distance
  -> StopGoReceiver / robot controller publishes stop or go commands
```

## Unity Setup

1. Install Unity `2022.3.62f3` or a compatible Unity 2022.3 LTS editor.
2. Open the `unity/` folder as a Unity project.
3. Open the scene:

```text
Assets/Scenes/Robotic arm.unity
```

4. Check these key components in the scene:

- `ER9ProFullController`: robot MQTT telemetry and control.
- `PoseBoneDriver`: receives Python 3D pose packets over UDP, default port `5005`.
- `RuntimeMetricsRecorder`: starts automatically and listens for safety metrics on UDP `5006`.
- `StopGoReceiver`: receives stop/go UDP commands on `5007` and republishes to MQTT.
- `RobotSafetyDistanceEvaluator`: evaluates robot-human distance and safety state.

5. Replace lab-specific MQTT broker IPs and topics in the Unity Inspector for your environment.

## Python Setup

From the repository root:

```powershell
cd python
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Set your camera streams before running live scripts. Do not commit credentials.

```powershell
$env:SMARTLAB_CAM1_HIGH = "rtsp://USER:PASSWORD@CAMERA_1_IP/h264"
$env:SMARTLAB_CAM2_HIGH = "rtsp://USER:PASSWORD@CAMERA_2_IP/h264"
$env:SMARTLAB_CAM1_LOW  = "rtsp://USER:PASSWORD@CAMERA_1_IP/mpeg4cif"
$env:SMARTLAB_CAM2_LOW  = "rtsp://USER:PASSWORD@CAMERA_2_IP/mpeg4cif"
$env:SMARTLAB_FFMPEG_EXE = "C:\path\to\ffmpeg.exe"
```

Run the main pose-to-Unity pipeline:

```powershell
cd yolo_pose_test
..\.venv\Scripts\python.exe pose3d_to_unity.py
```

For the safety pipeline with robot detection:

```powershell
..\.venv\Scripts\python.exe test_dual_stream_pipeline.py
```

The included default model files are:

- `yolo26n-pose.pt`
- `yolov8n-pose.pt`
- `yolo26n.pt`
- `runs/detect/robot_detector/weights/best.pt`

## Demonstration Videos

Example videos are included in:

```text
examples/videos/
```

- `demonstration_1_unity.mp4`: Unity view showing the robot, human pose, cameras, and calibration elements cooperating to create the digital twin.
- `demonstration_2_hololens.mp4`: HoloLens recording of the mixed-reality digital twin experience.

## Calibration And Robot Dimensions

Stereo calibration files are in `python/yolo_pose_test/`:

- `stereo_calibration_charuco_live.npz`
- `stereo_calibration_charuco_refined.npz`
- `stereo_calibration_charuco.npz`

Unity kinematic placeholder dimensions are in:

```text
unity/Assets/Resources/Kinematics/robot_kinematics.placeholder.json
```

Replace these placeholder link lengths and offsets with measured robot dimensions before using the safety output as an engineering result.

## Validation

Run the Python kinematics tests:

```powershell
cd kinematics_reference\DT\python
python -m unittest discover -p "test_*.py" -v
```

In Unity, attach or enable `RobotKinematicsValidationRunner` to print joint angles, FK joint positions, TCP position, and optional FK-vs-Unity TCP error.

## Notes

- This repository is a focused extraction of the robot-human digital twin, not the full SmartLab scene.
- The source project folder was not modified during export.
- Camera credentials were removed from the exported Python defaults. Use environment variables or command-line arguments for private stream URLs.
