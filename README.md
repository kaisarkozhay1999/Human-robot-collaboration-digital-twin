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

## Laboratory Safety-Assistance Repeated Trials

This repository also includes a repeated-trial validation workflow for the robot-arm digital twin. The workflow prepares protocol documents, run folders, log templates, column mapping, analysis tables, figures, and an honest Markdown report. It does not generate synthetic measurements and does not claim repeated trials were conducted unless real logs are present.

### 1. Prepare A New Run Folder

```bash
python python/analysis/repeated_trial_logger.py --root data/repeated_trials --scenario-config configs/repeated_trial_scenarios.csv --new-run
```

The logger creates the next non-overwriting `run_XX` folder with:

- `scenario_config.csv`
- `notes.md`
- `run_metadata.json`
- `runtime.csv`
- `safety.csv`
- `pose.csv`
- `robot_telemetry.csv`
- `command_log.csv`

The CSV files contain headers only until real laboratory data is copied or written into them.

### 2. Conduct The Physical Trial

Follow [docs/repeated_trial_protocol.md](docs/repeated_trial_protocol.md). Each scenario should be repeated at least three times for laboratory safety-assistance repeated trials.

Keep the measurement boundary clear:

- Runtime metrics describe software-stage runtime and command-generation behavior.
- Physical stop-time measurement is not independently measured unless an external timing reference is added.
- Do not edit logged measurements or enter fabricated rows.

### 3. Copy Real Logs Into The Run Folder

If the runtime pipeline writes logs elsewhere, copy the real files after the run:

- Python runtime metrics such as `python_frames.csv` can be copied to `runtime.csv`.
- Unity safety logs such as `unity_safety.csv` can be copied to `safety.csv`.
- Unity MQTT logs such as `unity_mqtt.csv` can be copied to `command_log.csv`.
- FK safety logs such as `fk_safety.csv` can be copied to `robot_telemetry.csv`.
- Pose-specific logs can be copied to `pose.csv` when available.

If column names differ, update [configs/column_mapping.yaml](configs/column_mapping.yaml) rather than changing the raw data.

### 4. Close Or Annotate The Run

```bash
python python/analysis/repeated_trial_logger.py --root data/repeated_trials --close-run run_01 --note "completed scenario sequence without manual intervention"
python python/analysis/repeated_trial_logger.py --root data/repeated_trials --append-note run_01 --note "S8 partial occlusion affected camera 2"
```

### 5. Analyze Repeated Trials

```bash
python python/analysis/analyze_repeated_trials.py --data-root data/repeated_trials --output analysis_outputs/repeated_trials
```

Outputs are written under `analysis_outputs/repeated_trials/`:

- `repeated_trial_report.md`
- `tables/per_trial_metrics.csv`
- `tables/per_scenario_metrics.csv`
- `tables/across_run_summary.csv`
- `tables/missing_data_warnings.csv`
- `figures/` with PNG, SVG, and PDF figures when the required data and matplotlib are available.

The report states whether multiple independent runs are actually present. If only one run exists, results are descriptive. If no real rows exist, the report says no repeated-trial results are reported.
