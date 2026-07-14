# Robot Kinematics and Human-Robot Safety Validation Report

Date: 2026-06-27

Project: SmartLab Unity digital twin

Robot model: ER9Pro digital twin, calibrated 5-link planar FK model

Scene: `Assets/Scenes/Robotic arm.unity`

## 1. Purpose

This report documents the integration and validation of a kinematic model inside the Unity-based robot arm digital twin. The goal was to support journal reporting by showing:

- how robot joint telemetry enters the Unity kinematics module;
- how robot forward kinematics (FK) produces a TCP/end-effector estimate;
- how the FK TCP estimate is compared against the Unity robot TCP transform;
- how human 3D pose data is used for minimum human-robot distance monitoring;
- what validation values were measured before and after calibration;
- what the measured values mean and what they do not prove.

The main quantitative validation in this report compares the calibrated FK-predicted TCP position against the Unity robot TCP transform (`Joint 6`) in the same Unity world coordinate frame.

## 2. System Overview

The project is a Unity digital twin for a robot arm with human-robot safety monitoring. The current runtime system includes:

- robot visualization and joint control through `ER9ProFullController`;
- robot telemetry/control over MQTT;
- human pose input through Pose3D/Unity skeleton components;
- Pose3D safety packets over UDP port `5006`;
- Unity-side minimum-distance evaluation and CSV logging;
- robot stop/go command output through existing command paths.

The relevant implemented data flow is:

```text
Robot telemetry / Unity robot controller
    -> ER9ProFullController
    -> RobotSafetyDistanceEvaluator.latestJointAnglesDegrees
    -> RobotKinematics.ForwardKinematics(...)
    -> FK TCP and FK link segment estimate
    -> RobotKinematicsCsvLogger
    -> fk_safety.csv validation rows
```

For human-robot distance monitoring, the evaluator reads human joint positions from the live pose skeleton:

```text
Pose3D / Unity pose skeleton
    -> PoseBoneDriver.CopyActiveJointWorldPositions(...)
    -> RobotSafetyDistanceEvaluator.CurrentHumanJointPositions
    -> minimum distance from human joints to robot geometry
    -> Warning / Stop / Safe decision
```

The active robot command path for the latest safety behavior is:

```text
Python Pose3D safety packet
    -> UDP :5006
    -> RuntimeMetricsRecorder
    -> StopGoReceiver / ER9ProFullController
    -> robot stop/go command
```

Important implementation note: the Unity FK evaluator has `publishSafetyCommands` disabled in the scene so that it does not send duplicate stop commands. Pose3D safety packets currently provide the active robot command source. The FK evaluator still computes/logs distances and decisions for validation and debugging.

## 3. Kinematic Model

The FK model is represented by a JSON configuration and loaded into Unity at runtime.

Primary files:

- `Assets/Kinematics/robot_kinematics.placeholder.json`
- `Assets/Resources/Kinematics/robot_kinematics.placeholder.json`
- `Assets/Scripts/Kinematics/RobotKinematicJsonLoader.cs`
- `Assets/Scripts/Kinematics/RobotKinematics.cs`
- `Assets/Scripts/Kinematics/RobotSafetyDistanceEvaluator.cs`
- `Assets/Scripts/Kinematics/RobotKinematicsCsvLogger.cs`

The calibrated model name is:

```text
ER9Pro_calibrated_5link_planar_20260627
```

The calibrated Unity-frame parameters are:

| Parameter | Value |
| --- | ---: |
| Base offset x | -0.095718 m |
| Base offset y | -0.016238 m |
| Base offset z | -0.233129 m |
| Horizontal axis | (0.315535, 0.944675, -0.089592) |
| Vertical axis | (0.637409, -0.770331, -0.017308) |

The calibrated link and joint-offset parameters are:

| Link | Length (m) | Joint offset (deg) |
| --- | ---: | ---: |
| Base to shoulder | 0.130248 | -51.237877 |
| Shoulder to elbow | 0.013049 | 80.512719 |
| Elbow to wrist pitch | 0.036949 | -4.241506 |
| Wrist pitch to roll | 0.077613 | 34.574345 |
| Roll to TCP | 0.030883 | 114.459221 |

These values are an empirical Unity-frame calibration. They should not be presented as direct physical link dimensions unless independently verified against the real robot CAD or mechanical drawings. They are calibrated parameters that make the FK implementation reproduce the Unity robot TCP position.

## 4. What Was Compared

The validation compares two independent estimates of the robot TCP position in the same Unity world coordinate frame.

### 4.1 FK-predicted TCP

The FK-predicted TCP is generated from:

- the calibrated kinematic JSON;
- the current robot joint angles;
- the Unity robot base transform plus calibrated base offset.

In the validation CSV these columns are:

```text
fk_tcp_x, fk_tcp_y, fk_tcp_z
```

### 4.2 Unity robot TCP transform

The Unity reference TCP is the live visual robot transform assigned as `Joint 6`.

In the validation CSV these columns are:

```text
unity_tcp_x, unity_tcp_y, unity_tcp_z
```

### 4.3 Error definition

For each sample `i`, the TCP error is the Euclidean distance between the FK-predicted TCP and the Unity transform TCP:

```text
e_i = || p_FK,i - p_UnityTCP,i ||_2
```

The CSV column is:

```text
fk_vs_unity_tcp_error_m
```

This error measures how well the calibrated FK model reproduces the Unity robot end-effector position from the same joint angles. It does not directly measure the accuracy of the human pose model or the real-world robot position.

## 5. Calibration Method

The initial model produced a large mismatch between FK TCP and the Unity robot TCP. A calibration step was performed using a previously recorded validation log:

```text
%USERPROFILE%/AppData/LocalLow/GPV/Smart Lab/metrics/fk_safety_20260627_212333/fk_safety.csv
```

Calibration used rows where robot data was fresh and the Unity TCP marker was available. The fitting objective minimized TCP position error between the FK model and Unity `Joint 6`.

The fitted parameters included:

- base position offset;
- horizontal axis direction;
- vertical axis direction;
- link lengths;
- joint angle offsets.

The calibration did not change the conceptual FK formulation from `Kinematics.nb`; it tuned the Unity implementation so that the model's coordinate frame, joint zero conventions, and TCP alignment match the actual Unity digital twin hierarchy.

## 6. Validation Protocol

After calibration, the Unity scene was run again and the runtime CSV logger recorded FK/safety rows at approximately 10 Hz.

Latest validation log:

```text
%USERPROFILE%/AppData/LocalLow/GPV/Smart Lab/metrics/fk_safety_20260627_215200/fk_safety.csv
```

The logger wrote:

- Unity frame number and timestamp;
- safety decision;
- robot/human data freshness flags;
- minimum human-robot distance;
- closest human joint and robot point;
- FK TCP position;
- Unity TCP position;
- FK-vs-Unity TCP error;
- joint angles;
- FK joint positions;
- FK link segments;
- human joint positions.

The CSV logger was explicitly configured to compute the calibrated model FK for validation, even though the safety distance evaluator can use Unity visual robot geometry for robust stopping behavior.

## 7. Results

### 7.1 Pre-calibration FK-vs-Unity TCP error

Source log:

```text
fk_safety_20260627_212333/fk_safety.csv
```

| Metric | Value |
| --- | ---: |
| Rows | 786 |
| TCP error rows | 786 |
| Mean TCP error | 0.655859 m |
| RMSE TCP error | 0.661162 m |
| Minimum TCP error | 0.530252 m |
| Median TCP error | 0.637422 m |
| P90 TCP error | 0.791262 m |
| P95 TCP error | 0.798562 m |
| Maximum TCP error | 0.803506 m |

This showed that the original placeholder model did not match the Unity robot TCP frame.

### 7.2 Post-calibration FK-vs-Unity TCP error

Source log:

```text
fk_safety_20260627_215200/fk_safety.csv
```

All rows:

| Metric | Value |
| --- | ---: |
| Rows | 858 |
| TCP error rows | 858 |
| Mean TCP error | 0.013738 m |
| RMSE TCP error | 0.032755 m |
| Minimum TCP error | 0.000330 m |
| Median TCP error | 0.005310 m |
| P90 TCP error | 0.056221 m |
| P95 TCP error | 0.058130 m |
| Maximum TCP error | 0.711048 m |

One startup synchronization outlier occurred at Unity frame 1. This row had a TCP error of 0.711048 m and distorted the all-row RMSE. The outlier appeared before the runtime pose/model state settled at the start of Play mode. Therefore, both all-row and post-startup statistics are reported.

Post-startup rows, excluding frame 1:

| Metric | Value |
| --- | ---: |
| Rows used | 857 |
| Mean TCP error | 0.012925 m |
| RMSE TCP error | 0.022005 m |
| Minimum TCP error | 0.000330 m |
| Median TCP error | 0.005310 m |
| P90 TCP error | 0.056167 m |
| P95 TCP error | 0.058130 m |
| Maximum TCP error | 0.058413 m |

Equivalent interpretation:

- mean TCP error: approximately 1.29 cm;
- RMSE TCP error: approximately 2.20 cm;
- median TCP error: approximately 0.53 cm;
- P95 TCP error: approximately 5.81 cm;
- maximum post-startup TCP error: approximately 5.84 cm.

### 7.3 Improvement from calibration

| Metric | Before calibration | After calibration, excluding startup | Improvement |
| --- | ---: | ---: | ---: |
| Mean TCP error | 0.655859 m | 0.012925 m | 98.0% lower |
| RMSE TCP error | 0.661162 m | 0.022005 m | 96.7% lower |
| Median TCP error | 0.637422 m | 0.005310 m | 99.2% lower |
| P95 TCP error | 0.798562 m | 0.058130 m | 92.7% lower |
| Maximum TCP error | 0.803506 m | 0.058413 m | 92.7% lower |

The calibrated FK model substantially reduced the TCP mismatch relative to the Unity robot transform.

### 7.4 Human-robot distance and safety states in the latest run

Latest log:

```text
fk_safety_20260627_215200/fk_safety.csv
```

Safety distance statistics:

| Metric | Value |
| --- | ---: |
| Distance rows | 858 |
| Minimum distance | 0.109164 m |
| Median distance | 0.233747 m |
| P95 distance | 0.295477 m |
| Maximum distance | 0.421487 m |

Safety decisions:

| Decision | Count |
| --- | ---: |
| Stop | 778 |
| Warning | 80 |

Freshness:

| Data fresh flag | Count |
| --- | ---: |
| 1 | 858 |

The measured minimum distance fell below the configured stop threshold of 0.25 m, which explains the large number of `Stop` decisions. The warning threshold was 0.45 m, the stop threshold was 0.25 m, and the release threshold was 0.35 m.

Important distinction: these distance values validate the live human-robot proximity logic in the Unity scene, but they should not be confused with the FK TCP validation metric. The FK validation metric is `fk_vs_unity_tcp_error_m`; the safety distance metric is `min_distance_m`.

## 8. Interpretation for Journal Use

The calibrated FK implementation reproduced the Unity digital twin TCP with low post-startup error. After excluding a single startup synchronization outlier, the mean TCP error was 0.012925 m and the RMSE was 0.022005 m over 857 validation samples. The median error was 0.005310 m, indicating that most samples were very closely aligned, while the P95 error was 0.058130 m.

This supports the statement that the Unity-frame FK implementation was successfully calibrated to the digital twin robot hierarchy. The validation is internal to the Unity digital twin: it compares the analytic/calibrated FK estimate against the Unity robot transform reference. It does not by itself prove real-world absolute robot positioning accuracy, because no external motion-capture or physical TCP measurement was used in this validation.

Recommended wording:

```text
The forward-kinematic model was validated by comparing its predicted TCP position against the corresponding Unity digital-twin TCP transform. After calibration of the model frame, link parameters, and joint offsets, the post-startup validation run produced a mean TCP error of 0.0129 m and an RMSE of 0.0220 m over 857 samples. The median error was 0.0053 m and the 95th percentile error was 0.0581 m. These results indicate that the calibrated FK model reproduced the Unity robot end-effector position with centimeter-level agreement in the digital-twin coordinate frame.
```

Recommended limitation wording:

```text
The reported FK validation measures agreement between the calibrated kinematic model and the Unity digital-twin robot transform. It should be interpreted as digital-twin consistency rather than external metrology of the physical robot. Additional validation against physical TCP measurements or motion-capture ground truth would be required to quantify real-world absolute accuracy.
```

## 9. Connection to Kinematics.nb

`Kinematics.nb` provides the theoretical or nominal kinematic basis for the robot model. The Unity implementation converts that idea into runtime code that produces joint positions, link segments, and TCP position in Unity coordinates.

The calibration performed here justifies the Unity implementation by showing that, after frame and offset alignment, the FK output agrees with the Unity robot reference. For journal writing, the safest claim is:

```text
The kinematic formulation was implemented in Unity and empirically aligned to the digital-twin coordinate frame. The calibrated implementation was then validated against the Unity robot TCP transform.
```

Avoid claiming that the fitted link parameters are exact physical dimensions unless you separately validate them against the real robot geometry.

## 10. Runtime Integration Summary

### Robot joint angles

Live robot joint angles enter the evaluator in:

```text
Assets/Scripts/Kinematics/RobotSafetyDistanceEvaluator.cs
```

The method `ApplyLiveRobotTelemetry()` reads:

```text
robotController.GetCurrentJointAnglesDegrees()
robotController.LastRobotStateTelemetryTime
```

and passes them into:

```text
ApplyJointTelemetry(...)
```

### FK link segments

FK is computed by:

```text
RobotKinematics.ForwardKinematics(parameters, latestJointAnglesDegrees)
```

The CSV logger now explicitly uses this model FK path for validation columns.

### Human joints

Human joints enter through:

```text
PoseBoneDriver.CopyActiveJointWorldPositions(...)
```

inside:

```text
RobotSafetyDistanceEvaluator.ReadHumanJointPositions(...)
```

### Minimum distance

Minimum distance is computed through:

```text
RobotKinematics.MinimumDistance(CurrentHumanJointPositions, CurrentKinematics.LinkSegments)
```

The output is stored as:

```text
RobotSafetyDistanceEvaluator.CurrentDistance
```

and logged as:

```text
min_distance_m
closest_human_joint_index
closest_robot_link_index
closest_human_x/y/z
closest_robot_x/y/z
```

### Safety state

Safety decision logic uses:

- data freshness;
- gesture stop/go state;
- warning threshold;
- stop threshold;
- release threshold;
- hysteresis through the previous decision.

The latest validation used:

```text
warningThresholdMeters = 0.45
stopThresholdMeters = 0.25
releaseThresholdMeters = 0.35
```

### Active stop/go command path

The latest active stop/go command path is handled by:

```text
Assets/RuntimeMetricsRecorder.cs
```

It receives Pose3D safety packets on UDP port `5006` and publishes stop/go through the existing robot command path.

## 11. Debug and Validation Outputs

The Unity scene can visualize:

- robot joint points;
- robot link segments;
- TCP marker;
- human skeleton joints;
- closest-distance line;
- current minimum distance;
- current safety state.

The CSV validation logger writes one `fk_safety.csv` per run under:

```text
%USERPROFILE%/AppData/LocalLow/GPV/Smart Lab/metrics/fk_safety_YYYYMMDD_HHMMSS/fk_safety.csv
```

Important CSV columns:

| Column | Meaning |
| --- | --- |
| `fk_tcp_x/y/z` | calibrated FK TCP position |
| `unity_tcp_x/y/z` | Unity `Joint 6` TCP reference |
| `fk_vs_unity_tcp_error_m` | Euclidean TCP error in meters |
| `min_distance_m` | minimum human-robot distance |
| `decision` | Warning/Stop/Safe decision |
| `joint_angles_deg` | joint angles used by FK |
| `fk_joint_positions` | FK joint positions |
| `fk_link_segments` | FK link segment endpoints |
| `human_joint_positions` | human skeleton joint positions |

## 12. Reproducibility Steps

To reproduce the validation:

1. Open Unity scene:

```text
Assets/Scenes/Robotic arm.unity
```

2. Ensure `RobotKinematicsCsvLogger` is enabled and `logOnStart` is true.

3. Ensure `unityTcpMarker` is assigned to the robot `Joint 6` transform.

4. Run the existing human Pose3D/dual-camera pipeline so Unity receives live human joints and safety packets.

5. Press Play in Unity and let the simulation run for at least 30-60 seconds.

6. Stop Play mode.

7. Read the newest CSV file under:

```text
%USERPROFILE%/AppData/LocalLow/GPV/Smart Lab/metrics/fk_safety_*/
```

8. Calculate statistics for:

```text
fk_vs_unity_tcp_error_m
min_distance_m
decision
data_fresh
```

9. Exclude the first startup frame if it contains a clear synchronization outlier, and report both all-row and post-startup statistics if needed.

## 13. Limitations and Future Validation

The current validation is strong evidence that the FK implementation is consistent with the Unity digital twin. However:

- it is not an external measurement of physical robot TCP accuracy;
- the fitted model parameters are empirical Unity-frame parameters;
- the first frame of Play mode may contain a startup synchronization outlier;
- safety distances depend on the quality of Pose3D human joint estimation;
- the current active stop/go command path is driven by Pose3D safety packets, while FK/Unity distance logic is logged and available for validation/debugging;
- further physical validation would require independent ground truth, such as measured robot TCP positions, calibrated camera-space coordinates, or motion-capture markers.

Recommended future validation:

- compare Unity TCP against measured physical TCP positions at known joint configurations;
- record repeated trials with different human approach trajectories;
- report latency from camera frame to Unity command output;
- evaluate false stop and missed stop rates;
- compare Pose3D distance estimates against manually measured distances or calibrated marker positions.

## 14. Main Takeaway

The calibrated FK implementation reduced the digital-twin TCP mismatch from approximately 0.66 m RMSE before calibration to approximately 0.022 m RMSE after calibration, excluding the single startup outlier. This demonstrates centimeter-level agreement between the calibrated FK model and the Unity robot TCP transform in the digital twin coordinate frame.

For journal use, the most defensible claim is:

```text
The kinematic model was implemented and empirically calibrated in the Unity digital twin. Validation against the Unity robot TCP transform produced a post-startup mean TCP error of 0.0129 m and an RMSE of 0.0220 m over 857 samples, supporting the use of the calibrated model for digital-twin consistency analysis and runtime visualization.
```
