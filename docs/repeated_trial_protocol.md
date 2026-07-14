# Laboratory Safety-Assistance Repeated-Trial Protocol

This protocol prepares repeated laboratory trials for the robot-arm digital twin validation workflow. It is intended for laboratory safety-assistance validation of the bidirectional digital twin pipeline, including Unity visualization/control, robot telemetry, RTSP cameras, stereo calibration, marker-based registration, YOLO-based human pose estimation, robot detection, forward kinematics, and proximity- or gesture-based warning/stop/release logic.

The protocol measures software-stage runtime, safety-state behavior, stop/resume transitions, distance-source availability, proximity-triggered stop behavior, gesture-triggered stop behavior, release behavior, robustness under occlusion or missing data, and repeatability across independent runs.

Important measurement boundary:

- Runtime values reported by the software logs describe software-stage processing, communication, and command-generation behavior when the corresponding columns are available.
- Physical stop-time measurement is not independently measured unless external timing is added, such as a synchronized high-speed camera, robot-controller timestamp, or hardware trigger.
- Laboratory stop/resume behavior must be interpreted as system behavior observed in this experimental setup, not as a formal safety assessment.

## General Setup

- Verify the robot workcell is supervised by the responsible operator.
- Confirm that the robot emergency stop is accessible before each trial.
- Confirm that the robot trajectory, speed, and workspace are appropriate for laboratory validation.
- Confirm that the Unity digital twin, robot telemetry, MQTT/UDP command path, RTSP cameras, stereo calibration, marker registration, pose estimator, robot detector, and forward-kinematics modules are running.
- Confirm that software logging is enabled for runtime, safety state, pose, robot telemetry, and command events where available.
- Create a new run folder with `python/analysis/repeated_trial_logger.py --root data/repeated_trials --scenario-config configs/repeated_trial_scenarios.csv --new-run`.
- Perform at least three repetitions per scenario. A run folder may contain a complete scenario sequence or one scenario, as long as `scenario_config.csv` records the start and end time for each trial.
- Do not overwrite previous run folders.
- Record manual notes immediately after each trial, especially unexpected robot behavior, lighting changes, occlusion, camera issues, manual intervention, and operator deviations from the planned action.

## Logs To Collect

Each run folder should contain:

- `scenario_config.csv`: trial segmentation and expected behavior.
- `notes.md`: operator notes, setup notes, safety observations, deviations, and timestamps.
- `runtime.csv`: software-stage timing, frame rate, and stage latencies.
- `safety.csv`: safety state, stop flag, release flag if available, stale-data status, distance, and distance source.
- `pose.csv`: pose-detection availability, confidence, tracked joints, and camera/triangulation status.
- `robot_telemetry.csv`: robot joint state, forward-kinematics state, command acknowledgement, and distance from FK-based evaluator if available.
- `command_log.csv`: stop/resume command generation, UDP/MQTT publication, acknowledgement if available, and send latency if available.

If the runtime pipeline writes logs elsewhere, copy the real log files into the current run folder after the run. Do not modify logged data values.

## Pass/Fail Interpretation

The following criteria are intended for laboratory safety-assistance repeated trials:

- A trial passes if its observed software states, stop/resume commands, stale-data handling, and distance-source behavior match the expected behavior for the scenario without manual safety intervention beyond the planned procedure.
- A trial fails if the expected warning/stop/release behavior is absent, if a stop command is missed when the expected state requires STOP, if a false stop occurs in a SAFE-only scenario, or if missing/stale inputs are not detected conservatively when required.
- A trial is marked inconclusive if logs are missing, timestamps are unusable, the scenario was not performed as planned, or a manual operator intervention prevents interpretation.

## Scenario S1_human_far_static

- Scenario ID: `S1_human_far_static`
- Scenario name: Human far static
- Objective: Check baseline SAFE behavior and false-stop rate when the human is clearly outside the warning and stop zones.
- Setup: Robot is visible to the cameras. Human stands in a marked safe region away from the robot. Normal lighting and camera placement.
- Start condition: Robot/digital twin telemetry and camera streams are stable. Human is already in the far region.
- Operator action: Stand still or make only small natural posture changes.
- Robot state/action: Robot may be idle or executing a low-risk repeatable motion that remains clear of the human.
- Expected safety behavior: SAFE state remains active. No proximity stop or gesture stop should be generated.
- Duration: 20-30 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if SAFE is maintained and no stop command is generated. Fail if a STOP command occurs without a planned gesture or proximity cause.
- Manual notes to record: Human position, robot motion status, visible pose quality, robot visibility, lighting, and any false stop or warning.
- Safety precautions: Keep the human outside all marked warning/stop regions and maintain emergency-stop access.

## Scenario S2_human_approaches_robot

- Scenario ID: `S2_human_approaches_robot`
- Scenario name: Human approaches robot
- Objective: Validate SAFE to WARNING to STOP behavior as human-robot distance decreases.
- Setup: Human begins in the far safe region. Robot is visible. The approach path is marked and free of obstacles.
- Start condition: System is in SAFE state with valid pose and robot detection.
- Operator action: Slowly approach the robot along the marked path without abrupt movement.
- Robot state/action: Robot may be idle or moving in the predefined laboratory trajectory.
- Expected safety behavior: State sequence should progress from SAFE to WARNING to STOP as distance decreases. A stop command should be generated when the stop threshold is crossed.
- Duration: 20-30 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if warning/stop behavior is detected in the expected order and a stop command is generated at or after the unsafe approach condition. Fail if the stop condition is missed.
- Manual notes to record: Approach path, approximate crossing time of warning/stop markers, any pose loss, and any operator hesitation.
- Safety precautions: Approach slowly, keep the operator prepared to stop, and do not enter unplanned robot contact regions.

## Scenario S3_human_inside_stop_zone

- Scenario ID: `S3_human_inside_stop_zone`
- Scenario name: Human inside stop zone
- Objective: Check that STOP remains active while the human remains inside the unsafe/stop zone.
- Setup: Human stands in the pre-marked stop zone with robot motion constrained for the trial.
- Start condition: System has detected the unsafe proximity condition or begins with the human inside the stop zone.
- Operator action: Remain inside the stop zone with minimal movement.
- Robot state/action: Robot should remain stopped or inhibited by the laboratory stop/resume behavior.
- Expected safety behavior: STOP state remains active and no premature release is generated.
- Duration: 15-20 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if STOP remains active for the unsafe interval. Fail if SAFE or RELEASE is generated before the human exits the stop zone.
- Manual notes to record: Body pose, closest body part to robot, whether both cameras see the human, and any stale-data indication.
- Safety precautions: Use a low-risk robot state, keep the emergency stop accessible, and avoid physical contact with the robot.

## Scenario S4_human_moves_away_release

- Scenario ID: `S4_human_moves_away_release`
- Scenario name: Human moves away for release
- Objective: Validate STOP to RELEASE/SAFE behavior after the human moves outside the release threshold and any confirmation condition is satisfied.
- Setup: Human starts inside or near the stop zone. Release threshold is marked if possible.
- Start condition: STOP is active or expected to become active at the start of the trial.
- Operator action: Slowly move away from the robot until clearly beyond the release threshold.
- Robot state/action: Robot remains stopped until release/resume conditions are satisfied.
- Expected safety behavior: STOP transitions to RELEASE or SAFE only after the release threshold and confirmation condition are satisfied.
- Duration: 20-30 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if release/resume occurs only under safe proximity conditions. Fail if release occurs while the human is still inside the unsafe zone or if release never occurs despite valid safe conditions.
- Manual notes to record: Approximate time of movement away, threshold crossing, resume command timing if visible, and any manual override.
- Safety precautions: Move slowly and keep the robot in a supervised laboratory trajectory.

## Scenario S5_dynamic_motion_near_boundary

- Scenario ID: `S5_dynamic_motion_near_boundary`
- Scenario name: Dynamic motion near boundary
- Objective: Check hysteresis and repeatability when the human moves near the warning/stop/release boundary.
- Setup: Mark warning, stop, and release threshold regions. Robot and cameras are visible.
- Start condition: Human begins near the warning boundary but outside the stop zone.
- Operator action: Move slowly back and forth near the boundary without intentional fast crossing.
- Robot state/action: Robot follows the selected laboratory motion or remains idle.
- Expected safety behavior: Hysteresis prevents rapid oscillation between STOP and release/SAFE.
- Duration: 30-45 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if state transitions are stable and do not rapidly oscillate near the threshold. Fail if repeated rapid stop/release toggling occurs without corresponding operator movement.
- Manual notes to record: Boundary positions, operator movement pattern, any observed state flicker, and camera visibility.
- Safety precautions: Keep movement slow and avoid entering the robot contact path unexpectedly.

## Scenario S6_hands_up_gesture_stop

- Scenario ID: `S6_hands_up_gesture_stop`
- Scenario name: Hands-up gesture stop
- Objective: Validate gesture-triggered stop behavior using the hands-up gesture.
- Setup: Human stands in a region where proximity alone should not force STOP unless the test intentionally combines proximity and gesture.
- Start condition: System is in SAFE or WARNING state with valid human pose.
- Operator action: Raise both hands clearly and hold the gesture long enough for the configured confirmation logic.
- Robot state/action: Robot may be idle or executing the selected supervised trajectory.
- Expected safety behavior: Gesture-based stop is triggered and a stop command is generated.
- Duration: 15-20 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if the stop gesture is detected and STOP/stop command is generated. Fail if the gesture is not detected despite valid pose visibility.
- Manual notes to record: Gesture start/end times, hand visibility, pose confidence, and any proximity state at gesture time.
- Safety precautions: Confirm the gesture area is clear and that robot motion is supervised.

## Scenario S7_t_pose_or_resume_gesture

- Scenario ID: `S7_t_pose_or_resume_gesture`
- Scenario name: T-pose or resume gesture
- Objective: Validate the resume gesture, if implemented, and confirm that resume is allowed only when proximity conditions are safe.
- Setup: Human begins near a STOP or release-test condition. Resume gesture behavior must be enabled before the trial.
- Start condition: STOP is active or the system is ready to test resume gating.
- Operator action: Move to a safe region, then perform the implemented resume gesture such as T-pose if supported.
- Robot state/action: Robot remains stopped until both the gesture and proximity conditions allow release/resume.
- Expected safety behavior: Resume is allowed only when proximity conditions are safe. If resume gesture is not implemented, the system should not generate a resume from gesture alone.
- Duration: 15-20 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if resume is gated by safe proximity and the configured gesture/confirmation condition. Fail if resume occurs while proximity is unsafe.
- Manual notes to record: Whether resume gesture is implemented, gesture time, proximity status, and resume command timing.
- Safety precautions: Use a controlled laboratory state and confirm the operator is outside the unsafe zone before attempting release/resume.

## Scenario S8_partial_occlusion

- Scenario ID: `S8_partial_occlusion`
- Scenario name: Partial occlusion
- Objective: Evaluate robustness when the human body is partially occluded.
- Setup: Use a safe occluder or body orientation that partially hides the human from one or both cameras without entering unplanned robot contact regions.
- Start condition: System initially has valid pose and robot visibility.
- Operator action: Introduce partial occlusion while remaining in the planned region.
- Robot state/action: Robot remains in the selected supervised state.
- Expected safety behavior: The system either maintains a conservative safety state, uses a fallback source, or reports missing/low-confidence data.
- Duration: 20-30 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if occlusion is handled conservatively or explicitly reported as missing/low-confidence data. Fail if missing data creates an unsafe release or unreported invalid state.
- Manual notes to record: Which body parts/camera views are occluded, pose confidence if visible, and source fallback behavior.
- Safety precautions: Do not use occlusion objects that can interfere with the robot or cameras in a hazardous way.

## Scenario S9_missing_pose_or_camera_dropout

- Scenario ID: `S9_missing_pose_or_camera_dropout`
- Scenario name: Missing pose or camera dropout
- Objective: Check stale/missing-data detection and conservative handling when pose input or one camera stream is unavailable.
- Setup: Prepare a safe method to disable one camera stream, hide the human from pose detection, or remove valid pose input. Only perform camera dropout if it is safe for the robot state.
- Start condition: System has valid pose and distance source before dropout.
- Operator action: Temporarily remove valid pose input or disconnect/disable one camera stream if safely possible, then restore it.
- Robot state/action: Robot remains in the supervised laboratory state and should not resume from stale data.
- Expected safety behavior: Stale/missing data is detected and treated conservatively.
- Duration: 15-20 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if missing/stale data is detected and the safety state remains conservative. Fail if missing data is treated as valid safe proximity.
- Manual notes to record: Dropout start/end time, affected camera or pose source, recovery behavior, and any command generated.
- Safety precautions: Use a low-risk robot state and keep external supervision active throughout the dropout.

## Scenario S10_robot_motion_with_human_present

- Scenario ID: `S10_robot_motion_with_human_present`
- Scenario name: Robot motion with human present
- Objective: Validate proximity-state changes while the robot moves and the human remains visible.
- Setup: Robot follows the selected repeatable laboratory motion. Human starts in a safe region.
- Start condition: Robot and human are visible, robot telemetry is available, and the system begins in SAFE state.
- Operator action: Remain in the safe region, then slowly approach as defined by the run plan.
- Robot state/action: Robot moves according to the controlled trajectory until warning/stop logic requires stopping.
- Expected safety behavior: The system detects proximity change during robot motion and triggers warning/stop when needed.
- Duration: 30-45 s.
- Number of repetitions: Minimum 3.
- Logs to collect: `runtime.csv`, `safety.csv`, `pose.csv`, `robot_telemetry.csv`, `command_log.csv`, `notes.md`.
- Pass/fail criteria: Pass if proximity changes are detected during robot motion and stop behavior occurs when required. Fail if required warning/stop behavior is missed.
- Manual notes to record: Robot trajectory, approach timing, telemetry availability, distance-source changes, and any command acknowledgement.
- Safety precautions: Use a predefined low-risk trajectory and maintain the emergency stop and operator supervision.

## Manual Notes Checklist

For every trial, record:

- Run ID and trial ID.
- Scenario ID and repetition number.
- Operator name or initials.
- Date/time and approximate start/end timestamps.
- Robot program or motion state.
- Camera configuration and any camera issue.
- Lighting and occlusion conditions.
- Whether pose, marker registration, robot detector, and robot telemetry appeared valid.
- Any manual stop, emergency intervention, or deviation from the scenario.
- Whether external timing was used. If not, note that physical stop-time was not independently measured.

## Analysis Boundary

The analysis scripts segment logs using `scenario_config.csv`. Frame-level values may be summarized as latency distributions, but frames inside the same run are correlated and must not be treated as independent experiments. Repeatability and mean +/- standard deviation should be computed at the trial level across independent repetitions.
