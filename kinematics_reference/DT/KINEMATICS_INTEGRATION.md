# Kinematic Modeling Layer

This workspace only contained `Guide.md` and `Kinematics.nb`; there were no existing Unity scripts, Python receivers, MQTT/UDP modules, or safety-distance files to modify directly. The implementation is therefore additive and can be wired into the current digital twin project without deleting existing receiver or visualization logic.

## Added Unity Files

- `Assets/Scripts/Kinematics/RobotKinematicParameters.cs`
  - ScriptableObject model parameters: base frame, planar axes, link lengths, and per-joint angle offsets.
- `Assets/Scripts/Kinematics/RobotKinematics.cs`
  - Pure math layer for forward kinematics, link segment generation, point-to-segment distance, minimum human-robot distance, safety threshold decisions, and optional 3-link Newton-Raphson IK.
- `Assets/Scripts/Kinematics/RobotKinematicJsonLoader.cs`
  - Loads a JSON TextAsset using the placeholder schema and can assign it to a `RobotSafetyDistanceEvaluator`.
- `Assets/Scripts/Kinematics/RobotSafetyDistanceEvaluator.cs`
  - Unity-facing evaluator that reads human skeleton transforms, computes FK link segments from latest joint telemetry, evaluates minimum distance, preserves stale-data unsafe behavior, and applies warning/stop/release hysteresis plus gesture stop/go inputs.
- `Assets/Scripts/Kinematics/RobotKinematicsHud.cs`
  - Displays safety state, end-effector position, minimum distance, human joint index, and robot link index.
- `Assets/Scripts/Kinematics/RobotKinematicsValidationRunner.cs`
  - Scene/debug script that prints test joint angles, joint positions, end-effector pose, and minimum distance to a sample human point.
- `Assets/Tests/RobotKinematicsTests.cs`
  - NUnit tests for FK consistency, point-to-segment distance, safety hysteresis, and stale-data unsafe behavior.
- `Assets/Kinematics/robot_kinematics.placeholder.json`
  - Unity-importable placeholder config. Use it as a TextAsset with `RobotKinematicJsonLoader`.

## Added Python Files

- `python/robot_kinematics.py`
  - Python mirror of the Unity math for upstream safety computation before UDP/MQTT packets are sent.
- `python/validate_kinematics.py`
  - Prints validation cases from the placeholder JSON.
- `python/test_robot_kinematics.py`
  - Unit tests for FK, distance, invalid human points, hysteresis, and stale data.
- `Config/robot_kinematics.placeholder.json`
  - Pipeline/default placeholder JSON.

## Configuring Robot Dimensions

The placeholder link lengths are not asserted as ER9Pro truth:

```json
"links": [
  { "name": "Link1_placeholder", "linkLengthMeters": 0.25, "jointOffsetDegrees": 0.0 },
  { "name": "Link2_placeholder", "linkLengthMeters": 0.20, "jointOffsetDegrees": 0.0 },
  { "name": "Link3_placeholder", "linkLengthMeters": 0.15, "jointOffsetDegrees": 0.0 }
]
```

Replace those values with measured link lengths and calibrated joint zero offsets. If a full DH model is later available, `RobotKinematics.ForwardKinematics` is the narrow place to extend from the current planar notebook model to a full spatial serial-chain model while keeping the safety-distance API unchanged.

## Unity Wiring

1. Create a `RobotKinematicParameters` asset from `Assets > Create > Digital Twin > Robot Kinematic Parameters`, or add `RobotKinematicJsonLoader` to a GameObject and assign `Assets/Kinematics/robot_kinematics.placeholder.json` as its TextAsset.
2. Add `RobotSafetyDistanceEvaluator` to a scene object.
3. Assign its `parameters` field, or set the JSON loader's `targetEvaluator`.
4. Assign `humanJointTransforms` to the skeleton joint/keypoint transforms already produced by the human tracking visualization.
5. In the existing MQTT/UDP joint receiver, call:

```csharp
evaluator.ApplyJointTelemetry(jointAnglesDegrees, Time.time);
```

6. Keep the existing gesture logic by setting `gestureStopRequested` and `gestureGoRequested` on the evaluator, or call `RobotKinematics.DecideSafety(...)` directly from the existing safety controller.

The evaluator starts unsafe until fresh telemetry arrives because `LastJointTelemetryTime` defaults to negative infinity.

## Safety Pipeline Connection

The model converts live joint telemetry into ordered robot body segments:

```csharp
ForwardKinematicsResult fk = RobotKinematics.ForwardKinematics(parameters, jointAnglesDegrees);
HumanRobotDistanceResult d = RobotKinematics.MinimumDistance(humanJointPositions, fk.LinkSegments);
```

`MinimumDistance` computes:

```text
d_min = min distance between each human skeleton joint and each robot link segment
```

using finite point-to-line-segment projection. The result includes the distance, closest point on the robot, human joint index, and robot link index, so the UI and logs can identify which pair produced the limiting distance.

## Running Validation

Python:

```powershell
python python\validate_kinematics.py
python -m unittest discover -s python
```

Unity:

- Add `RobotKinematicsValidationRunner` to a scene object.
- Assign `RobotKinematicParameters`.
- Press Play, or use the component context menu `Run Kinematics Validation`.
- Run `Assets/Tests/RobotKinematicsTests.cs` from Unity Test Runner.

## Article Claim Supported

The implemented layer supports the claim:

> The robot is represented as a set of forward-kinematic link segments derived from joint telemetry, enabling human-robot distance estimation between reconstructed human skeleton joints and the robot body geometry, rather than relying only on coarse robot bounding boxes or detected robot centers.
