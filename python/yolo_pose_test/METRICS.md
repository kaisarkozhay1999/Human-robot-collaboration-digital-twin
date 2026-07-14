# Runtime metrics

The measurement boundary begins when each decoded camera frame becomes available
to Python. Camera exposure, camera encoder buffering, monitor scan-out, and physical
motion are not measurable without external equipment and are not included.

## Pose pipeline

Run the normal pipeline. Metrics are enabled by default:

```powershell
venv\Scripts\python.exe pose3d_to_unity.py
```

Python writes `metrics/pose_<timestamp>_<pid>/python_frames.csv` and a rolling
`python_summary.json`. Unity writes `unity_frames.csv` and `unity_safety.csv` under:

```text
Application.persistentDataPath/metrics/unity_<timestamp>/
```

The Unity Console prints the exact directory at startup.

The Unity recorder starts automatically in Play mode. `unity_frames.csv` measures
UDP arrival, main-thread pose application, and `Application.onBeforeRender` for
each pose packet. `unity_safety.csv` listens on UDP port 5006 and correlates
safety events by Python frame number. It also forces Unity to continue updating
while the Python/OpenCV window has focus. No scene component needs to be added.

`unity_mqtt.csv` records MQTT connect/publish call duration, telemetry message
size, and UDP callback-to-Unity-Update queue delay.
It does not claim broker-to-physical-device actuation latency because the existing
command messages do not carry correlation IDs or acknowledgements.

## Safety pipeline

```powershell
venv\Scripts\python.exe test_dual_stream_pipeline.py
```

This creates `metrics/safety_<timestamp>_<pid>/` and includes pose inference,
robot detection/overlay, distance calculation, safety decision/send, display,
resource, and safety-state measurements.

## Combined report

```powershell
venv\Scripts\python.exe analyze_runtime_metrics.py `
  --python-csv metrics\pose_<session>\python_frames.csv `
  --unity-csv <UnityPersistentDataPath>\metrics\unity_<session>\unity_frames.csv `
  --mqtt-csv <UnityPersistentDataPath>\metrics\unity_<session>\unity_mqtt.csv
```

For a safety run, use its Python CSV and add
`--safety-csv <UnityPersistentDataPath>\metrics\unity_<session>\unity_safety.csv`.

Use P50, P90, P95, and P99 when reporting latency. `software_e2e_ms` is decoded
frame availability in Python to Unity's `onBeforeRender` callback. `udp_delay_ms`
uses the common Windows wall clock because Python and Unity run on the same PC.
