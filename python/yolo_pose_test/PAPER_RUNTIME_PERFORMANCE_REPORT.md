# Runtime Performance Evaluation of the Stereo Vision–Unity Safety System

## Abstract

This report evaluates the runtime performance of a dual-camera human-pose and robot-safety pipeline connected to a Unity application. Two experiments were conducted: a stereo 3D pose experiment containing 453 processed frames and a safety experiment containing 1,166 processed frames. Because the latency distributions contained isolated multi-second transients, median and tail percentiles (P90, P95, and P99) are emphasized instead of the arithmetic mean.

The pose pipeline achieved a median processing latency of 125.1 ms and a median output rate of 8.0 frames/s. ArUco marker detection was the dominant stage, with a median latency of 95.5 ms, while batched pose inference required 18.2 ms. The safety pipeline achieved a median processing latency of 38.8 ms and a median output rate of 21.2 frames/s. Within that pipeline, pose inference required 17.1 ms, robot detection and visualization required 21.0 ms, distance computation required 0.77 ms, and the safety-decision/UDP-send stage required 0.003 ms at the median.

The Python-stage results are suitable as descriptive results for the evaluated runs. The recorded Python-to-Unity end-to-end values are not reportable: Unity stopped updating while its window was unfocused, causing safety packets to remain queued for several seconds. Unity background execution has since been enabled, but a new run is required before an end-to-end latency claim can be made. Similarly, MQTT connections failed during the experiment, so the measurements do not establish command-to-robot actuation latency.

## 1. Experimental objective

The evaluation addressed four questions:

1. What latency and throughput are achieved by the stereo 3D pose pipeline?
2. Which processing stages dominate pose-pipeline execution time?
3. What latency and throughput are achieved by the human–robot safety pipeline?
4. Can the current experiment support a defensible Python-to-Unity or command-to-robot end-to-end latency claim?

## 2. System and software configuration

The experiments were executed on Windows 11 using an NVIDIA GeForce RTX 5060 Laptop GPU with 8,151 MiB of GPU memory and 31.4 GiB of system memory. The processor exposed 20 logical processing units. The relevant software versions were:

| Component | Version |
|---|---:|
| Python | 3.14.4 |
| PyTorch | 2.11.0+cu128 |
| CUDA runtime used by PyTorch | 12.8 |
| OpenCV | 4.13.0 |
| Ultralytics | 8.4.41 |
| NumPy | 2.4.4 |
| Unity | 2022.3.62f3 |

Both pipelines used two RTSP camera streams. Human pose was estimated using `yolo26n-pose.pt` with an inference image size of 480 pixels and a person-confidence threshold of 0.35. The safety pipeline used the same pose model and a custom robot detector at 480 pixels with a confidence threshold of 0.25. Robot detection was scheduled every second processed frame. The safety controller used a 0.75 m stop threshold, a 0.90 m release threshold, and a maximum command interval of 0.25 s.

## 3. Measurement methodology

### 3.1 Measurement boundary

Python timing began immediately before retrieval of the latest decoded camera frames and ended after completion of the frame-processing iteration. The measured software stages included camera-frame retrieval, model inference, stereo association, marker detection, triangulation/filtering, safety-distance computation, serialization, UDP transmission, and display-image construction.

CUDA synchronization was performed immediately before and after GPU inference. Therefore, the reported inference values include completion of the GPU work rather than only asynchronous kernel-launch overhead.

The following quantities were outside the software measurement boundary:

- camera exposure time;
- on-camera encoding and internal buffering;
- network transit before a frame became available to the decoder;
- physical monitor scan-out and pixel response;
- MQTT broker-to-robot transport;
- physical robot-controller response and mechanical actuation.

Consequently, `pipeline_ms` must be interpreted as software processing latency, not physical motion-to-photon latency.

### 3.2 Statistical treatment

Every processed frame was written to CSV. Results are reported using the median (P50) and the P90, P95, and P99 percentiles. These statistics were selected because real-time systems are affected by tail latency and because arithmetic means can be distorted by isolated initialization or I/O stalls. Maximum values are retained for diagnostic purposes but are not treated as representative steady-state behavior.

This report describes one run of each pipeline. The results are therefore descriptive measurements of the tested configuration, not confidence-bounded estimates of a broader population. Repeated independent trials are required for inferential comparisons or formal claims of repeatability.

## 4. Stereo 3D pose results

### 4.1 Runtime latency and throughput

The pose experiment processed 453 frames over approximately 88.9 s. The first frame contained a 30.1 s inference transient, producing a total first-frame pipeline time of 30.3 s. This event is reported as an initialization/outlier event and explains why the raw mean pipeline latency (196.3 ms) is substantially larger than the median. Percentiles below include all recorded frames; their robustness prevents the single event from dominating the representative values.

| Pose-pipeline metric | Mean | P50 | P90 | P95 | P99 | Maximum |
|---|---:|---:|---:|---:|---:|---:|
| Total pipeline latency (ms) | 196.31 | 125.11 | 153.27 | 156.54 | 161.52 | 30,279.37 |
| Batched pose inference (ms) | 85.30 | 18.17 | 24.23 | 25.45 | 28.09 | 30,131.03 |
| ArUco detection (ms) | 99.11 | 95.47 | 120.73 | 122.68 | 125.66 | 129.93 |
| Camera-frame retrieval (ms) | 1.75 | 1.81 | 2.07 | 2.26 | 2.67 | 3.72 |
| Camera decode skew (ms) | 18.11 | 11.27 | 40.63 | 50.29 | 59.70 | 68.99 |
| Stereo person matching (ms) | 0.27 | 0.24 | 0.31 | 0.37 | 0.95 | 2.36 |
| 3D triangulation/filtering (ms) | 0.39 | 0.36 | 0.49 | 0.56 | 1.28 | 1.66 |
| JSON serialization (ms) | 0.11 | 0.11 | 0.13 | 0.13 | 0.16 | 0.20 |
| UDP send call (ms) | 0.11 | 0.09 | 0.13 | 0.17 | 0.94 | 0.99 |
| Display construction (ms) | 8.03 | 7.60 | 9.46 | 9.92 | 10.74 | 12.08 |
| Output rate (frames/s) | 7.76 | 7.97 | 8.67 | 8.86 | 9.06 | 9.19 |

Median camera-frame age was 28.3 ms for camera 1 and 29.5 ms for camera 2. Their P95 ages were 64.9 ms and 57.7 ms, respectively. The median inter-camera decode skew was 11.3 ms, increasing to 50.3 ms at P95. This skew is important because stereo reconstruction assumes that corresponding observations represent approximately the same physical instant.

### 4.2 Pose tracking quality

Stereo person association succeeded on 450 of 453 frames (99.3%). Camera 1 detected one person on 451 frames, while camera 2 detected one person on 448 frames. The median tracked-joint ratio was 82.4%, and the mean tracked-joint ratio was 80.5%.

These results indicate that person association was stable during the run, although approximately one-fifth of the expected joints were not reconstructed on an average frame. The joint-coverage result should be interpreted jointly with camera visibility, confidence thresholds, occlusion, and stereo geometry rather than as a model-only accuracy measure.

### 4.3 Pose-pipeline bottleneck

ArUco detection was the main steady-state bottleneck. Its 95.5 ms median represented approximately 76% of the 125.1 ms median pipeline time. By comparison, pose inference contributed approximately 15% at the median. Stereo matching, triangulation, JSON serialization, and UDP transmission each contributed less than 1 ms at P95.

The most effective route to higher pose throughput is therefore to reduce marker-processing cost—for example, by detecting markers less frequently, restricting detection to regions of interest, using a lower marker-processing resolution, or moving marker alignment to a separate asynchronous cadence. Optimizing UDP serialization would not materially change total latency.

## 5. Human–robot safety results

### 5.1 Runtime latency and throughput

The safety experiment processed 1,166 frames over approximately 63.9 s. The median pipeline latency was 38.8 ms, and the median output rate was 21.2 frames/s.

| Safety-pipeline metric | Mean | P50 | P90 | P95 | P99 | Maximum |
|---|---:|---:|---:|---:|---:|---:|
| Total pipeline latency (ms) | 46.75 | 38.84 | 68.77 | 72.74 | 76.14 | 2,103.95 |
| Pose inference (ms) | 17.57 | 17.08 | 23.54 | 24.62 | 26.83 | 69.04 |
| Robot detection/overlay interval (ms) | 23.12 | 20.99 | 41.61 | 43.66 | 45.99 | 2,071.63 |
| Safety-distance calculation (ms) | 0.73 | 0.77 | 1.06 | 1.34 | 1.88 | 3.71 |
| Safety decision and UDP send (ms) | 0.02 | 0.003 | 0.10 | 0.10 | 0.13 | 0.97 |
| Camera-frame retrieval (ms) | 1.94 | 1.93 | 2.11 | 2.22 | 2.86 | 3.76 |
| Display construction (ms) | 3.35 | 3.24 | 4.11 | 4.33 | 4.72 | 5.33 |
| Output rate (frames/s) | 21.58 | 21.20 | 32.06 | 32.59 | 38.16 | 43.64 |

The robot-processing interval is bimodal because robot detection was executed every second frame. Two approximately 2.1 s stalls occurred at frames 1 and 1,128. Inspection of the code path showed that this metric also contains the periodic high-resolution camera refresh. The maximum must therefore not be attributed solely to neural-network robot detection. Future instrumentation should record high-resolution refresh time in a separate column.

### 5.2 Safety-state coverage

The safety controller used a 3D distance source on 1,035 frames (88.8%), a 2D fallback on 112 frames (9.6%), and no valid source on 19 frames (1.6%). The stop state was active on 292 frames (25.0%). Robot detection was positive on 99.8% of camera-1 frames and 90.5% of camera-2 frames.

The decision stage itself was inexpensive: P99 decision-and-send latency was 0.13 ms. Therefore, safety response within Python was governed primarily by image acquisition cadence and perception latency, not by threshold evaluation or UDP transmission.

### 5.3 Implication for software response time

At P95, the Python safety pipeline required 72.7 ms. At P99 it required 76.1 ms. These values describe the time needed by the evaluated software iteration after requesting the latest decoded frames. They do not include the unknown age accumulated during camera exposure, encoding, RTSP transport, or decoder buffering. They also do not include Unity rendering, MQTT delivery, robot-controller processing, or mechanical braking.

Accordingly, the defensible statement is:

> In the evaluated run, the Python safety pipeline completed within 72.7 ms for 95% of processed frames and within 76.1 ms for 99% of processed frames, excluding camera-side latency and downstream physical actuation.

It would not be defensible to describe 72.7 ms as total human-motion-to-robot-stop latency.

## 6. Unity and MQTT measurements

### 6.1 Unity safety timing

Unity received 225 safety messages that could be correlated with Python frame identifiers. However, Unity was not configured to continue updating while its window was unfocused. While the OpenCV display had focus, the Unity main thread stopped consuming its concurrent UDP queue. As a result, the recorded median queue delay was 32.9 s and the recorded P95 queue delay was 61.0 s.

These values measure an editor-focus artifact rather than system communication latency and must not be included as end-to-end performance results. The issue has been corrected by setting `Application.runInBackground = true` in the runtime recorder. A new experiment is required to obtain valid Unity queue, pre-render, and software end-to-end percentiles.

### 6.2 Pose-to-Unity timing

The Unity pose CSV contained no data rows because the scene's `PoseBoneDriver` component was inactive during the pose experiment. The Python pose values remain valid, but the experiment cannot support a pose-to-Unity end-to-end latency result. The receiver must be active during the repeated experiment.

### 6.3 MQTT timing

Two MQTT connection attempts failed, each after approximately 21 s. Six control publications were consequently marked as skipped. Because there was no successful publication, correlation identifier, or robot acknowledgement, no MQTT delivery or physical robot-actuation latency can be reported.

A valid actuation experiment requires a unique command identifier and timestamp in every command, followed by a robot-side acknowledgement carrying the same identifier. For physical stopping latency, the preferred reference is an independent sensor or high-speed camera observing the command indicator and the first measurable change in robot motion.

## 7. Resource utilization

| Resource metric | Pose pipeline | Safety pipeline |
|---|---:|---:|
| Median process CPU utilization | 105.3% | 77.8% |
| P95 process CPU utilization | 111.8% | 93.3% |
| Median process resident memory | 1,906.1 MiB | 1,802.0 MiB |
| P95 process resident memory | 1,907.4 MiB | 1,805.3 MiB |

Process CPU utilization may exceed 100% because it is expressed relative to one logical CPU and the process uses multiple threads. GPU memory allocation was recorded by PyTorch, but reliable GPU utilization percentages were not present in these runs; no GPU-utilization claim should be made from this dataset.

## 8. Threats to validity and limitations

The following limitations should accompany any publication of these values:

1. **Single-run evaluation.** Only one run of each pipeline was analyzed. Variation between launches, scenes, camera conditions, and thermal states was not estimated.
2. **Initialization transients.** The pose run contained a 30.1 s first-inference event. The safety run contained two approximately 2.1 s high-resolution refresh stalls.
3. **Mixed timing categories.** The current `robot_detection_overlay_ms` field includes periodic high-resolution camera refresh work and must not be interpreted as pure detector latency.
4. **Camera-side latency excluded.** Exposure, encoding, RTSP buffering, and decoder delay before frame availability were not measured.
5. **Stereo temporal skew.** Inter-camera decode skew reached 50.3 ms at P95, potentially affecting dynamic 3D reconstruction.
6. **Invalid Unity focus condition.** The recorded Unity end-to-end distribution is invalid because main-thread processing paused while unfocused.
7. **Inactive Unity pose receiver.** No pose frame reached the Unity pose recorder.
8. **No successful MQTT actuation path.** Broker connections failed and no robot acknowledgement was available.
9. **Runtime exceptions in the Unity scene.** Repeated Mixed Reality Toolkit `TintEffect` null-reference exceptions were present in the Unity Console. These should be corrected before final benchmarking because exception logging can perturb frame timing.
10. **No physical ground-truth timing.** Software clocks alone cannot establish physical motion-to-display or command-to-mechanical-stop latency.

## 9. Recommended final evaluation protocol

For the final paper dataset, perform at least five independent runs per pipeline, preferably 5–10 minutes each, using the following protocol:

1. Start Unity and verify that `PoseBoneDriver` is active for pose experiments.
2. Verify that Unity reports the metrics directory and remains active while unfocused.
3. Confirm MQTT connectivity before starting an actuation experiment.
4. Warm the models and camera streams for at least 30 s before beginning the recorded interval.
5. Separate high-resolution refresh timing from robot detection timing.
6. Clear or resolve Unity runtime exceptions before recording.
7. Exercise safe, warning, and stop regions repeatedly across both camera views.
8. Record environmental conditions, camera placement, resolution, model files, thresholds, and GPU power mode.
9. Report per-run P50/P95/P99 results and aggregate the independent-run statistics, rather than pooling all frames without identifying their run.
10. For end-to-end physical latency, use command correlation and an external high-speed or electronic reference.

## 10. Paper-ready results paragraph

The following paragraph can be adapted directly for the results section:

> Runtime performance was evaluated using frame-level telemetry from a 453-frame stereo pose run and a 1,166-frame safety run. The stereo pose pipeline achieved a median latency of 125.1 ms (P95: 156.5 ms; P99: 161.5 ms) and a median throughput of 8.0 frames/s. ArUco detection dominated steady-state processing, requiring 95.5 ms at the median, compared with 18.2 ms for batched pose inference. Stereo person association succeeded on 99.3% of frames, and the median reconstructed-joint ratio was 82.4%. The safety pipeline achieved a median latency of 38.8 ms (P95: 72.7 ms; P99: 76.1 ms) and a median throughput of 21.2 frames/s. Median pose inference, robot-processing, distance-computation, and decision/send latencies were 17.1 ms, 21.0 ms, 0.77 ms, and 0.003 ms, respectively. These values represent software processing after retrieval of decoded camera frames and exclude camera exposure/encoding, Unity rendering, MQTT delivery, and physical robot actuation.

## 11. Publication-safe conclusions

The evaluated implementation demonstrates that the Python safety logic itself is computationally lightweight and that perception dominates the response budget. The safety pipeline remained below 76.1 ms on 99% of processed frames, aside from isolated high-resolution refresh stalls. The pose pipeline was slower because marker detection dominated its runtime, limiting median throughput to approximately 8 frames/s. Pose association was stable, but joint reconstruction remained incomplete on a portion of frames and stereo timing skew was substantial in the upper tail.

The present data support claims about Python-stage latency, throughput, and tracking coverage only. They do not yet support a Python-to-Unity end-to-end latency claim or a physical robot-stop latency claim. Those claims require the corrected Unity background configuration, an active pose receiver, successful MQTT communication, command acknowledgements, repeated trials, and preferably an independent physical timing reference.

## Data provenance

- Pose source: `metrics/pose_20260620_154830_16920/python_frames.csv`
- Safety source: `metrics/safety_20260620_155206_4576/python_frames.csv`
- Unity source: `unity_20260620_154753/`
- Generated pose summary: `metrics_report/pose_20260620_154830/summary.json`
- Generated safety summary: `metrics_report/safety_20260620_155206/summary.json`

