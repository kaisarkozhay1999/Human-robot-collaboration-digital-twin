"""Low-overhead runtime telemetry for the vision-to-Unity pipelines.

The recorder deliberately measures only software-observable quantities. Camera
exposure/encoder latency is outside its boundary; ``camera_frame_age_ms`` starts
when a decoded frame becomes available to Python.
"""

from __future__ import annotations

import csv
import json
import math
import os
import platform
import statistics
import time
from collections import defaultdict, deque
from pathlib import Path


try:
    import psutil
except ImportError:  # optional
    psutil = None


METRIC_COLUMNS = [
    "frame", "unix_ms", "frame_interval_ms", "output_fps",
    "cam1_sequence", "cam2_sequence", "cam1_sequence_gap", "cam2_sequence_gap",
    "cam1_frame_age_ms", "cam2_frame_age_ms", "camera_decode_skew_ms",
    "camera_duplicate_1", "camera_duplicate_2", "camera_read_ms", "resize_ms",
    "inference_ms", "stereo_match_ms", "quality_metrics_ms", "aruco_detection_ms", "gesture_ms",
    "marker_anchor_ms", "aruco_triangulation_ms", "people_triangulation_filter_ms",
    "payload_build_ms", "json_serialize_ms", "udp_send_ms", "pipeline_ms",
    "display_build_ms", "persons_cam1", "persons_cam2", "matched_people",
    "output_people", "tracked_joints", "tracked_joint_ratio", "mean_joint_confidence",
    "aruco_cam1", "aruco_cam2", "aruco_output", "anchor_visible", "anchor_used",
    "reprojection_mean_px", "reprojection_p95_px", "epipolar_mean_px",
    "packet_bytes", "gesture_detected", "gesture_sent",
    "pose_inference_ms", "robot_detection_overlay_ms", "safety_distance_ms",
    "safety_decision_send_ms", "safety_stop", "safety_source", "safety_distance_m",
    "safety_distance_px", "robot_detected_cam1", "robot_detected_cam2",
    "process_cpu_percent", "process_rss_mb", "system_cpu_percent", "system_memory_percent",
    "gpu_allocated_mb", "gpu_reserved_mb", "gpu_utilization_percent", "gpu_memory_percent",
]


class RunningMetric:
    def __init__(self, window_size=10000):
        self.count = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.minimum = math.inf
        self.maximum = -math.inf
        self.window = deque(maxlen=window_size)

    def add(self, value):
        if value is None:
            return
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        if not math.isfinite(value):
            return
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)
        self.window.append(value)

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return None
        ordered = sorted(values)
        position = (len(ordered) - 1) * percentile / 100.0
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

    def summary(self):
        if self.count == 0:
            return {"count": 0}
        return {
            "count": self.count,
            "mean": self.mean,
            "stddev": math.sqrt(self.m2 / max(self.count - 1, 1)),
            "min": self.minimum,
            "p50": self._percentile(self.window, 50),
            "p90": self._percentile(self.window, 90),
            "p95": self._percentile(self.window, 95),
            "p99": self._percentile(self.window, 99),
            "max": self.maximum,
            "percentiles_window_samples": len(self.window),
        }


class ResourceSampler:
    def __init__(self, interval_sec=1.0):
        self.interval_sec = max(float(interval_sec), 0.1)
        self.last_sample = 0.0
        self.last = {}
        self.process = psutil.Process(os.getpid()) if psutil is not None else None
        if self.process is not None:
            self.process.cpu_percent(None)
            psutil.cpu_percent(None)

    def sample(self, torch_module=None):
        now = time.perf_counter()
        if now - self.last_sample < self.interval_sec:
            return dict(self.last)
        result = {}
        if self.process is not None:
            try:
                result.update({
                    "process_cpu_percent": self.process.cpu_percent(None),
                    "process_rss_mb": self.process.memory_info().rss / (1024.0 * 1024.0),
                    "system_cpu_percent": psutil.cpu_percent(None),
                    "system_memory_percent": psutil.virtual_memory().percent,
                })
            except (psutil.Error, OSError):
                pass
        if torch_module is not None:
            try:
                if torch_module.cuda.is_available():
                    result["gpu_allocated_mb"] = torch_module.cuda.memory_allocated() / (1024.0 * 1024.0)
                    result["gpu_reserved_mb"] = torch_module.cuda.memory_reserved() / (1024.0 * 1024.0)
                    # Supported by recent PyTorch/NVML builds; absent on some systems.
                    if hasattr(torch_module.cuda, "utilization"):
                        result["gpu_utilization_percent"] = torch_module.cuda.utilization()
                    if hasattr(torch_module.cuda, "memory_usage"):
                        result["gpu_memory_percent"] = torch_module.cuda.memory_usage()
            except Exception:
                pass
        self.last = result
        self.last_sample = now
        return dict(result)


class FrameTimer:
    def __init__(self, frame_index):
        self.frame_index = int(frame_index)
        self.started_ns = time.perf_counter_ns()
        self.last_ns = self.started_ns
        self.values = {
            "frame": self.frame_index,
            "unix_ms": time.time_ns() / 1_000_000.0,
            "_started_ns": self.started_ns,
        }

    def mark(self, name):
        now = time.perf_counter_ns()
        self.values[name] = (now - self.last_ns) / 1_000_000.0
        self.last_ns = now
        return self.values[name]

    def set(self, **values):
        self.values.update(values)

    def finish(self):
        self.values["pipeline_ms"] = (time.perf_counter_ns() - self.started_ns) / 1_000_000.0
        return self.values


class MetricsSession:
    def __init__(self, output_root, prefix="pose", summary_interval_sec=5.0, enabled=True):
        self.enabled = bool(enabled)
        self.output_root = Path(output_root)
        self.started_unix = time.time()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.started_unix))
        self.session_dir = self.output_root / f"{prefix}_{stamp}_{os.getpid()}"
        self.csv_path = self.session_dir / "python_frames.csv"
        self.summary_path = self.session_dir / "python_summary.json"
        self.metadata_path = self.session_dir / "session.json"
        self.summary_interval_sec = max(float(summary_interval_sec), 0.5)
        self.last_summary = 0.0
        self.file = None
        self.writer = None
        self.stats = defaultdict(RunningMetric)
        self.frames = 0
        self.last_frame_unix_ms = None
        self.resource_sampler = ResourceSampler()
        if self.enabled:
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self.file = self.csv_path.open("w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.file, fieldnames=METRIC_COLUMNS, extrasaction="ignore")
            self.writer.writeheader()
            self._write_metadata()

    def _write_metadata(self):
        metadata = {
            "schema_version": 1,
            "boundary": "decoded_frames_available_in_python_to_unity_before_render",
            "excluded": ["camera exposure", "camera encoding", "physical display response"],
            "started_unix": self.started_unix,
            "pid": os.getpid(),
            "python": platform.python_version(),
            "platform": platform.platform(),
            "columns": METRIC_COLUMNS,
        }
        self.metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    def new_frame(self, frame_index):
        return FrameTimer(frame_index)

    def record(self, values, torch_module=None):
        if not self.enabled:
            return
        row = dict(values)
        unix_ms = float(row.get("unix_ms", time.time_ns() / 1_000_000.0))
        if self.last_frame_unix_ms is not None:
            interval = unix_ms - self.last_frame_unix_ms
            row.setdefault("frame_interval_ms", interval)
            row.setdefault("output_fps", 1000.0 / interval if interval > 0.0 else None)
        self.last_frame_unix_ms = unix_ms
        row.update(self.resource_sampler.sample(torch_module))
        self.writer.writerow({column: row.get(column, "") for column in METRIC_COLUMNS})
        self.file.flush()
        self.frames += 1
        for name, value in row.items():
            if name in ("frame", "unix_ms") or isinstance(value, bool):
                continue
            self.stats[name].add(value)
        now = time.perf_counter()
        if now - self.last_summary >= self.summary_interval_sec:
            self.write_summary()
            self.last_summary = now

    def payload_metrics(self, values):
        return {
            "python_frame_start_unix_ms": float(values.get("unix_ms", 0.0)),
            "python_send_unix_ms": time.time_ns() / 1_000_000.0,
            "camera_frame_age_ms": max(
                float(values.get("cam1_frame_age_ms", 0.0)),
                float(values.get("cam2_frame_age_ms", 0.0)),
            ),
            "camera_decode_skew_ms": float(values.get("camera_decode_skew_ms", 0.0)),
            "inference_ms": float(values.get("inference_ms", 0.0)),
            "pipeline_ms": (time.perf_counter_ns() - int(values.get("_started_ns", time.perf_counter_ns()))) / 1_000_000.0,
        }

    def write_summary(self):
        if not self.enabled:
            return
        summary = {
            "schema_version": 1,
            "frames": self.frames,
            "duration_sec": max(time.time() - self.started_unix, 0.0),
            "metrics": {name: metric.summary() for name, metric in sorted(self.stats.items())},
        }
        temp = self.summary_path.with_suffix(".tmp")
        temp.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        temp.replace(self.summary_path)

    def close(self):
        if not self.enabled:
            return
        self.write_summary()
        if self.file is not None:
            self.file.flush()
            self.file.close()
            self.file = None


def mean_or_none(values):
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return statistics.fmean(finite) if finite else None
