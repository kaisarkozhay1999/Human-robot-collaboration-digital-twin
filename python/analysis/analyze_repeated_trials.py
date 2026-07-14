"""Analyze real repeated-trial logs for the robot-arm digital twin.

The script is intentionally conservative:

- It never fabricates rows or fills missing measurements.
- It segments logs with scenario_config.csv.
- It reports frame-level latency distributions separately from trial-level
  repeatability statistics.
- It does not report physical stop-time unless an explicit log column exists.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any


RUN_RE = re.compile(r"^run_(\d+)$", re.IGNORECASE)

DEFAULT_MAPPING: dict[str, dict[str, Any]] = {
    "logical_fields": {
        "timestamp": [
            "timestamp",
            "unix_ms",
            "unix_time_ms",
            "time_unix_ms",
            "python_unix_ms",
            "unity_unix_ms",
            "command_timestamp",
            "time_seconds",
            "time_s",
            "t",
            "unity_receive_unix_ms",
        ],
        "frame_id": ["frame_id", "frame", "unity_frame", "unity_frame_count"],
        "total_latency_ms": ["total_latency_ms", "software_e2e_ms", "pipeline_ms", "python_pipeline_ms", "latency_ms"],
        "pose_latency_ms": ["pose_latency_ms", "pose_inference_ms", "inference_ms", "python_pose_inference_ms", "python_inference_ms"],
        "robot_detection_latency_ms": ["robot_detection_latency_ms", "robot_detection_overlay_ms", "python_robot_detection_overlay_ms"],
        "marker_latency_ms": ["marker_latency_ms", "aruco_detection_ms", "marker_anchor_ms", "python_aruco_detection_ms", "python_marker_anchor_ms"],
        "camera_retrieval_latency_ms": ["camera_retrieval_latency_ms", "camera_read_ms", "python_camera_read_ms"],
        "triangulation_latency_ms": [
            "triangulation_latency_ms",
            "aruco_triangulation_ms",
            "people_triangulation_filter_ms",
            "stereo_match_ms",
            "python_aruco_triangulation_ms",
            "python_people_triangulation_filter_ms",
            "python_stereo_match_ms",
        ],
        "distance_latency_ms": ["distance_latency_ms", "safety_distance_ms", "python_safety_distance_ms"],
        "decision_send_latency_ms": ["decision_send_latency_ms", "safety_decision_send_ms", "udp_send_ms", "python_safety_decision_send_ms", "python_udp_send_ms", "call_duration_ms"],
        "fps": ["fps", "output_fps", "python_output_fps"],
        "safety_state": ["safety_state", "state", "decision", "current_decision"],
        "stop_flag": ["stop_flag", "safety_stop", "stop", "unity_stop", "is_stop"],
        "release_flag": ["release_flag", "release", "resume_flag", "go_flag"],
        "stale_data_flag": ["stale_data", "stale", "data_stale", "no_valid_source", "missing_data"],
        "data_fresh_flag": ["data_fresh", "robot_data_fresh", "human_data_fresh"],
        "min_distance_m": ["min_distance_m", "safety_distance_m", "distance_m"],
        "min_distance_px": ["min_distance_px", "safety_distance_px", "distance_px"],
        "distance": ["distance"],
        "distance_unit": ["unit", "distance_unit"],
        "distance_source": ["distance_source", "safety_source", "source"],
        "command_type": ["command_type", "command", "message"],
        "command_timestamp": ["command_timestamp", "unix_ms", "timestamp"],
        "command_acknowledged": ["command_acknowledged", "acknowledged", "ack", "success"],
        "command_send_latency_ms": ["command_send_latency_ms", "send_latency_ms", "call_duration_ms", "mqtt_publish_ms"],
        "warning_threshold_m": ["warning_threshold_m", "warning_distance_m"],
        "stop_threshold_m": ["stop_threshold_m", "stop_distance_m"],
        "release_threshold_m": ["release_threshold_m", "release_distance_m"],
        "warning_threshold_px": ["warning_threshold_px", "warning_distance_px"],
        "stop_threshold_px": ["stop_threshold_px", "stop_distance_px"],
        "release_threshold_px": ["release_threshold_px", "release_distance_px"],
    },
    "role_files": {
        "runtime": ["runtime.csv", "python_frames.csv", "joined_frames.csv", "joined_safety.csv", "unity_frames.csv"],
        "safety": ["safety.csv", "unity_safety.csv", "fk_safety.csv", "joined_safety.csv", "runtime.csv", "python_frames.csv"],
        "pose": ["pose.csv", "python_frames.csv", "runtime.csv"],
        "robot_telemetry": ["robot_telemetry.csv", "fk_safety.csv"],
        "command_log": ["command_log.csv", "unity_mqtt.csv"],
    },
    "analysis": {
        "stop_states": ["STOP"],
        "warning_states": ["WARNING"],
        "release_states": ["RELEASE", "RESUME", "GO"],
        "safe_states": ["SAFE", "RUN"],
        "stale_states": ["HOLD", "STALE", "MISSING", "NO_VALID_SOURCE"],
        "oscillation_window_sec": 2.0,
        "threshold_margin_m": 0.05,
        "threshold_margin_px": 10.0,
    },
}

SCENARIO_DEFINITIONS = {
    "S1": "human_far_static",
    "S1_human_far_static": "human_far_static",
    "S2": "human_approaches_robot",
    "S2_human_approaches_robot": "human_approaches_robot",
    "S3": "human_inside_stop_zone",
    "S3_human_inside_stop_zone": "human_inside_stop_zone",
    "S4": "human_moves_away_release",
    "S4_human_moves_away_release": "human_moves_away_release",
    "S5": "dynamic_motion_near_boundary",
    "S5_dynamic_motion_near_boundary": "dynamic_motion_near_boundary",
    "S6": "hands_up_gesture_stop",
    "S6_hands_up_gesture_stop": "hands_up_gesture_stop",
    "S7": "t_pose_or_resume_gesture",
    "S7_t_pose_or_resume_gesture": "t_pose_or_resume_gesture",
    "S8": "partial_occlusion",
    "S8_partial_occlusion": "partial_occlusion",
    "S9": "missing_pose_or_camera_dropout",
    "S9_missing_pose_or_camera_dropout": "missing_pose_or_camera_dropout",
    "S10": "robot_motion_with_human_present",
    "S10_robot_motion_with_human_present": "robot_motion_with_human_present",
}

STAGE_FIELDS = [
    ("camera_retrieval_latency_ms", "camera_retrieval_mean_ms", "Camera retrieval"),
    ("pose_latency_ms", "pose_inference_mean_ms", "Pose inference"),
    ("robot_detection_latency_ms", "robot_detection_mean_ms", "Robot detection"),
    ("marker_latency_ms", "marker_mean_ms", "Marker/ArUco"),
    ("triangulation_latency_ms", "triangulation_mean_ms", "Triangulation"),
    ("distance_latency_ms", "distance_calc_mean_ms", "Distance calc"),
    ("decision_send_latency_ms", "decision_send_mean_ms", "Decision/send"),
]


def deep_copy_mapping(mapping: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    copied: dict[str, dict[str, Any]] = {}
    for section, values in mapping.items():
        copied[section] = {}
        for key, value in values.items():
            copied[section][key] = list(value) if isinstance(value, list) else value
    return copied


def parse_scalar(raw: str) -> Any:
    text = raw.strip()
    if not text:
        return ""
    if text.startswith("[") and text.endswith("]"):
        inner = text[1:-1].strip()
        if not inner:
            return []
        return [part.strip().strip("'\"") for part in inner.split(",")]
    lowered = text.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return text.strip("'\"")


def load_mapping(path: Path | None) -> tuple[dict[str, dict[str, Any]], list[str]]:
    mapping = deep_copy_mapping(DEFAULT_MAPPING)
    warnings: list[str] = []
    if path is None:
        return mapping, warnings
    if not path.exists():
        warnings.append(f"Column mapping file not found: {path}. Built-in mapping was used.")
        return mapping, warnings

    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore

        loaded = yaml.safe_load(text) or {}
        if isinstance(loaded, dict):
            for section in ("logical_fields", "role_files", "analysis"):
                if isinstance(loaded.get(section), dict):
                    for key, value in loaded[section].items():
                        mapping.setdefault(section, {})[key] = value
            return mapping, warnings
    except Exception:
        warnings.append("PyYAML was not available or failed to parse the mapping; using the built-in minimal YAML parser.")

    current_section: str | None = None
    for raw_line in text.splitlines():
        line_without_comment = raw_line.split("#", 1)[0].rstrip()
        if not line_without_comment.strip():
            continue
        if not line_without_comment.startswith(" ") and line_without_comment.endswith(":"):
            current_section = line_without_comment[:-1].strip()
            mapping.setdefault(current_section, {})
            continue
        if current_section and line_without_comment.startswith("  ") and ":" in line_without_comment:
            key, value = line_without_comment.strip().split(":", 1)
            mapping[current_section][key.strip()] = parse_scalar(value)
    return mapping, warnings


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    if not path.exists():
        return [], []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        rows = [dict(row) for row in reader]
    for row in rows:
        row["__source_file"] = str(path)
    return rows, fields


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    ensure_dir(path.parent)
    if fields is None:
        field_order: list[str] = []
        for row in rows:
            for key in row.keys():
                if key not in field_order:
                    field_order.append(key)
        fields = field_order
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def clean(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip()


def to_float(value: Any) -> float | None:
    text = clean(value)
    if not text:
        return None
    try:
        result = float(text)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def to_bool(value: Any) -> bool | None:
    text = clean(value).lower()
    if not text:
        return None
    if text in {"1", "true", "yes", "y", "stop", "stopped", "active"}:
        return True
    if text in {"0", "false", "no", "n", "safe", "run", "go", "inactive"}:
        return False
    return None


def first_value(row: dict[str, Any], aliases: list[str]) -> tuple[Any, str | None]:
    lowered = {key.lower(): key for key in row.keys()}
    for alias in aliases:
        key = lowered.get(alias.lower())
        if key is None:
            continue
        value = row.get(key)
        if clean(value) != "":
            return value, key
    return None, None


def logical_value(row: dict[str, Any], mapping: dict[str, dict[str, Any]], field: str) -> tuple[Any, str | None]:
    aliases = mapping["logical_fields"].get(field, [])
    if isinstance(aliases, str):
        aliases = [aliases]
    return first_value(row, list(aliases))


def logical_float(row: dict[str, Any], mapping: dict[str, dict[str, Any]], field: str) -> float | None:
    value, _ = logical_value(row, mapping, field)
    return to_float(value)


def logical_bool(row: dict[str, Any], mapping: dict[str, dict[str, Any]], field: str) -> bool | None:
    value, _ = logical_value(row, mapping, field)
    return to_bool(value)


def timestamp_seconds(row: dict[str, Any], mapping: dict[str, dict[str, Any]], field_name: str = "timestamp") -> float | None:
    value, key = logical_value(row, mapping, field_name)
    numeric = to_float(value)
    if numeric is None:
        return None
    key_lower = (key or "").lower()
    if key_lower.endswith("_ms") or key_lower in {"unix_ms", "python_unix_ms", "unity_unix_ms", "command_timestamp"}:
        return numeric / 1000.0
    if numeric > 1_000_000_000_000:
        return numeric / 1000.0
    return numeric


def parse_config_time(value: Any) -> tuple[float | None, bool]:
    """Return (seconds, is_absolute_epoch_time)."""
    text = clean(value)
    if not text:
        return None, False
    numeric = to_float(text)
    if numeric is not None:
        if abs(numeric) > 1_000_000_000_000:
            return numeric / 1000.0, True
        if abs(numeric) > 1_000_000_000:
            return numeric, True
        return numeric, False
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None, False
    if parsed.tzinfo is None:
        return None, False
    return parsed.timestamp(), True


def config_duration_seconds(start_time: str, end_time: str) -> float | None:
    start, start_abs = parse_config_time(start_time)
    end, end_abs = parse_config_time(end_time)
    if start is None or end is None:
        return None
    if start_abs == end_abs and end >= start:
        return end - start
    return None


def numeric_values(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]], field: str) -> list[float]:
    values = []
    for row in rows:
        value = logical_float(row, mapping, field)
        if value is not None:
            values.append(value)
    return values


def percentile(values: list[float], p: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    if len(finite) == 1:
        return finite[0]
    position = (len(finite) - 1) * p / 100.0
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return finite[low]
    return finite[low] + (finite[high] - finite[low]) * (position - low)


def mean(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.fmean(finite) if finite else None


def sample_std(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    if len(finite) < 2:
        return None
    return statistics.stdev(finite)


def summary_stats(values: list[float], prefix: str) -> dict[str, Any]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {
            f"{prefix}_count": 0,
            f"{prefix}_mean": "",
            f"{prefix}_median": "",
            f"{prefix}_std": "",
            f"{prefix}_p5": "",
            f"{prefix}_p90": "",
            f"{prefix}_p95": "",
            f"{prefix}_p99": "",
            f"{prefix}_min": "",
            f"{prefix}_max": "",
        }
    return {
        f"{prefix}_count": len(finite),
        f"{prefix}_mean": statistics.fmean(finite),
        f"{prefix}_median": percentile(finite, 50),
        f"{prefix}_std": statistics.stdev(finite) if len(finite) >= 2 else 0.0,
        f"{prefix}_p5": percentile(finite, 5),
        f"{prefix}_p90": percentile(finite, 90),
        f"{prefix}_p95": percentile(finite, 95),
        f"{prefix}_p99": percentile(finite, 99),
        f"{prefix}_min": min(finite),
        f"{prefix}_max": max(finite),
    }


def ci95(values: list[float]) -> tuple[float | None, float | None]:
    finite = [value for value in values if math.isfinite(value)]
    n = len(finite)
    if n < 2:
        return None, None
    m = statistics.fmean(finite)
    s = statistics.stdev(finite)
    t_table = {
        1: 12.706,
        2: 4.303,
        3: 3.182,
        4: 2.776,
        5: 2.571,
        6: 2.447,
        7: 2.365,
        8: 2.306,
        9: 2.262,
        10: 2.228,
        11: 2.201,
        12: 2.179,
        13: 2.160,
        14: 2.145,
        15: 2.131,
        16: 2.120,
        17: 2.110,
        18: 2.101,
        19: 2.093,
        20: 2.086,
        21: 2.080,
        22: 2.074,
        23: 2.069,
        24: 2.064,
        25: 2.060,
        26: 2.056,
        27: 2.052,
        28: 2.048,
        29: 2.045,
        30: 2.042,
    }
    critical = t_table.get(n - 1, 1.96)
    half_width = critical * s / math.sqrt(n)
    return m - half_width, m + half_width


def normalized_scenario_id(scenario_id: str) -> str:
    text = clean(scenario_id)
    if text in SCENARIO_DEFINITIONS:
        return text
    if "_" in text:
        prefix = text.split("_", 1)[0]
        if prefix in SCENARIO_DEFINITIONS:
            return prefix
    return text


def scenario_name_from_row(row: dict[str, Any]) -> str:
    scenario_name = clean(row.get("scenario_name"))
    if scenario_name:
        return scenario_name
    scenario_id = clean(row.get("scenario_id"))
    return SCENARIO_DEFINITIONS.get(scenario_id, scenario_id)


def load_scenario_config(run_dir: Path, global_config: Path | None) -> tuple[list[dict[str, str]], list[str]]:
    warnings: list[str] = []
    config_path = run_dir / "scenario_config.csv"
    if not config_path.exists() and global_config and global_config.exists():
        config_path = global_config
    if not config_path.exists():
        warnings.append(f"{run_dir.name}: no scenario_config.csv found; unsegmented descriptive analysis will be used if logs exist.")
        return [], warnings
    rows, _ = read_csv(config_path)
    config_rows = []
    for idx, row in enumerate(rows, start=1):
        config_rows.append(
            {
                "run_id": clean(row.get("run_id")) or run_dir.name,
                "trial_id": clean(row.get("trial_id")) or f"trial_{idx:02d}",
                "scenario_id": clean(row.get("scenario_id")) or "UNSPECIFIED",
                "scenario_name": scenario_name_from_row(row),
                "start_time": clean(row.get("start_time")),
                "end_time": clean(row.get("end_time")),
                "expected_state": clean(row.get("expected_state")),
                "expected_transition": clean(row.get("expected_transition")),
                "notes": clean(row.get("notes")),
            }
        )
    return config_rows, warnings


def csv_has_data_rows(path: Path) -> bool:
    try:
        with path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.reader(handle)
            next(reader, None)
            return next(reader, None) is not None
    except OSError:
        return False


def find_role_files(run_dir: Path, mapping: dict[str, dict[str, Any]], role: str) -> list[Path]:
    candidates = mapping["role_files"].get(role, [])
    if isinstance(candidates, str):
        candidates = [candidates]
    first_empty_group: list[Path] = []
    for candidate in candidates:
        found: list[Path] = []
        seen = set()
        direct = run_dir / candidate
        if direct.exists() and direct.is_file() and str(direct) not in seen:
            found.append(direct)
            seen.add(str(direct))
        for nested in run_dir.rglob(candidate):
            if nested.is_file() and str(nested) not in seen:
                found.append(nested)
                seen.add(str(nested))
        if found:
            non_empty = [path for path in found if csv_has_data_rows(path)]
            if non_empty:
                return non_empty
            if not first_empty_group:
                first_empty_group = found
    return first_empty_group


def load_role_rows(run_dir: Path, mapping: dict[str, dict[str, Any]]) -> tuple[dict[str, list[dict[str, Any]]], dict[str, list[Path]], list[dict[str, Any]]]:
    role_rows: dict[str, list[dict[str, Any]]] = {}
    role_files: dict[str, list[Path]] = {}
    file_summary: list[dict[str, Any]] = []
    for role in mapping["role_files"].keys():
        files = find_role_files(run_dir, mapping, role)
        role_files[role] = files
        rows: list[dict[str, Any]] = []
        for path in files:
            loaded_rows, fields = read_csv(path)
            rows.extend(loaded_rows)
            file_summary.append(
                {
                    "run_id": run_dir.name,
                    "role": role,
                    "path": str(path),
                    "rows": len(loaded_rows),
                    "columns": ";".join(fields),
                }
            )
        role_rows[role] = rows
    return role_rows, role_files, file_summary


def rows_with_elapsed(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]], timestamp_field: str = "timestamp") -> list[tuple[dict[str, Any], float | None]]:
    timestamps = [timestamp_seconds(row, mapping, timestamp_field) for row in rows]
    valid = [ts for ts in timestamps if ts is not None]
    if not valid:
        return [(row, None) for row in rows]
    first = min(valid)
    result = []
    for row, ts in zip(rows, timestamps):
        result.append((row, None if ts is None else ts - first))
    return result


def segment_rows(
    rows: list[dict[str, Any]],
    mapping: dict[str, dict[str, Any]],
    start_time: str,
    end_time: str,
    timestamp_field: str = "timestamp",
) -> list[dict[str, Any]]:
    if not rows:
        return []
    start, start_absolute = parse_config_time(start_time)
    end, end_absolute = parse_config_time(end_time)
    if start is None and end is None:
        return list(rows)
    if start_absolute or end_absolute:
        selected = []
        for row in rows:
            ts = timestamp_seconds(row, mapping, timestamp_field)
            if ts is None:
                continue
            if start is not None and ts < start:
                continue
            if end is not None and ts > end:
                continue
            selected.append(row)
        return selected
    elapsed_rows = rows_with_elapsed(rows, mapping, timestamp_field)
    selected = []
    for row, elapsed in elapsed_rows:
        if elapsed is None:
            continue
        if start is not None and elapsed < start:
            continue
        if end is not None and elapsed > end:
            continue
        selected.append(row)
    return selected


def normalize_state(value: Any) -> str | None:
    text = clean(value).upper().replace(" ", "_").replace("-", "_")
    if not text:
        return None
    if text in {"STOP", "STOPPED", "EMERGENCY_STOP"}:
        return "STOP"
    if text in {"WARNING", "WARN"}:
        return "WARNING"
    if text in {"RELEASE", "RESUME", "GO"}:
        return "RELEASE"
    if text in {"SAFE", "RUN", "CLEAR"}:
        return "SAFE"
    if text in {"HOLD", "STALE", "MISSING", "NO_VALID_SOURCE", "NO_VALID", "UNKNOWN"}:
        return "HOLD_STALE"
    if "STOP" in text:
        return "STOP"
    if "WARN" in text:
        return "WARNING"
    if "RELEASE" in text or "RESUME" in text:
        return "RELEASE"
    if "STALE" in text or "MISSING" in text or "NO_VALID" in text:
        return "HOLD_STALE"
    if "SAFE" in text or "RUN" in text:
        return "SAFE"
    return text


def derive_state(row: dict[str, Any], mapping: dict[str, dict[str, Any]]) -> str | None:
    raw_state, _ = logical_value(row, mapping, "safety_state")
    state = normalize_state(raw_state)
    stale = logical_bool(row, mapping, "stale_data_flag")
    data_fresh = logical_bool(row, mapping, "data_fresh_flag")
    if stale is True or data_fresh is False:
        if state is None or state == "SAFE":
            return "HOLD_STALE"
    if state is not None:
        return state
    stop = logical_bool(row, mapping, "stop_flag")
    if stop is True:
        return "STOP"
    if stop is False:
        return "SAFE"
    release = logical_bool(row, mapping, "release_flag")
    if release is True:
        return "RELEASE"
    return None


def is_stop_state(state: str | None) -> bool:
    return state == "STOP"


def distance_m(row: dict[str, Any], mapping: dict[str, dict[str, Any]]) -> float | None:
    direct = logical_float(row, mapping, "min_distance_m")
    if direct is not None:
        return direct
    generic = logical_float(row, mapping, "distance")
    unit, _ = logical_value(row, mapping, "distance_unit")
    if generic is not None and clean(unit).lower() in {"m", "meter", "meters"}:
        return generic
    return None


def distance_px(row: dict[str, Any], mapping: dict[str, dict[str, Any]]) -> float | None:
    direct = logical_float(row, mapping, "min_distance_px")
    if direct is not None:
        return direct
    generic = logical_float(row, mapping, "distance")
    unit, _ = logical_value(row, mapping, "distance_unit")
    if generic is not None and clean(unit).lower() in {"px", "pixel", "pixels"}:
        return generic
    return None


def normalize_source(row: dict[str, Any], mapping: dict[str, dict[str, Any]]) -> str:
    raw, _ = logical_value(row, mapping, "distance_source")
    source = clean(raw).lower()
    unit, _ = logical_value(row, mapping, "distance_unit")
    unit_text = clean(unit).lower()
    if "3d" in source or source in {"m", "meter", "meters"} or unit_text in {"m", "meter", "meters"}:
        return "3D"
    if "2d" in source or "cam" in source or source in {"px", "pixel", "pixels"} or unit_text in {"px", "pixel", "pixels"}:
        return "2D"
    if source in {"", "none", "null", "nan", "no_valid", "no_valid_source", "missing", "stale"}:
        if distance_m(row, mapping) is not None:
            return "3D"
        if distance_px(row, mapping) is not None:
            return "2D"
        return "NO_VALID"
    return source.upper()


def duration_seconds(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]], timestamp_field: str = "timestamp") -> float | None:
    timestamps = [timestamp_seconds(row, mapping, timestamp_field) for row in rows]
    valid = [ts for ts in timestamps if ts is not None]
    if len(valid) >= 2:
        return max(valid) - min(valid)
    return None


def event_counts(stop_flags: list[bool], states: list[str | None]) -> tuple[int, int]:
    stop_events = 0
    release_events = 0
    previous_stop = False
    previous_release = False
    for stop, state in zip(stop_flags, states):
        release_state = state == "RELEASE"
        if stop and not previous_stop:
            stop_events += 1
        if previous_stop and not stop:
            release_events += 1
        elif release_state and not previous_release:
            release_events += 1
        previous_stop = stop
        previous_release = release_state
    return stop_events, release_events


def stop_duration(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]], stop_flags: list[bool]) -> float | None:
    timestamps = [timestamp_seconds(row, mapping) for row in rows]
    if len(rows) != len(stop_flags) or len(rows) < 2:
        return None
    total = 0.0
    for idx in range(len(rows) - 1):
        if timestamps[idx] is None or timestamps[idx + 1] is None:
            continue
        if stop_flags[idx]:
            delta = timestamps[idx + 1] - timestamps[idx]
            if delta >= 0:
                total += delta
    return total


def release_delay(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]], stop_flags: list[bool]) -> float | None:
    if not rows or not stop_flags:
        return None
    timestamps = [timestamp_seconds(row, mapping) for row in rows]
    stop_start: float | None = None
    for idx, stop in enumerate(stop_flags):
        ts = timestamps[idx]
        if ts is None:
            continue
        if stop and stop_start is None:
            stop_start = ts
        elif stop_start is not None and not stop:
            delay = ts - stop_start
            return delay if delay >= 0 else None
    return None


def oscillation_count(
    rows: list[dict[str, Any]],
    mapping: dict[str, dict[str, Any]],
    states: list[str | None],
    stop_flags: list[bool],
    window_sec: float,
) -> int:
    if not rows or len(rows) != len(states):
        return 0
    timestamps = [timestamp_seconds(row, mapping) for row in rows]
    transitions: list[tuple[int, float | None, bool]] = []
    previous = stop_flags[0] if stop_flags else False
    for idx, stop in enumerate(stop_flags[1:], start=1):
        if stop != previous:
            transitions.append((idx, timestamps[idx], stop))
        previous = stop
    count = 0
    for first, second in zip(transitions, transitions[1:]):
        _, t1, stop1 = first
        _, t2, stop2 = second
        if t1 is None or t2 is None:
            continue
        if stop1 != stop2 and 0 <= (t2 - t1) <= window_sec:
            count += 1
    return count


def command_metrics(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]]) -> dict[str, Any]:
    commands = []
    ack_values = []
    send_latencies = []
    for row in rows:
        raw_command, _ = logical_value(row, mapping, "command_type")
        command = clean(raw_command).lower()
        is_stop_command = "stop" in command
        is_resume_command = command in {"go", "resume", "release"} or "resume" in command
        is_command_row = is_stop_command or is_resume_command
        if command:
            commands.append(command)
        if not is_command_row:
            continue
        ack = logical_bool(row, mapping, "command_acknowledged")
        if ack is not None:
            ack_values.append(ack)
        latency = logical_float(row, mapping, "command_send_latency_ms")
        if latency is not None:
            send_latencies.append(latency)
    stop_count = sum(1 for command in commands if "stop" in command)
    resume_count = sum(1 for command in commands if command in {"go", "resume", "release"} or "resume" in command)
    result = {
        "command_rows": len(rows),
        "stop_command_count": stop_count,
        "resume_command_count": resume_count,
        "command_ack_count": sum(1 for value in ack_values if value),
        "command_ack_available_count": len(ack_values),
    }
    result.update(summary_stats(send_latencies, "command_send_latency_ms"))
    return result


def expectation_flags(expected_state: str, expected_transition: str) -> tuple[bool, bool]:
    state_text = expected_state.upper().replace("-", "_")
    transition_text = expected_transition.upper().replace("-", "_")
    no_stop_tokens = {"NO_STOP", "NOSTOP", "NO STOP"}
    state_has_alternatives = "_OR_" in state_text or " OR " in state_text
    expects_stop_from_state = "STOP" in state_text and not state_has_alternatives
    expects_stop = expects_stop_from_state or ("STOP" in transition_text and transition_text not in no_stop_tokens)
    expects_safe_only = "SAFE" in state_text and not expects_stop
    return expects_stop, expects_safe_only


def field_available(rows: list[dict[str, Any]], mapping: dict[str, dict[str, Any]], field: str) -> bool:
    aliases = mapping["logical_fields"].get(field, [])
    if isinstance(aliases, str):
        aliases = [aliases]
    for row in rows:
        value, _ = first_value(row, aliases)
        if clean(value):
            return True
    return False


def analyze_trial(
    run_id: str,
    config_row: dict[str, str],
    role_rows: dict[str, list[dict[str, Any]]],
    mapping: dict[str, dict[str, Any]],
    warnings: list[dict[str, Any]],
    timeseries: list[dict[str, Any]],
    events: list[dict[str, Any]],
) -> tuple[dict[str, Any], dict[str, list[float]]]:
    trial_id = config_row["trial_id"]
    scenario_id = normalized_scenario_id(config_row["scenario_id"])
    scenario_name = config_row["scenario_name"]
    start = config_row.get("start_time", "")
    end = config_row.get("end_time", "")

    segmented = {
        role: segment_rows(rows, mapping, start, end, "command_timestamp" if role == "command_log" else "timestamp")
        for role, rows in role_rows.items()
    }
    runtime_rows = segmented.get("runtime", [])
    safety_rows = segmented.get("safety", [])
    if not safety_rows and runtime_rows:
        safety_rows = runtime_rows
    robot_rows = segmented.get("robot_telemetry", [])
    command_rows = segmented.get("command_log", [])

    trial_key = {
        "run_id": run_id,
        "trial_id": trial_id,
        "scenario_id": scenario_id,
        "scenario_name": scenario_name,
    }

    latency_values = numeric_values(runtime_rows, mapping, "total_latency_ms")
    fps_values = numeric_values(runtime_rows, mapping, "fps")
    runtime_duration = duration_seconds(runtime_rows, mapping)
    safety_duration = duration_seconds(safety_rows, mapping)
    duration = runtime_duration if runtime_duration is not None else safety_duration
    if duration is None:
        duration = config_duration_seconds(start, end)

    result: dict[str, Any] = {
        **trial_key,
        "expected_state": config_row.get("expected_state", ""),
        "expected_transition": config_row.get("expected_transition", ""),
        "start_time": start,
        "end_time": end,
        "runtime_frame_count": len(runtime_rows),
        "safety_sample_count": len(safety_rows),
        "pose_sample_count": len(segmented.get("pose", [])),
        "robot_telemetry_sample_count": len(robot_rows),
        "duration_sec": duration if duration is not None else "",
        "output_rate_fps": mean(fps_values) if fps_values else ((len(runtime_rows) / duration) if duration and duration > 0 else ""),
    }
    result.update(summary_stats(latency_values, "total_latency_ms"))
    result["mean_total_latency_ms"] = result.get("total_latency_ms_mean", "")
    result["median_total_latency_ms"] = result.get("total_latency_ms_median", "")
    result["p90_total_latency_ms"] = result.get("total_latency_ms_p90", "")
    result["p95_total_latency_ms"] = result.get("total_latency_ms_p95", "")
    result["p99_total_latency_ms"] = result.get("total_latency_ms_p99", "")
    result["max_total_latency_ms"] = result.get("total_latency_ms_max", "")

    for logical_field, output_field, _label in STAGE_FIELDS:
        stage_values = numeric_values(runtime_rows, mapping, logical_field)
        result[output_field] = mean(stage_values) if stage_values else ""

    states = [derive_state(row, mapping) for row in safety_rows]
    stop_flags = [is_stop_state(state) or logical_bool(row, mapping, "stop_flag") is True for row, state in zip(safety_rows, states)]
    state_counts = Counter(state for state in states if state)
    state_total = sum(state_counts.values())
    for state in ["SAFE", "WARNING", "STOP", "RELEASE", "HOLD_STALE"]:
        result[f"{state.lower()}_pct"] = (100.0 * state_counts.get(state, 0) / state_total) if state_total else ""

    if safety_rows:
        stop_events, release_events = event_counts(stop_flags, states)
        result["stop_event_count"] = stop_events
        result["release_event_count"] = release_events
        result["stop_duration_sec"] = stop_duration(safety_rows, mapping, stop_flags)
        result["release_delay_sec"] = release_delay(safety_rows, mapping, stop_flags)
        result["oscillation_count"] = oscillation_count(
            safety_rows,
            mapping,
            states,
            stop_flags,
            float(mapping["analysis"].get("oscillation_window_sec", 2.0)),
        )

        expected_stop, expected_safe_only = expectation_flags(result["expected_state"], result["expected_transition"])
        any_stop = any(stop_flags)
        result["stop_success"] = 1 if expected_stop and any_stop else (0 if expected_stop else "")
        result["false_stop_count"] = stop_events if expected_safe_only else ""
        result["missed_stop_count"] = 1 if expected_stop and not any_stop else ""
    else:
        result["stop_event_count"] = ""
        result["release_event_count"] = ""
        result["stop_duration_sec"] = ""
        result["release_delay_sec"] = ""
        result["oscillation_count"] = ""
        result["stop_success"] = ""
        result["false_stop_count"] = ""
        result["missed_stop_count"] = ""

    distance_rows = safety_rows if safety_rows else robot_rows
    distance_m_values = [value for row in distance_rows if (value := distance_m(row, mapping)) is not None]
    distance_px_values = [value for row in distance_rows if (value := distance_px(row, mapping)) is not None]
    result.update(summary_stats(distance_m_values, "min_distance_m"))
    result.update(summary_stats(distance_px_values, "min_distance_px"))

    sources = [normalize_source(row, mapping) for row in distance_rows]
    source_counts = Counter(sources)
    source_total = sum(source_counts.values())
    result["distance_source_3d_pct"] = (100.0 * source_counts.get("3D", 0) / source_total) if source_total else ""
    result["distance_source_2d_pct"] = (100.0 * source_counts.get("2D", 0) / source_total) if source_total else ""
    result["distance_source_no_valid_pct"] = (100.0 * source_counts.get("NO_VALID", 0) / source_total) if source_total else ""
    stale_count = sum(1 for row, state in zip(distance_rows, states if len(states) == len(distance_rows) else [None] * len(distance_rows)) if state == "HOLD_STALE" or normalize_source(row, mapping) == "NO_VALID")
    result["stale_or_no_valid_pct"] = (100.0 * stale_count / len(distance_rows)) if distance_rows else ""

    result.update(command_metrics(command_rows, mapping))

    if runtime_rows and not latency_values:
        warnings.append({**trial_key, "warning": "Runtime rows exist but no total latency column was mapped."})
    if not runtime_rows:
        warnings.append({**trial_key, "warning": "No runtime rows available for this trial."})
    if safety_rows and not state_total:
        warnings.append({**trial_key, "warning": "Safety rows exist but no safety state or stop flag was mapped."})
    if not safety_rows:
        warnings.append({**trial_key, "warning": "No safety rows available for this trial."})
    if distance_rows and not distance_m_values and not distance_px_values:
        warnings.append({**trial_key, "warning": "Distance rows exist but no mapped distance values were available."})
    if command_rows and result["stop_command_count"] == 0 and result["resume_command_count"] == 0:
        warnings.append({**trial_key, "warning": "Command rows exist but no stop/resume command values were mapped."})

    # Time-series rows for plots.
    elapsed_safety = rows_with_elapsed(safety_rows, mapping)
    for row, elapsed in elapsed_safety:
        if elapsed is None:
            continue
        dm = distance_m(row, mapping)
        dpx = distance_px(row, mapping)
        state = derive_state(row, mapping)
        stop = is_stop_state(state) or logical_bool(row, mapping, "stop_flag") is True
        timeseries.append(
            {
                **trial_key,
                "elapsed_s": elapsed,
                "distance_m": dm,
                "distance_px": dpx,
                "stop_flag": int(stop),
                "state": state or "",
                "source": normalize_source(row, mapping),
                "warning_threshold_m": logical_float(row, mapping, "warning_threshold_m"),
                "stop_threshold_m": logical_float(row, mapping, "stop_threshold_m"),
                "release_threshold_m": logical_float(row, mapping, "release_threshold_m"),
                "warning_threshold_px": logical_float(row, mapping, "warning_threshold_px"),
                "stop_threshold_px": logical_float(row, mapping, "stop_threshold_px"),
                "release_threshold_px": logical_float(row, mapping, "release_threshold_px"),
            }
        )
    previous_stop = False
    for item in timeseries:
        if item["run_id"] != run_id or item["trial_id"] != trial_id:
            continue
        current_stop = bool(item["stop_flag"])
        if current_stop and not previous_stop:
            events.append({**trial_key, "elapsed_s": item["elapsed_s"], "event": "stop"})
        if previous_stop and not current_stop:
            events.append({**trial_key, "elapsed_s": item["elapsed_s"], "event": "release"})
        previous_stop = current_stop

    latency_groups = {
        "scenario": latency_values,
        "trial": latency_values,
    }
    return result, latency_groups


def real_data_row_count(role_rows: dict[str, list[dict[str, Any]]]) -> int:
    unique_rows = 0
    for rows in role_rows.values():
        unique_rows += len(rows)
    return unique_rows


def discover_runs(data_root: Path) -> list[Path]:
    if not data_root.exists():
        return []
    run_dirs = []
    for child in data_root.iterdir():
        if child.is_dir() and RUN_RE.match(child.name):
            run_dirs.append(child)
    return sorted(run_dirs, key=lambda p: p.name)


def summarize_group(rows: list[dict[str, Any]], group_key: str, latency_pools: dict[str, list[float]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[clean(row.get(group_key))].append(row)

    metrics = [
        "mean_total_latency_ms",
        "median_total_latency_ms",
        "p90_total_latency_ms",
        "p95_total_latency_ms",
        "p99_total_latency_ms",
        "safe_pct",
        "warning_pct",
        "stop_pct",
        "release_pct",
        "hold_stale_pct",
        "distance_source_3d_pct",
        "distance_source_2d_pct",
        "distance_source_no_valid_pct",
        "stale_or_no_valid_pct",
        "stop_event_count",
        "release_event_count",
        "oscillation_count",
        "stop_success",
        "false_stop_count",
        "missed_stop_count",
    ]
    for _logical, output_field, _label in STAGE_FIELDS:
        metrics.append(output_field)

    summaries = []
    for key, group_rows in sorted(grouped.items()):
        out: dict[str, Any] = {group_key: key, "n_trials": len(group_rows), "n_runs": len({row.get("run_id") for row in group_rows})}
        if group_key == "scenario_id":
            names = [clean(row.get("scenario_name")) for row in group_rows if clean(row.get("scenario_name"))]
            out["scenario_name"] = Counter(names).most_common(1)[0][0] if names else ""
        for metric in metrics:
            values = [to_float(row.get(metric)) for row in group_rows]
            finite = [value for value in values if value is not None]
            out[f"{metric}_mean"] = mean(finite) if finite else ""
            out[f"{metric}_std"] = sample_std(finite) if len(finite) >= 2 else ""
            lo, hi = ci95(finite)
            out[f"{metric}_ci95_low"] = lo if lo is not None else ""
            out[f"{metric}_ci95_high"] = hi if hi is not None else ""
        pooled = latency_pools.get(key, [])
        pooled_stats = summary_stats(pooled, "pooled_frame_latency_ms")
        out.update(pooled_stats)
        summaries.append(out)
    return summaries


def summarize_runs(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[clean(row.get("run_id"))].append(row)
    summaries = []
    for run_id, group_rows in sorted(grouped.items()):
        mean_latencies = [value for row in group_rows if (value := to_float(row.get("mean_total_latency_ms"))) is not None]
        stop_success_values = [value for row in group_rows if (value := to_float(row.get("stop_success"))) is not None]
        summaries.append(
            {
                "run_id": run_id,
                "n_trials": len(group_rows),
                "n_scenarios": len({row.get("scenario_id") for row in group_rows}),
                "mean_total_latency_ms_mean": mean(mean_latencies) if mean_latencies else "",
                "mean_total_latency_ms_std": sample_std(mean_latencies) if len(mean_latencies) >= 2 else "",
                "stop_success_rate": mean(stop_success_values) if stop_success_values else "",
                "stop_event_count_total": sum(int(to_float(row.get("stop_event_count")) or 0) for row in group_rows),
                "release_event_count_total": sum(int(to_float(row.get("release_event_count")) or 0) for row in group_rows),
            }
        )
    if rows:
        mean_latencies = [value for row in rows if (value := to_float(row.get("mean_total_latency_ms"))) is not None]
        stop_success_values = [value for row in rows if (value := to_float(row.get("stop_success"))) is not None]
        summaries.append(
            {
                "run_id": "ALL_RUNS",
                "n_trials": len(rows),
                "n_scenarios": len({row.get("scenario_id") for row in rows}),
                "mean_total_latency_ms_mean": mean(mean_latencies) if mean_latencies else "",
                "mean_total_latency_ms_std": sample_std(mean_latencies) if len(mean_latencies) >= 2 else "",
                "stop_success_rate": mean(stop_success_values) if stop_success_values else "",
                "stop_event_count_total": sum(int(to_float(row.get("stop_event_count")) or 0) for row in rows),
                "release_event_count_total": sum(int(to_float(row.get("release_event_count")) or 0) for row in rows),
            }
        )
    return summaries


def maybe_import_matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt, None
    except Exception as exc:
        return None, str(exc)


def save_figure(fig: Any, figures_dir: Path, name: str, generated: list[str]) -> None:
    for ext in ("png", "svg", "pdf"):
        path = figures_dir / f"{name}.{ext}"
        fig.savefig(path, bbox_inches="tight", dpi=200 if ext == "png" else None)
        generated.append(str(path))


def sample_points(points: list[dict[str, Any]], max_points: int = 5000) -> list[dict[str, Any]]:
    if len(points) <= max_points:
        return points
    step = max(1, len(points) // max_points)
    return points[::step]


def generate_plots(
    output_dir: Path,
    per_trial: list[dict[str, Any]],
    per_scenario: list[dict[str, Any]],
    latency_by_scenario: dict[str, list[float]],
    latency_by_trial: dict[str, list[float]],
    timeseries: list[dict[str, Any]],
    events: list[dict[str, Any]],
    warnings: list[dict[str, Any]],
) -> list[str]:
    figures_dir = output_dir / "figures"
    ensure_dir(figures_dir)
    generated: list[str] = []
    plt, error = maybe_import_matplotlib()
    if plt is None:
        warnings.append({"run_id": "", "trial_id": "", "scenario_id": "", "scenario_name": "", "warning": f"matplotlib unavailable; plots skipped: {error}"})
        return generated

    def has_values(groups: dict[str, list[float]]) -> bool:
        return any(values for values in groups.values())

    if has_values(latency_by_scenario):
        labels = [key for key, values in sorted(latency_by_scenario.items()) if values]
        data = [latency_by_scenario[key] for key in labels]
        fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
        ax.boxplot(data, labels=labels, showfliers=False)
        ax.set_title("Software-stage total latency by scenario")
        ax.set_xlabel("Scenario")
        ax.set_ylabel("Latency (ms)")
        ax.tick_params(axis="x", rotation=30)
        ax.grid(axis="y", alpha=0.3)
        save_figure(fig, figures_dir, "latency_boxplot_by_scenario", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Latency boxplot by scenario skipped because no latency values were available."})

    if has_values(latency_by_trial):
        labels = [key for key, values in sorted(latency_by_trial.items()) if values]
        data = [latency_by_trial[key] for key in labels]
        fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
        ax.boxplot(data, labels=labels, showfliers=False)
        ax.set_title("Software-stage total latency by trial")
        ax.set_xlabel("Trial")
        ax.set_ylabel("Latency (ms)")
        ax.tick_params(axis="x", rotation=45)
        ax.grid(axis="y", alpha=0.3)
        save_figure(fig, figures_dir, "latency_boxplot_by_trial", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Latency boxplot by trial skipped because no latency values were available."})

    if per_scenario and any(to_float(row.get("safe_pct_mean")) is not None for row in per_scenario):
        labels = [row["scenario_id"] for row in per_scenario]
        states = [
            ("safe_pct_mean", "SAFE"),
            ("warning_pct_mean", "WARNING"),
            ("stop_pct_mean", "STOP"),
            ("release_pct_mean", "RELEASE"),
            ("hold_stale_pct_mean", "HOLD/STALE"),
        ]
        fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
        bottoms = [0.0] * len(labels)
        for field, label in states:
            values = [to_float(row.get(field)) or 0.0 for row in per_scenario]
            ax.bar(labels, values, bottom=bottoms, label=label)
            bottoms = [a + b for a, b in zip(bottoms, values)]
        ax.set_title("Safety-state percentage by scenario")
        ax.set_xlabel("Scenario")
        ax.set_ylabel("Samples (%)")
        ax.set_ylim(0, 100)
        ax.tick_params(axis="x", rotation=30)
        ax.legend(loc="upper right")
        save_figure(fig, figures_dir, "safety_state_stacked_bar_by_scenario", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Safety-state stacked bar skipped because no mapped safety states were available."})

    distance_points_m = [point for point in timeseries if point.get("distance_m") is not None]
    distance_points_px = [point for point in timeseries if point.get("distance_px") is not None]
    distance_points = distance_points_m or distance_points_px
    if distance_points:
        use_m = bool(distance_points_m)
        y_field = "distance_m" if use_m else "distance_px"
        unit = "m" if use_m else "px"
        points = sample_points(distance_points)
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for scenario_id in sorted({point["scenario_id"] for point in points}):
            scenario_points = [point for point in points if point["scenario_id"] == scenario_id and point.get(y_field) is not None]
            if not scenario_points:
                continue
            ax.plot([point["elapsed_s"] for point in scenario_points], [point[y_field] for point in scenario_points], ".", markersize=2, label=scenario_id)
        threshold_fields = [
            ("warning_threshold_m" if use_m else "warning_threshold_px", "warning"),
            ("stop_threshold_m" if use_m else "stop_threshold_px", "stop"),
            ("release_threshold_m" if use_m else "release_threshold_px", "release"),
        ]
        for field, label in threshold_fields:
            values = [point.get(field) for point in points if point.get(field) is not None]
            if values:
                ax.axhline(statistics.fmean(values), linestyle="--", linewidth=1.0, label=f"{label} threshold")
        ax.set_title("Minimum distance over time")
        ax.set_xlabel("Elapsed time within log (s)")
        ax.set_ylabel(f"Minimum distance ({unit})")
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        save_figure(fig, figures_dir, "distance_over_time_with_thresholds", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Distance-over-time plot skipped because no mapped distance values were available."})

    if timeseries:
        points = sample_points(timeseries)
        fig, ax = plt.subplots(figsize=(9, 4.0))
        for scenario_id in sorted({point["scenario_id"] for point in points}):
            scenario_points = [point for point in points if point["scenario_id"] == scenario_id]
            ax.step(
                [point["elapsed_s"] for point in scenario_points],
                [point["stop_flag"] for point in scenario_points],
                where="post",
                label=scenario_id,
                linewidth=1.0,
            )
        ax.set_title("Stop flag over time")
        ax.set_xlabel("Elapsed time within log (s)")
        ax.set_ylabel("Stop flag")
        ax.set_yticks([0, 1])
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        save_figure(fig, figures_dir, "stop_flag_over_time", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Stop flag/state plot skipped because no safety time series was available."})

    if per_scenario and any(to_float(row.get("distance_source_3d_pct_mean")) is not None for row in per_scenario):
        labels = [row["scenario_id"] for row in per_scenario]
        fields = [
            ("distance_source_3d_pct_mean", "3D"),
            ("distance_source_2d_pct_mean", "2D"),
            ("distance_source_no_valid_pct_mean", "No valid"),
        ]
        fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
        bottoms = [0.0] * len(labels)
        for field, label in fields:
            values = [to_float(row.get(field)) or 0.0 for row in per_scenario]
            ax.bar(labels, values, bottom=bottoms, label=label)
            bottoms = [a + b for a, b in zip(bottoms, values)]
        ax.set_title("Distance-source availability by scenario")
        ax.set_xlabel("Scenario")
        ax.set_ylabel("Samples (%)")
        ax.set_ylim(0, 100)
        ax.tick_params(axis="x", rotation=30)
        ax.legend(loc="upper right")
        save_figure(fig, figures_dir, "distance_source_availability_by_scenario", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Distance-source availability plot skipped because no source values were available."})

    if events:
        labels = [f"{event['run_id']}:{event['trial_id']}" for event in events]
        unique_labels = list(dict.fromkeys(labels))
        y_lookup = {label: idx for idx, label in enumerate(unique_labels)}
        colors = {"stop": "tab:red", "release": "tab:green"}
        fig, ax = plt.subplots(figsize=(9, max(3.5, len(unique_labels) * 0.3)))
        for event in events:
            label = f"{event['run_id']}:{event['trial_id']}"
            ax.scatter(event["elapsed_s"], y_lookup[label], color=colors.get(event["event"], "tab:blue"), label=event["event"])
        handles, handle_labels = ax.get_legend_handles_labels()
        dedup = dict(zip(handle_labels, handles))
        ax.legend(dedup.values(), dedup.keys(), loc="best")
        ax.set_title("Stop/release event timeline")
        ax.set_xlabel("Elapsed time within trial/log (s)")
        ax.set_yticks(range(len(unique_labels)))
        ax.set_yticklabels(unique_labels)
        ax.grid(axis="x", alpha=0.3)
        save_figure(fig, figures_dir, "stop_release_event_timeline", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Stop/release event timeline skipped because no events were detected."})

    if per_scenario:
        stage_labels = [(field, label) for _logical, field, label in STAGE_FIELDS]
        if any(to_float(row.get(f"{field}_mean")) is not None for row in per_scenario for field, _label in stage_labels):
            labels = [row["scenario_id"] for row in per_scenario]
            fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
            bottoms = [0.0] * len(labels)
            for field, label in stage_labels:
                summary_field = f"{field}_mean"
                values = [to_float(row.get(summary_field)) or 0.0 for row in per_scenario]
                ax.bar(labels, values, bottom=bottoms, label=label)
                bottoms = [a + b for a, b in zip(bottoms, values)]
            ax.set_title("Runtime stage breakdown by scenario")
            ax.set_xlabel("Scenario")
            ax.set_ylabel("Mean latency contribution (ms)")
            ax.tick_params(axis="x", rotation=30)
            ax.legend(loc="best", fontsize=8)
            save_figure(fig, figures_dir, "runtime_stage_breakdown_by_scenario", generated)
            plt.close(fig)
        else:
            warnings.append({"warning": "Runtime stage breakdown skipped because no stage latency columns were available."})

    if per_scenario and any(to_float(row.get("mean_total_latency_ms_mean")) is not None for row in per_scenario):
        labels = [row["scenario_id"] for row in per_scenario]
        values = [to_float(row.get("mean_total_latency_ms_mean")) or 0.0 for row in per_scenario]
        errors = [to_float(row.get("mean_total_latency_ms_std")) or 0.0 for row in per_scenario]
        fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
        ax.errorbar(labels, values, yerr=errors, fmt="o", capsize=4)
        ax.set_title("Repeated-trial mean latency by scenario")
        ax.set_xlabel("Scenario")
        ax.set_ylabel("Trial-level mean latency (ms)")
        ax.tick_params(axis="x", rotation=30)
        ax.grid(axis="y", alpha=0.3)
        save_figure(fig, figures_dir, "repeated_trial_mean_std_latency", generated)
        plt.close(fig)
    else:
        warnings.append({"warning": "Repeated-trial summary latency plot skipped because trial-level latency was unavailable."})

    return generated


def markdown_table(rows: list[dict[str, Any]], columns: list[str], max_rows: int = 20) -> str:
    if not rows:
        return "No rows available.\n"
    selected = rows[:max_rows]
    lines = []
    lines.append("| " + " | ".join(columns) + " |")
    lines.append("| " + " | ".join("---" for _ in columns) + " |")
    for row in selected:
        values = []
        for column in columns:
            value = row.get(column, "")
            if isinstance(value, float):
                values.append(f"{value:.4g}")
            else:
                values.append(clean(value).replace("|", "/"))
        lines.append("| " + " | ".join(values) + " |")
    if len(rows) > max_rows:
        lines.append(f"\nShowing {max_rows} of {len(rows)} rows. See CSV outputs for the complete table.")
    return "\n".join(lines) + "\n"


def write_report(
    output_dir: Path,
    data_root: Path,
    run_summaries: list[dict[str, Any]],
    file_summary: list[dict[str, Any]],
    scenario_rows: list[dict[str, Any]],
    per_trial: list[dict[str, Any]],
    per_scenario: list[dict[str, Any]],
    across_run: list[dict[str, Any]],
    warnings: list[dict[str, Any]],
    figures: list[str],
    real_run_ids: list[str],
) -> None:
    report_path = output_dir / "repeated_trial_report.md"
    lines: list[str] = []
    lines.append("# Repeated-Trial Validation Report")
    lines.append("")
    lines.append(f"Generated UTC: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}")
    lines.append(f"Data root: `{data_root}`")
    lines.append("")
    lines.append("## Overview")
    lines.append("")
    lines.append(
        "This report analyzes laboratory safety-assistance repeated trials using available real log files. "
        "Frame-level values are used for latency distributions, while repeatability statistics use trial-level summaries."
    )
    lines.append("")
    lines.append("## Repeated-Trial Presence")
    lines.append("")
    if len(real_run_ids) == 0:
        lines.append("No repeated-trial run with analyzable log rows was found. The workflow is prepared, but no repeated-trial results are reported.")
    elif len(real_run_ids) == 1:
        lines.append("Only one repeated-trial run was found. Results are descriptive and do not provide between-trial repeatability.")
    else:
        lines.append(f"Repeated independent trials were analyzed across {len(real_run_ids)} runs.")
    lines.append("")
    lines.append("## Runs Found")
    lines.append("")
    lines.append(markdown_table(run_summaries, ["run_id", "data_rows", "config_trials", "runtime_rows", "safety_rows", "command_rows"], max_rows=50))
    lines.append("")
    lines.append("## Scenario Definitions")
    lines.append("")
    scenario_columns = ["run_id", "trial_id", "scenario_id", "scenario_name", "start_time", "end_time", "expected_state", "expected_transition"]
    lines.append(markdown_table(scenario_rows, scenario_columns, max_rows=50))
    lines.append("")
    lines.append("## Per-Trial Summary")
    lines.append("")
    trial_columns = [
        "run_id",
        "trial_id",
        "scenario_id",
        "runtime_frame_count",
        "duration_sec",
        "mean_total_latency_ms",
        "p95_total_latency_ms",
        "stop_event_count",
        "release_event_count",
        "stop_success",
        "false_stop_count",
        "missed_stop_count",
        "distance_source_3d_pct",
        "distance_source_2d_pct",
        "distance_source_no_valid_pct",
    ]
    lines.append(markdown_table(per_trial, trial_columns, max_rows=50))
    lines.append("")
    lines.append("## Per-Scenario Summary")
    lines.append("")
    scenario_summary_columns = [
        "scenario_id",
        "scenario_name",
        "n_trials",
        "n_runs",
        "mean_total_latency_ms_mean",
        "mean_total_latency_ms_std",
        "mean_total_latency_ms_ci95_low",
        "mean_total_latency_ms_ci95_high",
        "stop_pct_mean",
        "stop_success_mean",
        "distance_source_3d_pct_mean",
        "distance_source_2d_pct_mean",
        "distance_source_no_valid_pct_mean",
    ]
    lines.append(markdown_table(per_scenario, scenario_summary_columns, max_rows=50))
    lines.append("")
    lines.append("## Across-Run Statistics")
    lines.append("")
    lines.append(markdown_table(across_run, ["run_id", "n_trials", "n_scenarios", "mean_total_latency_ms_mean", "mean_total_latency_ms_std", "stop_success_rate", "stop_event_count_total", "release_event_count_total"], max_rows=50))
    lines.append("")
    lines.append("## Runtime Results")
    lines.append("")
    lines.append(
        "Runtime latency metrics are software-stage values from mapped timing columns such as `pipeline_ms`, "
        "`software_e2e_ms`, or `total_latency_ms`. Physical robot stop-time is not inferred from these values."
    )
    lines.append("")
    lines.append("## Safety-State Results")
    lines.append("")
    lines.append(
        "Safety percentages are computed from mapped safety-state or stop-flag samples. "
        "Stop success, false stop, and missed stop metrics are only computed when expected states/transitions are present in the scenario config."
    )
    lines.append("")
    lines.append("## Distance-Source Availability Results")
    lines.append("")
    lines.append(
        "Distance-source percentages classify mapped rows as 3D, 2D fallback, or no-valid source. "
        "Missing or stale inputs are reported only when corresponding columns are available or no-valid sources are logged."
    )
    lines.append("")
    lines.append("## Command-Log Results")
    lines.append("")
    lines.append(
        "Command-log metrics count mapped stop/resume commands and command-send timing where available. "
        "Command generation or send timing must not be described as physical robot stop-time without an external timing reference."
    )
    lines.append("")
    lines.append("## Generated Figure List")
    lines.append("")
    if figures:
        for figure in figures:
            lines.append(f"- `{figure}`")
    else:
        lines.append("No figures were generated because required mapped data were unavailable or plotting dependencies were missing.")
    lines.append("")
    lines.append("## Missing Columns / Missing Data Warnings")
    lines.append("")
    warning_rows = [{key: row.get(key, "") for key in ["run_id", "trial_id", "scenario_id", "warning"]} for row in warnings]
    lines.append(markdown_table(warning_rows, ["run_id", "trial_id", "scenario_id", "warning"], max_rows=80))
    lines.append("")
    lines.append("## Limitations")
    lines.append("")
    lines.append(
        "Physical robot stop-time latency was not independently measured with an external timing reference. "
        "Therefore, the reported timing values describe software-stage processing and command-generation behavior, "
        "not formal end-to-end safety response."
    )
    lines.append("")
    lines.append(
        "Frames inside a trial are correlated. Frame-level latency distributions are useful for runtime characterization, "
        "but trial-level summaries should be used for repeated-trial repeatability."
    )
    lines.append("")
    if not real_run_ids:
        lines.append("No real repeated-trial log rows were available in the repeated-trials data root at analysis time.")
    lines.append("")
    lines.append("## Suggested Manuscript Text Snippets")
    lines.append("")
    if len(real_run_ids) >= 2:
        lines.append(
            f"Laboratory safety-assistance repeated trials were analyzed across {len(real_run_ids)} independent runs. "
            "The analysis segmented runtime, safety-state, distance-source, and command logs by the pre-defined scenario configuration."
        )
    elif len(real_run_ids) == 1:
        lines.append(
            "A single repeated-trial run was analyzed descriptively. Additional independent runs are required before claiming between-trial repeatability."
        )
    else:
        lines.append(
            "The repeated-trial protocol, logging structure, and analysis workflow were prepared before physical trial execution. "
            "No repeated-trial results are reported until real run logs are collected."
        )
    lines.append("")
    lines.append(
        "Runtime values are reported as software-stage timing and command-generation measurements. "
        "Physical stop-time requires an external timing reference and was not inferred from software logs alone."
    )
    lines.append("")
    lines.append("## Source Files")
    lines.append("")
    lines.append(markdown_table(file_summary, ["run_id", "role", "path", "rows"], max_rows=80))
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(args: argparse.Namespace) -> int:
    data_root = Path(args.data_root)
    output_dir = Path(args.output)
    ensure_dir(output_dir)
    tables_dir = output_dir / "tables"
    ensure_dir(tables_dir)

    mapping, mapping_warnings = load_mapping(Path(args.column_mapping) if args.column_mapping else None)
    warnings: list[dict[str, Any]] = [{"warning": warning} for warning in mapping_warnings]

    run_dirs = discover_runs(data_root)
    all_per_trial: list[dict[str, Any]] = []
    all_scenario_rows: list[dict[str, Any]] = []
    all_file_summary: list[dict[str, Any]] = []
    run_summaries: list[dict[str, Any]] = []
    latency_by_scenario: dict[str, list[float]] = defaultdict(list)
    latency_by_trial: dict[str, list[float]] = defaultdict(list)
    timeseries: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    real_run_ids: list[str] = []

    global_config = Path(args.scenario_config) if args.scenario_config else None
    if not run_dirs:
        warnings.append({"warning": f"No run_XX folders found under {data_root}."})
        template_config = global_config or Path("configs/repeated_trial_scenarios.csv")
        if template_config.exists():
            template_rows, _ = read_csv(template_config)
            for row in template_rows:
                all_scenario_rows.append(
                    {
                        "run_id": "template",
                        "trial_id": clean(row.get("trial_id")),
                        "scenario_id": clean(row.get("scenario_id")),
                        "scenario_name": scenario_name_from_row(row),
                        "start_time": clean(row.get("start_time")),
                        "end_time": clean(row.get("end_time")),
                        "expected_state": clean(row.get("expected_state")),
                        "expected_transition": clean(row.get("expected_transition")),
                        "notes": clean(row.get("notes")),
                    }
                )

    for run_dir in run_dirs:
        config_rows, config_warnings = load_scenario_config(run_dir, global_config)
        for warning in config_warnings:
            warnings.append({"run_id": run_dir.name, "warning": warning})
        role_rows, _role_files, file_summary = load_role_rows(run_dir, mapping)
        all_file_summary.extend(file_summary)
        data_rows = real_data_row_count(role_rows)
        if data_rows > 0:
            real_run_ids.append(run_dir.name)
        run_summaries.append(
            {
                "run_id": run_dir.name,
                "data_rows": data_rows,
                "config_trials": len(config_rows),
                "runtime_rows": len(role_rows.get("runtime", [])),
                "safety_rows": len(role_rows.get("safety", [])),
                "command_rows": len(role_rows.get("command_log", [])),
            }
        )

        if not config_rows and data_rows > 0:
            config_rows = [
                {
                    "run_id": run_dir.name,
                    "trial_id": "unsegmented",
                    "scenario_id": "UNSEGMENTED",
                    "scenario_name": "unsegmented",
                    "start_time": "",
                    "end_time": "",
                    "expected_state": "",
                    "expected_transition": "",
                    "notes": "No scenario_config.csv was found.",
                }
            ]
        for config_row in config_rows:
            config_row = dict(config_row)
            config_row["run_id"] = run_dir.name
            all_scenario_rows.append(config_row)
            if config_row.get("scenario_id") != "UNSEGMENTED":
                start_value, _ = parse_config_time(config_row.get("start_time"))
                end_value, _ = parse_config_time(config_row.get("end_time"))
                if start_value is None or end_value is None:
                    warnings.append(
                        {
                            "run_id": run_dir.name,
                            "trial_id": config_row.get("trial_id", ""),
                            "scenario_id": config_row.get("scenario_id", ""),
                            "scenario_name": config_row.get("scenario_name", ""),
                            "warning": "Scenario row skipped because start_time/end_time are not set to parseable values.",
                        }
                    )
                    continue
            trial_result, latency_groups = analyze_trial(run_dir.name, config_row, role_rows, mapping, warnings, timeseries, events)
            all_per_trial.append(trial_result)
            scenario_key = trial_result["scenario_id"]
            trial_key = f"{trial_result['run_id']}:{trial_result['trial_id']}"
            latency_by_scenario[scenario_key].extend(latency_groups["scenario"])
            latency_by_trial[trial_key].extend(latency_groups["trial"])

    per_scenario = summarize_group(all_per_trial, "scenario_id", latency_by_scenario)
    across_run = summarize_runs(all_per_trial)

    write_csv(tables_dir / "per_trial_metrics.csv", all_per_trial)
    write_csv(tables_dir / "per_scenario_metrics.csv", per_scenario)
    write_csv(tables_dir / "across_run_summary.csv", across_run)
    write_csv(tables_dir / "run_file_summary.csv", all_file_summary)
    write_csv(tables_dir / "scenario_definitions_used.csv", all_scenario_rows)

    figures = generate_plots(output_dir, all_per_trial, per_scenario, latency_by_scenario, latency_by_trial, timeseries, events, warnings)
    write_csv(tables_dir / "missing_data_warnings.csv", warnings)
    write_report(
        output_dir,
        data_root,
        run_summaries,
        all_file_summary,
        all_scenario_rows,
        all_per_trial,
        per_scenario,
        across_run,
        warnings,
        figures,
        real_run_ids,
    )
    print(output_dir / "repeated_trial_report.md")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze real repeated-trial logs without fabricating missing data.")
    parser.add_argument("--data-root", default="data/repeated_trials", help="Root containing run_XX folders.")
    parser.add_argument("--output", default="analysis_outputs/repeated_trials", help="Output directory for tables, figures, and report.")
    parser.add_argument("--scenario-config", help="Optional fallback scenario config if a run lacks scenario_config.csv.")
    parser.add_argument("--column-mapping", default="configs/column_mapping.yaml", help="YAML mapping of project columns to logical fields.")
    return parser.parse_args()


def main() -> int:
    try:
        return analyze(parse_args())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
