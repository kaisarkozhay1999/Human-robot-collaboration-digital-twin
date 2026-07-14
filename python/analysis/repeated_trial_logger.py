"""Create and manage repeated-trial run folders.

This helper does not run the robot pipeline. It prepares a non-overwriting
folder for real laboratory logs and records run-level metadata/notes.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


RUN_RE = re.compile(r"^run_(\d+)$", re.IGNORECASE)

LOG_HEADERS = {
    "runtime.csv": [
        "timestamp",
        "frame_id",
        "total_latency_ms",
        "camera_retrieval_latency_ms",
        "pose_latency_ms",
        "robot_detection_latency_ms",
        "marker_latency_ms",
        "triangulation_latency_ms",
        "distance_latency_ms",
        "decision_send_latency_ms",
        "fps",
    ],
    "safety.csv": [
        "timestamp",
        "frame_id",
        "safety_state",
        "stop_flag",
        "release_flag",
        "stale_data_flag",
        "data_fresh_flag",
        "min_distance_m",
        "min_distance_px",
        "distance_source",
        "warning_threshold_m",
        "stop_threshold_m",
        "release_threshold_m",
        "warning_threshold_px",
        "stop_threshold_px",
        "release_threshold_px",
    ],
    "pose.csv": [
        "timestamp",
        "frame_id",
        "pose_valid",
        "tracked_joints",
        "tracked_joint_ratio",
        "mean_joint_confidence",
        "persons_cam1",
        "persons_cam2",
        "matched_people",
        "output_people",
        "distance_source",
    ],
    "robot_telemetry.csv": [
        "timestamp",
        "frame_id",
        "joint_angles_deg",
        "robot_data_fresh",
        "human_data_fresh",
        "data_fresh",
        "min_distance_m",
        "safety_state",
        "command_acknowledged",
    ],
    "command_log.csv": [
        "command_timestamp",
        "command_type",
        "transport",
        "command_send_latency_ms",
        "command_acknowledged",
        "message",
    ],
}

DEFAULT_SCENARIOS = [
    ("trial_01", "S1", "human_far_static", "0.0", "25.0", "SAFE", "no_stop", "human far from robot"),
    ("trial_02", "S2", "human_approaches_robot", "30.0", "60.0", "STOP", "SAFE_WARNING_STOP", "slow approach"),
    ("trial_03", "S3", "human_inside_stop_zone", "65.0", "83.0", "STOP", "STOP_HOLD", "human remains inside stop zone"),
    ("trial_04", "S4", "human_moves_away_release", "88.0", "118.0", "SAFE", "STOP_RELEASE_SAFE", "human moves away after stop"),
    ("trial_05", "S5", "dynamic_motion_near_boundary", "123.0", "168.0", "WARNING", "NO_RAPID_OSCILLATION", "movement near boundary"),
    ("trial_06", "S6", "hands_up_gesture_stop", "173.0", "193.0", "STOP", "SAFE_STOP", "hands raised"),
    ("trial_07", "S7", "t_pose_or_resume_gesture", "198.0", "218.0", "SAFE", "STOP_RELEASE_SAFE", "resume gesture only after proximity is safe"),
    ("trial_08", "S8", "partial_occlusion", "223.0", "253.0", "HOLD_OR_SAFE", "CONSERVATIVE_OR_FALLBACK", "partial body occlusion"),
    ("trial_09", "S9", "missing_pose_or_camera_dropout", "258.0", "278.0", "HOLD_OR_STALE", "STALE_OR_MISSING_DATA", "temporary pose/camera dropout if safe"),
    ("trial_10", "S10", "robot_motion_with_human_present", "283.0", "328.0", "STOP", "SAFE_WARNING_STOP", "robot moves while human starts safe then approaches"),
]


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def run_command(args: list[str], cwd: Path) -> str | None:
    try:
        completed = subprocess.run(
            args,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout.strip()


def collect_environment(cwd: Path) -> dict:
    env = {
        "python": sys.version.replace("\n", " "),
        "platform": platform.platform(),
        "processor": platform.processor(),
        "machine": platform.machine(),
        "cpu_count": os.cpu_count(),
        "cwd": str(cwd),
    }
    git_commit = run_command(["git", "rev-parse", "HEAD"], cwd)
    git_status = run_command(["git", "status", "--short"], cwd)
    if git_commit:
        env["git_commit"] = git_commit
    if git_status is not None:
        env["git_status_short"] = git_status
    return env


def existing_run_numbers(root: Path) -> list[int]:
    numbers = []
    if not root.exists():
        return numbers
    for child in root.iterdir():
        if not child.is_dir():
            continue
        match = RUN_RE.match(child.name)
        if match:
            numbers.append(int(match.group(1)))
    return sorted(numbers)


def next_run_id(root: Path) -> str:
    numbers = existing_run_numbers(root)
    next_number = numbers[-1] + 1 if numbers else 1
    return f"run_{next_number:02d}"


def write_header_only_csv(path: Path, headers: list[str]) -> None:
    if path.exists():
        raise FileExistsError(f"Refusing to overwrite existing log file: {path}")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)


def default_config_rows(run_id: str) -> list[dict[str, str]]:
    rows = []
    for trial_id, scenario_id, scenario_name, start, end, expected_state, transition, notes in DEFAULT_SCENARIOS:
        rows.append(
            {
                "run_id": run_id,
                "trial_id": trial_id,
                "scenario_id": scenario_id,
                "scenario_name": scenario_name,
                "start_time": start,
                "end_time": end,
                "expected_state": expected_state,
                "expected_transition": transition,
                "notes": notes,
            }
        )
    return rows


def copy_or_create_scenario_config(source: Path | None, destination: Path, run_id: str) -> str:
    fieldnames = [
        "run_id",
        "trial_id",
        "scenario_id",
        "scenario_name",
        "start_time",
        "end_time",
        "expected_state",
        "expected_transition",
        "notes",
    ]
    if source and source.exists():
        with source.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            source_fields = reader.fieldnames or fieldnames
            rows = [dict(row) for row in reader]
        if "run_id" not in source_fields:
            source_fields = ["run_id"] + list(source_fields)
        for row in rows:
            row["run_id"] = run_id
        out_fields = list(dict.fromkeys(source_fields + fieldnames))
        config_source = str(source)
    else:
        rows = default_config_rows(run_id)
        out_fields = fieldnames
        config_source = "built-in default scenarios"

    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=out_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return config_source


def write_notes_template(path: Path, run_id: str, started_at: str, operator: str | None, initial_note: str | None) -> None:
    lines = [
        f"# Repeated Trial Notes: {run_id}",
        "",
        f"- Started UTC: {started_at}",
        f"- Operator: {operator or ''}",
        "- External physical timing used: no",
        "- Robot program / trajectory:",
        "- Camera setup:",
        "- Calibration files:",
        "- Unity scene / build:",
        "- MQTT/UDP configuration:",
        "",
        "## Run-Level Notes",
        "",
        initial_note or "",
        "",
        "## Trial Notes",
        "",
        "| trial_id | scenario_id | repetition | notes | pass/fail/inconclusive |",
        "| --- | --- | --- | --- | --- |",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def create_run(args: argparse.Namespace) -> Path:
    root = Path(args.root)
    root.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id or next_run_id(root)
    run_dir = root / run_id
    if run_dir.exists():
        raise FileExistsError(f"Run folder already exists; refusing to overwrite: {run_dir}")
    run_dir.mkdir(parents=False)

    started_at = utc_now_iso()
    scenario_source = copy_or_create_scenario_config(
        Path(args.scenario_config) if args.scenario_config else None,
        run_dir / "scenario_config.csv",
        run_id,
    )
    write_notes_template(run_dir / "notes.md", run_id, started_at, args.operator, args.note)
    for filename, headers in LOG_HEADERS.items():
        write_header_only_csv(run_dir / filename, headers)

    metadata = {
        "schema_version": 1,
        "run_id": run_id,
        "started_at_utc": started_at,
        "started_unix": time.time(),
        "ended_at_utc": None,
        "ended_unix": None,
        "scenario_config_source": scenario_source,
        "operator": args.operator,
        "initial_note": args.note,
        "measurement_boundary": "software-stage runtime and command-generation logging; physical stop-time requires external timing",
        "expected_files": sorted(LOG_HEADERS.keys()) + ["scenario_config.csv", "notes.md"],
        "environment": collect_environment(Path.cwd()),
    }
    (run_dir / "run_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return run_dir


def append_note(run_dir: Path, note: str) -> None:
    notes_path = run_dir / "notes.md"
    if not notes_path.exists():
        notes_path.write_text(f"# Repeated Trial Notes: {run_dir.name}\n\n", encoding="utf-8")
    timestamp = utc_now_iso()
    with notes_path.open("a", encoding="utf-8") as handle:
        handle.write(f"\n## Note {timestamp}\n\n{note}\n")


def update_metadata(run_dir: Path, updates: dict) -> None:
    metadata_path = run_dir / "run_metadata.json"
    metadata = {}
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata.update(updates)
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def close_run(args: argparse.Namespace) -> Path:
    root = Path(args.root)
    if not args.close_run:
        raise ValueError("--close-run requires a run ID")
    run_dir = root / args.close_run
    if not run_dir.exists():
        raise FileNotFoundError(f"Run folder not found: {run_dir}")
    ended_at = utc_now_iso()
    update_metadata(run_dir, {"ended_at_utc": ended_at, "ended_unix": time.time()})
    if args.note:
        append_note(run_dir, args.note)
    return run_dir


def add_note(args: argparse.Namespace) -> Path:
    root = Path(args.root)
    if not args.append_note:
        raise ValueError("--append-note requires a run ID")
    if not args.note:
        raise ValueError("--note is required with --append-note")
    run_dir = root / args.append_note
    if not run_dir.exists():
        raise FileNotFoundError(f"Run folder not found: {run_dir}")
    append_note(run_dir, args.note)
    return run_dir


def list_runs(root: Path) -> None:
    if not root.exists():
        print(f"No run root exists: {root}")
        return
    for number in existing_run_numbers(root):
        run_id = f"run_{number:02d}"
        metadata_path = root / run_id / "run_metadata.json"
        status = ""
        if metadata_path.exists():
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                status = f" started={metadata.get('started_at_utc')} ended={metadata.get('ended_at_utc')}"
            except json.JSONDecodeError:
                status = " metadata_unreadable"
        print(f"{run_id}{status}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare repeated-trial run folders without overwriting prior logs.")
    parser.add_argument("--root", default="data/repeated_trials", help="Repeated-trial data root.")
    parser.add_argument("--scenario-config", help="Scenario config CSV to copy into a new run.")
    parser.add_argument("--new-run", action="store_true", help="Create the next run_XX folder.")
    parser.add_argument("--run-id", help="Optional explicit run ID for --new-run, for example run_07.")
    parser.add_argument("--operator", help="Operator name or initials for metadata/notes.")
    parser.add_argument("--note", help="Initial, closing, or appended note text.")
    parser.add_argument("--close-run", metavar="RUN_ID", help="Mark a run as ended and optionally append --note.")
    parser.add_argument("--append-note", metavar="RUN_ID", help="Append --note to an existing run notes.md.")
    parser.add_argument("--list-runs", action="store_true", help="List existing run folders.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.new_run:
            run_dir = create_run(args)
            print(run_dir)
            return 0
        if args.close_run:
            run_dir = close_run(args)
            print(f"closed {run_dir}")
            return 0
        if args.append_note:
            run_dir = add_note(args)
            print(f"updated {run_dir / 'notes.md'}")
            return 0
        if args.list_runs:
            list_runs(Path(args.root))
            return 0
        raise ValueError("Choose one action: --new-run, --close-run, --append-note, or --list-runs.")
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
