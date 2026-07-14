"""Summarize and optionally join Python/Unity runtime metric CSV files."""

import argparse
import csv
import json
import math
from pathlib import Path


def percentile(values, p):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * p / 100.0
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (position - low)


def load_csv(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def numeric_summary(rows):
    columns = {}
    for row in rows:
        for key, raw in row.items():
            if raw is None or raw == "":
                continue
            try:
                value = float(raw)
            except ValueError:
                continue
            if math.isfinite(value):
                columns.setdefault(key, []).append(value)
    result = {}
    for key, values in columns.items():
        result[key] = {
            "count": len(values),
            "mean": sum(values) / len(values),
            "min": min(values),
            "p50": percentile(values, 50),
            "p90": percentile(values, 90),
            "p95": percentile(values, 95),
            "p99": percentile(values, 99),
            "max": max(values),
        }
    return result


def join_by_frame(python_rows, unity_rows):
    unity_by_frame = {row.get("frame"): row for row in unity_rows if row.get("frame")}
    joined = []
    for python_row in python_rows:
        frame = python_row.get("frame")
        unity_row = unity_by_frame.get(frame)
        if unity_row is None:
            continue
        combined = {"frame": frame}
        combined.update({"python_" + key: value for key, value in python_row.items() if key != "frame"})
        combined.update({"unity_" + key: value for key, value in unity_row.items() if key != "frame"})
        joined.append(combined)
    return joined


def write_csv(path, rows):
    if not rows:
        return
    fields = list(rows[0].keys())
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--python-csv", required=True)
    parser.add_argument("--unity-csv")
    parser.add_argument("--safety-csv", help="Unity unity_safety.csv file")
    parser.add_argument("--mqtt-csv", help="Unity unity_mqtt.csv file")
    parser.add_argument("--output-dir", default="metrics_report")
    return parser.parse_args()


def main():
    args = parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    python_rows = load_csv(args.python_csv)
    report = {
        "python_frames": len(python_rows),
        "python": numeric_summary(python_rows),
    }
    if args.unity_csv:
        unity_rows = load_csv(args.unity_csv)
        joined = join_by_frame(python_rows, unity_rows)
        report.update({
            "unity_frames": len(unity_rows),
            "joined_frames": len(joined),
            "unity": numeric_summary(unity_rows),
            "joined": numeric_summary(joined),
        })
        write_csv(output / "joined_frames.csv", joined)
    if args.safety_csv:
        safety_rows = load_csv(args.safety_csv)
        joined_safety = join_by_frame(python_rows, safety_rows)
        report.update({
            "unity_safety_events": len(safety_rows),
            "joined_safety_events": len(joined_safety),
            "unity_safety": numeric_summary(safety_rows),
            "joined_safety": numeric_summary(joined_safety),
        })
        write_csv(output / "joined_safety.csv", joined_safety)
    if args.mqtt_csv:
        mqtt_rows = load_csv(args.mqtt_csv)
        report.update({
            "unity_mqtt_events": len(mqtt_rows),
            "unity_mqtt": numeric_summary(mqtt_rows),
        })
    (output / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(output / "summary.json")


if __name__ == "__main__":
    main()
