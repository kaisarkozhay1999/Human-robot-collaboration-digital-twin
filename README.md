# SmartLab Robot-Arm Digital Twin

## Laboratory Safety-Assistance Repeated Trials

This repository includes a repeated-trial validation workflow for the robot-arm digital twin. The workflow prepares protocol documents, run folders, log templates, column mapping, analysis tables, figures, and an honest Markdown report. It does not generate synthetic measurements and does not claim repeated trials were conducted unless real logs are present.

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
