# ER9Pro FK Calibration 2026-06-27

Source validation log:

`%USERPROFILE%/AppData/LocalLow/GPV/Smart Lab/metrics/fk_safety_20260627_212333/fk_safety.csv`

The calibration fitted the Unity planar FK parameters against rows where robot telemetry was fresh and the Unity TCP marker (`Joint 6`) was assigned.

## Fitted Model

- Model: `ER9Pro_calibrated_5link_planar_20260627`
- Base offset: `(-0.095718, -0.016238, -0.233129)` m
- Horizontal axis: `(0.315535, 0.944675, -0.089592)`
- Vertical axis: `(0.637409, -0.770331, -0.017308)`

## Validation Error

Filtered samples: 514

| Metric | Before calibration | After calibration |
| --- | ---: | ---: |
| Mean TCP error | 0.6561 m | 0.0123 m |
| RMSE TCP error | 0.6622 m | 0.0205 m |
| Median TCP error | 0.6239 m | 0.0055 m |
| P95 TCP error | 0.8010 m | 0.0572 m |
| Max TCP error | 0.8035 m | 0.0584 m |

The calibrated model is an empirical Unity-frame fit. Re-run calibration if the robot scale, TCP marker, joint zero convention, or robot hierarchy changes.
