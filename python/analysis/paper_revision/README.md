# Paper revision analyses

Scripts behind Sections 4.7 and 4.8 and Fig. 13 of the manuscript
"Source-aware decision records for evaluating proximity monitoring in a
human-robot digital twin".

| Script | Output | Manuscript |
|---|---|---|
| `distance_uncertainty.py` | `distance_uncertainty_results.json` | Section 4.7, Eq. (8), Fig. 13a |
| `message_loss_simulation.py` | `message_loss_results.json` | Section 4.8, Fig. 13b–c |
| `plot_fig13.py` | Fig. 13 (PNG, 300 dpi) | Fig. 13 |

Run from the repository root (requires numpy, opencv-python, matplotlib):

```
python python/analysis/paper_revision/distance_uncertainty.py python/yolo_pose_test/stereo_calibration_charuco_live.npz python/analysis/paper_revision/distance_uncertainty_results.json
python python/analysis/paper_revision/message_loss_simulation.py python/analysis/paper_revision/message_loss_results.json
python python/analysis/paper_revision/plot_fig13.py python/analysis/paper_revision/distance_uncertainty_results.json python/analysis/paper_revision/message_loss_results.json fig13.png
```

Both analyses use a fixed random seed (0), so the results are reproducible.
They are simulations: the first propagates random keypoint noise through the
live stereo calibration, the second applies the implemented resend (0.25 s)
and freshness (0.5 s) rules to simulated message loss.
