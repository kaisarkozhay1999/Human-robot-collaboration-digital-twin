"""Monte Carlo propagation of 2D keypoint noise to the stereo human-robot distance.

Uses the live (1280 x 720) stereo calibration of the workcell
(stereo_calibration_charuco_live.npz from the project repository).

Model
-----
* Camera 1 defines the reconstruction frame: P1 = K1 [I | 0], P2 = K2 [R | T].
* Workspace: points within +/-1 m (per axis) of the point where the two optical
  axes pass closest to each other, kept only if they project inside both
  1280 x 720 images and lie in front of both cameras.
* A robot reference point r is drawn from the workspace and a human joint h is
  placed at a true distance d0 from r in a random direction (also visible).
* Independent isotropic Gaussian noise of standard deviation sigma (pixels) is
  added to the image coordinates of both points in both cameras; both points
  are triangulated and the estimated distance is compared with d0.
* Lens distortion is neglected in the propagation (noise is applied to
  undistorted image coordinates).
"""
import json
import sys

import cv2
import numpy as np

calib_path = sys.argv[1] if len(sys.argv) > 1 else "stereo_calibration_charuco_live.npz"
calib_name = calib_path.replace(chr(92), '/').split('/')[-1]
out_json = sys.argv[2] if len(sys.argv) > 2 else "distance_uncertainty_results.json"

c = np.load(calib_path, allow_pickle=False)
K1, K2, R, T = c["K1"], c["K2"], c["R"], c["T"].reshape(3, 1)
W, H = int(c["image_width"]), int(c["image_height"])
P1 = K1 @ np.hstack([np.eye(3), np.zeros((3, 1))])
P2 = K2 @ np.hstack([R, T])

# convergence point of the optical axes (camera-1 frame)
C1, d1 = np.zeros(3), np.array([0.0, 0.0, 1.0])
C2 = (-R.T @ T).ravel()
d2 = R.T @ np.array([0.0, 0.0, 1.0])
w0 = C1 - C2
a, b, cc, d, e = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ w0, d2 @ w0
s = (b * e - cc * d) / (a * cc - b * b)
t = (a * e - b * d) / (a * cc - b * b)
conv = 0.5 * ((C1 + s * d1) + (C2 + t * d2))
gap = np.linalg.norm((C1 + s * d1) - (C2 + t * d2))

rng = np.random.default_rng(0)


def project(P, X):
    x = P @ np.vstack([X.T, np.ones(len(X))])
    return (x[:2] / x[2]).T, x[2]


def visible(X):
    u1, z1 = project(P1, X)
    u2, z2 = project(P2, X)
    ok = (z1 > 0.2) & (z2 > 0.2)
    for u in (u1, u2):
        ok &= (u[:, 0] >= 0) & (u[:, 0] < W) & (u[:, 1] >= 0) & (u[:, 1] < H)
    return ok


def sample_pairs(n, d0):
    rs, hs = [], []
    while sum(len(x) for x in rs) < n:
        r = conv + rng.uniform(-1, 1, size=(4 * n, 3))
        v = rng.normal(size=(4 * n, 3))
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        h = r + d0 * v
        ok = visible(r) & visible(h)
        rs.append(r[ok]); hs.append(h[ok])
    return np.vstack(rs)[:n], np.vstack(hs)[:n]


def triangulate(u1, u2):
    X = cv2.triangulatePoints(P1, P2, u1.T.astype(np.float64), u2.T.astype(np.float64))
    return (X[:3] / X[3]).T


def noisy(X, sigma):
    u1, _ = project(P1, X)
    u2, _ = project(P2, X)
    u1 = u1 + rng.normal(0, sigma, u1.shape)
    u2 = u2 + rng.normal(0, sigma, u2.shape)
    return triangulate(u1, u2)


N = 20000
results = {
    "calibration": calib_name,
    "image_size": [W, H],
    "baseline_m": float(np.linalg.norm(T)),
    "convergence_point_cam1_m": conv.round(3).tolist(),
    "axis_gap_m": float(gap),
    "convergence_depth_cam1_m": float(np.linalg.norm(conv)),
    "convergence_depth_cam2_m": float(np.linalg.norm(conv - C2)),
    "n_pairs": N,
    "sweep": [],
}
r, h = sample_pairs(N, 0.75)
results["depth_cam1_median_m"] = float(np.median(np.linalg.norm(r, axis=1)))
for sigma in (0.5, 1, 2, 4, 8, 16):
    rh, hh = noisy(r, sigma), noisy(h, sigma)
    pt = np.linalg.norm(rh - r, axis=1)
    err = np.linalg.norm(hh - rh, axis=1) - 0.75
    results["sweep"].append({
        "sigma_px": sigma,
        "point_error_rms_m": float(np.sqrt(np.mean(pt ** 2))),
        "point_error_p95_m": float(np.percentile(pt, 95)),
        "distance_error_sd_m": float(np.std(err)),
        "distance_error_abs_p95_m": float(np.percentile(np.abs(err), 95)),
        "share_abs_error_gt_0p075": float(np.mean(np.abs(err) > 0.075)),
        "share_abs_error_gt_0p15": float(np.mean(np.abs(err) > 0.15)),
    })

# probability that a person at the release distance (0.90 m) is estimated at or
# below the stop distance (0.75 m), and vice versa
r9, h9 = sample_pairs(N, 0.90)
r6, h6 = sample_pairs(N, 0.60)
for row in results["sweep"]:
    sg = row["sigma_px"]
    d9 = np.linalg.norm(noisy(h9, sg) - noisy(r9, sg), axis=1)
    d6 = np.linalg.norm(noisy(h6, sg) - noisy(r6, sg), axis=1)
    row["p_est_le_0p75_given_true_0p90"] = float(np.mean(d9 <= 0.75))
    row["p_est_ge_0p90_given_true_0p60"] = float(np.mean(d6 >= 0.90))

json.dump(results, open(out_json, "w"), indent=2)
print(json.dumps(results, indent=2))
