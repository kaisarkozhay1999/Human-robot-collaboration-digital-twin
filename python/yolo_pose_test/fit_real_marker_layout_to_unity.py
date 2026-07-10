import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from solve_unity_camera_poses_from_aruco import (
    ARUCO_DICT,
    CV_FROM_UNITY,
    CAM1_HIGH,
    CAM2_HIGH,
    FFMPEG_EXE,
    build_detector,
    capture_pair,
    detect_markers,
    load_calibration,
    save_debug,
    scale_camera_matrix,
)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "stereo_calibration_charuco_refined.npz"
DEFAULT_UNITY_MARKERS = SCRIPT_DIR / "unity_marker_centers_current.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Fit real ArUco marker layout to Unity markers and solve static camera transforms."
    )
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--unity-marker-centers", default=str(DEFAULT_UNITY_MARKERS))
    parser.add_argument("--output", default=str(SCRIPT_DIR / "real_marker_layout_fit.json"))
    parser.add_argument("--debug-dir", default=str(SCRIPT_DIR / "real_marker_layout_debug"))
    parser.add_argument("--cam1", default=CAM1_HIGH)
    parser.add_argument("--cam2", default=CAM2_HIGH)
    parser.add_argument("--cam1-image", default=None)
    parser.add_argument("--cam2-image", default=None)
    parser.add_argument("--high-width", type=int, default=2592)
    parser.add_argument("--high-height", type=int, default=1944)
    parser.add_argument("--live-width", type=int, default=1280)
    parser.add_argument("--live-height", type=int, default=720)
    parser.add_argument("--settle-sec", type=float, default=6.0)
    parser.add_argument("--transport", choices=["tcp", "udp"], default="tcp")
    parser.add_argument("--ffmpeg-exe", default=FFMPEG_EXE)
    parser.add_argument("--opencv-capture", action="store_true")
    parser.add_argument("--min-common-markers", type=int, default=1)
    return parser.parse_args()


def load_unity_marker_centers(path):
    raw = json.loads(Path(path).read_text())
    return {int(marker_id): np.asarray(position, dtype=np.float64) for marker_id, position in raw.items()}


def marker_object_points(side_length):
    half = side_length * 0.5
    return np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_marker_pose_unit(corners, camera_matrix, dist_coeffs):
    object_points = marker_object_points(1.0)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        np.asarray(corners, dtype=np.float64),
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            np.asarray(corners, dtype=np.float64),
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    if not ok:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    return {
        "t_unit": tvec.reshape(3).astype(np.float64),
        "rotation_marker_to_camera": rotation.astype(np.float64),
    }


def estimate_all_marker_poses(detected, camera_matrix, dist_coeffs):
    poses = {}
    for marker_id, corners in detected.items():
        pose = estimate_marker_pose_unit(corners, camera_matrix, dist_coeffs)
        if pose is not None:
            poses[marker_id] = pose
    return poses


def estimate_marker_side_m(cam1_poses, cam2_poses, calibration, min_common):
    common_ids = sorted(set(cam1_poses) & set(cam2_poses))
    if len(common_ids) < min_common:
        raise RuntimeError(f"Need at least {min_common} common marker(s) to estimate marker size, got {common_ids}")

    rt = calibration["R"].T
    offset = rt @ calibration["T"].reshape(3)
    a_terms = []
    b_terms = []
    for marker_id in common_ids:
        cam1_unit = cam1_poses[marker_id]["t_unit"]
        cam2_unit_in_cam1 = rt @ cam2_poses[marker_id]["t_unit"]
        a_terms.append(cam1_unit - cam2_unit_in_cam1)
        b_terms.append(offset)

    a = np.vstack(a_terms)
    b = np.vstack(b_terms)
    denom = float(np.sum(a * a))
    if denom <= 1e-12:
        raise RuntimeError("Could not estimate marker size: degenerate common-marker geometry")

    side = -float(np.sum(a * b)) / denom
    if side <= 0:
        raise RuntimeError(f"Estimated non-positive marker side length: {side}")
    return side, common_ids


def real_marker_centers_cam1(cam1_poses, cam2_poses, calibration, marker_side_m):
    rt = calibration["R"].T
    centers = {}
    sources = {}
    for marker_id, pose in cam1_poses.items():
        centers.setdefault(marker_id, []).append(marker_side_m * pose["t_unit"])
        sources.setdefault(marker_id, []).append("cam1")
    for marker_id, pose in cam2_poses.items():
        center_cam2 = marker_side_m * pose["t_unit"]
        center_cam1 = rt @ (center_cam2 - calibration["T"].reshape(3))
        centers.setdefault(marker_id, []).append(center_cam1)
        sources.setdefault(marker_id, []).append("cam2")

    averaged = {marker_id: np.mean(np.vstack(values), axis=0) for marker_id, values in centers.items()}
    return averaged, sources


def fit_similarity(source_points, target_points):
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = (target_centered.T @ source_centered) / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        correction[-1, -1] = -1
    rotation = u @ correction @ vt
    variance = np.mean(np.sum(source_centered * source_centered, axis=1))
    scale = float(np.sum(singular_values * np.diag(correction)) / max(variance, 1e-12))
    translation = target_mean - scale * (rotation @ source_mean)
    return scale, rotation, translation


def transform_point(point, scale, rotation, translation):
    return scale * (rotation @ point) + translation


def euler_from_basis(rotation):
    # Unity-compatible XYZ Euler via a temporary Rodrigues-free matrix decomposition is done in Unity.
    return rotation.tolist()


def main():
    args = parse_args()
    calibration = load_calibration(args.calibration)
    unity_centers = load_unity_marker_centers(args.unity_marker_centers)

    frame1, frame2 = capture_pair(args)
    frame_size = (frame1.shape[1], frame1.shape[0])
    k1 = scale_camera_matrix(calibration["K1"], calibration["image_size"], frame_size)
    k2 = scale_camera_matrix(calibration["K2"], calibration["image_size"], frame_size)

    detector = build_detector()
    markers1, corners1, ids1, _ = detect_markers(frame1, detector)
    markers2, corners2, ids2, _ = detect_markers(frame2, detector)
    debug_dir = Path(args.debug_dir)
    save_debug(debug_dir, frame1, corners1, ids1, "cam1")
    save_debug(debug_dir, frame2, corners2, ids2, "cam2")

    cam1_poses = estimate_all_marker_poses(markers1, k1, calibration["dist1"])
    cam2_poses = estimate_all_marker_poses(markers2, k2, calibration["dist2"])
    marker_side_m, common_ids = estimate_marker_side_m(
        cam1_poses,
        cam2_poses,
        calibration,
        args.min_common_markers,
    )
    real_centers_cv, marker_sources = real_marker_centers_cam1(
        cam1_poses,
        cam2_poses,
        calibration,
        marker_side_m,
    )

    fit_ids = sorted(set(real_centers_cv) & set(unity_centers))
    if len(fit_ids) < 3:
        raise RuntimeError(f"Need at least 3 markers to fit layout; got {fit_ids}")

    real_centers_unity_camera = {
        marker_id: CV_FROM_UNITY @ center for marker_id, center in real_centers_cv.items()
    }
    source = np.vstack([real_centers_unity_camera[marker_id] for marker_id in fit_ids])
    target = np.vstack([unity_centers[marker_id] for marker_id in fit_ids])
    scale, rotation, translation = fit_similarity(source, target)

    fitted_markers = {}
    residuals = []
    for marker_id in fit_ids:
        fitted = transform_point(real_centers_unity_camera[marker_id], scale, rotation, translation)
        error = float(np.linalg.norm(fitted - unity_centers[marker_id]))
        residuals.append(error)
        fitted_markers[str(marker_id)] = {
            "current_unity_position": unity_centers[marker_id].tolist(),
            "fitted_unity_position": fitted.tolist(),
            "move_delta": (fitted - unity_centers[marker_id]).tolist(),
            "fit_error_before_move": error,
            "sources": marker_sources.get(marker_id, []),
        }

    cam1_position = translation
    cam1_rotation = rotation
    cam2_center_cv = -(calibration["R"].T @ calibration["T"]).reshape(3)
    cam2_position = transform_point(CV_FROM_UNITY @ cam2_center_cv, scale, rotation, translation)
    cam2_relative = CV_FROM_UNITY @ calibration["R"].T @ CV_FROM_UNITY
    cam2_rotation = rotation @ cam2_relative

    output = {
        "timestamp": time.time(),
        "calibration": str(args.calibration),
        "detected_ids": {
            "cam1": sorted(markers1.keys()),
            "cam2": sorted(markers2.keys()),
            "common_for_marker_size": common_ids,
        },
        "marker_side_m_estimated": marker_side_m,
        "fit_marker_ids": fit_ids,
        "scale_real_m_to_unity": scale,
        "rotation_real_cam1_unity_to_unity_world": rotation.tolist(),
        "translation_real_cam1_unity_to_unity_world": translation.tolist(),
        "mean_marker_error_before_move": float(np.mean(residuals)),
        "max_marker_error_before_move": float(np.max(residuals)),
        "markers": fitted_markers,
        "cameras": {
            "cam1": {
                "position": cam1_position.tolist(),
                "rotation_matrix": euler_from_basis(cam1_rotation),
            },
            "cam2": {
                "position": cam2_position.tolist(),
                "rotation_matrix": euler_from_basis(cam2_rotation),
            },
        },
        "debug_images": {
            "cam1": str(debug_dir / "cam1_high_detected.png"),
            "cam2": str(debug_dir / "cam2_high_detected.png"),
        },
    }
    Path(args.output).write_text(json.dumps(output, indent=2))
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
