import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from pose3d_to_unity import CAM1_LIVE, CAM2_LIVE, FFMPEG_EXE, FFMPEG_TRANSPORT, LIVE_IMAGE_SIZE, CamReader


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "stereo_calibration_charuco_live.npz"
ARUCO_DICT = cv2.aruco.DICT_4X4_50
OPENCV_TO_UNITY = np.diag([1.0, -1.0, 1.0])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect stereo ArUco anchors and compute real-camera to Unity-scene alignment."
    )
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--cam1", default=CAM1_LIVE)
    parser.add_argument("--cam2", default=CAM2_LIVE)
    parser.add_argument("--cam1-image", default=None, help="Use an existing cam1 image instead of RTSP capture.")
    parser.add_argument("--cam2-image", default=None, help="Use an existing cam2 image instead of RTSP capture.")
    parser.add_argument("--output", default="aruco_unity_alignment.json")
    parser.add_argument("--debug-dir", default="aruco_alignment_debug")
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument("--live-width", type=int, default=LIVE_IMAGE_SIZE[0])
    parser.add_argument("--live-height", type=int, default=LIVE_IMAGE_SIZE[1])
    parser.add_argument("--transport", choices=["tcp", "udp"], default=FFMPEG_TRANSPORT)
    parser.add_argument("--ffmpeg-exe", default=FFMPEG_EXE)
    parser.add_argument("--opencv-capture", action="store_true")
    parser.add_argument(
        "--marker-ids",
        default="0,1,2,3",
        help="Comma-separated ArUco IDs to use for alignment.",
    )
    parser.add_argument(
        "--unity-markers-json",
        default=None,
        help="JSON object mapping ArUco ID to Unity world position, e.g. {\"0\":[0,0,0],...}.",
    )
    return parser.parse_args()


def load_calibration(path):
    data = np.load(path)
    required = ["K1", "dist1", "K2", "dist2", "R", "T", "image_width", "image_height"]
    missing = [key for key in required if key not in data.files]
    if missing:
        raise RuntimeError(f"Calibration file is missing keys: {missing}")

    return {
        "K1": data["K1"].astype(np.float64),
        "dist1": data["dist1"].astype(np.float64),
        "K2": data["K2"].astype(np.float64),
        "dist2": data["dist2"].astype(np.float64),
        "R": data["R"].astype(np.float64),
        "T": data["T"].astype(np.float64).reshape(3, 1),
        "image_size": (int(data["image_width"]), int(data["image_height"])),
    }


def calibration_for_frame_size(calibration, frame_size):
    if frame_size == calibration["image_size"]:
        return calibration

    calib_w, calib_h = calibration["image_size"]
    frame_w, frame_h = frame_size
    scale_x = frame_w / float(calib_w)
    scale_y = frame_h / float(calib_h)

    scaled = dict(calibration)
    scaled["K1"] = calibration["K1"].copy()
    scaled["K2"] = calibration["K2"].copy()
    scaled["K1"][0, :] *= scale_x
    scaled["K1"][1, :] *= scale_y
    scaled["K2"][0, :] *= scale_x
    scaled["K2"][1, :] *= scale_y
    scaled["image_size"] = frame_size
    return scaled


def capture_pair(args):
    if args.cam1_image and args.cam2_image:
        frame1 = cv2.imread(args.cam1_image)
        frame2 = cv2.imread(args.cam2_image)
        if frame1 is None:
            raise RuntimeError(f"Could not read cam1 image: {args.cam1_image}")
        if frame2 is None:
            raise RuntimeError(f"Could not read cam2 image: {args.cam2_image}")
        return frame1, frame2

    use_ffmpeg = not args.opencv_capture
    frame_size = (args.live_width, args.live_height)
    cam1 = CamReader(
        args.cam1,
        "Align Cam1",
        use_ffmpeg=use_ffmpeg,
        ffmpeg_exe=args.ffmpeg_exe,
        transport=args.transport,
        frame_size=frame_size,
    )
    time.sleep(0.5)
    cam2 = CamReader(
        args.cam2,
        "Align Cam2",
        use_ffmpeg=use_ffmpeg,
        ffmpeg_exe=args.ffmpeg_exe,
        transport=args.transport,
        frame_size=frame_size,
    )

    try:
        deadline = time.time() + max(args.settle_sec, 0.1)
        last1 = None
        last2 = None
        while time.time() < deadline:
            ok1, frame1 = cam1.read()
            ok2, frame2 = cam2.read()
            if ok1 and frame1 is not None:
                last1 = frame1
            if ok2 and frame2 is not None:
                last2 = frame2
            time.sleep(0.02)

        if last1 is None or last2 is None:
            raise RuntimeError("Could not capture frames from both RTSP cameras.")
        return last1, last2
    finally:
        cam1.release()
        cam2.release()


def build_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 4.0
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def detect_markers(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    markers = {}
    if ids is None:
        return markers, corners, ids, rejected

    for index, marker_id in enumerate(ids.flatten()):
        markers[int(marker_id)] = corners[index].reshape(4, 2).astype(np.float64)
    return markers, corners, ids, rejected


def triangulate_points(points1, points2, calibration):
    pts1 = np.asarray(points1, dtype=np.float64).reshape(-1, 1, 2)
    pts2 = np.asarray(points2, dtype=np.float64).reshape(-1, 1, 2)

    u1 = cv2.undistortPoints(pts1, calibration["K1"], calibration["dist1"])
    u2 = cv2.undistortPoints(pts2, calibration["K2"], calibration["dist2"])

    p1 = np.hstack((np.eye(3), np.zeros((3, 1)))).astype(np.float64)
    p2 = np.hstack((calibration["R"], calibration["T"])).astype(np.float64)
    points_4d = cv2.triangulatePoints(p1, p2, u1.reshape(-1, 2).T, u2.reshape(-1, 2).T)
    points_3d = (points_4d[:3] / points_4d[3]).T
    return points_3d


def marker_pose_from_corners(corners_3d):
    center = np.mean(corners_3d, axis=0)
    right = corners_3d[1] - corners_3d[0]
    down = corners_3d[3] - corners_3d[0]
    normal = np.cross(right, down)
    normal_norm = np.linalg.norm(normal)
    if normal_norm > 1e-9:
        normal = normal / normal_norm
    return center, normal


def fit_similarity(source_points, target_points):
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source_points and target_points must both be Nx3")
    if len(source) < 3:
        raise ValueError("At least 3 point pairs are required for similarity alignment.")

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean

    covariance = (target_centered.T @ source_centered) / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0.0:
        correction[-1, -1] = -1.0

    rotation = u @ correction @ vt
    variance = np.mean(np.sum(source_centered * source_centered, axis=1))
    scale = float(np.sum(singular_values * np.diag(correction)) / max(variance, 1e-12))
    translation = target_mean - scale * (rotation @ source_mean)
    return scale, rotation, translation


def matrix_columns(rotation):
    return {
        "right": rotation[:, 0].tolist(),
        "up": rotation[:, 1].tolist(),
        "forward": rotation[:, 2].tolist(),
    }


def transform_point(point, scale, rotation, translation):
    return scale * (rotation @ point) + translation


def transform_rotation(rotation_local, rotation_alignment):
    return rotation_alignment @ rotation_local


def parse_marker_ids(value):
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def parse_unity_markers(value):
    if value is None:
        return None
    path = Path(value)
    text = path.read_text() if path.exists() else value
    raw = json.loads(text)
    return {int(key): np.asarray(pos, dtype=np.float64) for key, pos in raw.items()}


def save_debug_images(debug_dir, frame1, frame2, corners1, ids1, corners2, ids2):
    debug_dir.mkdir(parents=True, exist_ok=True)
    vis1 = frame1.copy()
    vis2 = frame2.copy()
    if ids1 is not None:
        cv2.aruco.drawDetectedMarkers(vis1, corners1, ids1)
    if ids2 is not None:
        cv2.aruco.drawDetectedMarkers(vis2, corners2, ids2)
    cv2.imwrite(str(debug_dir / "cam1_detected.png"), vis1)
    cv2.imwrite(str(debug_dir / "cam2_detected.png"), vis2)


def main():
    args = parse_args()
    marker_ids = parse_marker_ids(args.marker_ids)
    unity_markers = parse_unity_markers(args.unity_markers_json)

    frame1, frame2 = capture_pair(args)
    if frame1.shape[:2] != frame2.shape[:2]:
        raise RuntimeError(f"Camera frame sizes differ: {frame1.shape[:2]} vs {frame2.shape[:2]}")

    calibration = load_calibration(args.calibration)
    calibration = calibration_for_frame_size(calibration, (frame1.shape[1], frame1.shape[0]))

    detector = build_detector()
    markers1, corners1, ids1, _ = detect_markers(frame1, detector)
    markers2, corners2, ids2, _ = detect_markers(frame2, detector)
    save_debug_images(Path(args.debug_dir), frame1, frame2, corners1, ids1, corners2, ids2)

    common_ids = sorted(set(markers1) & set(markers2) & set(marker_ids))
    missing = sorted(set(marker_ids) - set(common_ids))
    if missing:
        print(f"Missing requested marker IDs in both cameras: {missing}")
    if len(common_ids) < 3:
        raise RuntimeError(f"Need at least 3 common markers, got {common_ids}")

    real_markers = {}
    source_points = []
    target_points = []
    for marker_id in common_ids:
        corners_3d_cv = triangulate_points(markers1[marker_id], markers2[marker_id], calibration)
        center_cv, normal_cv = marker_pose_from_corners(corners_3d_cv)
        center_unity_camera = OPENCV_TO_UNITY @ center_cv
        normal_unity_camera = OPENCV_TO_UNITY @ normal_cv
        real_markers[marker_id] = {
            "corners_cam1_m": corners_3d_cv.tolist(),
            "center_cam1_m": center_cv.tolist(),
            "center_unity_camera_m": center_unity_camera.tolist(),
            "normal_unity_camera": normal_unity_camera.tolist(),
            "cam1_pixels": markers1[marker_id].tolist(),
            "cam2_pixels": markers2[marker_id].tolist(),
        }
        if unity_markers is not None and marker_id in unity_markers:
            source_points.append(center_unity_camera)
            target_points.append(unity_markers[marker_id])

    alignment = None
    if unity_markers is not None:
        if len(source_points) < 3:
            raise RuntimeError("Need at least 3 detected markers with Unity positions to compute alignment.")

        scale, rotation, translation = fit_similarity(np.asarray(source_points), np.asarray(target_points))
        residuals = []
        fitted_markers = {}
        for marker_id in common_ids:
            source = np.asarray(real_markers[marker_id]["center_unity_camera_m"], dtype=np.float64)
            fitted = transform_point(source, scale, rotation, translation)
            target = unity_markers.get(marker_id)
            error = None if target is None else float(np.linalg.norm(fitted - target))
            residuals.append(error if error is not None else 0.0)
            fitted_markers[marker_id] = {
                "position": fitted.tolist(),
                "target_position": None if target is None else target.tolist(),
                "error_to_current_unity_m": error,
            }

        c1_position = translation
        c1_rotation = rotation

        cam2_center_cv = -(calibration["R"].T @ calibration["T"]).reshape(3)
        cam2_center_u = OPENCV_TO_UNITY @ cam2_center_cv
        cam2_position = transform_point(cam2_center_u, scale, rotation, translation)
        cam2_to_cam1_u_rotation = OPENCV_TO_UNITY @ calibration["R"].T @ OPENCV_TO_UNITY
        cam2_rotation = transform_rotation(cam2_to_cam1_u_rotation, rotation)

        alignment = {
            "scale_real_m_to_unity": scale,
            "rotation_real_unity_camera_to_unity_world": rotation.tolist(),
            "translation_real_unity_camera_to_unity_world": translation.tolist(),
            "mean_marker_fit_error_m": float(np.mean(residuals)),
            "max_marker_fit_error_m": float(np.max(residuals)),
            "camera_1": {
                "position": c1_position.tolist(),
                "rotation_matrix_columns": matrix_columns(c1_rotation),
                "rotation_matrix": c1_rotation.tolist(),
            },
            "camera_2": {
                "position": cam2_position.tolist(),
                "rotation_matrix_columns": matrix_columns(cam2_rotation),
                "rotation_matrix": cam2_rotation.tolist(),
            },
            "fitted_markers": fitted_markers,
        }

    output = {
        "timestamp": time.time(),
        "calibration": str(args.calibration),
        "frame_size": [int(frame1.shape[1]), int(frame1.shape[0])],
        "detected_cam1_ids": sorted(markers1.keys()),
        "detected_cam2_ids": sorted(markers2.keys()),
        "common_alignment_ids": common_ids,
        "markers": {str(key): value for key, value in real_markers.items()},
        "alignment": alignment,
        "debug_images": {
            "cam1": str(Path(args.debug_dir) / "cam1_detected.png"),
            "cam2": str(Path(args.debug_dir) / "cam2_detected.png"),
        },
    }

    Path(args.output).write_text(json.dumps(output, indent=2))
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
