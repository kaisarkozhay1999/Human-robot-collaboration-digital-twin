import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from pose3d_to_unity import CAM1_HIGH, CAM2_HIGH, FFMPEG_EXE, FFMPEG_TRANSPORT, CamReader


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "stereo_calibration_charuco_refined.npz"
DEFAULT_MARKER_CORNERS = SCRIPT_DIR / "unity_marker_corners_current.json"
ARUCO_DICT = cv2.aruco.DICT_4X4_50
CV_FROM_UNITY = np.diag([1.0, -1.0, 1.0])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Solve real camera poses in Unity world from high-resolution ArUco observations."
    )
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--unity-marker-corners", default=str(DEFAULT_MARKER_CORNERS))
    parser.add_argument("--output", default=str(SCRIPT_DIR / "unity_camera_pose_solution.json"))
    parser.add_argument("--debug-dir", default=str(SCRIPT_DIR / "aruco_camera_pose_debug"))
    parser.add_argument("--cam1", default=CAM1_HIGH)
    parser.add_argument("--cam2", default=CAM2_HIGH)
    parser.add_argument("--cam1-image", default=None)
    parser.add_argument("--cam2-image", default=None)
    parser.add_argument("--high-width", type=int, default=2592)
    parser.add_argument("--high-height", type=int, default=1944)
    parser.add_argument("--live-width", type=int, default=1280)
    parser.add_argument("--live-height", type=int, default=720)
    parser.add_argument("--settle-sec", type=float, default=5.0)
    parser.add_argument("--transport", choices=["tcp", "udp"], default=FFMPEG_TRANSPORT)
    parser.add_argument("--ffmpeg-exe", default=FFMPEG_EXE)
    parser.add_argument("--opencv-capture", action="store_true")
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


def scale_camera_matrix(k, source_size, target_size):
    if source_size == target_size:
        return k.copy()

    source_w, source_h = source_size
    target_w, target_h = target_size
    sx = target_w / float(source_w)
    sy = target_h / float(source_h)
    scaled = k.copy()
    scaled[0, :] *= sx
    scaled[1, :] *= sy
    return scaled


def load_marker_corners(path):
    raw = json.loads(Path(path).read_text())
    markers = {}
    for marker_id, corners in raw.items():
        unity = np.asarray(corners, dtype=np.float64).reshape(4, 3)
        cv_world = (CV_FROM_UNITY @ unity.T).T
        markers[int(marker_id)] = {
            "unity": unity,
            "cv_world": cv_world,
        }
    return markers


def capture_one(reader, settle_sec):
    deadline = time.time() + max(settle_sec, 0.1)
    latest = None
    while time.time() < deadline:
        ok, frame = reader.read()
        if ok and frame is not None:
            latest = frame
        time.sleep(0.02)
    return latest


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
    frame_size = (args.high_width, args.high_height)
    cam1 = CamReader(
        args.cam1,
        "PnP Cam1 HIGH",
        use_ffmpeg=use_ffmpeg,
        ffmpeg_exe=args.ffmpeg_exe,
        transport=args.transport,
        frame_size=frame_size,
    )
    time.sleep(0.5)
    cam2 = CamReader(
        args.cam2,
        "PnP Cam2 HIGH",
        use_ffmpeg=use_ffmpeg,
        ffmpeg_exe=args.ffmpeg_exe,
        transport=args.transport,
        frame_size=frame_size,
    )

    try:
        frame1 = capture_one(cam1, args.settle_sec)
        frame2 = capture_one(cam2, args.settle_sec)
        if frame1 is None or frame2 is None:
            raise RuntimeError("Could not capture high-resolution frames from both cameras.")
        return frame1, frame2
    finally:
        cam1.release()
        cam2.release()


def build_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.006
    params.maxMarkerPerimeterRate = 4.0
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def detect_markers(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    markers = {}
    if ids is not None:
        for index, marker_id in enumerate(ids.flatten()):
            markers[int(marker_id)] = corners[index].reshape(4, 2).astype(np.float64)
    return markers, corners, ids, rejected


def collect_pnp_points(detected_markers, unity_markers):
    object_points = []
    image_points = []
    used_ids = []

    for marker_id in sorted(set(detected_markers) & set(unity_markers)):
        object_points.extend(unity_markers[marker_id]["cv_world"])
        image_points.extend(detected_markers[marker_id])
        used_ids.append(marker_id)

    return (
        np.asarray(object_points, dtype=np.float64),
        np.asarray(image_points, dtype=np.float64),
        used_ids,
    )


def corner_permutations(corners):
    base = np.asarray(corners, dtype=np.float64).reshape(4, 3)
    orders = []
    for reverse in (False, True):
        order = [0, 1, 2, 3] if not reverse else [0, 3, 2, 1]
        for shift in range(4):
            shifted = order[shift:] + order[:shift]
            orders.append(base[shifted])
    return orders


def collect_pnp_points_with_permutation(detected_markers, unity_markers, marker_order_choices):
    object_points = []
    image_points = []
    used_ids = []

    for marker_id in sorted(set(detected_markers) & set(unity_markers)):
        choice = marker_order_choices.get(marker_id, 0)
        object_points.extend(corner_permutations(unity_markers[marker_id]["cv_world"])[choice])
        image_points.extend(detected_markers[marker_id])
        used_ids.append(marker_id)

    return (
        np.asarray(object_points, dtype=np.float64),
        np.asarray(image_points, dtype=np.float64),
        used_ids,
    )


def solve_pnp_and_error(object_points, image_points, camera_matrix, dist_coeffs):
    if len(object_points) < 4:
        return None

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points, axis=1)
    return rvec, tvec, errors


def choose_best_corner_orders(detected_markers, unity_markers, camera_matrix, dist_coeffs):
    marker_ids = sorted(set(detected_markers) & set(unity_markers))
    if not marker_ids:
        return {}, None, None, None, []

    best = None
    choices = {}

    def search(index):
        nonlocal best
        if index == len(marker_ids):
            object_points, image_points, used_ids = collect_pnp_points_with_permutation(
                detected_markers,
                unity_markers,
                choices,
            )
            solved = solve_pnp_and_error(object_points, image_points, camera_matrix, dist_coeffs)
            if solved is None:
                return
            rvec, tvec, errors = solved
            score = float(np.mean(errors))
            if best is None or score < best[0]:
                best = (score, dict(choices), rvec, tvec, errors, used_ids)
            return

        marker_id = marker_ids[index]
        for choice in range(8):
            choices[marker_id] = choice
            search(index + 1)
        choices.pop(marker_id, None)

    search(0)
    if best is None:
        return {}, None, None, None, marker_ids

    _, best_choices, rvec, tvec, errors, used_ids = best
    return best_choices, rvec, tvec, errors, used_ids


def solve_camera_pose(name, detected_markers, unity_markers, camera_matrix, dist_coeffs):
    object_points, image_points, used_ids = collect_pnp_points(detected_markers, unity_markers)
    if len(object_points) < 4:
        return {
            "name": name,
            "success": False,
            "reason": f"Need at least 4 marker corners; got {len(object_points)}.",
            "used_marker_ids": used_ids,
        }

    marker_order_choices, rvec, tvec, errors, used_ids = choose_best_corner_orders(
        detected_markers,
        unity_markers,
        camera_matrix,
        dist_coeffs,
    )
    if rvec is None:
        return {
            "name": name,
            "success": False,
            "reason": "cv2.solvePnP returned false for all marker corner permutations.",
            "used_marker_ids": used_ids,
        }

    rotation_world_cv_to_camera_cv, _ = cv2.Rodrigues(rvec)
    camera_center_cv_world = -(rotation_world_cv_to_camera_cv.T @ tvec).reshape(3)
    camera_center_unity = CV_FROM_UNITY @ camera_center_cv_world

    right_unity = CV_FROM_UNITY @ (rotation_world_cv_to_camera_cv.T @ np.array([1.0, 0.0, 0.0]))
    up_unity = CV_FROM_UNITY @ (rotation_world_cv_to_camera_cv.T @ np.array([0.0, -1.0, 0.0]))
    forward_unity = CV_FROM_UNITY @ (rotation_world_cv_to_camera_cv.T @ np.array([0.0, 0.0, 1.0]))
    right_unity /= max(np.linalg.norm(right_unity), 1e-12)
    up_unity /= max(np.linalg.norm(up_unity), 1e-12)
    forward_unity /= max(np.linalg.norm(forward_unity), 1e-12)

    return {
        "name": name,
        "success": True,
        "used_marker_ids": used_ids,
        "marker_corner_order_choices": {str(key): value for key, value in marker_order_choices.items()},
        "corner_count": int(len(object_points)),
        "position": camera_center_unity.tolist(),
        "right": right_unity.tolist(),
        "up": up_unity.tolist(),
        "forward": forward_unity.tolist(),
        "world_cv_to_camera_cv_rotation": rotation_world_cv_to_camera_cv.tolist(),
        "world_cv_to_camera_cv_translation": tvec.reshape(3).tolist(),
        "mean_reprojection_error_px": float(np.mean(errors)),
        "max_reprojection_error_px": float(np.max(errors)),
    }


def map_marker_pixels_to_live(detected_markers, high_size, live_size):
    high_w, high_h = high_size
    live_w, live_h = live_size
    sx = live_w / float(high_w)
    sy = live_h / float(high_h)
    mapped = {}
    for marker_id, corners in detected_markers.items():
        live = corners.copy()
        live[:, 0] *= sx
        live[:, 1] *= sy
        mapped[str(marker_id)] = live.tolist()
    return mapped


def save_debug(debug_dir, frame, corners, ids, name):
    debug_dir.mkdir(parents=True, exist_ok=True)
    vis = frame.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    cv2.imwrite(str(debug_dir / f"{name}_high_detected.png"), vis)


def main():
    args = parse_args()
    calibration = load_calibration(args.calibration)
    unity_markers = load_marker_corners(args.unity_marker_corners)

    frame1, frame2 = capture_pair(args)
    frame1_size = (frame1.shape[1], frame1.shape[0])
    frame2_size = (frame2.shape[1], frame2.shape[0])
    if frame1_size != frame2_size:
        raise RuntimeError(f"Captured frame sizes differ: {frame1_size} vs {frame2_size}")

    k1 = scale_camera_matrix(calibration["K1"], calibration["image_size"], frame1_size)
    k2 = scale_camera_matrix(calibration["K2"], calibration["image_size"], frame2_size)

    detector = build_detector()
    markers1, corners1, ids1, _ = detect_markers(frame1, detector)
    markers2, corners2, ids2, _ = detect_markers(frame2, detector)

    debug_dir = Path(args.debug_dir)
    save_debug(debug_dir, frame1, corners1, ids1, "cam1")
    save_debug(debug_dir, frame2, corners2, ids2, "cam2")

    cam1_solution = solve_camera_pose("cam1", markers1, unity_markers, k1, calibration["dist1"])
    cam2_solution = solve_camera_pose("cam2", markers2, unity_markers, k2, calibration["dist2"])
    real_to_unity_scale = None
    if cam1_solution.get("success") and cam2_solution.get("success"):
        cam1_pos = np.asarray(cam1_solution["position"], dtype=np.float64)
        cam2_pos = np.asarray(cam2_solution["position"], dtype=np.float64)
        unity_baseline = float(np.linalg.norm(cam2_pos - cam1_pos))
        real_cam2_in_cam1 = -(calibration["R"].T @ calibration["T"]).reshape(3)
        real_baseline = float(np.linalg.norm(real_cam2_in_cam1))
        if real_baseline > 1e-9:
            real_to_unity_scale = unity_baseline / real_baseline

    output = {
        "timestamp": time.time(),
        "calibration": str(args.calibration),
        "unity_marker_corners": str(args.unity_marker_corners),
        "high_frame_size": [frame1_size[0], frame1_size[1]],
        "live_frame_size": [args.live_width, args.live_height],
        "detected_high": {
            "cam1_ids": sorted(markers1.keys()),
            "cam2_ids": sorted(markers2.keys()),
        },
        "detected_live_mapped_pixels": {
            "cam1": map_marker_pixels_to_live(markers1, frame1_size, (args.live_width, args.live_height)),
            "cam2": map_marker_pixels_to_live(markers2, frame2_size, (args.live_width, args.live_height)),
        },
        "cameras": {
            "cam1": cam1_solution,
            "cam2": cam2_solution,
        },
        "real_to_unity_scale": real_to_unity_scale,
        "debug_images": {
            "cam1": str(debug_dir / "cam1_high_detected.png"),
            "cam2": str(debug_dir / "cam2_high_detected.png"),
        },
    }

    Path(args.output).write_text(json.dumps(output, indent=2))
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
