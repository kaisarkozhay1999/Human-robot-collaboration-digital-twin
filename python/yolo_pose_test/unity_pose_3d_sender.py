import argparse
import json
from pathlib import Path
import socket
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from test_dual_stream_pipeline import CAM1_HIGH, CAM2_HIGH, CamReader


SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = SCRIPT_DIR / "stereo_calibration_charuco_refined.npz"
YOLO_MODEL_PATH = SCRIPT_DIR / "yolov8n-pose.pt"

UNITY_HOST = "127.0.0.1"
UNITY_PORT = 5005

YOLO_IMGSZ = 320
PERSON_CONF = 0.35
MIN_KEYPOINT_CONF = 0.35
MAX_POINT_DISTANCE_M = 20.0

COCO_KEYPOINT_NAMES = [
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
]

SKELETON = [
    (5, 7), (7, 9),
    (6, 8), (8, 10),
    (5, 6),
    (5, 11), (6, 12),
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16),
]


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


def load_model(model_path, device):
    model = YOLO(model_path)

    if device == "cpu":
        print("Using CPU for YOLO.")
        return model, "cpu", False

    if device in ("auto", "cuda") and torch.cuda.is_available():
        try:
            model.to("cuda")
            torch.backends.cudnn.benchmark = True
            print("Using CUDA for YOLO.")
            return model, "cuda", True
        except Exception as exc:
            if device == "cuda":
                raise
            print(f"CUDA failed, falling back to CPU: {exc}")

    print("Using CPU for YOLO.")
    return model, "cpu", False


def run_pose(model, frame, use_half):
    results = model(
        frame,
        verbose=False,
        imgsz=YOLO_IMGSZ,
        conf=PERSON_CONF,
        half=use_half,
    )
    result = results[0]

    if result.keypoints is None or len(result.keypoints) == 0:
        return None

    keypoints_xy = result.keypoints.xy.cpu().numpy()
    keypoints_conf = result.keypoints.conf.cpu().numpy()

    person_index = 0
    person_score = 0.0
    if result.boxes is not None and len(result.boxes) > 0:
        box_conf = result.boxes.conf.cpu().numpy()
        person_index = int(np.argmax(box_conf))
        person_score = float(box_conf[person_index])

    return {
        "points": keypoints_xy[person_index].astype(np.float64),
        "confidence": keypoints_conf[person_index].astype(np.float64),
        "person_score": person_score,
    }


def triangulate_pose(pose1, pose2, calibration, min_conf):
    kpts1 = pose1["points"]
    kpts2 = pose2["points"]
    conf1 = pose1["confidence"]
    conf2 = pose2["confidence"]

    valid = (conf1 >= min_conf) & (conf2 >= min_conf)
    joints_3d = np.full((len(COCO_KEYPOINT_NAMES), 3), np.nan, dtype=np.float64)

    valid_indices = np.where(valid)[0]
    if len(valid_indices) == 0:
        return joints_3d, valid

    pts1 = kpts1[valid_indices].reshape(-1, 1, 2)
    pts2 = kpts2[valid_indices].reshape(-1, 1, 2)

    undistorted1 = cv2.undistortPoints(pts1, calibration["K1"], calibration["dist1"])
    undistorted2 = cv2.undistortPoints(pts2, calibration["K2"], calibration["dist2"])

    projection1 = np.hstack((np.eye(3), np.zeros((3, 1)))).astype(np.float64)
    projection2 = np.hstack((calibration["R"], calibration["T"])).astype(np.float64)

    points_4d = cv2.triangulatePoints(
        projection1,
        projection2,
        undistorted1.reshape(-1, 2).T,
        undistorted2.reshape(-1, 2).T,
    )
    points_3d = (points_4d[:3] / points_4d[3]).T

    cam2_points = (calibration["R"] @ points_3d.T + calibration["T"]).T
    in_front = (points_3d[:, 2] > 0.0) & (cam2_points[:, 2] > 0.0)
    finite = np.isfinite(points_3d).all(axis=1)
    plausible = np.linalg.norm(points_3d, axis=1) <= MAX_POINT_DISTANCE_M

    keep = in_front & finite & plausible
    for local_index, joint_index in enumerate(valid_indices):
        if keep[local_index]:
            joints_3d[joint_index] = points_3d[local_index]
        else:
            valid[joint_index] = False

    return joints_3d, valid


def opencv_to_unity(point_m, scale):
    # OpenCV camera coordinates: X right, Y down, Z forward.
    # Unity avatar coordinates here: X right, Y up, Z forward.
    return {
        "x": float(point_m[0] * scale),
        "y": float(-point_m[1] * scale),
        "z": float(point_m[2] * scale),
    }


def make_payload(frame_index, joints_3d, valid, pose1, pose2, scale):
    joints = []
    for joint_id, name in enumerate(COCO_KEYPOINT_NAMES):
        tracked = bool(valid[joint_id] and np.isfinite(joints_3d[joint_id]).all())
        confidence = float(min(pose1["confidence"][joint_id], pose2["confidence"][joint_id]))

        if tracked:
            coords = opencv_to_unity(joints_3d[joint_id], scale)
        else:
            coords = {"x": 0.0, "y": 0.0, "z": 0.0}

        joints.append({
            "id": joint_id,
            "name": name,
            "tracked": tracked,
            "confidence": confidence,
            **coords,
        })

    return {
        "type": "pose3d",
        "frame": frame_index,
        "timestamp": time.time(),
        "units": "meters",
        "coordinate_space": "unity_from_cam1",
        "person_score_cam1": float(pose1["person_score"]),
        "person_score_cam2": float(pose2["person_score"]),
        "joints": joints,
    }


def draw_pose(frame, pose, color):
    vis = frame.copy()
    if pose is None:
        return vis

    points = pose["points"]
    conf = pose["confidence"]

    for joint_id, point in enumerate(points):
        if conf[joint_id] >= MIN_KEYPOINT_CONF:
            cv2.circle(vis, tuple(point.astype(int)), 4, color, -1)

    for a, b in SKELETON:
        if conf[a] >= MIN_KEYPOINT_CONF and conf[b] >= MIN_KEYPOINT_CONF:
            cv2.line(vis, tuple(points[a].astype(int)), tuple(points[b].astype(int)), color, 2)

    return vis


def resize_to_width(frame, width):
    h, w = frame.shape[:2]
    scale = width / float(w)
    return cv2.resize(frame, (width, int(h * scale)), interpolation=cv2.INTER_AREA)


def parse_args():
    parser = argparse.ArgumentParser(description="Triangulate YOLO pose joints and stream them to Unity over UDP.")
    parser.add_argument("--calibration", default=CALIBRATION_FILE)
    parser.add_argument("--model", default=YOLO_MODEL_PATH)
    parser.add_argument("--cam1", default=CAM1_HIGH, help="Left/primary camera RTSP URL.")
    parser.add_argument("--cam2", default=CAM2_HIGH, help="Right/secondary camera RTSP URL.")
    parser.add_argument("--host", default=UNITY_HOST)
    parser.add_argument("--port", type=int, default=UNITY_PORT)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--send-hz", type=float, default=15.0)
    parser.add_argument("--min-keypoint-conf", type=float, default=MIN_KEYPOINT_CONF)
    parser.add_argument("--unity-scale", type=float, default=1.0)
    parser.add_argument("--display-width", type=int, default=520)
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    calibration = load_calibration(args.calibration)
    print(f"Loaded calibration: {args.calibration}")
    print(f"Calibration image size: {calibration['image_size'][0]}x{calibration['image_size'][1]}")

    model, _, use_half = load_model(args.model, args.device)
    model(np.zeros((256, 256, 3), dtype=np.uint8), verbose=False, imgsz=YOLO_IMGSZ)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    unity_address = (args.host, args.port)
    print(f"Sending UDP pose packets to {args.host}:{args.port}")

    print("Opening high-resolution streams...")
    print(f"  Cam1 source: {args.cam1}")
    print(f"  Cam2 source: {args.cam2}")
    cam1 = CamReader(args.cam1, "Cam1 HIGH")
    cam2 = CamReader(args.cam2, "Cam2 HIGH")
    cameras = [cam1, cam2]

    if not cam1.is_opened() or not cam2.is_opened():
        for camera in cameras:
            camera.release()
        raise RuntimeError(
            "Both high-resolution camera streams are required for 3D triangulation. "
            f"Failed sources: cam1={args.cam1!r}, cam2={args.cam2!r}. "
            "Check that both cameras are powered, reachable from this PC, and that the RTSP URLs are correct."
        )

    frame_interval = 1.0 / max(args.send_hz, 0.1)
    next_send_time = 0.0
    frame_index = 0
    last_payload = None
    active_calibration = calibration
    active_frame_size = None

    window_name = "Unity 3D Pose Sender"
    if not args.no_display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1400, 800)

    print("Press ESC or Q to quit.")

    try:
        while True:
            ok1, frame1 = cam1.read()
            ok2, frame2 = cam2.read()
            now = time.perf_counter()

            if ok1 and ok2 and frame1 is not None and frame2 is not None and now >= next_send_time:
                next_send_time = now + frame_interval
                frame_index += 1

                frame_size_1 = (frame1.shape[1], frame1.shape[0])
                frame_size_2 = (frame2.shape[1], frame2.shape[0])
                if frame_size_1 != frame_size_2:
                    print(f"Skipping frame: camera sizes differ {frame_size_1} vs {frame_size_2}")
                    continue

                if active_frame_size != frame_size_1:
                    active_calibration = calibration_for_frame_size(calibration, frame_size_1)
                    active_frame_size = frame_size_1
                    if active_frame_size != calibration["image_size"]:
                        print(
                            "Live frame size differs from calibration; "
                            f"scaled intrinsics to {active_frame_size[0]}x{active_frame_size[1]}"
                        )

                pose1 = run_pose(model, frame1, use_half)
                pose2 = run_pose(model, frame2, use_half)

                if pose1 is not None and pose2 is not None:
                    joints_3d, valid = triangulate_pose(
                        pose1,
                        pose2,
                        active_calibration,
                        args.min_keypoint_conf,
                    )
                    payload = make_payload(frame_index, joints_3d, valid, pose1, pose2, args.unity_scale)
                    message = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                    sock.sendto(message, unity_address)
                    last_payload = payload

                    tracked_count = sum(1 for joint in payload["joints"] if joint["tracked"])
                    print(f"frame={frame_index} sent_joints={tracked_count}", end="\r")

                if not args.no_display:
                    vis1 = draw_pose(frame1, pose1, (0, 255, 255))
                    vis2 = draw_pose(frame2, pose2, (0, 255, 255))
                    vis1 = resize_to_width(vis1, args.display_width)
                    vis2 = resize_to_width(vis2, args.display_width)
                    h = max(vis1.shape[0], vis2.shape[0])
                    vis1 = cv2.resize(vis1, (vis1.shape[1], h))
                    vis2 = cv2.resize(vis2, (vis2.shape[1], h))
                    combined = np.hstack((vis1, vis2))
                    tracked = 0 if last_payload is None else sum(
                        1 for joint in last_payload["joints"] if joint["tracked"]
                    )
                    cv2.putText(
                        combined,
                        f"UDP {args.host}:{args.port}  tracked joints: {tracked}",
                        (20, 34),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow(window_name, combined)

            if not args.no_display:
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break
            else:
                time.sleep(0.001)

    finally:
        print()
        for camera in cameras:
            camera.release()
        sock.close()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
