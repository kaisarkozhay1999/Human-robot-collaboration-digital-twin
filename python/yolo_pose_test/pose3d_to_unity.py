import os
import argparse
import atexit
import json
import socket
import subprocess
import threading
import time
from pathlib import Path

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;tcp"
    "|fflags;nobuffer"
    "|flags;low_delay"
    "|analyzeduration;1000000"
    "|probesize;1048576"
)

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from runtime_metrics import MetricsSession, mean_or_none

try:
    import imageio_ffmpeg
except ImportError:
    imageio_ffmpeg = None


# ============================================================
# PATHS
# ============================================================
SCRIPT_DIR = Path(__file__).resolve().parent

LIVE_CALIBRATION_FILE = SCRIPT_DIR / "stereo_calibration_charuco_live.npz"
REFINED_CALIBRATION_FILE = SCRIPT_DIR / "stereo_calibration_charuco_refined.npz"
BASE_CALIBRATION_FILE    = SCRIPT_DIR / "stereo_calibration_charuco.npz"

CALIBRATION_FILE = next(
    (
        path for path in (
            LIVE_CALIBRATION_FILE,
            REFINED_CALIBRATION_FILE,
            BASE_CALIBRATION_FILE,
        )
        if path.exists()
    ),
    LIVE_CALIBRATION_FILE,
)

YOLO_MODEL_PATH = SCRIPT_DIR / "yolo26n-pose.pt"


# ============================================================
# CAMERA STREAMS
# ============================================================
CAM1_HIGH = os.environ.get("SMARTLAB_CAM1_HIGH", "rtsp://CAMERA_1_IP/h264")
CAM2_HIGH = os.environ.get("SMARTLAB_CAM2_HIGH", "rtsp://CAMERA_2_IP/h264")
CAM1_LIVE = CAM1_HIGH
CAM2_LIVE = CAM2_HIGH

LIVE_IMAGE_SIZE = (1280, 720)
USE_FFMPEG_READER = True
FFMPEG_TRANSPORT = "tcp"
FFMPEG_EXE = os.environ.get("SMARTLAB_FFMPEG_EXE", "")
FFMPEG_STREAM_SIZES = {
    CAM1_HIGH: LIVE_IMAGE_SIZE,
    CAM2_HIGH: LIVE_IMAGE_SIZE,
}


# ============================================================
# UNITY UDP
# ============================================================
UNITY_HOST = "127.0.0.1"
UNITY_PORT = 5005
UNITY_COMMAND_PORT = 5007
GESTURE_BRIDGE_HOST = "127.0.0.1"
GESTURE_BRIDGE_PORT = 5010


# ============================================================
# SETTINGS
# ============================================================
YOLO_IMGSZ           = 480
PERSON_CONF          = 0.35
MIN_KEYPOINT_CONF    = 0.35
MAX_POINT_DISTANCE_M = 20.0
PROCESS_WIDTH        = 0     # keep live frame size so calibration stays aligned
ARUCO_DICT           = cv2.aruco.DICT_4X4_50
MAX_MARKER_DISTANCE_M = 20.0
MAX_TRACK_MATCH_DISTANCE_M = 0.75
TRACK_STALE_FRAMES = 15
CV_FROM_UNITY = np.diag([1.0, -1.0, 1.0])
MARKER_ANCHOR_ENABLED = True
MARKER_ANCHOR_SMOOTHING = 0.12
MARKER_ANCHOR_MAX_TRANSLATION_M = 0.35
MARKER_ANCHOR_MAX_ROTATION_DEG = 25.0
JOINT_FILTER_ENABLED = True
JOINT_FILTER_ALPHA = 0.18
JOINT_FILTER_FAST_ALPHA = 0.65
JOINT_FILTER_FAST_DISTANCE = 0.025
JOINT_FILTER_DEADBAND = 0.0015
JOINT_FILTER_SNAP_DISTANCE = 0.08
JOINT_FILTER_HOLD_FRAMES = 4
GESTURE_HOLD_SEC = 0.30
GESTURE_COOLDOWN_SEC = 1.50
HANDS_UP_MARGIN_RATIO = 0.20
HANDS_UP_MIN_MARGIN_PX = 20.0
TPOSE_VERTICAL_TOLERANCE_RATIO = 0.30
TPOSE_MIN_SPAN_RATIO = 1.80

person_tracks = {}
next_person_id = 1
joint_filter_states = {}


COCO_KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]

SKELETON = [
    (5, 7), (7, 9), (6, 8), (8, 10), (5, 6),
    (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
]


# ============================================================
# CAMERA READER
# ============================================================
def resolve_ffmpeg_exe(configured_path=None):
    if configured_path:
        path = Path(configured_path)
        if path.exists():
            return str(path)

    if FFMPEG_EXE:
        path = Path(FFMPEG_EXE)
        if path.exists():
            return str(path)

    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        winget_root = Path(local_app_data) / "Microsoft" / "WinGet" / "Packages"
        candidates = sorted(winget_root.glob("Gyan.FFmpeg*/*/bin/ffmpeg.exe"))
        if candidates:
            return str(candidates[-1])

    return imageio_ffmpeg.get_ffmpeg_exe() if imageio_ffmpeg is not None else "ffmpeg"


class CamReader:
    def __init__(
        self,
        rtsp_url,
        name="Camera",
        use_ffmpeg=USE_FFMPEG_READER,
        ffmpeg_exe=None,
        transport=FFMPEG_TRANSPORT,
        frame_size=None,
    ):
        self.name = name
        self.rtsp_url = rtsp_url
        self.use_ffmpeg = use_ffmpeg
        self.ffmpeg_exe = ffmpeg_exe
        self.transport = transport
        self.proc = None
        self.stderr_thread = None
        self.cap = None
        self.width, self.height = frame_size or FFMPEG_STREAM_SIZES.get(rtsp_url, LIVE_IMAGE_SIZE)
        self.frame_size = self.width * self.height * 3
        self.read_count = 0
        self.frame_received_ns = 0

        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.stopped = False

        if self.use_ffmpeg:
            try:
                self._start_ffmpeg()
            except Exception as exc:
                print(f"{self.name}: FFmpeg reader failed, falling back to OpenCV: {exc}")
                self.use_ffmpeg = False

        if not self.use_ffmpeg:
            self._start_opencv()

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _start_ffmpeg(self):
        ffmpeg_exe = resolve_ffmpeg_exe(self.ffmpeg_exe)
        cmd = [
            ffmpeg_exe,
            "-hide_banner",
            "-loglevel", "warning",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "1000000",
            "-probesize", "1048576",
            "-rtsp_transport", self.transport,
            "-i", self.rtsp_url,
            "-an",
            "-vf", f"scale={self.width}:{self.height}",
            "-fps_mode", "passthrough",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-",
        ]

        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.stderr_thread = threading.Thread(target=self._stderr_loop, daemon=True)
        self.stderr_thread.start()
        print(f"{self.name}: FFmpeg reader {self.width}x{self.height} transport={self.transport}")

    def _start_opencv(self):
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        try:
            self.ok, self.frame = self.cap.read()
        except Exception:
            self.ok, self.frame = False, None
        print(f"{self.name}: OpenCV reader")

    def _stderr_loop(self):
        while not self.stopped and self.proc is not None and self.proc.stderr is not None:
            line = self.proc.stderr.readline()
            if not line:
                break
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                print(f"{self.name} ffmpeg: {text}")

    def _read_exact(self, size):
        data = bytearray(size)
        view = memoryview(data)
        offset = 0
        while offset < size and not self.stopped:
            n = self.proc.stdout.readinto(view[offset:])
            if not n:
                return None
            offset += n
        return data

    def _loop(self):
        if self.use_ffmpeg:
            self._loop_ffmpeg()
        else:
            self._loop_opencv()

    def _loop_ffmpeg(self):
        while not self.stopped:
            data = self._read_exact(self.frame_size)
            if data is None:
                with self.lock:
                    self.ok = False
                time.sleep(0.002)
                continue

            frame = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()
            with self.lock:
                self.ok = True
                self.frame = frame
                self.read_count += 1
                self.frame_received_ns = time.perf_counter_ns()

    def _loop_opencv(self):
        while not self.stopped:
            if self.cap is None or not self.cap.isOpened():
                with self.lock:
                    self.ok = False
                time.sleep(0.05)
                continue

            if not self.cap.grab():
                with self.lock:
                    self.ok = False
                time.sleep(0.01)
                continue

            ok, frame = self.cap.retrieve()
            if ok and frame is not None:
                with self.lock:
                    self.ok = True
                    self.frame = frame
                    self.read_count += 1
                    self.frame_received_ns = time.perf_counter_ns()
            else:
                time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def read_with_metadata(self):
        with self.lock:
            if self.frame is None:
                return False, None, None
            return self.ok, self.frame.copy(), {
                "sequence": self.read_count,
                "received_ns": self.frame_received_ns,
            }

    def is_opened(self):
        if self.use_ffmpeg:
            return self.proc is not None and self.proc.poll() is None
        return self.cap is not None and self.cap.isOpened()

    def release(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.use_ffmpeg:
            if self.proc is not None and self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            if self.stderr_thread is not None and self.stderr_thread.is_alive():
                self.stderr_thread.join(timeout=1)
        elif self.cap is not None:
            self.cap.release()


# ============================================================
# DISPLAY THREAD
# ============================================================
class DisplayThread:
    def __init__(self, window_name, display_width):
        self.window_name   = window_name
        self.display_width = display_width
        self._lock         = threading.Lock()
        self._frame        = None
        self._stop         = False
        self._quit         = False

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1400, 800)

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def push(self, frame):
        with self._lock:
            self._frame = frame

    @property
    def quit_requested(self):
        return self._quit

    def _loop(self):
        while not self._stop:
            frame = None
            with self._lock:
                if self._frame is not None:
                    frame       = self._frame
                    self._frame = None
            if frame is not None:
                cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                self._quit = True
                break
            time.sleep(0.001)

    def stop(self):
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=2)
        cv2.destroyAllWindows()


# ============================================================
# CALIBRATION
# ============================================================
def load_calibration(path):
    path = Path(path)
    if not path.exists():
        raise RuntimeError(f"Calibration file not found: {path}")
    data     = np.load(path)
    required = ["K1", "dist1", "K2", "dist2", "R", "T", "image_width", "image_height"]
    missing  = [k for k in required if k not in data.files]
    if missing:
        raise RuntimeError(f"Calibration file missing keys: {missing}")
    return {
        "K1":         data["K1"].astype(np.float64),
        "dist1":      data["dist1"].astype(np.float64),
        "K2":         data["K2"].astype(np.float64),
        "dist2":      data["dist2"].astype(np.float64),
        "R":          data["R"].astype(np.float64),
        "T":          data["T"].astype(np.float64).reshape(3, 1),
        "image_size": (int(data["image_width"]), int(data["image_height"])),
    }


def calibration_for_frame_size(calibration, frame_size):
    if frame_size == calibration["image_size"]:
        return calibration
    calib_w, calib_h = calibration["image_size"]
    frame_w, frame_h = frame_size
    scale_x          = frame_w / float(calib_w)
    scale_y          = frame_h / float(calib_h)
    scaled           = dict(calibration)
    scaled["K1"]     = calibration["K1"].copy()
    scaled["K2"]     = calibration["K2"].copy()
    scaled["K1"][0, :] *= scale_x
    scaled["K1"][1, :] *= scale_y
    scaled["K2"][0, :] *= scale_x
    scaled["K2"][1, :] *= scale_y
    scaled["image_size"] = frame_size
    return scaled


# ============================================================
# YOLO
# ============================================================
def load_model(model_path, device):
    model = YOLO(model_path)

    print("Torch:", torch.__version__)
    print("CUDA available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("CUDA device:", torch.cuda.get_device_name(0))

    if device == "cpu":
        print("Using CPU.")
        return model, "cpu", False

    if device in ("auto", "cuda") and torch.cuda.is_available():
        try:
            model.to("cuda")
            torch.backends.cudnn.benchmark = True
            print("Using CUDA.")
            return model, "cuda", True
        except Exception as exc:
            if device == "cuda":
                raise
            print(f"CUDA failed, falling back to CPU: {exc}")

    print("Using CPU.")
    return model, "cpu", False


def run_pose_batch(model, frame1, frame2, use_half):
    """Single batched forward pass for both cameras."""
    results = model(
        [frame1, frame2],
        verbose=False,
        imgsz=YOLO_IMGSZ,
        conf=PERSON_CONF,
        half=use_half,
    )
    poses = []
    for result in results:
        if result.keypoints is None or len(result.keypoints) == 0:
            poses.append(None)
            continue
        kp_xy   = result.keypoints.xy.cpu().numpy()
        kp_conf = result.keypoints.conf.cpu().numpy()
        idx     = 0
        score   = 0.0
        if result.boxes is not None and len(result.boxes) > 0:
            bc    = result.boxes.conf.cpu().numpy()
            idx   = int(np.argmax(bc))
            score = float(bc[idx])
        poses.append({
            "points":       kp_xy[idx].astype(np.float64),
            "confidence":   kp_conf[idx].astype(np.float64),
            "person_score": score,
        })
    return poses[0], poses[1]


def run_pose_batch_multi(model, frame1, frame2, use_half):
    results = model(
        [frame1, frame2],
        verbose=False,
        imgsz=YOLO_IMGSZ,
        conf=PERSON_CONF,
        half=use_half,
    )

    all_poses = []
    for result in results:
        poses = []
        if result.keypoints is None or len(result.keypoints) == 0:
            all_poses.append(poses)
            continue

        kp_xy = result.keypoints.xy.cpu().numpy()
        kp_conf = result.keypoints.conf.cpu().numpy()
        box_conf = (
            result.boxes.conf.cpu().numpy()
            if result.boxes is not None and len(result.boxes) > 0
            else np.ones(len(kp_xy), dtype=np.float64)
        )

        boxes_xywh = (
            result.boxes.xywh.cpu().numpy()
            if result.boxes is not None and len(result.boxes) > 0
            else None
        )

        for i in range(len(kp_xy)):
            if boxes_xywh is not None and i < len(boxes_xywh):
                center = boxes_xywh[i, :2].astype(np.float64)
            else:
                confident = kp_conf[i] >= MIN_KEYPOINT_CONF
                if np.any(confident):
                    center = np.mean(kp_xy[i][confident], axis=0).astype(np.float64)
                else:
                    center = np.mean(kp_xy[i], axis=0).astype(np.float64)

            poses.append({
                "points": kp_xy[i].astype(np.float64),
                "confidence": kp_conf[i].astype(np.float64),
                "person_score": float(box_conf[i]) if i < len(box_conf) else 0.0,
                "center": center,
            })

        poses.sort(key=lambda p: float(p["center"][0]))
        all_poses.append(poses)

    return all_poses[0], all_poses[1]


# ============================================================
# TRIANGULATION
# ============================================================
def triangulate_pose(pose1, pose2, calibration, min_conf):
    kpts1 = pose1["points"]
    kpts2 = pose2["points"]
    conf1 = pose1["confidence"]
    conf2 = pose2["confidence"]
    valid = (conf1 >= min_conf) & (conf2 >= min_conf)

    joints_3d     = np.full((len(COCO_KEYPOINT_NAMES), 3), np.nan, dtype=np.float64)
    valid_indices = np.where(valid)[0]

    if len(valid_indices) == 0:
        return joints_3d, valid

    pts1 = kpts1[valid_indices].reshape(-1, 1, 2)
    pts2 = kpts2[valid_indices].reshape(-1, 1, 2)

    u1 = cv2.undistortPoints(pts1, calibration["K1"], calibration["dist1"])
    u2 = cv2.undistortPoints(pts2, calibration["K2"], calibration["dist2"])

    P1    = np.hstack((np.eye(3), np.zeros((3, 1)))).astype(np.float64)
    P2    = np.hstack((calibration["R"], calibration["T"])).astype(np.float64)
    pts4d = cv2.triangulatePoints(P1, P2, u1.reshape(-1, 2).T, u2.reshape(-1, 2).T)
    pts3d = (pts4d[:3] / pts4d[3]).T

    cam2_pts  = (calibration["R"] @ pts3d.T + calibration["T"]).T
    in_front  = (pts3d[:, 2] > 0.0) & (cam2_pts[:, 2] > 0.0)
    finite    = np.isfinite(pts3d).all(axis=1)
    plausible = np.linalg.norm(pts3d, axis=1) <= MAX_POINT_DISTANCE_M
    keep      = in_front & finite & plausible

    for local_i, joint_i in enumerate(valid_indices):
        if keep[local_i]:
            joints_3d[joint_i] = pts3d[local_i]
        else:
            valid[joint_i] = False

    return joints_3d, valid


def stereo_quality_metrics(matches, calibration, min_conf):
    """Return software-only geometric consistency errors for accepted pose pairs."""
    reprojection_errors = []
    epipolar_errors = []
    k1 = calibration["K1"]
    k2 = calibration["K2"]
    r = calibration["R"]
    t = calibration["T"].reshape(3, 1)
    tx = np.array([
        [0.0, -t[2, 0], t[1, 0]],
        [t[2, 0], 0.0, -t[0, 0]],
        [-t[1, 0], t[0, 0], 0.0],
    ], dtype=np.float64)
    fundamental = np.linalg.inv(k2).T @ tx @ r @ np.linalg.inv(k1)

    for pose1, pose2 in matches:
        joints_3d, valid = triangulate_pose(pose1, pose2, calibration, min_conf)
        indices = np.where(valid & np.isfinite(joints_3d).all(axis=1))[0]
        if len(indices) == 0:
            continue
        points_3d = joints_3d[indices].reshape(-1, 1, 3)
        projected1, _ = cv2.projectPoints(
            points_3d, np.zeros((3, 1)), np.zeros((3, 1)), k1, calibration["dist1"]
        )
        rvec2, _ = cv2.Rodrigues(r)
        projected2, _ = cv2.projectPoints(points_3d, rvec2, t, k2, calibration["dist2"])
        observed1 = pose1["points"][indices]
        observed2 = pose2["points"][indices]
        reprojection_errors.extend(np.linalg.norm(projected1.reshape(-1, 2) - observed1, axis=1))
        reprojection_errors.extend(np.linalg.norm(projected2.reshape(-1, 2) - observed2, axis=1))

        homogeneous1 = np.column_stack((observed1, np.ones(len(indices))))
        homogeneous2 = np.column_stack((observed2, np.ones(len(indices))))
        lines2 = (fundamental @ homogeneous1.T).T
        denominator = np.linalg.norm(lines2[:, :2], axis=1)
        numerator = np.abs(np.sum(lines2 * homogeneous2, axis=1))
        good = denominator > 1e-12
        epipolar_errors.extend(numerator[good] / denominator[good])

    reprojection_errors = np.asarray(reprojection_errors, dtype=np.float64)
    epipolar_errors = np.asarray(epipolar_errors, dtype=np.float64)
    return {
        "reprojection_mean_px": float(np.mean(reprojection_errors)) if reprojection_errors.size else None,
        "reprojection_p95_px": float(np.percentile(reprojection_errors, 95)) if reprojection_errors.size else None,
        "epipolar_mean_px": float(np.mean(epipolar_errors)) if epipolar_errors.size else None,
    }


def pose_pair_score(pose1, pose2):
    conf = np.minimum(pose1["confidence"], pose2["confidence"])
    return int(np.count_nonzero(conf >= MIN_KEYPOINT_CONF))


def match_pose_lists(poses1, poses2, calibration, min_conf):
    if not poses1 or not poses2:
        return []

    candidates = []
    for i, pose1 in enumerate(poses1):
        for j, pose2 in enumerate(poses2):
            joints_3d, valid = triangulate_pose(pose1, pose2, calibration, min_conf)
            score = int(np.count_nonzero(valid))
            if score <= 0:
                continue
            center_distance = float(np.linalg.norm(pose1["center"] - pose2["center"]))
            candidates.append((-score, center_distance, i, j))

    candidates.sort()
    used1 = set()
    used2 = set()
    matches = []
    for _, _, i, j in candidates:
        if i in used1 or j in used2:
            continue
        used1.add(i)
        used2.add(j)
        matches.append((poses1[i], poses2[j]))
    return matches


def person_track_position(joints_3d, valid):
    hip_ids = [11, 12]
    hip_points = [joints_3d[i] for i in hip_ids if valid[i] and np.isfinite(joints_3d[i]).all()]
    if hip_points:
        return np.mean(np.asarray(hip_points), axis=0)

    valid_points = joints_3d[valid]
    valid_points = valid_points[np.isfinite(valid_points).all(axis=1)]
    if len(valid_points) == 0:
        return None
    return np.mean(valid_points, axis=0)


def assign_person_id(track_position, frame_index, used_ids):
    global next_person_id

    if track_position is None:
        person_id = next_person_id
        next_person_id += 1
        used_ids.add(person_id)
        return person_id

    best_id = None
    best_distance = MAX_TRACK_MATCH_DISTANCE_M
    for person_id, track in person_tracks.items():
        if person_id in used_ids:
            continue
        distance = float(np.linalg.norm(track["position"] - track_position))
        if distance < best_distance:
            best_distance = distance
            best_id = person_id

    if best_id is None:
        best_id = next_person_id
        next_person_id += 1

    person_tracks[best_id] = {
        "position": track_position,
        "last_frame": frame_index,
    }
    used_ids.add(best_id)

    stale_ids = [
        person_id for person_id, track in person_tracks.items()
        if frame_index - int(track["last_frame"]) > TRACK_STALE_FRAMES
    ]
    for person_id in stale_ids:
        person_tracks.pop(person_id, None)

    return best_id


def smooth_payload_joints(
    person_id,
    joints,
    frame_index,
    enabled=True,
    alpha=JOINT_FILTER_ALPHA,
    fast_alpha=JOINT_FILTER_FAST_ALPHA,
    fast_distance=JOINT_FILTER_FAST_DISTANCE,
    deadband=JOINT_FILTER_DEADBAND,
    snap_distance=JOINT_FILTER_SNAP_DISTANCE,
    hold_frames=JOINT_FILTER_HOLD_FRAMES,
):
    if not enabled:
        return joints

    state = joint_filter_states.setdefault(int(person_id), {})
    alpha = float(np.clip(alpha, 0.0, 1.0))
    fast_alpha = float(np.clip(fast_alpha, alpha, 1.0))
    fast_distance = max(float(fast_distance), 1e-6)
    deadband = max(float(deadband), 0.0)
    snap_distance = max(float(snap_distance), deadband)
    hold_frames = max(int(hold_frames), 0)

    for joint in joints:
        joint_id = int(joint["id"])
        previous = state.get(joint_id)

        if not joint.get("tracked", False):
            if previous is not None and frame_index - int(previous["frame"]) <= hold_frames:
                pos = previous["position"]
                joint["tracked"] = True
                joint["confidence"] = max(float(joint.get("confidence", 0.0)), MIN_KEYPOINT_CONF)
                joint["x"] = float(pos[0])
                joint["y"] = float(pos[1])
                joint["z"] = float(pos[2])
            continue

        measured = np.asarray([joint["x"], joint["y"], joint["z"]], dtype=np.float64)
        if not np.isfinite(measured).all():
            joint["tracked"] = False
            continue

        if previous is None:
            filtered = measured
        else:
            previous_pos = np.asarray(previous["position"], dtype=np.float64)
            delta = measured - previous_pos
            distance = float(np.linalg.norm(delta))

            if distance <= deadband:
                filtered = previous_pos
            elif distance >= snap_distance:
                filtered = measured
            else:
                speed_factor = float(np.clip(distance / fast_distance, 0.0, 1.0))
                blend = alpha + (fast_alpha - alpha) * speed_factor
                filtered = previous_pos + blend * delta

        state[joint_id] = {
            "position": filtered,
            "frame": int(frame_index),
        }
        joint["x"] = float(filtered[0])
        joint["y"] = float(filtered[1])
        joint["z"] = float(filtered[2])

    return joints


def prune_joint_filter_states(frame_index, stale_frames=90):
    stale_people = []
    for person_id, joints in joint_filter_states.items():
        stale_joints = [
            joint_id for joint_id, state in joints.items()
            if frame_index - int(state.get("frame", 0)) > stale_frames
        ]
        for joint_id in stale_joints:
            joints.pop(joint_id, None)
        if not joints:
            stale_people.append(person_id)

    for person_id in stale_people:
        joint_filter_states.pop(person_id, None)


def make_person_payload(
    person_id,
    joints_3d,
    valid,
    pose1,
    pose2,
    scale,
    alignment=None,
    frame_index=0,
    filter_config=None,
):
    joints = []
    for joint_id, name in enumerate(COCO_KEYPOINT_NAMES):
        tracked = bool(valid[joint_id] and np.isfinite(joints_3d[joint_id]).all())
        confidence = float(min(pose_confidence(pose1, joint_id), pose_confidence(pose2, joint_id)))
        coords = point_to_payload_coords(joints_3d[joint_id], scale, alignment) if tracked else {"x": 0.0, "y": 0.0, "z": 0.0}
        joints.append({"id": joint_id, "name": name, "tracked": tracked, "confidence": confidence, **coords})

    if filter_config is not None:
        smooth_payload_joints(person_id, joints, frame_index, **filter_config)

    return {
        "id": int(person_id),
        "tracked": True,
        "confidence": float(min(pose_score(pose1), pose_score(pose2))),
        "joints": joints,
    }


def build_people_payload(matches, calibration, min_conf, scale, frame_index, alignment=None, filter_config=None):
    people = []
    best = None
    best_count = -1
    used_ids = set()

    for pose1, pose2 in matches:
        joints_3d, valid = triangulate_pose(pose1, pose2, calibration, min_conf)
        tracked_count = int(np.count_nonzero(valid))
        if tracked_count == 0:
            continue

        track_position = person_track_position(joints_3d, valid)
        person_id = assign_person_id(track_position, frame_index, used_ids)
        person = make_person_payload(
            person_id,
            joints_3d,
            valid,
            pose1,
            pose2,
            scale,
            alignment,
            frame_index=frame_index,
            filter_config=filter_config,
        )
        people.append(person)

        if tracked_count > best_count:
            best_count = tracked_count
            best = (joints_3d, valid, pose1, pose2)

    return people, best


def build_aruco_detector():
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


def detect_aruco_markers(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    markers = {}
    if ids is None or len(ids) == 0:
        return markers, corners, ids

    ids_flat = ids.flatten()
    for i, marker_id in enumerate(ids_flat):
        markers[int(marker_id)] = corners[i].reshape(4, 2).astype(np.float64)
    return markers, corners, ids


def triangulate_points(points1, points2, calibration):
    pts1 = np.asarray(points1, dtype=np.float64).reshape(-1, 1, 2)
    pts2 = np.asarray(points2, dtype=np.float64).reshape(-1, 1, 2)

    u1 = cv2.undistortPoints(pts1, calibration["K1"], calibration["dist1"])
    u2 = cv2.undistortPoints(pts2, calibration["K2"], calibration["dist2"])

    p1 = np.hstack((np.eye(3), np.zeros((3, 1)))).astype(np.float64)
    p2 = np.hstack((calibration["R"], calibration["T"])).astype(np.float64)
    pts4d = cv2.triangulatePoints(p1, p2, u1.reshape(-1, 2).T, u2.reshape(-1, 2).T)
    pts3d = (pts4d[:3] / pts4d[3]).T
    cam2_pts = (calibration["R"] @ pts3d.T + calibration["T"]).T

    in_front = (pts3d[:, 2] > 0.0) & (cam2_pts[:, 2] > 0.0)
    finite = np.isfinite(pts3d).all(axis=1)
    plausible = np.linalg.norm(pts3d, axis=1) <= MAX_MARKER_DISTANCE_M
    keep = in_front & finite & plausible
    return pts3d, keep


def triangulate_aruco_markers(markers1, markers2, calibration, scale, alignment=None):
    payload_markers = []
    matched_ids = sorted(set(markers1.keys()) & set(markers2.keys()))

    for marker_id in matched_ids:
        corners1 = markers1[marker_id]
        corners2 = markers2[marker_id]
        points_3d, keep = triangulate_points(corners1, corners2, calibration)
        if len(points_3d) == 0 or not bool(np.all(keep)):
            continue

        center_3d = np.mean(points_3d, axis=0)
        if not np.isfinite(center_3d).all():
            continue

        coords = point_to_payload_coords(center_3d, scale, alignment)
        payload_markers.append({
            "id": int(marker_id),
            "tracked": True,
            "confidence": 1.0,
            **coords,
        })

    return payload_markers


def aruco_object_points(marker_side_m):
    half = float(marker_side_m) * 0.5
    return np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_single_marker_center(corners, camera_matrix, dist_coeffs, marker_side_m):
    if marker_side_m is None or marker_side_m <= 0:
        return None

    object_points = aruco_object_points(marker_side_m)
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    ok, _, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        ok, _, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    if not ok:
        return None

    center = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not np.isfinite(center).all() or np.linalg.norm(center) > MAX_MARKER_DISTANCE_M:
        return None
    return center


def visible_marker_centers_cam1(markers1, markers2, calibration, alignment):
    centers = {}
    marker_side_m = None if alignment is None else alignment.get("marker_side_m")

    for marker_id in sorted(set(markers1.keys()) & set(markers2.keys())):
        points_3d, keep = triangulate_points(markers1[marker_id], markers2[marker_id], calibration)
        if len(points_3d) == 4 and bool(np.all(keep)):
            center = np.mean(points_3d, axis=0)
            if np.isfinite(center).all():
                centers[int(marker_id)] = {
                    "position": center,
                    "source": "stereo",
                }

    for marker_id, corners in markers1.items():
        marker_id = int(marker_id)
        if marker_id in centers:
            continue
        center = estimate_single_marker_center(corners, calibration["K1"], calibration["dist1"], marker_side_m)
        if center is not None:
            centers[marker_id] = {
                "position": center,
                "source": "cam1",
            }

    for marker_id, corners in markers2.items():
        marker_id = int(marker_id)
        if marker_id in centers:
            continue
        center_cam2 = estimate_single_marker_center(corners, calibration["K2"], calibration["dist2"], marker_side_m)
        if center_cam2 is None:
            continue
        center_cam1 = calibration["R"].T @ (center_cam2 - calibration["T"].reshape(3))
        if np.isfinite(center_cam1).all() and np.linalg.norm(center_cam1) <= MAX_MARKER_DISTANCE_M:
            centers[marker_id] = {
                "position": center_cam1,
                "source": "cam2",
            }

    return centers


def rotation_between_vectors(source, target):
    source = np.asarray(source, dtype=np.float64).reshape(3)
    target = np.asarray(target, dtype=np.float64).reshape(3)
    source_norm = np.linalg.norm(source)
    target_norm = np.linalg.norm(target)
    if source_norm < 1e-9 or target_norm < 1e-9:
        return np.eye(3, dtype=np.float64)

    a = source / source_norm
    b = target / target_norm
    cross = np.cross(a, b)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    cross_norm = np.linalg.norm(cross)

    if cross_norm < 1e-9:
        if dot > 0.0:
            return np.eye(3, dtype=np.float64)
        axis = np.cross(a, np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(a, np.asarray([0.0, 1.0, 0.0], dtype=np.float64))
        axis /= max(np.linalg.norm(axis), 1e-9)
        return cv2.Rodrigues(axis * np.pi)[0]

    axis = cross / cross_norm
    angle = np.arctan2(cross_norm, dot)
    return cv2.Rodrigues(axis * angle)[0]


def estimate_rigid_observed_to_reference(observed_points, reference_points):
    observed = np.asarray(observed_points, dtype=np.float64).reshape(-1, 3)
    reference = np.asarray(reference_points, dtype=np.float64).reshape(-1, 3)
    count = len(observed)

    if count == 0:
        return None, None
    if count == 1:
        rotation = np.eye(3, dtype=np.float64)
        translation = reference[0] - observed[0]
        return rotation, translation
    if count == 2:
        observed_center = np.mean(observed, axis=0)
        reference_center = np.mean(reference, axis=0)
        rotation = rotation_between_vectors(observed[1] - observed[0], reference[1] - reference[0])
        translation = reference_center - rotation @ observed_center
        return rotation, translation

    observed_center = np.mean(observed, axis=0)
    reference_center = np.mean(reference, axis=0)
    observed_zero = observed - observed_center
    reference_zero = reference - reference_center
    covariance = observed_zero.T @ reference_zero
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = reference_center - rotation @ observed_center
    return rotation, translation


def rotation_angle_degrees(rotation):
    rvec, _ = cv2.Rodrigues(np.asarray(rotation, dtype=np.float64))
    return float(np.linalg.norm(rvec) * 180.0 / np.pi)


def smooth_anchor_transform(anchor_state, rotation, translation, alpha):
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if anchor_state.get("rotation") is None or anchor_state.get("translation") is None:
        anchor_state["rotation"] = rotation
        anchor_state["translation"] = translation
        return rotation, translation

    old_rotation = anchor_state["rotation"]
    old_translation = anchor_state["translation"]
    delta_rotation = rotation @ old_rotation.T
    delta_rvec, _ = cv2.Rodrigues(delta_rotation)
    smoothed_delta = cv2.Rodrigues(delta_rvec.reshape(3) * alpha)[0]
    smoothed_rotation = smoothed_delta @ old_rotation
    smoothed_translation = (1.0 - alpha) * old_translation + alpha * translation

    anchor_state["rotation"] = smoothed_rotation
    anchor_state["translation"] = smoothed_translation
    return smoothed_rotation, smoothed_translation


def marker_anchor_alignment(
    alignment,
    markers1,
    markers2,
    calibration,
    anchor_state,
    enabled=True,
    smoothing=MARKER_ANCHOR_SMOOTHING,
    max_translation_m=MARKER_ANCHOR_MAX_TRANSLATION_M,
    max_rotation_deg=MARKER_ANCHOR_MAX_ROTATION_DEG,
):
    if not enabled or alignment is None or not alignment.get("reference_markers_cam1"):
        return alignment, {"used": 0, "visible": 0, "source": "disabled"}

    visible = visible_marker_centers_cam1(markers1, markers2, calibration, alignment)
    observed = []
    reference = []
    used_ids = []

    for marker_id, marker in sorted(visible.items()):
        reference_point = alignment["reference_markers_cam1"].get(marker_id)
        if reference_point is None:
            continue
        observed.append(marker["position"])
        reference.append(reference_point)
        used_ids.append(marker_id)

    if not observed:
        return alignment, {"used": 0, "visible": len(visible), "source": "no_reference_match"}

    rotation, translation = estimate_rigid_observed_to_reference(observed, reference)
    if rotation is None:
        return alignment, {"used": 0, "visible": len(visible), "source": "fit_failed"}

    translation_m = float(np.linalg.norm(translation))
    rotation_deg = rotation_angle_degrees(rotation)
    if translation_m > max_translation_m or rotation_deg > max_rotation_deg:
        return alignment, {
            "used": 0,
            "visible": len(visible),
            "source": "rejected",
            "translation_m": translation_m,
            "rotation_deg": rotation_deg,
        }

    rotation, translation = smooth_anchor_transform(anchor_state, rotation, translation, smoothing)
    anchored = dict(alignment)
    anchored["marker_anchor_rotation_cv"] = rotation
    anchored["marker_anchor_translation_cv"] = translation
    return anchored, {
        "used": len(used_ids),
        "visible": len(visible),
        "ids": used_ids,
        "source": "visible_markers",
        "translation_m": float(np.linalg.norm(translation)),
        "rotation_deg": rotation_angle_degrees(rotation),
    }


def make_camera_payloads(calibration, scale, alignment=None):
    if alignment is not None and alignment.get("cameras"):
        payloads = []
        for camera_id, camera_name in ((1, "cam1"), (2, "cam2")):
            camera = alignment["cameras"].get(camera_name)
            if not camera or not camera.get("success", True):
                continue
            position = np.asarray(camera["position"], dtype=np.float64)
            if "rotation_matrix" in camera:
                matrix = np.asarray(camera["rotation_matrix"], dtype=np.float64)
                right = matrix[:, 0]
                up = matrix[:, 1]
                forward = matrix[:, 2]
            else:
                right = np.asarray(camera["right"], dtype=np.float64)
                up = np.asarray(camera["up"], dtype=np.float64)
                forward = np.asarray(camera["forward"], dtype=np.float64)
            payloads.append({
                "id": camera_id,
                "name": camera_name,
                "tracked": True,
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
                "hasOrientation": True,
                "rightX": float(right[0]),
                "rightY": float(right[1]),
                "rightZ": float(right[2]),
                "upX": float(up[0]),
                "upY": float(up[1]),
                "upZ": float(up[2]),
                "forwardX": float(forward[0]),
                "forwardY": float(forward[1]),
                "forwardZ": float(forward[2]),
            })
        return payloads

    cam1_cv = np.zeros(3, dtype=np.float64)
    cam2_cv = -(calibration["R"].T @ calibration["T"]).reshape(3)
    cam1_orientation = camera_orientation_payload(np.eye(3, dtype=np.float64))
    cam2_orientation = camera_orientation_payload(calibration["R"].T)
    return [
        {
            "id": 1,
            "name": "cam1",
            "tracked": True,
            **opencv_to_unity(cam1_cv, scale),
            **cam1_orientation,
        },
        {
            "id": 2,
            "name": "cam2",
            "tracked": True,
            **opencv_to_unity(cam2_cv, scale),
            **cam2_orientation,
        },
    ]


# ============================================================
# PAYLOAD
# ============================================================
def opencv_to_unity(point_m, scale):
    return {
        "x": float(point_m[0] * scale),
        "y": float(-point_m[1] * scale),
        "z": float(point_m[2] * scale),
    }


def opencv_cam1_to_unity_world(point_m, alignment):
    point = np.asarray(point_m, dtype=np.float64).reshape(3)
    if "marker_anchor_rotation_cv" in alignment and "marker_anchor_translation_cv" in alignment:
        point = alignment["marker_anchor_rotation_cv"] @ point + alignment["marker_anchor_translation_cv"]

    if alignment.get("mode") == "similarity":
        rotation = alignment["rotation_real_cam1_unity_to_unity_world"]
        translation = alignment["translation_real_cam1_unity_to_unity_world"]
        world_unity = alignment["scale_real_m_to_unity"] * (rotation @ (CV_FROM_UNITY @ point)) + translation
        return {
            "x": float(world_unity[0]),
            "y": float(world_unity[1]),
            "z": float(world_unity[2]),
        }

    rotation = alignment["cam1_world_cv_to_camera_cv_rotation"]
    translation = alignment["cam1_world_cv_to_camera_cv_translation"]
    real_to_unity_scale = alignment["real_to_unity_scale"]
    point_cv_units = point * real_to_unity_scale
    world_cv = rotation.T @ (point_cv_units - translation)
    world_unity = CV_FROM_UNITY @ world_cv
    return {
        "x": float(world_unity[0]),
        "y": float(world_unity[1]),
        "z": float(world_unity[2]),
    }


def point_to_payload_coords(point_m, scale, alignment=None):
    if alignment is not None:
        return opencv_cam1_to_unity_world(point_m, alignment)
    return opencv_to_unity(point_m, scale)


def load_unity_alignment(path):
    if not path:
        return None

    path = Path(path)
    if not path.exists():
        return None

    data = json.loads(path.read_text())
    if "scale_real_m_to_unity" in data:
        scale_real_m_to_unity = float(data["scale_real_m_to_unity"])
        rotation_real_cam1_unity_to_unity_world = np.asarray(
            data["rotation_real_cam1_unity_to_unity_world"],
            dtype=np.float64,
        )
        translation_real_cam1_unity_to_unity_world = np.asarray(
            data["translation_real_cam1_unity_to_unity_world"],
            dtype=np.float64,
        )
        reference_markers_cam1 = {}
        for marker_id, marker_data in data.get("markers", {}).items():
            unity_position = marker_data.get("fitted_unity_position") or marker_data.get("current_unity_position")
            if unity_position is None:
                continue
            unity_position = np.asarray(unity_position, dtype=np.float64).reshape(3)
            cam1_unity = rotation_real_cam1_unity_to_unity_world.T @ (
                (unity_position - translation_real_cam1_unity_to_unity_world) / scale_real_m_to_unity
            )
            reference_markers_cam1[int(marker_id)] = CV_FROM_UNITY @ cam1_unity

        return {
            "mode": "similarity",
            "path": str(path),
            "scale_real_m_to_unity": scale_real_m_to_unity,
            "real_to_unity_scale": scale_real_m_to_unity,
            "rotation_real_cam1_unity_to_unity_world": rotation_real_cam1_unity_to_unity_world,
            "translation_real_cam1_unity_to_unity_world": translation_real_cam1_unity_to_unity_world,
            "marker_side_m": data.get("marker_side_m_estimated"),
            "reference_markers_cam1": reference_markers_cam1,
            "cameras": data.get("cameras", {}),
        }

    cam1 = data.get("cameras", {}).get("cam1", {})
    if not cam1.get("success"):
        raise RuntimeError(f"Unity camera pose file has no successful cam1 solve: {path}")

    real_to_unity_scale = data.get("real_to_unity_scale")
    if real_to_unity_scale is None or real_to_unity_scale <= 0:
        raise RuntimeError(f"Unity camera pose file has invalid real_to_unity_scale: {path}")

    return {
        "mode": "pnp",
        "path": str(path),
        "real_to_unity_scale": float(real_to_unity_scale),
        "cam1_world_cv_to_camera_cv_rotation": np.asarray(
            cam1["world_cv_to_camera_cv_rotation"],
            dtype=np.float64,
        ),
        "cam1_world_cv_to_camera_cv_translation": np.asarray(
            cam1["world_cv_to_camera_cv_translation"],
            dtype=np.float64,
        ),
        "marker_side_m": data.get("marker_side_m_estimated"),
        "reference_markers_cam1": {},
        "cameras": data.get("cameras", {}),
    }


def opencv_direction_to_unity(direction):
    direction = np.asarray(direction, dtype=np.float64).reshape(3)
    return {
        "x": float(direction[0]),
        "y": float(-direction[1]),
        "z": float(direction[2]),
    }


def camera_orientation_payload(rotation_cam_to_cam1_unity):
    right = opencv_direction_to_unity(rotation_cam_to_cam1_unity[:, 0])
    up = opencv_direction_to_unity(rotation_cam_to_cam1_unity[:, 1])
    forward = opencv_direction_to_unity(rotation_cam_to_cam1_unity[:, 2])
    return {
        "hasOrientation": True,
        "rightX": right["x"],
        "rightY": right["y"],
        "rightZ": right["z"],
        "upX": up["x"],
        "upY": up["y"],
        "upZ": up["z"],
        "forwardX": forward["x"],
        "forwardY": forward["y"],
        "forwardZ": forward["z"],
    }


def pose_confidence(pose, joint_id):
    if pose is None:
        return 0.0
    return float(pose["confidence"][joint_id])


def pose_score(pose):
    return 0.0 if pose is None else float(pose["person_score"])


def make_payload(
    frame_index,
    joints_3d,
    valid,
    pose1,
    pose2,
    scale,
    aruco_markers=None,
    cameras=None,
    people=None,
    alignment=None,
    filter_config=None,
):
    joints = []
    for joint_id, name in enumerate(COCO_KEYPOINT_NAMES):
        tracked    = bool(valid[joint_id] and np.isfinite(joints_3d[joint_id]).all())
        confidence = float(min(pose_confidence(pose1, joint_id), pose_confidence(pose2, joint_id)))
        coords     = point_to_payload_coords(joints_3d[joint_id], scale, alignment) if tracked else {"x": 0.0, "y": 0.0, "z": 0.0}
        joints.append({"id": joint_id, "name": name, "tracked": tracked, "confidence": confidence, **coords})

    if filter_config is not None:
        smooth_payload_joints(0, joints, frame_index, **filter_config)

    return {
        "type":              "pose3d",
        "frame":             frame_index,
        "timestamp":         time.time(),
        "units":             "meters",
        "coordinate_space":  "unity_world" if alignment is not None else "unity_from_cam1",
        "person_score_cam1": pose_score(pose1),
        "person_score_cam2": pose_score(pose2),
        "aruco_markers":     aruco_markers or [],
        "cameras":           cameras or [],
        "people":            people or [],
        "joints":            joints,
    }


class StopGoCommandSender:
    def __init__(self, host, port):
        self.address = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, command):
        if command not in ("stop", "go"):
            return False

        self.sock.sendto(command.encode("utf-8"), self.address)
        print(f"[GESTURE] forwarded '{command}' to Unity {self.address[0]}:{self.address[1]}")
        return True

    def close(self):
        self.sock.close()


def _valid_pose_keypoint(pose, idx, min_conf=MIN_KEYPOINT_CONF):
    if pose is None:
        return False
    points = pose.get("points")
    confidence = pose.get("confidence")
    return (
        points is not None and
        confidence is not None and
        idx < len(points) and
        idx < len(confidence) and
        confidence[idx] >= min_conf
    )


def detect_hands_up_pose(pose):
    required = [5, 6, 9, 10]  # shoulders, wrists
    if not all(_valid_pose_keypoint(pose, idx) for idx in required):
        return False

    points = pose["points"]
    left_shoulder = np.asarray(points[5], dtype=np.float32)
    right_shoulder = np.asarray(points[6], dtype=np.float32)
    left_wrist = np.asarray(points[9], dtype=np.float32)
    right_wrist = np.asarray(points[10], dtype=np.float32)

    shoulder_width = float(np.linalg.norm(left_shoulder - right_shoulder))
    margin = max(HANDS_UP_MIN_MARGIN_PX, shoulder_width * HANDS_UP_MARGIN_RATIO)
    required_y = min(float(left_shoulder[1]), float(right_shoulder[1])) - margin

    return float(left_wrist[1]) < required_y and float(right_wrist[1]) < required_y


def detect_tpose_pose(pose):
    required = [5, 6, 7, 8, 9, 10]  # shoulders, elbows, wrists
    if not all(_valid_pose_keypoint(pose, idx) for idx in required):
        return False

    points = pose["points"]
    left_shoulder = np.asarray(points[5], dtype=np.float32)
    right_shoulder = np.asarray(points[6], dtype=np.float32)
    left_elbow = np.asarray(points[7], dtype=np.float32)
    right_elbow = np.asarray(points[8], dtype=np.float32)
    left_wrist = np.asarray(points[9], dtype=np.float32)
    right_wrist = np.asarray(points[10], dtype=np.float32)

    shoulder_width = float(np.linalg.norm(left_shoulder - right_shoulder))
    if shoulder_width < 1.0:
        return False

    shoulder_y = (float(left_shoulder[1]) + float(right_shoulder[1])) * 0.5
    max_vertical_offset = shoulder_width * TPOSE_VERTICAL_TOLERANCE_RATIO
    arms_level = (
        abs(float(left_elbow[1]) - shoulder_y) <= max_vertical_offset and
        abs(float(right_elbow[1]) - shoulder_y) <= max_vertical_offset and
        abs(float(left_wrist[1]) - shoulder_y) <= max_vertical_offset and
        abs(float(right_wrist[1]) - shoulder_y) <= max_vertical_offset
    )

    wrist_span = abs(float(left_wrist[0]) - float(right_wrist[0]))
    wrists_outside_shoulders = (
        min(float(left_wrist[0]), float(right_wrist[0])) <
        min(float(left_shoulder[0]), float(right_shoulder[0])) and
        max(float(left_wrist[0]), float(right_wrist[0])) >
        max(float(left_shoulder[0]), float(right_shoulder[0]))
    )

    return arms_level and wrists_outside_shoulders and wrist_span >= shoulder_width * TPOSE_MIN_SPAN_RATIO


def detect_gesture_command(poses1, poses2):
    poses = list(poses1 or []) + list(poses2 or [])
    if any(detect_hands_up_pose(pose) for pose in poses):
        return "stop"
    if any(detect_tpose_pose(pose) for pose in poses):
        return "go"
    return None


class GestureCommandState:
    def __init__(self, command_sender):
        self.command_sender = command_sender
        self.pending_command = None
        self.pending_since = 0.0
        self.last_sent_command = None
        self.last_send_time = 0.0

    def update(self, command):
        now = time.time()

        if command is None:
            self.pending_command = None
            return None

        if command != self.pending_command:
            self.pending_command = command
            self.pending_since = now
            return None

        if now - self.pending_since < GESTURE_HOLD_SEC:
            return None

        if command == self.last_sent_command and now - self.last_send_time < GESTURE_COOLDOWN_SEC:
            return None

        if self.command_sender.send(command):
            self.last_sent_command = command
            self.last_send_time = now
            return command

        return None


class GestureCommandReceiver:
    def __init__(self, host, port):
        self.address = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.address)
        self.sock.setblocking(False)

    def poll(self):
        latest_command = None
        while True:
            try:
                data, _ = self.sock.recvfrom(128)
            except BlockingIOError:
                break

            command = data.decode("utf-8", errors="ignore").strip().lower()
            if command in ("stop", "go"):
                latest_command = command

        return latest_command

    def close(self):
        self.sock.close()


class GestureCommandBridge:
    def __init__(self, host, port, command_sender):
        self.address = (host, port)
        self.command_sender = command_sender
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.address)
        self.sock.settimeout(0.1)
        self.running = False
        self.thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(128)
            except socket.timeout:
                continue
            except OSError:
                break

            command = data.decode("utf-8", errors="ignore").strip().lower()
            if command in ("stop", "go"):
                self.command_sender.send(command)

    def close(self):
        self.running = False
        try:
            self.sock.close()
        finally:
            if self.thread is not None:
                self.thread.join(timeout=0.5)


# ============================================================
# DISPLAY HELPERS
# ============================================================
def resize_for_processing(frame, target_width):
    if target_width <= 0:
        return frame
    h, w = frame.shape[:2]
    if w == target_width:
        return frame
    scale = target_width / float(w)
    return cv2.resize(frame, (target_width, int(h * scale)), interpolation=cv2.INTER_AREA)


def resize_to_width(frame, width):
    h, w  = frame.shape[:2]
    scale = width / float(w)
    return cv2.resize(frame, (width, int(h * scale)), interpolation=cv2.INTER_AREA)


def draw_pose(frame, pose, color):
    vis = frame.copy()
    if pose is None:
        return vis
    points = pose["points"]
    conf   = pose["confidence"]
    for joint_id, point in enumerate(points):
        if conf[joint_id] >= MIN_KEYPOINT_CONF:
            cv2.circle(vis, tuple(point.astype(int)), 4, color, -1)
    for a, b in SKELETON:
        if conf[a] >= MIN_KEYPOINT_CONF and conf[b] >= MIN_KEYPOINT_CONF:
            cv2.line(vis, tuple(points[a].astype(int)), tuple(points[b].astype(int)), color, 2)
    return vis


def build_display_frame(
    frame1, frame2, pose1, pose2, display_width, fps, tracked,
    host, port, aruco1=None, aruco2=None, gesture_command=None
):
    vis1     = draw_pose(frame1, pose1, (0, 255, 255))
    vis2     = draw_pose(frame2, pose2, (0, 255, 255))
    if aruco1 is not None:
        corners1, ids1 = aruco1
        if ids1 is not None and corners1 is not None:
            cv2.aruco.drawDetectedMarkers(vis1, corners1, ids1)
    if aruco2 is not None:
        corners2, ids2 = aruco2
        if ids2 is not None and corners2 is not None:
            cv2.aruco.drawDetectedMarkers(vis2, corners2, ids2)
    vis1     = resize_to_width(vis1, display_width)
    vis2     = resize_to_width(vis2, display_width)
    h        = max(vis1.shape[0], vis2.shape[0])
    vis1     = cv2.resize(vis1, (vis1.shape[1], h))
    vis2     = cv2.resize(vis2, (vis2.shape[1], h))
    combined = np.hstack((vis1, vis2))
    cv2.putText(
        combined,
        f"UDP {host}:{port} | tracked: {tracked} | FPS: {fps:.1f}",
        (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA,
    )
    cv2.putText(
        combined,
        f"Gesture: {gesture_command if gesture_command is not None else 'none'}",
        (20, combined.shape[0] - 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255) if gesture_command == "stop" else ((0, 255, 0) if gesture_command == "go" else (255, 255, 255)),
        2,
        cv2.LINE_AA,
    )
    return combined


# ============================================================
# ARGUMENTS
# ============================================================
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration",       default=CALIBRATION_FILE)
    parser.add_argument("--model",             default=YOLO_MODEL_PATH)
    parser.add_argument("--cam1",              default=CAM1_LIVE)
    parser.add_argument("--cam2",              default=CAM2_LIVE)
    parser.add_argument("--host",              default=UNITY_HOST)
    parser.add_argument("--port",              type=int,   default=UNITY_PORT)
    parser.add_argument("--command-host",      default=None)
    parser.add_argument("--command-port",      type=int,   default=UNITY_COMMAND_PORT)
    parser.add_argument("--gesture-bridge-host", default=GESTURE_BRIDGE_HOST)
    parser.add_argument("--gesture-bridge-port", type=int, default=GESTURE_BRIDGE_PORT)
    parser.add_argument("--disable-local-gestures", action="store_true")
    parser.add_argument("--disable-gesture-bridge", action="store_true")
    parser.add_argument("--bridge-only", action="store_true")
    parser.add_argument("--device",            choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--send-hz",           type=float, default=30.0)
    parser.add_argument("--process-width",     type=int,   default=PROCESS_WIDTH)
    parser.add_argument("--min-keypoint-conf", type=float, default=MIN_KEYPOINT_CONF)
    parser.add_argument("--unity-scale",       type=float, default=1.0)
    parser.add_argument("--display-width",     type=int,   default=520)
    parser.add_argument("--live-width",        type=int,   default=LIVE_IMAGE_SIZE[0])
    parser.add_argument("--live-height",       type=int,   default=LIVE_IMAGE_SIZE[1])
    parser.add_argument("--transport",         choices=["tcp", "udp"], default=FFMPEG_TRANSPORT)
    parser.add_argument("--ffmpeg-exe",        default=FFMPEG_EXE)
    parser.add_argument(
        "--unity-camera-poses",
        default=str(SCRIPT_DIR / "real_marker_layout_fit.json"),
        help="Static high-resolution ArUco layout/camera solve. When present, sends Unity-world coordinates.",
    )
    parser.add_argument(
        "--no-marker-anchor",
        action="store_true",
        help="Disable runtime visible-ArUco correction for human joints.",
    )
    parser.add_argument(
        "--marker-anchor-smoothing",
        type=float,
        default=MARKER_ANCHOR_SMOOTHING,
        help="0..1 smoothing for the runtime visible-marker correction. Higher follows markers faster.",
    )
    parser.add_argument(
        "--marker-anchor-max-translation-m",
        type=float,
        default=MARKER_ANCHOR_MAX_TRANSLATION_M,
        help="Reject a visible-marker correction above this translation magnitude.",
    )
    parser.add_argument(
        "--marker-anchor-max-rotation-deg",
        type=float,
        default=MARKER_ANCHOR_MAX_ROTATION_DEG,
        help="Reject a visible-marker correction above this rotation magnitude.",
    )
    parser.add_argument(
        "--no-joint-filter",
        action="store_true",
        help="Disable temporal filtering of outgoing avatar joints.",
    )
    parser.add_argument(
        "--joint-filter-alpha",
        type=float,
        default=JOINT_FILTER_ALPHA,
        help="Base smoothing alpha for small joint movements. Lower is steadier.",
    )
    parser.add_argument(
        "--joint-filter-fast-alpha",
        type=float,
        default=JOINT_FILTER_FAST_ALPHA,
        help="Smoothing alpha for larger intentional movements.",
    )
    parser.add_argument(
        "--joint-filter-fast-distance",
        type=float,
        default=JOINT_FILTER_FAST_DISTANCE,
        help="Unity-world distance where the joint filter reaches fast alpha.",
    )
    parser.add_argument(
        "--joint-filter-deadband",
        type=float,
        default=JOINT_FILTER_DEADBAND,
        help="Unity-world movement ignored as camera/triangulation noise.",
    )
    parser.add_argument(
        "--joint-filter-snap-distance",
        type=float,
        default=JOINT_FILTER_SNAP_DISTANCE,
        help="Unity-world movement that bypasses smoothing to avoid lag after large real motion.",
    )
    parser.add_argument(
        "--joint-filter-hold-frames",
        type=int,
        default=JOINT_FILTER_HOLD_FRAMES,
        help="Hold last filtered joint for this many missed frames.",
    )
    parser.add_argument("--opencv-capture",    action="store_true")
    parser.add_argument("--no-display",        action="store_true")
    parser.add_argument("--metrics-dir", default=str(SCRIPT_DIR / "metrics"))
    parser.add_argument("--no-metrics", action="store_true")
    parser.add_argument("--metrics-summary-sec", type=float, default=5.0)
    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================
def main():
    args = parse_args()

    command_host = args.command_host or args.host
    if args.bridge_only:
        command_sender = StopGoCommandSender(command_host, args.command_port)
        gesture_bridge = GestureCommandBridge(
            args.gesture_bridge_host,
            args.gesture_bridge_port,
            command_sender,
        )
        gesture_bridge.start()
        print(f"Bridge only: listening on {args.gesture_bridge_host}:{args.gesture_bridge_port}")
        print(f"Bridge only: forwarding commands to Unity {command_host}:{args.command_port}")
        print("Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            gesture_bridge.close()
            command_sender.close()
        return

    live_size = (args.live_width, args.live_height)
    use_ffmpeg = not args.opencv_capture
    metrics = MetricsSession(
        args.metrics_dir,
        prefix="pose",
        summary_interval_sec=args.metrics_summary_sec,
        enabled=not args.no_metrics,
    )
    atexit.register(metrics.close)
    if metrics.enabled:
        print(f"Metrics -> {metrics.session_dir}")

    print(
        f"send_hz={args.send_hz}  process_width={args.process_width}  "
        f"live_size={live_size[0]}x{live_size[1]}  ffmpeg={use_ffmpeg}"
    )

    calibration = load_calibration(args.calibration)
    print(f"Calibration: {args.calibration}")
    print(f"Calibration size: {calibration['image_size'][0]}x{calibration['image_size'][1]}")
    unity_alignment = load_unity_alignment(args.unity_camera_poses)
    if unity_alignment is not None:
        print(
            "Unity static camera alignment: "
            f"{unity_alignment['path']}  scale={unity_alignment['real_to_unity_scale']:.6f}"
        )

    if "-pose" not in str(args.model):
        print(f"WARNING: {args.model} is not a pose model. Use a '*-pose.pt' checkpoint to get joints.")

    model, model_device, use_half = load_model(args.model, args.device)

    print("Warming up YOLO...")
    dummy = np.zeros((256, 256, 3), dtype=np.uint8)
    model([dummy, dummy], verbose=False, imgsz=YOLO_IMGSZ)
    print("Warmup done.")

    aruco_detector = build_aruco_detector()

    sock          = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    unity_address = (args.host, args.port)
    print(f"UDP -> {args.host}:{args.port}")
    command_sender = StopGoCommandSender(command_host, args.command_port)
    gesture_state = None
    if not args.disable_local_gestures:
        gesture_state = GestureCommandState(command_sender)
        print("Local gestures: hands-up=stop, T-pose=go")

    gesture_bridge = None
    if not args.disable_gesture_bridge:
        gesture_bridge = GestureCommandBridge(
            args.gesture_bridge_host,
            args.gesture_bridge_port,
            command_sender,
        )
        gesture_bridge.start()
        print(f"Gesture bridge <- {args.gesture_bridge_host}:{args.gesture_bridge_port}")
        print(f"Gesture commands -> Unity {command_host}:{args.command_port}")

    print("Opening streams...")
    cam1 = CamReader(
        args.cam1,
        "Cam1",
        use_ffmpeg=use_ffmpeg,
        ffmpeg_exe=args.ffmpeg_exe,
        transport=args.transport,
        frame_size=live_size,
    )
    time.sleep(1.0)
    cam2 = CamReader(
        args.cam2,
        "Cam2",
        use_ffmpeg=use_ffmpeg,
        ffmpeg_exe=args.ffmpeg_exe,
        transport=args.transport,
        frame_size=live_size,
    )
    time.sleep(1.0)

    if not cam1.is_opened() or not cam2.is_opened():
        cam1.release()
        cam2.release()
        raise RuntimeError("Could not open one or both RTSP streams.")

    display_thread = None
    if not args.no_display:
        display_thread = DisplayThread("3D Pose -> Unity", args.display_width)

    frame_interval     = 1.0 / max(args.send_hz, 0.1)
    next_send_time     = 0.0
    frame_index        = 0
    active_calibration = calibration
    active_frame_size  = None
    last_payload       = None
    marker_anchor_state = {
        "rotation": None,
        "translation": None,
    }
    joint_filter_config = None
    if not args.no_joint_filter:
        joint_filter_config = {
            "enabled": True,
            "alpha": args.joint_filter_alpha,
            "fast_alpha": args.joint_filter_fast_alpha,
            "fast_distance": args.joint_filter_fast_distance,
            "deadband": args.joint_filter_deadband,
            "snap_distance": args.joint_filter_snap_distance,
            "hold_frames": args.joint_filter_hold_frames,
        }
    prev_time          = time.perf_counter()
    previous_cam1_sequence = None
    previous_cam2_sequence = None

    print(f"Running at {args.send_hz} Hz. Press ESC or Q to quit.")

    try:
        while True:
            if display_thread is not None and display_thread.quit_requested:
                break

            now = time.perf_counter()
            if now < next_send_time:
                time.sleep(0.0005)
                continue

            # Read freshest frame directly — no draining needed at GPU speeds
            frame_timer = metrics.new_frame(frame_index + 1)
            ok1, frame1_raw, cam1_meta = cam1.read_with_metadata()
            ok2, frame2_raw, cam2_meta = cam2.read_with_metadata()
            frame_timer.mark("camera_read_ms")

            if not (ok1 and ok2 and frame1_raw is not None and frame2_raw is not None):
                time.sleep(0.005)
                continue

            next_send_time = now + frame_interval
            frame_index   += 1

            sample_ns = time.perf_counter_ns()
            cam1_sequence = int(cam1_meta["sequence"])
            cam2_sequence = int(cam2_meta["sequence"])
            frame_timer.set(
                cam1_sequence=cam1_sequence,
                cam2_sequence=cam2_sequence,
                cam1_sequence_gap=0 if previous_cam1_sequence is None else max(cam1_sequence - previous_cam1_sequence - 1, 0),
                cam2_sequence_gap=0 if previous_cam2_sequence is None else max(cam2_sequence - previous_cam2_sequence - 1, 0),
                camera_duplicate_1=int(previous_cam1_sequence == cam1_sequence),
                camera_duplicate_2=int(previous_cam2_sequence == cam2_sequence),
                cam1_frame_age_ms=(sample_ns - int(cam1_meta["received_ns"])) / 1_000_000.0,
                cam2_frame_age_ms=(sample_ns - int(cam2_meta["received_ns"])) / 1_000_000.0,
                camera_decode_skew_ms=abs(int(cam1_meta["received_ns"]) - int(cam2_meta["received_ns"])) / 1_000_000.0,
            )
            previous_cam1_sequence = cam1_sequence
            previous_cam2_sequence = cam2_sequence

            frame1 = resize_for_processing(frame1_raw, args.process_width)
            frame2 = resize_for_processing(frame2_raw, args.process_width)
            frame_timer.mark("resize_ms")

            size1 = (frame1.shape[1], frame1.shape[0])
            size2 = (frame2.shape[1], frame2.shape[0])

            if size1 != size2:
                print(f"Size mismatch {size1} vs {size2}, skipping")
                continue

            if active_frame_size != size1:
                active_calibration = calibration_for_frame_size(calibration, size1)
                active_frame_size  = size1
                print(f"Processing size: {size1[0]}x{size1[1]}")

            # Batched inference — single GPU forward pass for both cameras
            if model_device == "cuda":
                torch.cuda.synchronize()
            poses1, poses2 = run_pose_batch_multi(model, frame1, frame2, use_half)
            if model_device == "cuda":
                torch.cuda.synchronize()
            frame_timer.mark("inference_ms")
            matches = match_pose_lists(poses1, poses2, active_calibration, args.min_keypoint_conf)
            frame_timer.mark("stereo_match_ms")
            frame_timer.set(**stereo_quality_metrics(matches, active_calibration, args.min_keypoint_conf))
            frame_timer.mark("quality_metrics_ms")

            markers1, aruco_corners1, aruco_ids1 = detect_aruco_markers(frame1, aruco_detector)
            markers2, aruco_corners2, aruco_ids2 = detect_aruco_markers(frame2, aruco_detector)
            frame_timer.mark("aruco_detection_ms")
            gesture_command = detect_gesture_command(poses1, poses2)
            sent_gesture_command = (
                gesture_state.update(gesture_command)
                if gesture_state is not None
                else None
            )
            frame_timer.mark("gesture_ms")
            frame_alignment, marker_anchor_info = marker_anchor_alignment(
                unity_alignment,
                markers1,
                markers2,
                active_calibration,
                marker_anchor_state,
                enabled=not args.no_marker_anchor,
                smoothing=args.marker_anchor_smoothing,
                max_translation_m=args.marker_anchor_max_translation_m,
                max_rotation_deg=args.marker_anchor_max_rotation_deg,
            )
            frame_timer.mark("marker_anchor_ms")
            aruco_markers = triangulate_aruco_markers(
                markers1, markers2, active_calibration, args.unity_scale, frame_alignment
            )
            camera_payloads = make_camera_payloads(active_calibration, args.unity_scale, unity_alignment)
            frame_timer.mark("aruco_triangulation_ms")

            joints_3d = np.full((len(COCO_KEYPOINT_NAMES), 3), np.nan, dtype=np.float64)
            valid = np.zeros(len(COCO_KEYPOINT_NAMES), dtype=bool)
            pose1 = None
            pose2 = None

            people, best_person = build_people_payload(
                matches,
                active_calibration,
                args.min_keypoint_conf,
                args.unity_scale,
                frame_index,
                frame_alignment,
                filter_config=joint_filter_config,
            )
            if best_person is not None:
                joints_3d, valid, pose1, pose2 = best_person
            frame_timer.mark("people_triangulation_filter_ms")

            payload = make_payload(
                frame_index,
                joints_3d,
                valid,
                pose1,
                pose2,
                args.unity_scale,
                aruco_markers=aruco_markers,
                cameras=camera_payloads,
                people=people,
                alignment=frame_alignment,
                filter_config=joint_filter_config,
            )
            prune_joint_filter_states(frame_index)
            payload["gesture_command"] = gesture_command
            payload["sent_gesture_command"] = sent_gesture_command
            tracked_count = sum(1 for j in payload["joints"] if j["tracked"])
            tracked_confidences = [j["confidence"] for j in payload["joints"] if j["tracked"]]
            frame_timer.set(
                persons_cam1=len(poses1), persons_cam2=len(poses2), matched_people=len(matches),
                output_people=len(people), tracked_joints=tracked_count,
                tracked_joint_ratio=tracked_count / float(len(COCO_KEYPOINT_NAMES)),
                mean_joint_confidence=mean_or_none(tracked_confidences),
                aruco_cam1=len(markers1), aruco_cam2=len(markers2), aruco_output=len(aruco_markers),
                anchor_visible=marker_anchor_info["visible"], anchor_used=marker_anchor_info["used"],
                gesture_detected=gesture_command or "", gesture_sent=sent_gesture_command or "",
            )
            frame_timer.mark("payload_build_ms")
            payload["metrics"] = metrics.payload_metrics(frame_timer.values)
            message = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            frame_timer.mark("json_serialize_ms")
            sock.sendto(message, unity_address)
            frame_timer.mark("udp_send_ms")
            frame_timer.set(packet_bytes=len(message))
            last_payload  = payload
            person_count = len(people)

            fps       = 1.0 / max(now - prev_time, 1e-9)
            prev_time = now

            print(
                f"frame={frame_index:06d}  joints={tracked_count:02d}  "
                f"people={person_count:02d}  aruco={len(aruco_markers):02d}  "
                f"gesture={gesture_command or 'none':4s}  "
                f"anchor={marker_anchor_info['used']:01d}/{marker_anchor_info['visible']:01d}  "
                f"fps={fps:05.1f}   ",
                end="\r"
            )

            if display_thread is not None:
                tracked = 0 if last_payload is None else sum(
                    1 for j in last_payload["joints"] if j["tracked"]
                )
                combined = build_display_frame(
                    frame1, frame2, pose1, pose2,
                    args.display_width, fps, tracked, args.host, args.port,
                    aruco1=(aruco_corners1, aruco_ids1),
                    aruco2=(aruco_corners2, aruco_ids2),
                    gesture_command=gesture_command,
                )
                display_thread.push(combined)
                frame_timer.mark("display_build_ms")

            metrics.record(frame_timer.finish(), torch_module=torch)

    finally:
        print()
        cam1.release()
        cam2.release()
        sock.close()
        if gesture_bridge is not None:
            gesture_bridge.close()
        command_sender.close()
        if display_thread is not None:
            display_thread.stop()
        metrics.close()


if __name__ == "__main__":
    main()
