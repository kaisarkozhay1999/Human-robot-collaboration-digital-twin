import cv2
import atexit
import json
import subprocess
import threading
import time
import numpy as np
import os
import socket
import torch
from pathlib import Path
from ultralytics import YOLO
from runtime_metrics import MetricsSession

SCRIPT_DIR = Path(__file__).resolve().parent

try:
    import imageio_ffmpeg
except ImportError:
    imageio_ffmpeg = None

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;udp"
    "|fflags;nobuffer+discardcorrupt"
    "|flags;low_delay"
    "|framedrop;1"
    "|analyzeduration;1000000"
    "|probesize;1048576"
    "|sync;ext"
    "|avioflags;direct"
)

# ============================================================
# STREAM URLS
# ============================================================
CAM1_HIGH = os.environ.get("SMARTLAB_CAM1_HIGH", "rtsp://CAMERA_1_IP/h264")
CAM1_LOW  = os.environ.get("SMARTLAB_CAM1_LOW", "rtsp://CAMERA_1_IP/mpeg4cif")

CAM2_HIGH = os.environ.get("SMARTLAB_CAM2_HIGH", "rtsp://CAMERA_2_IP/h264")
CAM2_LOW  = os.environ.get("SMARTLAB_CAM2_LOW", "rtsp://CAMERA_2_IP/mpeg4cif")
CAM1_LIVE = CAM1_HIGH
CAM2_LIVE = CAM2_HIGH

# ============================================================
# SETTINGS
# ============================================================
DISPLAY_WIDTH = 1280
HIGH_REFRESH_SEC = 60
ARUCO_DICT = cv2.aruco.DICT_4X4_50
CALIBRATION_IMAGE_SIZE = (2592, 1944)
LIVE_IMAGE_SIZE = (1280, 720)
CALIBRATION_PATHS = [
    Path("stereo_calibration_charuco_live.npz"),
    Path("stereo_calibration_charuco_refined.npz"),
    Path("stereo_calibration_charuco_low.npz"),
]

USE_FFMPEG_READER = True
FFMPEG_TRANSPORT = "tcp"
FFMPEG_EXE = os.environ.get("SMARTLAB_FFMPEG_EXE", "")
USE_HIGH_STREAMS = False
FFMPEG_STREAM_SIZES = {
    CAM1_LOW: (640, 360),
    CAM2_LOW: (640, 360),
    CAM1_HIGH: LIVE_IMAGE_SIZE,
    CAM2_HIGH: LIVE_IMAGE_SIZE,
}


def resolve_ffmpeg_exe():
    if FFMPEG_EXE:
        return FFMPEG_EXE

    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        winget_root = Path(local_app_data) / "Microsoft" / "WinGet" / "Packages"
        candidates = sorted(winget_root.glob("Gyan.FFmpeg*/*/bin/ffmpeg.exe"))
        if candidates:
            return str(candidates[-1])

    return imageio_ffmpeg.get_ffmpeg_exe() if imageio_ffmpeg is not None else "ffmpeg"

YOLO_MODEL_PATH = "yolo26n-pose.pt"
YOLO_IMGSZ = 480
PERSON_CONF = 0.35
KEYPOINT_CONF = 0.35

ROBOT_MODEL_PATH = Path("runs/detect/robot_detector/weights/best.pt")
ROBOT_IMGSZ = 480
ROBOT_CONF = 0.25
ROBOT_DETECT_INTERVAL = 2
ROBOT_TRACK_HOLD_FRAMES = 24
ROBOT_TRACK_SMOOTHING = 0.35

MAX_TRIANGULATED_DISTANCE_M = 20.0
MAX_EPIPOLAR_ERROR_PX = 320.0
UNITY_SAFETY_HOST = "127.0.0.1"
UNITY_SAFETY_PORT = 5006
POSE3D_GESTURE_HOST = "127.0.0.1"
POSE3D_GESTURE_PORT = 5010
SAFETY_STOP_DISTANCE_M = 0.75
SAFETY_RELEASE_DISTANCE_M = 0.90
SAFETY_STOP_DISTANCE_PX = 75.0
SAFETY_RELEASE_DISTANCE_PX = 110.0
SAFETY_SINGLE_CAMERA_OCCLUSION_STOP_PX = 60.0
SAFETY_SINGLE_CAMERA_OCCLUSION_RELEASE_PX = 95.0
SAFETY_COMMAND_INTERVAL_SEC = 0.25
GESTURE_HOLD_SEC = 0.30
GESTURE_COOLDOWN_SEC = 1.50
HANDS_UP_MARGIN_RATIO = 0.20
HANDS_UP_MIN_MARGIN_PX = 20.0
TPOSE_VERTICAL_TOLERANCE_RATIO = 0.30
TPOSE_MIN_SPAN_RATIO = 1.80

# Robot reference colors given by you, interpreted as RGB
ROBOT_RGB_COLORS = [
    (20, 45, 78),
    (8, 28, 62),
    (6, 22, 54),
    (3, 12, 38),
]
ROBOT_DRAW_COLOR = (0, 0, 255)  # Red in OpenCV BGR

# Color tolerance in HSV
H_TOL = 8
S_TOL = 80
V_TOL = 70

# Ignore tiny color blobs
MIN_ROBOT_AREA = 700

# COCO skeleton pairs
SKELETON = [
    (5, 7), (7, 9),
    (6, 8), (8, 10),
    (5, 6),
    (5, 11), (6, 12),
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16)
]

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


class CamReader:
    def __init__(self, rtsp_url, name="Camera"):
        self.name = name
        self.rtsp_url = rtsp_url
        self.use_ffmpeg = USE_FFMPEG_READER
        self.proc = None
        self.stderr_thread = None
        self.frame_size = None
        self.width = None
        self.height = None
        self.read_count = 0

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
        self.width, self.height = FFMPEG_STREAM_SIZES.get(self.rtsp_url, (640, 360))
        self.frame_size = self.width * self.height * 3
        ffmpeg_exe = resolve_ffmpeg_exe()

        cmd = [
            ffmpeg_exe,
            "-hide_banner",
            "-loglevel", "warning",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "1000000",
            "-probesize", "1048576",
            "-rtsp_transport", FFMPEG_TRANSPORT,
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
        print(f"{self.name}: FFmpeg reader {self.width}x{self.height} transport={FFMPEG_TRANSPORT}")

    def _start_opencv(self):
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ok, self.frame = self.cap.read()
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

    def _loop_opencv(self):
        while not self.stopped:
            ok = self.cap.grab()
            if not ok:
                with self.lock:
                    self.ok = False
                time.sleep(0.01)
                continue

            ok, frame = self.cap.retrieve()
            if ok:
                with self.lock:
                    self.ok = True
                    self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def is_opened(self):
        if self.use_ffmpeg:
            return self.proc is not None and self.proc.poll() is None
        return self.cap.isOpened()

    def release(self):
        self.stopped = True
        self.thread.join(timeout=2)
        if self.use_ffmpeg:
            if self.proc is not None and self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            if self.stderr_thread is not None:
                self.stderr_thread.join(timeout=1)
        else:
            self.cap.release()


def read_full_resolution_frame(rtsp_url, timeout_sec=4.0):
    temp_sizes = FFMPEG_STREAM_SIZES.copy()
    FFMPEG_STREAM_SIZES[rtsp_url] = CALIBRATION_IMAGE_SIZE
    reader = None
    start = time.time()
    try:
        reader = CamReader(rtsp_url, "Aruco Refresh")
        while time.time() - start < timeout_sec:
            ok, frame = reader.read()
            if ok and frame is not None:
                return True, frame
            time.sleep(0.01)
        return False, None
    finally:
        FFMPEG_STREAM_SIZES.clear()
        FFMPEG_STREAM_SIZES.update(temp_sizes)
        if reader is not None:
            reader.release()


def resize_to_width(frame, target_w):
    h, w = frame.shape[:2]
    scale = target_w / w
    new_h = int(h * scale)
    return cv2.resize(frame, (target_w, new_h), interpolation=cv2.INTER_AREA)


def equalize_heights(img1, img2):
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    target_h = max(h1, h2)

    if h1 != target_h:
        img1 = cv2.resize(img1, (w1, target_h))
    if h2 != target_h:
        img2 = cv2.resize(img2, (w2, target_h))

    return img1, img2


def make_placeholder(width, height, text):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        img,
        text,
        (20, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2,
        cv2.LINE_AA
    )
    return img


def draw_text(frame, lines, x=12, y=28, dy=24, color=(0, 255, 0)):
    yy = y
    for line in lines:
        cv2.putText(
            frame,
            line,
            (x, yy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA
        )
        yy += dy


def detect_aruco_with_corners(frame, detector):
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None or len(ids) == 0:
        return [], []

    ids_flat = ids.flatten().tolist()
    marker_polygons = []

    for i, marker_id in enumerate(ids_flat):
        pts = np.array(corners[i][0], dtype=np.float32)
        marker_polygons.append({
            "id": int(marker_id),
            "corners": pts
        })

    return sorted(ids_flat), marker_polygons


def run_pose(model, frame):
    results = model(frame, verbose=False, imgsz=YOLO_IMGSZ, conf=PERSON_CONF)
    result = results[0]
    return parse_pose_result(result)


def parse_pose_result(result):

    num_persons = 0
    boxes_scaled = []
    keypoints_scaled = []
    keypoints_conf = []

    if result.boxes is not None and len(result.boxes) > 0:
        boxes_xyxy = result.boxes.xyxy.cpu().numpy()
        boxes_conf = result.boxes.conf.cpu().numpy()
        num_persons = len(boxes_xyxy)

        for i in range(num_persons):
            b = boxes_xyxy[i]
            boxes_scaled.append((
                int(b[0]), int(b[1]),
                int(b[2]), int(b[3]),
                float(boxes_conf[i])
            ))

    if result.keypoints is not None and len(result.keypoints) > 0:
        kpts_xy = result.keypoints.xy.cpu().numpy()
        kpts_conf = result.keypoints.conf.cpu().numpy()

        for i in range(len(kpts_xy)):
            pts = []
            confs = []
            for j in range(kpts_xy.shape[1]):
                px = int(kpts_xy[i, j, 0])
                py = int(kpts_xy[i, j, 1])
                pc = float(kpts_conf[i, j])
                pts.append((px, py))
                confs.append(pc)
            keypoints_scaled.append(pts)
            keypoints_conf.append(confs)

    return num_persons, boxes_scaled, keypoints_scaled, keypoints_conf


def run_pose_batch(model, frame1, frame2):
    results = model([frame1, frame2], verbose=False, imgsz=YOLO_IMGSZ, conf=PERSON_CONF)
    return parse_pose_result(results[0]), parse_pose_result(results[1])


def draw_pose_on_frame(frame, pose_cache):
    vis = frame.copy()

    if pose_cache is None:
        return vis, 0

    num_persons, boxes_scaled, keypoints_scaled, keypoints_conf = pose_cache

    for i in range(num_persons):
        if i < len(boxes_scaled):
            x1, y1, x2, y2, score = boxes_scaled[i]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(
                vis,
                f"person {score:.2f}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 0),
                2,
                cv2.LINE_AA
            )

        if i < len(keypoints_scaled):
            pts = keypoints_scaled[i]
            confs = keypoints_conf[i]

            for j, (px, py) in enumerate(pts):
                if confs[j] > KEYPOINT_CONF:
                    cv2.circle(vis, (px, py), 3, (0, 255, 255), -1)

            for a, b in SKELETON:
                if a < len(pts) and b < len(pts):
                    if confs[a] > KEYPOINT_CONF and confs[b] > KEYPOINT_CONF:
                        cv2.line(vis, pts[a], pts[b], (0, 255, 0), 2)

    return vis, num_persons


def iter_valid_person_joints(pose_cache):
    if pose_cache is None:
        return

    _, _, keypoints_scaled, keypoints_conf = pose_cache

    for person_idx, pts in enumerate(keypoints_scaled):
        if person_idx >= len(keypoints_conf):
            continue

        confs = keypoints_conf[person_idx]
        for joint_idx, (px, py) in enumerate(pts):
            if joint_idx < len(confs) and confs[joint_idx] >= KEYPOINT_CONF:
                yield person_idx, joint_idx, (float(px), float(py)), float(confs[joint_idx])


def closest_point_on_rect(point, rect):
    x, y = point
    x1, y1, x2, y2 = rect
    return (
        min(max(x, x1), x2),
        min(max(y, y1), y2),
    )


def find_closest_person_robot_distance(pose_cache, robot_bbox):
    if robot_bbox is None:
        return None

    x, y, w, h = robot_bbox
    robot_rect = (x, y, x + w, y + h)
    best = None

    for person_idx, joint_idx, joint_pt, joint_conf in iter_valid_person_joints(pose_cache):
        robot_pt = closest_point_on_rect(joint_pt, robot_rect)
        distance_px = float(np.linalg.norm(np.subtract(joint_pt, robot_pt)))

        if best is None or distance_px < best["distance_px"]:
            best = {
                "distance_px": distance_px,
                "robot_rect": robot_rect,
                "person_index": person_idx,
                "joint_index": joint_idx,
                "joint_name": COCO_KEYPOINT_NAMES[joint_idx] if joint_idx < len(COCO_KEYPOINT_NAMES) else f"joint_{joint_idx}",
                "joint_conf": joint_conf,
                "person_point": joint_pt,
                "robot_point": robot_pt,
            }

    return best


def draw_closest_distance(frame, closest):
    if closest is None:
        return frame

    person_pt = tuple(int(round(v)) for v in closest["person_point"])
    robot_pt = tuple(int(round(v)) for v in closest["robot_point"])
    mid_pt = (
        int(round((person_pt[0] + robot_pt[0]) / 2)),
        int(round((person_pt[1] + robot_pt[1]) / 2)),
    )

    cv2.line(frame, person_pt, robot_pt, (0, 255, 255), 2)
    cv2.circle(frame, person_pt, 4, (0, 255, 255), -1)
    cv2.circle(frame, robot_pt, 4, (0, 255, 255), -1)
    cv2.putText(
        frame,
        f"{closest['joint_name']} {closest['distance_px']:.0f}px",
        (mid_pt[0] + 6, max(20, mid_pt[1] - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )
    return frame


def build_stereo_2d_safety_distance(closest_1, closest_2):
    if closest_1 is None or closest_2 is None:
        return None

    distance_1 = float(closest_1["distance_px"])
    distance_2 = float(closest_2["distance_px"])
    limiting_camera = "cam1" if distance_1 >= distance_2 else "cam2"
    limiting_closest = closest_1 if limiting_camera == "cam1" else closest_2

    return {
        "camera": "both",
        "mode": "both",
        "limiting_camera": limiting_camera,
        "distance_px": max(distance_1, distance_2),
        "cam1_distance_px": distance_1,
        "cam2_distance_px": distance_2,
        "joint_name": limiting_closest["joint_name"],
    }


def pose_has_person_joints(pose_cache):
    return any(True for _ in iter_valid_person_joints(pose_cache))


def _valid_keypoint(pts, confs, idx):
    return idx < len(pts) and idx < len(confs) and confs[idx] >= KEYPOINT_CONF


def detect_hands_up_pose(pose_cache):
    if pose_cache is None:
        return False

    _, _, keypoints_scaled, keypoints_conf = pose_cache
    for person_idx, pts in enumerate(keypoints_scaled):
        if person_idx >= len(keypoints_conf):
            continue

        confs = keypoints_conf[person_idx]
        required = [5, 6, 9, 10]  # shoulders, wrists
        if not all(_valid_keypoint(pts, confs, idx) for idx in required):
            continue

        left_shoulder = np.asarray(pts[5], dtype=np.float32)
        right_shoulder = np.asarray(pts[6], dtype=np.float32)
        left_wrist = np.asarray(pts[9], dtype=np.float32)
        right_wrist = np.asarray(pts[10], dtype=np.float32)

        shoulder_width = float(np.linalg.norm(left_shoulder - right_shoulder))
        margin = max(HANDS_UP_MIN_MARGIN_PX, shoulder_width * HANDS_UP_MARGIN_RATIO)
        required_y = min(float(left_shoulder[1]), float(right_shoulder[1])) - margin

        if float(left_wrist[1]) < required_y and float(right_wrist[1]) < required_y:
            return True

    return False


def detect_tpose_pose(pose_cache):
    if pose_cache is None:
        return False

    _, _, keypoints_scaled, keypoints_conf = pose_cache
    for person_idx, pts in enumerate(keypoints_scaled):
        if person_idx >= len(keypoints_conf):
            continue

        confs = keypoints_conf[person_idx]
        required = [5, 6, 7, 8, 9, 10]  # shoulders, elbows, wrists
        if not all(_valid_keypoint(pts, confs, idx) for idx in required):
            continue

        left_shoulder = np.asarray(pts[5], dtype=np.float32)
        right_shoulder = np.asarray(pts[6], dtype=np.float32)
        left_elbow = np.asarray(pts[7], dtype=np.float32)
        right_elbow = np.asarray(pts[8], dtype=np.float32)
        left_wrist = np.asarray(pts[9], dtype=np.float32)
        right_wrist = np.asarray(pts[10], dtype=np.float32)

        shoulder_width = float(np.linalg.norm(left_shoulder - right_shoulder))
        if shoulder_width < 1.0:
            continue

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

        if arms_level and wrists_outside_shoulders and wrist_span >= shoulder_width * TPOSE_MIN_SPAN_RATIO:
            return True

    return False


def detect_gesture_command(pose_cache_1, pose_cache_2):
    if detect_hands_up_pose(pose_cache_1) or detect_hands_up_pose(pose_cache_2):
        return "stop"
    if detect_tpose_pose(pose_cache_1) or detect_tpose_pose(pose_cache_2):
        return "go"
    return None


class GesturePrintState:
    def __init__(self):
        self.pending_command = None
        self.pending_since = 0.0
        self.last_printed_command = None
        self.last_print_time = 0.0

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

        if (
            command == self.last_printed_command and
            now - self.last_print_time < GESTURE_COOLDOWN_SEC
        ):
            return None

        print(command)
        self.last_printed_command = command
        self.last_print_time = now
        return command


class Pose3DGestureSender:
    def __init__(self, host, port):
        self.address = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, command):
        if command not in ("stop", "go"):
            return
        self.sock.sendto(command.encode("utf-8"), self.address)
        print(f"[GESTURE] sent '{command}' to pose3d {self.address[0]}:{self.address[1]}")

    def close(self):
        self.sock.close()


def build_2d_safety_distance(
    closest_1,
    closest_2,
    pose_cache_1,
    pose_cache_2,
    robot_held_1=False,
    robot_held_2=False,
):
    stereo = build_stereo_2d_safety_distance(closest_1, closest_2)
    if stereo is not None:
        return stereo

    cam1_has_person = pose_has_person_joints(pose_cache_1)
    cam2_has_person = pose_has_person_joints(pose_cache_2)

    if closest_1 is not None and cam2_has_person:
        return {
            "camera": "cam1",
            "mode": "single_camera",
            "other_camera_held_robot": bool(robot_held_2),
            "distance_px": float(closest_1["distance_px"]),
            "cam1_distance_px": float(closest_1["distance_px"]),
            "cam2_distance_px": -1.0,
            "joint_name": closest_1["joint_name"],
        }

    if closest_2 is not None and cam1_has_person:
        return {
            "camera": "cam2",
            "mode": "single_camera",
            "other_camera_held_robot": bool(robot_held_1),
            "distance_px": float(closest_2["distance_px"]),
            "cam1_distance_px": -1.0,
            "cam2_distance_px": float(closest_2["distance_px"]),
            "joint_name": closest_2["joint_name"],
        }

    return None


def load_stereo_calibration(paths):
    for path in paths:
        if path.exists():
            data = np.load(path)
            print(f"Loaded stereo calibration: {path}")
            return {
                "K1": data["K1"].astype(np.float64),
                "dist1": data["dist1"].astype(np.float64),
                "K2": data["K2"].astype(np.float64),
                "dist2": data["dist2"].astype(np.float64),
                "R": data["R"].astype(np.float64),
                "T": data["T"].astype(np.float64),
                "F": data["F"].astype(np.float64),
                "image_width": int(data["image_width"]),
                "image_height": int(data["image_height"]),
            }

    print("No stereo calibration found. 3D distance disabled.")
    return None


def live_point_to_calibration(point, frame_shape, calibration):
    h, w = frame_shape[:2]
    calib_w = float(calibration["image_width"])
    calib_h = float(calibration["image_height"])
    low_aspect = w / float(h)
    calib_aspect = calib_w / calib_h

    if low_aspect > calib_aspect:
        crop_h = calib_w / low_aspect
        crop_y = (calib_h - crop_h) / 2.0
        x = float(point[0]) * calib_w / float(w)
        y = crop_y + float(point[1]) * crop_h / float(h)
    elif low_aspect < calib_aspect:
        crop_w = calib_h * low_aspect
        crop_x = (calib_w - crop_w) / 2.0
        x = crop_x + float(point[0]) * crop_w / float(w)
        y = float(point[1]) * calib_h / float(h)
    else:
        x = float(point[0]) * calib_w / float(w)
        y = float(point[1]) * calib_h / float(h)

    return np.array([x, y], dtype=np.float64)


def map_live_point_to_calibration(point, frame_shape, calibration, live_to_calibration_h=None):
    if live_to_calibration_h is not None:
        pts = np.array([[[float(point[0]), float(point[1])]]], dtype=np.float64)
        mapped = cv2.perspectiveTransform(pts, live_to_calibration_h)[0, 0]
        return mapped.astype(np.float64)

    return live_point_to_calibration(point, frame_shape, calibration)


def live_to_calibration_scale_homography(frame_shape, calibration):
    h, w = frame_shape[:2]
    sx = calibration["image_width"] / float(w)
    sy = calibration["image_height"] / float(h)
    return np.array(
        [
            [sx, 0.0, 0.0],
            [0.0, sy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def epipolar_error(point1, point2, frame1_shape, frame2_shape, calibration, h1=None, h2=None):
    p1 = map_live_point_to_calibration(point1, frame1_shape, calibration, h1)
    p2 = map_live_point_to_calibration(point2, frame2_shape, calibration, h2)
    x1 = np.array([p1[0], p1[1], 1.0], dtype=np.float64)
    x2 = np.array([p2[0], p2[1], 1.0], dtype=np.float64)

    line2 = calibration["F"] @ x1
    line1 = calibration["F"].T @ x2

    denom2 = max(np.linalg.norm(line2[:2]), 1e-9)
    denom1 = max(np.linalg.norm(line1[:2]), 1e-9)
    dist2 = abs(float(x2 @ line2)) / denom2
    dist1 = abs(float(x1 @ line1)) / denom1
    return max(dist1, dist2)


def triangulate_live_point(point1, point2, frame1_shape, frame2_shape, calibration, h1=None, h2=None):
    p1 = map_live_point_to_calibration(point1, frame1_shape, calibration, h1).reshape(1, 1, 2)
    p2 = map_live_point_to_calibration(point2, frame2_shape, calibration, h2).reshape(1, 1, 2)

    u1 = cv2.undistortPoints(p1, calibration["K1"], calibration["dist1"])
    u2 = cv2.undistortPoints(p2, calibration["K2"], calibration["dist2"])

    P1 = np.hstack((np.eye(3), np.zeros((3, 1)))).astype(np.float64)
    P2 = np.hstack((calibration["R"], calibration["T"])).astype(np.float64)
    pts4d = cv2.triangulatePoints(P1, P2, u1.reshape(1, 2).T, u2.reshape(1, 2).T)

    if abs(float(pts4d[3, 0])) < 1e-9:
        return None

    point_3d = (pts4d[:3, 0] / pts4d[3, 0]).astype(np.float64)
    cam2_point = calibration["R"] @ point_3d.reshape(3, 1) + calibration["T"]

    if point_3d[2] <= 0.0 or float(cam2_point[2, 0]) <= 0.0:
        return None
    if not np.isfinite(point_3d).all():
        return None
    if np.linalg.norm(point_3d) > MAX_TRIANGULATED_DISTANCE_M:
        return None

    return point_3d


def find_closest_3d_person_robot_distance(
    pose_cache_1,
    pose_cache_2,
    robot_center_1,
    robot_center_2,
    frame1_shape,
    frame2_shape,
    calibration,
    h1=None,
    h2=None,
):
    if calibration is None or robot_center_1 is None or robot_center_2 is None:
        return None, "missing calibration/robot"
    if pose_cache_1 is None or pose_cache_2 is None:
        return None, "missing pose"

    robot_error = epipolar_error(robot_center_1, robot_center_2, frame1_shape, frame2_shape, calibration, h1, h2)
    if robot_error > MAX_EPIPOLAR_ERROR_PX:
        return None, f"robot epi {robot_error:.0f}px"

    robot_3d = triangulate_live_point(robot_center_1, robot_center_2, frame1_shape, frame2_shape, calibration, h1, h2)
    if robot_3d is None:
        return None, "robot triangulation failed"

    joints1 = list(iter_valid_person_joints(pose_cache_1))
    joints2 = list(iter_valid_person_joints(pose_cache_2))
    if not joints1 or not joints2:
        return None, f"joints cam1={len(joints1)} cam2={len(joints2)}"

    best = None
    matched_joint_pairs = 0
    epipolar_rejected = 0
    triangulation_rejected = 0

    for person_idx_1, joint_idx_1, joint_pt_1, joint_conf_1 in joints1:
        for person_idx_2, joint_idx_2, joint_pt_2, joint_conf_2 in joints2:
            if joint_idx_1 != joint_idx_2:
                continue

            matched_joint_pairs += 1

            epi_error = epipolar_error(joint_pt_1, joint_pt_2, frame1_shape, frame2_shape, calibration, h1, h2)
            if epi_error > MAX_EPIPOLAR_ERROR_PX:
                epipolar_rejected += 1
                continue

            joint_3d = triangulate_live_point(joint_pt_1, joint_pt_2, frame1_shape, frame2_shape, calibration, h1, h2)
            if joint_3d is None:
                triangulation_rejected += 1
                continue

            distance_m = float(np.linalg.norm(joint_3d - robot_3d))
            if best is None or distance_m < best["distance_m"]:
                best = {
                    "distance_m": distance_m,
                    "joint_name": COCO_KEYPOINT_NAMES[joint_idx_1] if joint_idx_1 < len(COCO_KEYPOINT_NAMES) else f"joint_{joint_idx_1}",
                    "joint_index": joint_idx_1,
                    "person_index_cam1": person_idx_1,
                    "person_index_cam2": person_idx_2,
                    "joint_conf": min(joint_conf_1, joint_conf_2),
                    "epipolar_error_px": epi_error,
                    "robot_epipolar_error_px": robot_error,
                    "joint_3d": joint_3d,
                    "robot_3d": robot_3d,
                }

    if best is None:
        return None, f"no valid joints pairs={matched_joint_pairs} epi_rej={epipolar_rejected} tri_rej={triangulation_rejected}"

    return best, "ok"


def draw_cached_marker_frames(frame_live, marker_polygons_high, display_scale):
    live_h, live_w = frame_live.shape[:2]
    sx_h2l = live_w / float(CALIBRATION_IMAGE_SIZE[0])
    sy_h2l = live_h / float(CALIBRATION_IMAGE_SIZE[1])

    for marker in marker_polygons_high:
        pts_high = marker["corners"]
        marker_id = marker["id"]

        pts_low = pts_high.copy()
        pts_low[:, 0] *= sx_h2l
        pts_low[:, 1] *= sy_h2l

        pts_disp = pts_low.copy()
        pts_disp[:, 0] *= display_scale
        pts_disp[:, 1] *= display_scale

        pts_int = pts_disp.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame_live, [pts_int], True, (0, 255, 0), 2)

        p0 = pts_int[0, 0]
        cv2.putText(
            frame_live,
            f"ID {marker_id}",
            (int(p0[0]), max(18, int(p0[1]) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )


def scale_marker_polygons(marker_polygons, scale):
    scaled = []
    for marker in marker_polygons:
        scaled.append({
            "id": marker["id"],
            "corners": marker["corners"].astype(np.float64) * float(scale),
        })
    return scaled


def rgb_to_hsv(rgb):
    arr = np.uint8([[[rgb[0], rgb[1], rgb[2]]]])
    hsv = cv2.cvtColor(arr, cv2.COLOR_RGB2HSV)[0, 0]
    return int(hsv[0]), int(hsv[1]), int(hsv[2])


def build_robot_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for rgb in ROBOT_RGB_COLORS:
        h, s, v = rgb_to_hsv(rgb)

        h_low = h - H_TOL
        h_high = h + H_TOL
        s_low = max(0, s - S_TOL)
        s_high = min(255, s + S_TOL)
        v_low = max(0, v - V_TOL)
        v_high = min(255, v + V_TOL)

        if h_low < 0:
            lower1 = np.array([0, s_low, v_low], dtype=np.uint8)
            upper1 = np.array([h_high, s_high, v_high], dtype=np.uint8)
            lower2 = np.array([180 + h_low, s_low, v_low], dtype=np.uint8)
            upper2 = np.array([179, s_high, v_high], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        elif h_high > 179:
            lower1 = np.array([h_low, s_low, v_low], dtype=np.uint8)
            upper1 = np.array([179, s_high, v_high], dtype=np.uint8)
            lower2 = np.array([0, s_low, v_low], dtype=np.uint8)
            upper2 = np.array([h_high - 180, s_high, v_high], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        else:
            lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
            upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)

        mask_total = cv2.bitwise_or(mask_total, mask)

    kernel = np.ones((5, 5), np.uint8)
    mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
    mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)
    return mask_total


def draw_robot_color_detection(frame, detection_frame=None):
    vis = frame.copy()
    if detection_frame is None:
        detection_frame = frame

    mask = build_robot_mask(detection_frame)

    overlay = vis.copy()
    overlay[mask > 0] = ROBOT_DRAW_COLOR
    vis = cv2.addWeighted(vis, 0.75, overlay, 0.25, 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    robot_found = False
    robot_bbox = None
    robot_center = None
    best_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_ROBOT_AREA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if area > best_area:
            best_area = area
            robot_bbox = (x, y, w, h)
            robot_center = (x + w // 2, y + h // 2)
            robot_found = True

    if robot_found and robot_bbox is not None:
        x, y, w, h = robot_bbox
        cv2.rectangle(vis, (x, y), (x + w, y + h), ROBOT_DRAW_COLOR, 2)
        cv2.circle(vis, robot_center, 4, (255, 255, 255), -1)
        cv2.putText(
            vis,
            f"Robot: {int(best_area)}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            ROBOT_DRAW_COLOR,
            2,
            cv2.LINE_AA
        )

    return vis, robot_found, best_area, robot_bbox, robot_center


def run_robot_detector(model, frame):
    if model is None:
        return False, 0.0, None, None

    results = model(frame, verbose=False, imgsz=ROBOT_IMGSZ, conf=ROBOT_CONF)
    result = results[0]

    if result.boxes is None or len(result.boxes) == 0:
        return False, 0.0, None, None

    boxes_xyxy = result.boxes.xyxy.cpu().numpy()
    boxes_conf = result.boxes.conf.cpu().numpy()
    best_idx = int(np.argmax(boxes_conf))
    x1, y1, x2, y2 = boxes_xyxy[best_idx]
    conf = float(boxes_conf[best_idx])

    x = int(round(x1))
    y = int(round(y1))
    w = int(round(x2 - x1))
    h = int(round(y2 - y1))
    robot_bbox = (x, y, w, h)
    robot_center = (x + w // 2, y + h // 2)
    return True, conf, robot_bbox, robot_center


class RobotTrack:
    def __init__(self):
        self.bbox = None
        self.score = 0.0
        self.missed = ROBOT_TRACK_HOLD_FRAMES + 1

    def update(self, found, score, bbox):
        if found and bbox is not None:
            if self.bbox is None:
                self.bbox = tuple(float(v) for v in bbox)
            else:
                alpha = ROBOT_TRACK_SMOOTHING
                self.bbox = tuple(
                    (1.0 - alpha) * old + alpha * new
                    for old, new in zip(self.bbox, bbox)
                )

            self.score = float(score)
            self.missed = 0
        elif self.bbox is not None:
            self.missed += 1
            self.score *= 0.75

        if self.bbox is None or self.missed > ROBOT_TRACK_HOLD_FRAMES:
            return False, 0.0, None, None, False

        x, y, w, h = (int(round(v)) for v in self.bbox)
        tracked_bbox = (x, y, max(1, w), max(1, h))
        tracked_center = (x + tracked_bbox[2] // 2, y + tracked_bbox[3] // 2)
        using_held_position = self.missed > 0
        return True, self.score, tracked_bbox, tracked_center, using_held_position

    def current(self):
        if self.bbox is None or self.missed > ROBOT_TRACK_HOLD_FRAMES:
            return False, 0.0, None, None, False

        x, y, w, h = (int(round(v)) for v in self.bbox)
        tracked_bbox = (x, y, max(1, w), max(1, h))
        tracked_center = (x + tracked_bbox[2] // 2, y + tracked_bbox[3] // 2)
        return True, self.score, tracked_bbox, tracked_center, self.missed > 0


class UnitySafetySender:
    def __init__(self, host, port):
        self.address = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_stop = None
        self.last_send_time = 0.0

    def update(self, closest_3d, closest_2d, frame_metrics=None):
        if closest_3d is not None:
            distance = closest_3d["distance_m"]
            stop_threshold = SAFETY_STOP_DISTANCE_M
            release_threshold = SAFETY_RELEASE_DISTANCE_M
            source = "3d"
            unit = "m"
            joint_name = closest_3d["joint_name"]
        elif closest_2d is not None:
            distance = closest_2d["distance_px"]
            if closest_2d.get("mode") == "single_camera":
                stop_threshold = SAFETY_SINGLE_CAMERA_OCCLUSION_STOP_PX
                release_threshold = SAFETY_SINGLE_CAMERA_OCCLUSION_RELEASE_PX
                source = f"{closest_2d['camera']}_single"
            else:
                stop_threshold = SAFETY_STOP_DISTANCE_PX
                release_threshold = SAFETY_RELEASE_DISTANCE_PX
                source = "both_2d"
            unit = "px"
            joint_name = closest_2d["joint_name"]
        else:
            return self.last_stop

        stop = self.last_stop

        if stop is None:
            stop = distance <= stop_threshold
        elif stop and distance >= release_threshold:
            stop = False
        elif not stop and distance <= stop_threshold:
            stop = True

        now = time.time()
        should_send = (
            stop != self.last_stop or
            now - self.last_send_time >= SAFETY_COMMAND_INTERVAL_SEC
        )

        if should_send:
            payload = {
                "type": "robot_safety",
                "frame": int(frame_metrics.get("frame", -1)) if frame_metrics is not None else -1,
                "stop": bool(stop),
                "source": source,
                "unit": unit,
                "distance": float(distance),
                "distance_m": float(closest_3d["distance_m"]) if closest_3d is not None else -1.0,
                "distance_px": float(closest_2d["distance_px"]) if closest_2d is not None else -1.0,
                "cam1_distance_px": float(closest_2d["cam1_distance_px"]) if closest_2d is not None else -1.0,
                "cam2_distance_px": float(closest_2d["cam2_distance_px"]) if closest_2d is not None else -1.0,
                "stop_distance_m": SAFETY_STOP_DISTANCE_M,
                "release_distance_m": SAFETY_RELEASE_DISTANCE_M,
                "stop_distance_px": SAFETY_STOP_DISTANCE_PX,
                "release_distance_px": SAFETY_RELEASE_DISTANCE_PX,
                "single_camera_occlusion_stop_px": SAFETY_SINGLE_CAMERA_OCCLUSION_STOP_PX,
                "single_camera_occlusion_release_px": SAFETY_SINGLE_CAMERA_OCCLUSION_RELEASE_PX,
                "joint": joint_name,
                "timestamp": now,
            }
            if frame_metrics is not None:
                payload["metrics"] = {
                    "python_frame_start_unix_ms": float(frame_metrics.get("unix_ms", 0.0)),
                    "python_send_unix_ms": time.time_ns() / 1_000_000.0,
                    "pipeline_ms": (
                        time.perf_counter_ns() - int(frame_metrics.get("_started_ns", time.perf_counter_ns()))
                    ) / 1_000_000.0,
                }
            data = json.dumps(payload).encode("utf-8")
            self.sock.sendto(data, self.address)
            self.last_send_time = now

        self.last_stop = stop
        return self.last_stop

    def close(self):
        self.sock.close()


def draw_robot_model_detection(frame, robot_found, robot_conf, robot_bbox, robot_center):
    if not robot_found or robot_bbox is None:
        return frame

    x, y, w, h = robot_bbox
    cv2.rectangle(frame, (x, y), (x + w, y + h), ROBOT_DRAW_COLOR, 2)
    cv2.circle(frame, robot_center, 4, (255, 255, 255), -1)
    cv2.putText(
        frame,
        f"Robot: {robot_conf:.2f}",
        (x, max(20, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        ROBOT_DRAW_COLOR,
        2,
        cv2.LINE_AA
    )
    return frame


def main():
    metrics = MetricsSession(SCRIPT_DIR / "metrics", prefix="safety", enabled=True)
    atexit.register(metrics.close)
    print(f"Metrics -> {metrics.session_dir}")
    print("Loading YOLO pose model...")
    if "-pose" not in YOLO_MODEL_PATH:
        print(f"WARNING: {YOLO_MODEL_PATH} is not a pose model. Use a '*-pose.pt' checkpoint to get joints.")

    model = YOLO(YOLO_MODEL_PATH)
    robot_model = None

    if ROBOT_MODEL_PATH.exists():
        print(f"Loading robot detector: {ROBOT_MODEL_PATH}")
        robot_model = YOLO(str(ROBOT_MODEL_PATH))
    else:
        print(f"Robot detector not found at {ROBOT_MODEL_PATH}. Using color fallback.")

    if torch.cuda.is_available():
        print("CUDA detected. Using GPU.")
        model.to("cuda")
        if robot_model is not None:
            robot_model.to("cuda")
    else:
        print("CUDA not detected. Using CPU.")

    dummy = np.zeros((256, 256, 3), dtype=np.uint8)
    model(dummy, verbose=False, imgsz=YOLO_IMGSZ)
    if robot_model is not None:
        robot_model(dummy, verbose=False, imgsz=ROBOT_IMGSZ)

    calibration = load_stereo_calibration(CALIBRATION_PATHS)

    print("Creating ArUco detector...")
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()

    aruco_params.adaptiveThreshWinSizeMin = 3
    aruco_params.adaptiveThreshWinSizeMax = 53
    aruco_params.adaptiveThreshWinSizeStep = 4
    aruco_params.minMarkerPerimeterRate = 0.01
    aruco_params.maxMarkerPerimeterRate = 4.0

    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print("Opening live streams...")
    cam1_low = CamReader(CAM1_LIVE, "Cam1 LIVE")
    cam2_low = CamReader(CAM2_LIVE, "Cam2 LIVE")
    cam1_high = CamReader(CAM1_HIGH, "Cam1 HIGH") if USE_HIGH_STREAMS else None
    cam2_high = CamReader(CAM2_HIGH, "Cam2 HIGH") if USE_HIGH_STREAMS else None

    streams = [cam for cam in [cam1_low, cam2_low, cam1_high, cam2_high] if cam is not None]
    if not all(cam.is_opened() for cam in streams):
        raise RuntimeError("Could not open one or more streams.")

    print(f"Opened {len(streams)} streams.")
    print("Press ESC or Q to quit.")

    last_high_sample = 0.0
    prev_time = time.perf_counter()
    frame_idx = 0

    cam1_cached_ids = []
    cam2_cached_ids = []
    cam1_cached_polygons = []
    cam2_cached_polygons = []

    pose_cache_1 = None
    pose_cache_2 = None
    robot_track_1 = RobotTrack()
    robot_track_2 = RobotTrack()
    safety_sender = UnitySafetySender(UNITY_SAFETY_HOST, UNITY_SAFETY_PORT)
    gesture_printer = GesturePrintState()
    gesture_sender = Pose3DGestureSender(POSE3D_GESTURE_HOST, POSE3D_GESTURE_PORT)
    safety_stop = None
    gesture_command = None
    robot_detection_cache_1 = (False, 0.0, None, None)
    robot_detection_cache_2 = (False, 0.0, None, None)

    window_name = "Dual Stream - Pose + Cached ArUco + Robot Color"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1400, 900)

    while True:
        frame_timer = metrics.new_frame(frame_idx + 1)
        ok1, frame1 = cam1_low.read()
        ok2, frame2 = cam2_low.read()
        frame_timer.mark("camera_read_ms")
        robot_center_1 = None
        robot_center_2 = None
        robot_bbox_1 = None
        robot_bbox_2 = None
        robot_held_1 = False
        robot_held_2 = False
        closest_1 = None
        closest_2 = None

        if ok1 and frame1 is not None and ok2 and frame2 is not None:
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            pose_cache_1, pose_cache_2 = run_pose_batch(model, frame1, frame2)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
        frame_timer.mark("pose_inference_ms")

        now = time.time()
        if now - last_high_sample >= HIGH_REFRESH_SEC:
            ok1h, frame1h = read_full_resolution_frame(CAM1_HIGH)
            ok2h, frame2h = read_full_resolution_frame(CAM2_HIGH)

            if ok1h and frame1h is not None:
                cam1_cached_ids, cam1_cached_polygons = detect_aruco_with_corners(frame1h, detector)
                print(f"[ARUCO REFRESH] Cam1 IDs: {cam1_cached_ids}")

            if ok2h and frame2h is not None:
                cam2_cached_ids, cam2_cached_polygons = detect_aruco_with_corners(frame2h, detector)
                print(f"[ARUCO REFRESH] Cam2 IDs: {cam2_cached_ids}")

            last_high_sample = now

        if ok1 and frame1 is not None:
            vis1, persons1 = draw_pose_on_frame(frame1, pose_cache_1)
            if robot_model is not None:
                if frame_idx % ROBOT_DETECT_INTERVAL == 0:
                    robot_detection_cache_1 = run_robot_detector(robot_model, frame1)
                    robot_found_1, robot_score_1, robot_bbox_1, robot_center_1 = robot_detection_cache_1
                else:
                    robot_found_1, robot_score_1, robot_bbox_1, robot_center_1 = (False, 0.0, None, None)
                robot_found_1, robot_score_1, robot_bbox_1, robot_center_1, robot_held_1 = robot_track_1.update(
                    robot_found_1, robot_score_1, robot_bbox_1
                )
                if robot_center_1 is None:
                    robot_found_1, robot_score_1, robot_bbox_1, robot_center_1, robot_held_1 = robot_track_1.current()
                vis1 = draw_robot_model_detection(vis1, robot_found_1, robot_score_1, robot_bbox_1, robot_center_1)
                robot_status_1 = f"{robot_score_1:.2f}{' held' if robot_held_1 else ''}" if robot_found_1 else "NO"
            else:
                vis1, robot_found_1, robot_area_1, robot_bbox_1, robot_center_1 = draw_robot_color_detection(vis1, frame1)
                robot_status_1 = "YES" if robot_found_1 else "NO"
            closest_1 = find_closest_person_robot_distance(pose_cache_1, robot_bbox_1)
            vis1 = draw_closest_distance(vis1, closest_1)

            low_h1, low_w1 = vis1.shape[:2]
            vis1 = resize_to_width(vis1, DISPLAY_WIDTH)
            display_scale_1 = vis1.shape[1] / float(low_w1)

            draw_cached_marker_frames(vis1, cam1_cached_polygons, display_scale_1)

            draw_text(vis1, [
                "Cam1 LIVE",
                f"{low_w1}x{low_h1}",
                f"Persons: {persons1}",
                f"ArUco visible: {cam1_cached_ids}",
                f"Robot: {robot_status_1}",
                f"Closest joint: {closest_1['joint_name']} {closest_1['distance_px']:.0f} px" if closest_1 else "Closest joint: N/A"
            ])
        else:
            vis1 = make_placeholder(DISPLAY_WIDTH, 360, "Cam1 LOW - No signal")

        if ok2 and frame2 is not None:
            vis2, persons2 = draw_pose_on_frame(frame2, pose_cache_2)
            if robot_model is not None:
                if frame_idx % ROBOT_DETECT_INTERVAL == 0:
                    robot_detection_cache_2 = run_robot_detector(robot_model, frame2)
                    robot_found_2, robot_score_2, robot_bbox_2, robot_center_2 = robot_detection_cache_2
                else:
                    robot_found_2, robot_score_2, robot_bbox_2, robot_center_2 = (False, 0.0, None, None)
                robot_found_2, robot_score_2, robot_bbox_2, robot_center_2, robot_held_2 = robot_track_2.update(
                    robot_found_2, robot_score_2, robot_bbox_2
                )
                if robot_center_2 is None:
                    robot_found_2, robot_score_2, robot_bbox_2, robot_center_2, robot_held_2 = robot_track_2.current()
                vis2 = draw_robot_model_detection(vis2, robot_found_2, robot_score_2, robot_bbox_2, robot_center_2)
                robot_status_2 = f"{robot_score_2:.2f}{' held' if robot_held_2 else ''}" if robot_found_2 else "NO"
            else:
                vis2, robot_found_2, robot_area_2, robot_bbox_2, robot_center_2 = draw_robot_color_detection(vis2, frame2)
                robot_status_2 = "YES" if robot_found_2 else "NO"
            closest_2 = find_closest_person_robot_distance(pose_cache_2, robot_bbox_2)
            vis2 = draw_closest_distance(vis2, closest_2)

            low_h2, low_w2 = vis2.shape[:2]
            vis2 = resize_to_width(vis2, DISPLAY_WIDTH)
            display_scale_2 = vis2.shape[1] / float(low_w2)

            draw_cached_marker_frames(vis2, cam2_cached_polygons, display_scale_2)

            draw_text(vis2, [
                "Cam2 LIVE",
                f"{low_w2}x{low_h2}",
                f"Persons: {persons2}",
                f"ArUco visible: {cam2_cached_ids}",
                f"Robot: {robot_status_2}",
                f"Closest joint: {closest_2['joint_name']} {closest_2['distance_px']:.0f} px" if closest_2 else "Closest joint: N/A"
            ])
        else:
            vis2 = make_placeholder(DISPLAY_WIDTH, 360, "Cam2 LOW - No signal")

        frame_timer.mark("robot_detection_overlay_ms")

        closest_3d = None
        closest_3d_reason = "waiting"
        closest_2d = build_2d_safety_distance(
            closest_1,
            closest_2,
            pose_cache_1,
            pose_cache_2,
            robot_held_1,
            robot_held_2,
        )
        if ok1 and ok2 and frame1 is not None and frame2 is not None:
            h1 = live_to_calibration_scale_homography(frame1.shape, calibration) if calibration is not None else None
            h2 = live_to_calibration_scale_homography(frame2.shape, calibration) if calibration is not None else None

            closest_3d, closest_3d_reason = find_closest_3d_person_robot_distance(
                pose_cache_1,
                pose_cache_2,
                robot_center_1,
                robot_center_2,
                frame1.shape,
                frame2.shape,
                calibration,
                h1,
                h2,
            )
            if closest_3d is None and calibration is not None:
                closest_3d_reason += f" calibIDs1={cam1_cached_ids} calibIDs2={cam2_cached_ids}"

        frame_timer.mark("safety_distance_ms")
        safety_stop = safety_sender.update(closest_3d, closest_2d, frame_timer.values)
        frame_timer.mark("safety_decision_send_ms")
        gesture_command = detect_gesture_command(pose_cache_1, pose_cache_2)
        printed_command = gesture_printer.update(gesture_command)
        if printed_command is not None:
            gesture_sender.send(printed_command)

        frame_idx += 1

        vis1, vis2 = equalize_heights(vis1, vis2)
        combined = np.hstack((vis1, vis2))

        now_perf = time.perf_counter()
        fps = 1.0 / max(now_perf - prev_time, 1e-9)
        prev_time = now_perf

        cv2.putText(
            combined,
            f"FPS: {fps:.1f}",
            (combined.shape[1] - 150, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        distance_text = (
            f"3D closest: {closest_3d['joint_name']} {closest_3d['distance_m']:.2f} m epi={closest_3d['epipolar_error_px']:.0f}px"
            if closest_3d else
            f"3D closest: N/A ({closest_3d_reason})"
        )
        cv2.putText(
            combined,
            distance_text,
            (20, combined.shape[0] - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 255, 255) if closest_3d else (0, 0, 255),
            2,
            cv2.LINE_AA
        )

        if closest_3d is not None:
            safety_source = f"3D {closest_3d['distance_m']:.2f}m"
        elif closest_2d is not None:
            if closest_2d.get("mode") == "single_camera":
                held_text = " held" if closest_2d.get("other_camera_held_robot") else ""
                safety_source = f"2D {closest_2d['camera']} only{held_text} {closest_2d['distance_px']:.0f}px"
            else:
                safety_source = f"2D both c1={closest_2d['cam1_distance_px']:.0f}px c2={closest_2d['cam2_distance_px']:.0f}px"
        else:
            safety_source = "need both cameras"

        safety_text = f"Safety: {'STOP' if safety_stop else 'RUN'} ({safety_source})"
        cv2.putText(
            combined,
            safety_text,
            (20, combined.shape[0] - 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 0, 255) if safety_stop else (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        gesture_text = f"Gesture: {gesture_command if gesture_command is not None else 'none'}"
        cv2.putText(
            combined,
            gesture_text,
            (20, combined.shape[0] - 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 0, 255) if gesture_command == "stop" else ((0, 255, 0) if gesture_command == "go" else (255, 255, 255)),
            2,
            cv2.LINE_AA
        )

        cv2.imshow(window_name, combined)
        frame_timer.mark("display_build_ms")
        frame_timer.set(
            safety_stop=int(bool(safety_stop)),
            safety_source=(closest_3d or closest_2d or {}).get("source", "3d" if closest_3d else "2d" if closest_2d else "none"),
            safety_distance_m=closest_3d.get("distance_m") if closest_3d else None,
            safety_distance_px=closest_2d.get("distance_px") if closest_2d else None,
            robot_detected_cam1=int(robot_bbox_1 is not None),
            robot_detected_cam2=int(robot_bbox_2 is not None),
            gesture_detected=gesture_command or "",
        )
        metrics.record(frame_timer.finish(), torch_module=torch)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    safety_sender.close()
    gesture_sender.close()
    for cam in streams:
        cam.release()
    cv2.destroyAllWindows()
    metrics.close()


if __name__ == "__main__":
    main()
