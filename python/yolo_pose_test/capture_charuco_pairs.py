import cv2
import threading
import time
import numpy as np
import os

try:
    import winsound
    HAS_WINSOUND = True
except ImportError:
    HAS_WINSOUND = False

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
# HIGH-RES STREAMS FOR ChArUco CAPTURE
# ============================================================
CAM1_HIGH = os.environ.get("SMARTLAB_CAM1_HIGH", "rtsp://CAMERA_1_IP/h264")
CAM2_HIGH = os.environ.get("SMARTLAB_CAM2_HIGH", "rtsp://CAMERA_2_IP/h264")

DISPLAY_WIDTH = 600
SAVE_DIR = "charuco_pairs"
MAX_PAIRS = 50

# ============================================================
# MUST MATCH YOUR PRINTED ChArUco BOARD
# ============================================================
ARUCO_DICT = cv2.aruco.DICT_4X4_50
SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.035
MARKER_LENGTH_M = 0.026


class CamReader:
    def __init__(self, rtsp_url, name="Camera"):
        self.name = name
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.stopped = False

        # initial read may fail at startup; that's okay
        try:
            self.ok, self.frame = self.cap.read()
        except Exception:
            self.ok, self.frame = False, None

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.stopped:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.05)
                continue

            ok = self.cap.grab()
            if not ok:
                with self.lock:
                    self.ok = False
                time.sleep(0.02)
                continue

            ok, frame = self.cap.retrieve()
            if ok and frame is not None:
                with self.lock:
                    self.ok = True
                    self.frame = frame
            else:
                with self.lock:
                    self.ok = False
                time.sleep(0.02)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def is_opened(self):
        return self.cap is not None and self.cap.isOpened()

    def release(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.cap is not None:
            self.cap.release()


def resize_to_width(frame, target_w):
    h, w = frame.shape[:2]
    if w == target_w:
        return frame
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


def draw_text(frame, lines, x=12, y=28, dy=28, color=(0, 255, 0)):
    yy = y
    for line in lines:
        cv2.putText(
            frame,
            line,
            (x, yy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA
        )
        yy += dy


def make_placeholder(width, height, text):
    img = np.full((height, width, 3), 40, dtype=np.uint8)
    cv2.putText(
        img,
        text,
        (20, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )
    return img


def beep():
    if HAS_WINSOUND:
        winsound.Beep(1200, 180)
    else:
        print("\a", end="", flush=True)


def annotate_charuco(img, detector, board, cam_name):
    vis = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    num_markers = 0
    num_charuco = 0
    charuco_ok = False

    if ids is not None and len(ids) > 0:
        num_markers = len(ids)
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )

        if charuco_corners is not None and charuco_ids is not None and len(charuco_ids) > 0:
            num_charuco = len(charuco_ids)
            charuco_ok = True
            cv2.aruco.drawDetectedCornersCharuco(
                vis, charuco_corners, charuco_ids, (0, 255, 0)
            )

    h, w = img.shape[:2]
    draw_text(vis, [
        cam_name,
        f"{w}x{h}",
        f"Markers: {num_markers}",
        f"ChArUco corners: {num_charuco}",
        f"Board visible: {'YES' if charuco_ok else 'NO'}"
    ])

    return vis, num_markers, num_charuco, charuco_ok


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        aruco_dict
    )

    print("Opening high-resolution streams for ChArUco capture...")
    cam1 = CamReader(CAM1_HIGH, "Cam1 HIGH")
    cam2 = CamReader(CAM2_HIGH, "Cam2 HIGH")

    if not cam1.is_opened() or not cam2.is_opened():
        raise RuntimeError("Could not open one or both high-resolution streams.")

    print("Press J to save one stereo pair.")
    print("Press Q or ESC to quit.")
    print(f"Target number of pairs: {MAX_PAIRS}")

    saved_count = 0

    window_name = "Stereo ChArUco Manual Capture"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1350, 800)

    prev_time = time.perf_counter()
    start_time = time.time()

    while True:
        ok1, frame1 = cam1.read()
        ok2, frame2 = cam2.read()

        # Show placeholders while streams are still warming up
        if ok1 and frame1 is not None:
            vis1, _, _, board_ok1 = annotate_charuco(frame1, detector, board, "Cam1 HIGH")
            vis1 = resize_to_width(vis1, DISPLAY_WIDTH)
        else:
            board_ok1 = False
            vis1 = make_placeholder(DISPLAY_WIDTH, 420, "Cam1 waiting for stream...")

        if ok2 and frame2 is not None:
            vis2, _, _, board_ok2 = annotate_charuco(frame2, detector, board, "Cam2 HIGH")
            vis2 = resize_to_width(vis2, DISPLAY_WIDTH)
        else:
            board_ok2 = False
            vis2 = make_placeholder(DISPLAY_WIDTH, 420, "Cam2 waiting for stream...")

        vis1, vis2 = equalize_heights(vis1, vis2)
        combined = np.hstack((vis1, vis2))

        now_perf = time.perf_counter()
        fps = 1.0 / max(now_perf - prev_time, 1e-9)
        prev_time = now_perf

        warmup_left = max(0, 3 - int(time.time() - start_time))

        cv2.putText(
            combined,
            f"FPS: {fps:.1f}",
            (combined.shape[1] - 170, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        status_color = (0, 255, 0) if (board_ok1 and board_ok2) else (0, 0, 255)
        cv2.putText(
            combined,
            f"Saved pairs: {saved_count}/{MAX_PAIRS}",
            (20, combined.shape[0] - 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color if saved_count < MAX_PAIRS else (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        if warmup_left > 0:
            cv2.putText(
                combined,
                f"Streams warming up... {warmup_left}s",
                (20, combined.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )
        else:
            cv2.putText(
                combined,
                "Press J to save stereo pair",
                (20, combined.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

        cv2.imshow(window_name, combined)

        key = cv2.waitKey(1) & 0xFF

        if (
            key == ord("j")
            and saved_count < MAX_PAIRS
            and ok1 and frame1 is not None
            and ok2 and frame2 is not None
        ):
            pair_idx = saved_count + 1
            cam1_path = os.path.join(SAVE_DIR, f"cam1_{pair_idx:04d}.png")
            cam2_path = os.path.join(SAVE_DIR, f"cam2_{pair_idx:04d}.png")

            cv2.imwrite(cam1_path, frame1)
            cv2.imwrite(cam2_path, frame2)

            saved_count += 1
            print(f"[SAVED] Pair {pair_idx:04d}   ({saved_count}/{MAX_PAIRS})")
            beep()

        elif key == 27 or key == ord("q"):
            break

    cam1.release()
    cam2.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
