import argparse
import os
import subprocess
import threading
import time

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;tdp"
    "|fflags;nobuffer+discardcorrupt"
    "|flags;low_delay"
    "|framedrop;1"
    "|analyzeduration;1000000"
    "|probesize;1048576"
    "|max_delay;0"
)

import cv2

try:
    import imageio_ffmpeg
except ImportError:
    imageio_ffmpeg = None


CAM1_LOW = os.environ.get("SMARTLAB_CAM1_LOW", "rtsp://CAMERA_1_IP/mpeg4cif")
CAM1_HIGH = os.environ.get("SMARTLAB_CAM1_HIGH", "rtsp://CAMERA_1_IP/h264")
CAM2_LOW = os.environ.get("SMARTLAB_CAM2_LOW", "rtsp://CAMERA_2_IP/mpeg4cif")
CAM2_HIGH = os.environ.get("SMARTLAB_CAM2_HIGH", "rtsp://CAMERA_2_IP/h264")


class LatestFrameReader:
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.read_count = 0
        self.ok = False
        self.stopped = False

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.stopped:
            ok = self.cap.grab()
            if not ok:
                with self.lock:
                    self.ok = False
                time.sleep(0.002)
                continue

            ok, frame = self.cap.retrieve()
            now = time.perf_counter()
            if ok and frame is not None:
                with self.lock:
                    self.ok = True
                    self.frame = frame
                    self.frame_time = now
                    self.read_count += 1

    def read_latest(self):
        with self.lock:
            if self.frame is None:
                return False, None, 0.0, self.read_count
            return self.ok, self.frame.copy(), self.frame_time, self.read_count

    def release(self):
        self.stopped = True
        self.thread.join(timeout=2)
        self.cap.release()


class FFmpegPipeReader:
    def __init__(self, url, width, height, transport, ffmpeg_exe=None):
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.read_count = 0
        self.ok = False
        self.stopped = False

        if ffmpeg_exe is None:
            ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe() if imageio_ffmpeg is not None else "ffmpeg"
        self.stderr_lines = []

        vf_args = []
        if width > 0 and height > 0:
            vf_args = ["-vf", f"scale={width}:{height}"]

        cmd = [
            ffmpeg_exe,
            "-hide_banner",
            "-loglevel", "warning",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "1000000",
            "-probesize", "1048576",
            "-rtsp_transport", transport,
            "-i", url,
            "-an",
            *vf_args,
            "-fps_mode", "passthrough",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-",
        ]

        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.stderr_thread = threading.Thread(target=self._stderr_loop, daemon=True)
        self.stderr_thread.start()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _stderr_loop(self):
        while not self.stopped and self.proc.stderr is not None:
            line = self.proc.stderr.readline()
            if not line:
                break
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                self.stderr_lines.append(text)
                self.stderr_lines = self.stderr_lines[-5:]
                print("ffmpeg:", text)

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
        while not self.stopped:
            data = self._read_exact(self.frame_size)
            if data is None:
                with self.lock:
                    self.ok = False
                time.sleep(0.002)
                continue

            import numpy as np
            image = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()
            now = time.perf_counter()

            with self.lock:
                self.ok = True
                self.frame = image
                self.frame_time = now
                self.read_count += 1

    def read_latest(self):
        with self.lock:
            if self.frame is None:
                return False, None, 0.0, self.read_count
            return self.ok, self.frame.copy(), self.frame_time, self.read_count

    def release(self):
        self.stopped = True
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.thread.join(timeout=2)
        self.stderr_thread.join(timeout=1)


def parse_args():
    parser = argparse.ArgumentParser(description="Minimal RTSP latency viewer with no YOLO.")
    parser.add_argument(
        "--camera",
        choices=["cam1-low", "cam1-high", "cam2-low", "cam2-high"],
        default="cam1-low",
    )
    parser.add_argument("--url", default=None, help="Override RTSP URL.")
    parser.add_argument("--width", type=int, default=960, help="Display width.")
    parser.add_argument("--backend", choices=["opencv", "ffmpeg"], default="ffmpeg")
    parser.add_argument("--transport", choices=["tcp", "udp"], default="tcp")
    parser.add_argument("--pipe-width", type=int, default=1280)
    parser.add_argument("--pipe-height", type=int, default=720)
    parser.add_argument("--ffmpeg-exe", default=None, help="Full path to ffmpeg.exe from the same folder as working ffplay.exe.")
    return parser.parse_args()


def choose_url(args):
    if args.url:
        return args.url

    return {
        "cam1-low": CAM1_LOW,
        "cam1-high": CAM1_HIGH,
        "cam2-low": CAM2_LOW,
        "cam2-high": CAM2_HIGH,
    }[args.camera]


def resize_to_width(frame, width):
    h, w = frame.shape[:2]
    if w == width:
        return frame
    scale = width / float(w)
    return cv2.resize(frame, (width, int(h * scale)), interpolation=cv2.INTER_AREA)


def main():
    args = parse_args()
    url = choose_url(args)
    print(f"Opening {args.camera}: {url}")

    if args.backend == "ffmpeg":
        reader = FFmpegPipeReader(url, args.pipe_width, args.pipe_height, args.transport, args.ffmpeg_exe)
    else:
        reader = LatestFrameReader(url)
    window = "RTSP Latency Test"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    prev_display = time.perf_counter()
    prev_count = 0
    last_count_change = time.perf_counter()
    fps = 0.0

    try:
        while True:
            ok, frame, frame_time, read_count = reader.read_latest()
            now = time.perf_counter()

            if not ok or frame is None:
                placeholder = "No frame"
                frame = cv2.UMat(360, 640, cv2.CV_8UC3).get()
                cv2.putText(frame, placeholder, (24, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                dt = max(now - prev_display, 1e-9)
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0.0 else 1.0 / dt
                prev_display = now

            frame_age_ms = max(0.0, (now - frame_time) * 1000.0) if frame_time > 0.0 else 0.0
            read_delta = read_count - prev_count
            if read_delta > 0:
                last_count_change = now
            prev_count = read_count
            capture_stall_ms = (now - last_count_change) * 1000.0

            vis = resize_to_width(frame, args.width)
            lines = [
                f"{args.camera}  display FPS: {fps:.1f}",
                f"latest-frame age in app: {frame_age_ms:.1f} ms",
                f"capture frames since last draw: {read_delta}",
                f"capture stall: {capture_stall_ms:.0f} ms",
                "Press Q or ESC to quit",
            ]

            y = 30
            for line in lines:
                cv2.putText(vis, line, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                y += 28

            cv2.imshow(window, vis)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
    finally:
        reader.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
