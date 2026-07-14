import cv2
import os
import time

cap = cv2.VideoCapture(
    os.environ.get("SMARTLAB_CAM1_HIGH", "rtsp://CAMERA_1_IP/h264"),
    cv2.CAP_FFMPEG
)

t0 = time.perf_counter()
for _ in range(30):
    cap.grab()
t1 = time.perf_counter()

print(f"30 grabs: {(t1-t0)*1000:.0f} ms")
print(f"Effective FPS: {30/(t1-t0):.1f}")
cap.release()
