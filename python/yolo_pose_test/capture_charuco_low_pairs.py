import argparse
import time
from pathlib import Path

import cv2

from test_dual_stream_pipeline import CAM1_LOW, CAM2_LOW, ARUCO_DICT, CamReader, equalize_heights


SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.048
MARKER_LENGTH_M = 0.036


def parse_args():
    parser = argparse.ArgumentParser(description="Capture low-stream ChArUco stereo pairs for live 3D calibration.")
    parser.add_argument("--output", default="charuco_low_pairs")
    parser.add_argument("--max-pairs", type=int, default=50)
    parser.add_argument("--interval", type=float, default=1.0)
    return parser.parse_args()


def next_index(output_dir):
    existing = sorted(output_dir.glob("cam1_*.png"))
    if not existing:
        return 1
    try:
        return int(existing[-1].stem.replace("cam1_", "")) + 1
    except ValueError:
        return len(existing) + 1


def marker_count(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    return 0 if ids is None else len(ids), corners, ids


def draw_status(frame, text, corners, ids):
    vis = frame.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    cv2.putText(vis, text, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
    return vis


def save_pair(output_dir, index, frame1, frame2):
    p1 = output_dir / f"cam1_{index:04d}.png"
    p2 = output_dir / f"cam2_{index:04d}.png"
    cv2.imwrite(str(p1), frame1)
    cv2.imwrite(str(p2), frame2)
    print(f"saved {p1.name} / {p2.name}")


def main():
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    cam1 = CamReader(CAM1_LOW, "Cam1 LOW")
    cam2 = CamReader(CAM2_LOW, "Cam2 LOW")
    streams = [cam1, cam2]

    if not all(cam.is_opened() for cam in streams):
        raise RuntimeError("Could not open one or more low streams.")

    pair_index = next_index(output_dir)
    auto_save = False
    last_save = 0.0

    print("Controls: S saves one pair, A toggles autosave, Q/ESC quits.")
    print("Move the ChArUco board around; keep many corners visible in both cameras.")

    try:
        while pair_index <= args.max_pairs:
            ok1, frame1 = cam1.read()
            ok2, frame2 = cam2.read()
            if not ok1 or frame1 is None or not ok2 or frame2 is None:
                time.sleep(0.02)
                continue

            count1, corners1, ids1 = marker_count(frame1, detector)
            count2, corners2, ids2 = marker_count(frame2, detector)
            status = f"next={pair_index:04d} auto={'ON' if auto_save else 'OFF'} markers cam1={count1} cam2={count2}"

            now = time.time()
            if auto_save and now - last_save >= args.interval:
                save_pair(output_dir, pair_index, frame1, frame2)
                pair_index += 1
                last_save = now

            vis1 = draw_status(frame1, "Cam1 " + status, corners1, ids1)
            vis2 = draw_status(frame2, "Cam2 " + status, corners2, ids2)
            vis1, vis2 = equalize_heights(vis1, vis2)
            cv2.imshow("Low ChArUco Capture", cv2.hconcat([vis1, vis2]))

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            if key == ord("a"):
                auto_save = not auto_save
                last_save = 0.0
            if key == ord("s"):
                save_pair(output_dir, pair_index, frame1, frame2)
                pair_index += 1
    finally:
        for cam in streams:
            cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
