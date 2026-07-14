import argparse
import time
from pathlib import Path

import cv2

from test_dual_stream_pipeline import CAM1_LOW, CAM2_LOW, CamReader, equalize_heights


def parse_args():
    parser = argparse.ArgumentParser(description="Capture robot training images from the low camera streams.")
    parser.add_argument("--output", default="robot_dataset/images/train", help="Directory to save captured images.")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between automatic captures.")
    parser.add_argument("--prefix", default="robot", help="Filename prefix.")
    parser.add_argument("--cam1", default=CAM1_LOW, help="Camera 1 stream URL.")
    parser.add_argument("--cam2", default=CAM2_LOW, help="Camera 2 stream URL.")
    return parser.parse_args()


def next_index(output_dir, prefix):
    existing = sorted(output_dir.glob(f"{prefix}_*.jpg"))
    if not existing:
        return 1

    last_stem = existing[-1].stem
    try:
        return int(last_stem.rsplit("_", 1)[1]) + 1
    except (IndexError, ValueError):
        return len(existing) + 1


def save_frame(output_dir, prefix, index, cam_name, frame):
    filename = output_dir / f"{prefix}_{index:05d}_{cam_name}.jpg"
    cv2.imwrite(str(filename), frame)
    print(f"saved {filename}")


def main():
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    cam1 = CamReader(args.cam1, "Cam1 LOW")
    cam2 = CamReader(args.cam2, "Cam2 LOW")
    cams = [cam1, cam2]

    if not all(cam.is_opened() for cam in cams):
        raise RuntimeError("Could not open one or more low streams.")

    print("Controls: S saves one pair, A toggles autosave, Q/ESC quits.")
    auto_save = False
    last_save = 0.0
    image_index = next_index(output_dir, args.prefix)

    try:
        while True:
            ok1, frame1 = cam1.read()
            ok2, frame2 = cam2.read()

            if not ok1 or frame1 is None or not ok2 or frame2 is None:
                time.sleep(0.02)
                continue

            now = time.time()
            if auto_save and now - last_save >= args.interval:
                save_frame(output_dir, args.prefix, image_index, "cam1", frame1)
                save_frame(output_dir, args.prefix, image_index, "cam2", frame2)
                image_index += 1
                last_save = now

            vis1, vis2 = equalize_heights(frame1.copy(), frame2.copy())
            combined = cv2.hconcat([vis1, vis2])
            status = f"autosave={'ON' if auto_save else 'OFF'}  next={image_index:05d}"
            cv2.putText(combined, status, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("Robot Dataset Capture", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            if key == ord("a"):
                auto_save = not auto_save
                last_save = 0.0
            if key == ord("s"):
                save_frame(output_dir, args.prefix, image_index, "cam1", frame1)
                save_frame(output_dir, args.prefix, image_index, "cam2", frame2)
                image_index += 1
    finally:
        for cam in cams:
            cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
