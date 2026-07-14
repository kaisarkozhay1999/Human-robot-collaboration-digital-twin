import argparse
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Derive a stereo calibration for a resized stream by scaling the intrinsic matrices."
    )
    parser.add_argument("--input", default="stereo_calibration_charuco_refined.npz")
    parser.add_argument("--output", default="stereo_calibration_charuco_live.npz")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


def scale_intrinsics(K, sx, sy):
    scaled = K.astype(np.float64).copy()
    scaled[0, 0] *= sx
    scaled[0, 1] *= sx
    scaled[0, 2] *= sx
    scaled[1, 0] *= sy
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


def main():
    args = parse_args()
    source = Path(args.input)
    if not source.exists():
        raise RuntimeError(f"Calibration file not found: {source}")

    data = np.load(source)
    source_w = int(data["image_width"])
    source_h = int(data["image_height"])
    sx = args.width / float(source_w)
    sy = args.height / float(source_h)

    K1 = scale_intrinsics(data["K1"], sx, sy)
    K2 = scale_intrinsics(data["K2"], sx, sy)
    F = data["F"].astype(np.float64).copy()
    scale_matrix = np.diag([sx, sy, 1.0])
    F = np.linalg.inv(scale_matrix).T @ F @ np.linalg.inv(scale_matrix)

    output = {
        key: data[key]
        for key in data.files
        if key not in {"K1", "K2", "F", "image_width", "image_height"}
    }
    output.update(
        K1=K1,
        K2=K2,
        F=F,
        image_width=np.array(args.width, dtype=np.int64),
        image_height=np.array(args.height, dtype=np.int64),
        source_calibration=str(source),
        source_image_width=np.array(source_w, dtype=np.int64),
        source_image_height=np.array(source_h, dtype=np.int64),
        resize_scale_x=np.array(sx, dtype=np.float64),
        resize_scale_y=np.array(sy, dtype=np.float64),
    )

    np.savez(args.output, **output)
    print(f"saved {args.output}")
    print(f"source: {source_w}x{source_h}")
    print(f"target: {args.width}x{args.height}")
    print(f"scale: sx={sx:.6f} sy={sy:.6f}")


if __name__ == "__main__":
    main()
