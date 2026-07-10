import argparse
from pathlib import Path

import cv2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert high-res ChArUco stereo pairs to the current live 1280x720 geometry."
    )
    parser.add_argument("--input", default="charuco_pairs")
    parser.add_argument("--output", default="charuco_live_pairs")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--mode",
        choices=["stretch", "center-crop"],
        default="stretch",
        help="stretch keeps the whole image; center-crop matches a 16:9 crop but can remove board corners.",
    )
    return parser.parse_args()


def center_crop_to_aspect(image, target_aspect):
    h, w = image.shape[:2]
    aspect = w / float(h)

    if aspect > target_aspect:
        crop_w = int(round(h * target_aspect))
        x0 = max(0, (w - crop_w) // 2)
        return image[:, x0:x0 + crop_w]

    if aspect < target_aspect:
        crop_h = int(round(w / target_aspect))
        y0 = max(0, (h - crop_h) // 2)
        return image[y0:y0 + crop_h, :]

    return image


def convert_image(path, output_path, width, height, mode):
    image = cv2.imread(str(path))
    if image is None:
        print(f"skip unreadable: {path}")
        return False

    if mode == "center-crop":
        target_aspect = width / float(height)
        image = center_crop_to_aspect(image, target_aspect)

    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(output_path), resized)
    return True


def main():
    args = parse_args()
    input_dir = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(input_dir.glob("cam*_*.png"))
    if not files:
        raise RuntimeError(f"No ChArUco images found in {input_dir}")

    converted = 0
    for path in files:
        output_path = output_dir / path.name
        if convert_image(path, output_path, args.width, args.height, args.mode):
            converted += 1

    print(f"converted {converted} images to {output_dir} at {args.width}x{args.height}")


if __name__ == "__main__":
    main()
