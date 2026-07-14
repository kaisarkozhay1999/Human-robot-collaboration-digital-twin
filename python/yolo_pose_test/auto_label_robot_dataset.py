import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args():
    parser = argparse.ArgumentParser(description="Auto-label robot images with trained YOLO weights.")
    parser.add_argument("--weights", default="runs/detect/robot_detector/weights/best.pt")
    parser.add_argument("--images", default="robot_dataset/images/unlabeled")
    parser.add_argument("--labels", default="robot_dataset/labels/autolabeled")
    parser.add_argument("--review-images", default="robot_dataset/review/autolabeled")
    parser.add_argument("--reject-images", default="robot_dataset/review/low_confidence")
    parser.add_argument("--conf", type=float, default=0.35, help="Minimum confidence to write a label.")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--copy-images", action="store_true", help="Copy confidently labeled images into review-images.")
    return parser.parse_args()


def list_images(images_dir):
    if not images_dir.exists():
        return []
    return sorted(p for p in images_dir.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS)


def box_to_yolo(box_xyxy, image_w, image_h):
    x1, y1, x2, y2 = box_xyxy
    x1 = max(0.0, min(float(image_w - 1), float(x1)))
    x2 = max(0.0, min(float(image_w - 1), float(x2)))
    y1 = max(0.0, min(float(image_h - 1), float(y1)))
    y2 = max(0.0, min(float(image_h - 1), float(y2)))

    xc = ((x1 + x2) / 2.0) / image_w
    yc = ((y1 + y2) / 2.0) / image_h
    bw = (x2 - x1) / image_w
    bh = (y2 - y1) / image_h
    return xc, yc, bw, bh


def draw_preview(image, box_xyxy, conf):
    preview = image.copy()
    x1, y1, x2, y2 = (int(round(v)) for v in box_xyxy)
    cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 0, 255), 2)
    cv2.putText(
        preview,
        f"robot {conf:.2f}",
        (x1, max(20, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return preview


def save_preview(output_dir, image_path, image):
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / image_path.name), image)


def main():
    args = parse_args()
    weights = Path(args.weights)
    images_dir = Path(args.images)
    labels_dir = Path(args.labels)
    review_dir = Path(args.review_images)
    reject_dir = Path(args.reject_images)

    if not weights.exists():
        raise RuntimeError(f"Robot weights not found: {weights}")

    images = list_images(images_dir)
    if not images:
        raise RuntimeError(f"No images found: {images_dir}")

    labels_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights))

    labeled = 0
    rejected = 0

    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"skip unreadable: {image_path}")
            continue

        result = model(image, verbose=False, imgsz=args.imgsz, conf=0.01)[0]
        if result.boxes is None or len(result.boxes) == 0:
            save_preview(reject_dir, image_path, image)
            rejected += 1
            continue

        confs = result.boxes.conf.cpu().numpy()
        boxes = result.boxes.xyxy.cpu().numpy()
        best_idx = int(np.argmax(confs))
        conf = float(confs[best_idx])
        box = boxes[best_idx]

        if conf < args.conf:
            save_preview(reject_dir, image_path, draw_preview(image, box, conf))
            rejected += 1
            continue

        h, w = image.shape[:2]
        xc, yc, bw, bh = box_to_yolo(box, w, h)
        label_path = labels_dir / f"{image_path.stem}.txt"
        label_path.write_text(f"0 {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")

        preview = draw_preview(image, box, conf)
        save_preview(review_dir, image_path, preview)
        if args.copy_images:
            image_out = review_dir / "images"
            image_out.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(image_out / image_path.name), image)

        labeled += 1

    print(f"auto-labeled: {labeled}")
    print(f"low-confidence/no-detection review: {rejected}")
    print(f"labels written to: {labels_dir}")
    print(f"previews written to: {review_dir}")
    print(f"rejections written to: {reject_dir}")


if __name__ == "__main__":
    main()
