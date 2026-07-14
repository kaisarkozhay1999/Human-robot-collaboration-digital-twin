import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="Train a one-class YOLO robot detector.")
    parser.add_argument("--data", default="robot_dataset/data.yaml", help="Ultralytics dataset YAML.")
    parser.add_argument("--model", default="yolo26n.pt", help="Base detection model checkpoint.")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--project", default="")
    parser.add_argument("--name", default="robot_detector")
    return parser.parse_args()


def require_labels(data_yaml):
    dataset_dir = Path(data_yaml).resolve().parent
    train_labels = list((dataset_dir / "labels" / "train").glob("*.txt"))
    val_labels = list((dataset_dir / "labels" / "val").glob("*.txt"))
    if not train_labels:
        raise RuntimeError(
            "No YOLO label files found in robot_dataset/labels/train. "
            "Capture images first, label robot boxes, then train."
        )
    if not val_labels:
        raise RuntimeError(
            "No YOLO label files found in robot_dataset/labels/val. "
            "Move about 20 percent of labeled images and labels into the val split."
        )


def main():
    args = parse_args()
    require_labels(args.data)

    device = 0 if torch.cuda.is_available() else "cpu"
    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name=args.name,
        device=device,
        exist_ok=True,
    )

    best = Path(model.trainer.save_dir) / "weights" / "best.pt"
    print(f"best weights: {best}")


if __name__ == "__main__":
    main()
