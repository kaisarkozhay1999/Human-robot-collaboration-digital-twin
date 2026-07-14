import argparse
from pathlib import Path

import cv2


CLASS_ID = 0
CLASS_NAME = "robot"


def parse_args():
    parser = argparse.ArgumentParser(description="Simple YOLO box labeler for the robot dataset.")
    parser.add_argument("--images", default="robot_dataset/images/train", help="Image directory.")
    parser.add_argument("--labels", default="robot_dataset/labels/train", help="YOLO label directory.")
    parser.add_argument("--start", type=int, default=0, help="Start image index.")
    return parser.parse_args()


def list_images(images_dir):
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    return sorted(p for p in images_dir.iterdir() if p.suffix.lower() in exts)


def label_path_for(image_path, labels_dir):
    return labels_dir / f"{image_path.stem}.txt"


def load_box(image_path, labels_dir, image_w, image_h):
    label_path = label_path_for(image_path, labels_dir)
    if not label_path.exists():
        return None

    line = label_path.read_text().strip().splitlines()
    if not line:
        return None

    parts = line[0].split()
    if len(parts) != 5:
        return None

    _, xc, yc, bw, bh = parts
    xc = float(xc) * image_w
    yc = float(yc) * image_h
    bw = float(bw) * image_w
    bh = float(bh) * image_h

    x1 = int(round(xc - bw / 2))
    y1 = int(round(yc - bh / 2))
    x2 = int(round(xc + bw / 2))
    y2 = int(round(yc + bh / 2))
    return normalize_box((x1, y1, x2, y2), image_w, image_h)


def save_box(image_path, labels_dir, box, image_w, image_h):
    labels_dir.mkdir(parents=True, exist_ok=True)
    label_path = label_path_for(image_path, labels_dir)

    if box is None:
        label_path.write_text("")
        print(f"saved empty label {label_path}")
        return

    x1, y1, x2, y2 = normalize_box(box, image_w, image_h)
    xc = ((x1 + x2) / 2.0) / image_w
    yc = ((y1 + y2) / 2.0) / image_h
    bw = (x2 - x1) / image_w
    bh = (y2 - y1) / image_h
    label_path.write_text(f"{CLASS_ID} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")
    print(f"saved {label_path}")


def normalize_box(box, image_w, image_h):
    x1, y1, x2, y2 = box
    x1, x2 = sorted((int(x1), int(x2)))
    y1, y2 = sorted((int(y1), int(y2)))
    x1 = max(0, min(image_w - 1, x1))
    x2 = max(0, min(image_w - 1, x2))
    y1 = max(0, min(image_h - 1, y1))
    y2 = max(0, min(image_h - 1, y2))
    return x1, y1, x2, y2


class LabelState:
    def __init__(self):
        self.box = None
        self.drawing = False
        self.start = None
        self.current = None


def draw_view(image, image_path, index, total, state, saved):
    view = image.copy()
    h, w = image.shape[:2]

    box = state.box
    if state.drawing and state.start is not None and state.current is not None:
        box = (*state.start, *state.current)

    if box is not None:
        x1, y1, x2, y2 = normalize_box(box, w, h)
        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(view, CLASS_NAME, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    status = "saved" if saved else "unsaved"
    lines = [
        f"{index + 1}/{total}  {image_path.name}  {status}",
        "Drag box | S save | C clear | D/Right next | A/Left previous | Q quit",
    ]

    y = 26
    for line in lines:
        cv2.putText(view, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        y += 26

    return view


def make_mouse_callback(state):
    def on_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            state.drawing = True
            state.start = (x, y)
            state.current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state.drawing:
            state.current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and state.drawing:
            state.drawing = False
            state.current = (x, y)
            state.box = (*state.start, *state.current)
    return on_mouse


def main():
    args = parse_args()
    images_dir = Path(args.images)
    labels_dir = Path(args.labels)
    images = list_images(images_dir)

    if not images:
        raise RuntimeError(f"No images found in {images_dir}")

    index = max(0, min(args.start, len(images) - 1))
    window = "Robot Labeler"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    state = LabelState()
    loaded_index = None
    saved = True

    while True:
        image_path = images[index]
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"could not read {image_path}")
            index = min(index + 1, len(images) - 1)
            continue

        h, w = image.shape[:2]
        if loaded_index != index:
            state = LabelState()
            state.box = load_box(image_path, labels_dir, w, h)
            cv2.setMouseCallback(window, make_mouse_callback(state))
            loaded_index = index
            saved = True

        cv2.imshow(window, draw_view(image, image_path, index, len(images), state, saved))
        key = cv2.waitKey(20) & 0xFF

        if key in (27, ord("q")):
            break
        if key in (ord("s"),):
            save_box(image_path, labels_dir, state.box, w, h)
            saved = True
        elif key in (ord("c"),):
            state.box = None
            saved = False
        elif key in (ord("d"), 83):
            index = min(index + 1, len(images) - 1)
        elif key in (ord("a"), 81):
            index = max(index - 1, 0)
        elif state.drawing:
            saved = False

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
