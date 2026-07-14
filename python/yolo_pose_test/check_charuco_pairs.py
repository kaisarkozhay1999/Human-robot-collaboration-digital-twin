import cv2
import os
import glob
import numpy as np

# ============================================================
# MUST MATCH YOUR PRINTED BOARD
# ============================================================
IMAGE_DIR = "charuco_pairs"

ARUCO_DICT = cv2.aruco.DICT_4X4_50
SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.035
MARKER_LENGTH_M = 0.026

# Smaller display so both images fit
DISPLAY_WIDTH = 600


def resize_to_width(frame, target_w):
    h, w = frame.shape[:2]
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


def annotate_image(img, detector, board, name):
    vis = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    num_markers = 0
    num_charuco = 0

    if ids is not None and len(ids) > 0:
        num_markers = len(ids)
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )

        if charuco_corners is not None and charuco_ids is not None:
            num_charuco = len(charuco_ids)
            cv2.aruco.drawDetectedCornersCharuco(
                vis, charuco_corners, charuco_ids, (0, 255, 0)
            )

    cv2.putText(
        vis,
        f"{name}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        vis,
        f"Markers: {num_markers}",
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        vis,
        f"ChArUco corners: {num_charuco}",
        (20, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    return vis, num_markers, num_charuco


def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        aruco_dict
    )

    cam1_files = sorted(glob.glob(os.path.join(IMAGE_DIR, "cam1_*.png")))
    cam2_files = sorted(glob.glob(os.path.join(IMAGE_DIR, "cam2_*.png")))

    if not cam1_files or not cam2_files:
        raise RuntimeError("No images found in charuco_pairs")

    if len(cam1_files) != len(cam2_files):
        raise RuntimeError("Different number of cam1/cam2 images")

    idx = 0
    total = len(cam1_files)

    window_name = "Check ChArUco Pairs"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1400, 800)

    while True:
        f1 = cam1_files[idx]
        f2 = cam2_files[idx]

        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)

        vis1, m1, c1 = annotate_image(img1, detector, board, f"Cam1 - {os.path.basename(f1)}")
        vis2, m2, c2 = annotate_image(img2, detector, board, f"Cam2 - {os.path.basename(f2)}")

        vis1 = resize_to_width(vis1, DISPLAY_WIDTH)
        vis2 = resize_to_width(vis2, DISPLAY_WIDTH)
        vis1, vis2 = equalize_heights(vis1, vis2)

        combined = np.hstack((vis1, vis2))

        cv2.putText(
            combined,
            f"Pair {idx + 1}/{total}   Left/Right arrows: navigate   Q or ESC: quit",
            (20, combined.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.imshow(window_name, combined)

        key = cv2.waitKey(0) & 0xFF

        if key == 27 or key == ord("q"):
            break
        elif key == 81 or key == ord("a"):   # left arrow or A
            idx = max(0, idx - 1)
        elif key == 83 or key == ord("d"):   # right arrow or D
            idx = min(total - 1, idx + 1)
        else:
            idx = min(total - 1, idx + 1)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()