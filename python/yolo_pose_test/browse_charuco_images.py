import cv2
import numpy as np
import os
import glob

IMAGE_DIR = "charuco_pairs"

ARUCO_DICT = cv2.aruco.DICT_4X4_50
SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.048
MARKER_LENGTH_M = 0.036

DISPLAY_WIDTH = 1400


def resize_to_width(frame, target_w):
    h, w = frame.shape[:2]
    scale = target_w / w
    new_h = int(h * scale)
    return cv2.resize(frame, (target_w, new_h), interpolation=cv2.INTER_AREA)


def draw_aruco_marker_corners(img, corners, ids):
    vis = img.copy()

    if ids is None or len(ids) == 0:
        return vis

    for i, marker_id in enumerate(ids.flatten()):
        pts = corners[i][0]
        pts_int = np.int32(pts).reshape((-1, 1, 2))
        cv2.polylines(vis, [pts_int], True, (255, 0, 0), 2)

        for j, (x, y) in enumerate(pts):
            x = int(x)
            y = int(y)
            cv2.circle(vis, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(
                vis,
                str(j),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA
            )

        x0, y0 = int(pts[0][0]), int(pts[0][1])
        cv2.putText(
            vis,
            f"ID {marker_id}",
            (x0, max(20, y0 - 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2,
            cv2.LINE_AA
        )

    return vis


def main():
    files = sorted(glob.glob(os.path.join(IMAGE_DIR, "cam1_*.png")))
    if not files:
        raise RuntimeError("No cam1 images found in charuco_pairs")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        aruco_dict
    )

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    idx = 0
    total = len(files)

    window_name = "Browse ChArUco Images"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1500, 900)

    while True:
        image_path = files[idx]
        img = cv2.imread(image_path)
        if img is None:
            raise RuntimeError(f"Could not read image: {image_path}")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)

        num_markers = 0 if ids is None else len(ids)

        vis = draw_aruco_marker_corners(img, corners, ids)

        num_charuco = 0
        if ids is not None and len(ids) > 0:
            ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board
            )

            if charuco_corners is not None and charuco_ids is not None:
                num_charuco = len(charuco_ids)

                for k, pt in enumerate(charuco_corners):
                    x, y = pt[0]
                    x = int(x)
                    y = int(y)
                    cv2.circle(vis, (x, y), 5, (0, 255, 0), -1)
                    cv2.putText(
                        vis,
                        f"C{int(charuco_ids[k][0])}",
                        (x + 8, y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA
                    )

        cv2.putText(
            vis,
            "Red = 4 marker corners, Green = ChArUco corners",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            vis,
            f"File: {os.path.basename(image_path)}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            vis,
            f"Markers detected: {num_markers}",
            (20, 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            vis,
            f"ChArUco corners detected: {num_charuco}",
            (20, 155),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            vis,
            "Left/Right arrows or A/D to browse, Q or ESC to quit",
            (20, 195),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        vis = resize_to_width(vis, DISPLAY_WIDTH)
        cv2.imshow(window_name, vis)

        key = cv2.waitKey(0) & 0xFF

        if key == 27 or key == ord("q"):
            break
        elif key == 81 or key == ord("a"):
            idx = max(0, idx - 1)
        elif key == 83 or key == ord("d"):
            idx = min(total - 1, idx + 1)
        else:
            idx = min(total - 1, idx + 1)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()