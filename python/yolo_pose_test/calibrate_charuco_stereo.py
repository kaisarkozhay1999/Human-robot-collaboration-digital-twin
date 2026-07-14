import cv2
import numpy as np
import os
import glob

# ============================================================
# SETTINGS - MUST MATCH YOUR PRINTED ChArUco BOARD
# ============================================================
IMAGE_DIR = "charuco_pairs"
OUTPUT_FILE = "stereo_calibration_charuco.npz"

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# A2 board we prepared
SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.048   # 48 mm
MARKER_LENGTH_M = 0.036   # 36 mm

MIN_CORNERS_PER_IMAGE = 12


def load_image_pairs(image_dir: str):
    cam1_files = sorted(glob.glob(os.path.join(image_dir, "cam1_*.png")))
    cam2_files = sorted(glob.glob(os.path.join(image_dir, "cam2_*.png")))

    if not cam1_files or not cam2_files:
        raise RuntimeError(f"No images found in folder: {image_dir}")

    if len(cam1_files) != len(cam2_files):
        raise RuntimeError("Different number of cam1 and cam2 images")

    pairs = []
    for f1, f2 in zip(cam1_files, cam2_files):
        n1 = os.path.basename(f1).replace("cam1_", "").replace(".png", "")
        n2 = os.path.basename(f2).replace("cam2_", "").replace(".png", "")
        if n1 != n2:
            print(f"Skipping mismatched pair: {f1} / {f2}")
            continue
        pairs.append((f1, f2))

    if not pairs:
        raise RuntimeError("No matching cam1/cam2 image pairs found")

    return pairs


def main():
    print("Preparing ChArUco board...")
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        aruco_dict
    )

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    pairs = load_image_pairs(IMAGE_DIR)
    print(f"Found {len(pairs)} matched stereo pairs")

    # Per-camera data for intrinsic calibration
    all_charuco_corners_1 = []
    all_charuco_ids_1 = []
    all_charuco_corners_2 = []
    all_charuco_ids_2 = []

    # Stereo data
    stereo_objpoints = []
    stereo_imgpoints_1 = []
    stereo_imgpoints_2 = []

    image_size = None

    for f1, f2 in pairs:
        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)

        if img1 is None or img2 is None:
            print(f"Skipping unreadable pair: {os.path.basename(f1)}")
            continue

        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray1.shape[::-1]

        corners1, ids1, _ = detector.detectMarkers(gray1)
        corners2, ids2, _ = detector.detectMarkers(gray2)

        if ids1 is None or ids2 is None:
            print(f"Skipping pair (no markers): {os.path.basename(f1)}")
            continue

        ret1, charuco_corners1, charuco_ids1 = cv2.aruco.interpolateCornersCharuco(
            corners1, ids1, gray1, board
        )
        ret2, charuco_corners2, charuco_ids2 = cv2.aruco.interpolateCornersCharuco(
            corners2, ids2, gray2, board
        )

        if (
            charuco_corners1 is None or charuco_ids1 is None or
            charuco_corners2 is None or charuco_ids2 is None
        ):
            print(f"Skipping pair (no ChArUco corners): {os.path.basename(f1)}")
            continue

        if len(charuco_ids1) < MIN_CORNERS_PER_IMAGE or len(charuco_ids2) < MIN_CORNERS_PER_IMAGE:
            print(f"Skipping pair (too few corners): {os.path.basename(f1)}")
            continue

        # Save for intrinsic calibration
        all_charuco_corners_1.append(charuco_corners1)
        all_charuco_ids_1.append(charuco_ids1)

        all_charuco_corners_2.append(charuco_corners2)
        all_charuco_ids_2.append(charuco_ids2)

        # Find common detected ChArUco IDs for stereo
        ids1_flat = charuco_ids1.flatten()
        ids2_flat = charuco_ids2.flatten()
        common_ids = np.intersect1d(ids1_flat, ids2_flat)

        if len(common_ids) < MIN_CORNERS_PER_IMAGE:
            print(f"Skipping stereo part (few common IDs): {os.path.basename(f1)}")
            continue

        obj_pts = []
        img_pts_1 = []
        img_pts_2 = []

        chessboard_corners = board.getChessboardCorners()

        for cid in common_ids:
            idx1 = np.where(ids1_flat == cid)[0][0]
            idx2 = np.where(ids2_flat == cid)[0][0]

            obj_pts.append(chessboard_corners[cid])
            img_pts_1.append(charuco_corners1[idx1][0])
            img_pts_2.append(charuco_corners2[idx2][0])

        stereo_objpoints.append(np.array(obj_pts, dtype=np.float32))
        stereo_imgpoints_1.append(np.array(img_pts_1, dtype=np.float32))
        stereo_imgpoints_2.append(np.array(img_pts_2, dtype=np.float32))

        print(f"Accepted pair: {os.path.basename(f1)}")

    if len(all_charuco_corners_1) < 10 or len(all_charuco_corners_2) < 10:
        raise RuntimeError("Too few valid images for intrinsic calibration")

    if len(stereo_objpoints) < 10:
        raise RuntimeError("Too few valid stereo pairs for stereo calibration")

    print(f"\nValid intrinsic images cam1: {len(all_charuco_corners_1)}")
    print(f"Valid intrinsic images cam2: {len(all_charuco_corners_2)}")
    print(f"Valid stereo pairs: {len(stereo_objpoints)}")

    # ============================================================
    # Intrinsic calibration
    # ============================================================
    ret1, K1, dist1, rvecs1, tvecs1 = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners_1,
        all_charuco_ids_1,
        board,
        image_size,
        None,
        None
    )

    ret2, K2, dist2, rvecs2, tvecs2 = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners_2,
        all_charuco_ids_2,
        board,
        image_size,
        None,
        None
    )

    print("\nIntrinsic reprojection error cam1:", ret1)
    print("Intrinsic reprojection error cam2:", ret2)

    # ============================================================
    # Stereo calibration
    # ============================================================
    flags = cv2.CALIB_FIX_INTRINSIC

    stereo_ret, K1, dist1, K2, dist2, R, T, E, F = cv2.stereoCalibrate(
        stereo_objpoints,
        stereo_imgpoints_1,
        stereo_imgpoints_2,
        K1,
        dist1,
        K2,
        dist2,
        image_size,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
        flags=flags
    )

    print("\nStereo reprojection error:", stereo_ret)
    print("Translation T:\n", T)
    print("Rotation R:\n", R)

    np.savez(
        OUTPUT_FILE,
        K1=K1,
        dist1=dist1,
        K2=K2,
        dist2=dist2,
        R=R,
        T=T,
        E=E,
        F=F,
        image_width=image_size[0],
        image_height=image_size[1],
        squares_x=SQUARES_X,
        squares_y=SQUARES_Y,
        square_length_m=SQUARE_LENGTH_M,
        marker_length_m=MARKER_LENGTH_M
    )

    print(f"\nSaved calibration to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()