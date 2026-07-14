import argparse
import glob
import os
from dataclasses import dataclass

import cv2
import numpy as np


# ============================================================
# SETTINGS - MUST MATCH YOUR PRINTED ChArUco BOARD
# ============================================================
IMAGE_DIR = "charuco_pairs"
OUTPUT_FILE = "stereo_calibration_charuco_refined.npz"

ARUCO_DICT = cv2.aruco.DICT_4X4_50

SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.048
MARKER_LENGTH_M = 0.036

MIN_CORNERS_PER_IMAGE = 12
MIN_COMMON_CORNERS = 12
MIN_STEREO_PAIRS = 10

# Keep the strongest stereo subset by default. The current image set scores best
# with the top 10 pair prefix while still meeting the minimum stereo view count.
DEFAULT_TOP_PAIRS = 10


@dataclass
class CharucoDetection:
    filename: str
    corners: np.ndarray
    ids: np.ndarray
    image_size: tuple[int, int]
    coverage: float


@dataclass
class StereoPairScore:
    pair_name: str
    cam1_file: str
    cam2_file: str
    cam1: CharucoDetection
    cam2: CharucoDetection
    common_ids: np.ndarray
    min_coverage: float
    avg_coverage: float
    score: float


def load_image_pairs(image_dir: str) -> list[tuple[str, str]]:
    cam1_files = sorted(glob.glob(os.path.join(image_dir, "cam1_*.png")))
    cam2_files = sorted(glob.glob(os.path.join(image_dir, "cam2_*.png")))

    if not cam1_files or not cam2_files:
        raise RuntimeError(f"No images found in folder: {image_dir}")

    cam1_by_id = {
        os.path.basename(path).replace("cam1_", "").replace(".png", ""): path
        for path in cam1_files
    }
    cam2_by_id = {
        os.path.basename(path).replace("cam2_", "").replace(".png", ""): path
        for path in cam2_files
    }

    common_ids = sorted(set(cam1_by_id) & set(cam2_by_id))
    if not common_ids:
        raise RuntimeError("No matching cam1/cam2 image pairs found")

    missing_cam2 = sorted(set(cam1_by_id) - set(cam2_by_id))
    missing_cam1 = sorted(set(cam2_by_id) - set(cam1_by_id))
    for pair_id in missing_cam2:
        print(f"Skipping cam1_{pair_id}.png: no matching cam2 image")
    for pair_id in missing_cam1:
        print(f"Skipping cam2_{pair_id}.png: no matching cam1 image")

    return [(cam1_by_id[pair_id], cam2_by_id[pair_id]) for pair_id in common_ids]


def create_board_and_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        aruco_dict,
    )

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    return board, detector


def charuco_coverage(charuco_corners: np.ndarray, image_size: tuple[int, int]) -> float:
    points = charuco_corners.reshape(-1, 2).astype(np.float32)
    if len(points) < 3:
        return 0.0

    hull = cv2.convexHull(points)
    hull_area = cv2.contourArea(hull)
    image_area = float(image_size[0] * image_size[1])
    if image_area <= 0.0:
        return 0.0

    return float(hull_area / image_area)


def detect_charuco(path: str, board, detector) -> CharucoDetection | None:
    image = cv2.imread(path)
    if image is None:
        print(f"Skipping unreadable image: {path}")
        return None

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    if marker_ids is None or len(marker_ids) == 0:
        return None

    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
    )
    if charuco_corners is None or charuco_ids is None:
        return None

    if len(charuco_ids) < MIN_CORNERS_PER_IMAGE:
        return None

    return CharucoDetection(
        filename=path,
        corners=charuco_corners,
        ids=charuco_ids,
        image_size=image_size,
        coverage=charuco_coverage(charuco_corners, image_size),
    )


def build_stereo_score(
    cam1_path: str,
    cam2_path: str,
    cam1: CharucoDetection,
    cam2: CharucoDetection,
) -> StereoPairScore | None:
    ids1 = cam1.ids.flatten()
    ids2 = cam2.ids.flatten()
    common_ids = np.intersect1d(ids1, ids2)

    if len(common_ids) < MIN_COMMON_CORNERS:
        return None

    min_coverage = min(cam1.coverage, cam2.coverage)
    avg_coverage = (cam1.coverage + cam2.coverage) * 0.5

    # A pair is strongest when both cameras see many of the same board corners
    # and the board occupies a meaningful image area in both images.
    score = float(len(common_ids) * min_coverage)

    pair_id = os.path.basename(cam1_path).replace("cam1_", "").replace(".png", "")
    return StereoPairScore(
        pair_name=pair_id,
        cam1_file=cam1_path,
        cam2_file=cam2_path,
        cam1=cam1,
        cam2=cam2,
        common_ids=common_ids,
        min_coverage=min_coverage,
        avg_coverage=avg_coverage,
        score=score,
    )


def choose_best_pairs(
    scores: list[StereoPairScore],
    top_pairs: int | None,
    keep_ratio: float | None,
    max_pairs: int | None,
) -> list[StereoPairScore]:
    if not scores:
        return []

    sorted_scores = sorted(
        scores,
        key=lambda item: (
            item.score,
            len(item.common_ids),
            item.min_coverage,
            item.avg_coverage,
        ),
        reverse=True,
    )

    if top_pairs is not None and top_pairs > 0:
        keep_count = top_pairs
    elif keep_ratio is not None:
        keep_count = int(np.ceil(len(sorted_scores) * keep_ratio))
    else:
        keep_count = len(sorted_scores)

    if max_pairs is not None and max_pairs > 0:
        keep_count = min(keep_count, max_pairs)

    keep_count = max(MIN_STEREO_PAIRS, keep_count)
    keep_count = min(len(sorted_scores), keep_count)

    return sorted_scores[:keep_count]


def stereo_points_from_pair(pair: StereoPairScore, chessboard_corners: np.ndarray):
    ids1 = pair.cam1.ids.flatten()
    ids2 = pair.cam2.ids.flatten()

    obj_points = []
    img_points_1 = []
    img_points_2 = []

    for corner_id in pair.common_ids:
        idx1 = np.where(ids1 == corner_id)[0][0]
        idx2 = np.where(ids2 == corner_id)[0][0]

        obj_points.append(chessboard_corners[corner_id])
        img_points_1.append(pair.cam1.corners[idx1][0])
        img_points_2.append(pair.cam2.corners[idx2][0])

    return (
        np.array(obj_points, dtype=np.float32),
        np.array(img_points_1, dtype=np.float32),
        np.array(img_points_2, dtype=np.float32),
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Filter ChArUco stereo pairs by corner count and board coverage, then recalibrate stereo."
    )
    parser.add_argument("--image-dir", default=IMAGE_DIR)
    parser.add_argument("--output", default=OUTPUT_FILE)
    parser.add_argument(
        "--top-pairs",
        type=int,
        default=DEFAULT_TOP_PAIRS,
        help="Keep this many strongest pairs. Use 0 with --keep-ratio to select by ratio.",
    )
    parser.add_argument(
        "--keep-ratio",
        type=float,
        default=None,
        help="Optional ratio of scored pairs to keep when --top-pairs is 0.",
    )
    parser.add_argument(
        "--max-pairs",
        type=int,
        default=None,
        help="Optional cap after applying --top-pairs or --keep-ratio.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.keep_ratio is not None and not 0.0 < args.keep_ratio <= 1.0:
        raise ValueError("--keep-ratio must be in the range (0, 1]")
    if args.top_pairs < 0:
        raise ValueError("--top-pairs must be >= 0")

    board, detector = create_board_and_detector()
    pairs = load_image_pairs(args.image_dir)
    print(f"Found {len(pairs)} matched stereo pairs")

    detections_1: list[CharucoDetection] = []
    detections_2: list[CharucoDetection] = []
    stereo_scores: list[StereoPairScore] = []
    image_size = None

    for cam1_path, cam2_path in pairs:
        cam1 = detect_charuco(cam1_path, board, detector)
        cam2 = detect_charuco(cam2_path, board, detector)

        if cam1 is not None:
            detections_1.append(cam1)
            image_size = image_size or cam1.image_size
        if cam2 is not None:
            detections_2.append(cam2)
            image_size = image_size or cam2.image_size

        if cam1 is None or cam2 is None:
            print(f"Rejected pair {os.path.basename(cam1_path)}: missing usable ChArUco detection")
            continue

        if cam1.image_size != cam2.image_size:
            print(f"Rejected pair {os.path.basename(cam1_path)}: image sizes differ")
            continue

        pair_score = build_stereo_score(cam1_path, cam2_path, cam1, cam2)
        if pair_score is None:
            print(f"Rejected pair {os.path.basename(cam1_path)}: too few common ChArUco corners")
            continue

        stereo_scores.append(pair_score)

    if image_size is None:
        raise RuntimeError("No readable images found")

    if len(detections_1) < MIN_STEREO_PAIRS or len(detections_2) < MIN_STEREO_PAIRS:
        raise RuntimeError("Too few valid images for intrinsic calibration")

    if len(stereo_scores) < MIN_STEREO_PAIRS:
        raise RuntimeError("Too few valid stereo pairs for stereo calibration")

    kept_pairs = choose_best_pairs(stereo_scores, args.top_pairs, args.keep_ratio, args.max_pairs)
    if len(kept_pairs) < MIN_STEREO_PAIRS:
        raise RuntimeError("Too few kept stereo pairs for stereo calibration")

    print(f"\nValid intrinsic images cam1: {len(detections_1)}")
    print(f"Valid intrinsic images cam2: {len(detections_2)}")
    print(f"Valid stereo pairs before filtering: {len(stereo_scores)}")
    print(f"Kept stereo pairs after filtering: {len(kept_pairs)}")

    print("\nKept pair list, strongest first:")
    for rank, pair in enumerate(kept_pairs, start=1):
        print(
            f"{rank:02d}. {pair.pair_name}  "
            f"score={pair.score:.4f}  "
            f"common={len(pair.common_ids):2d}  "
            f"coverage1={pair.cam1.coverage:.4f}  "
            f"coverage2={pair.cam2.coverage:.4f}"
        )

    print("\nRunning intrinsic calibration from all usable detections...")
    ret1, K1, dist1, _, _ = cv2.aruco.calibrateCameraCharuco(
        [detection.corners for detection in detections_1],
        [detection.ids for detection in detections_1],
        board,
        image_size,
        None,
        None,
    )
    ret2, K2, dist2, _, _ = cv2.aruco.calibrateCameraCharuco(
        [detection.corners for detection in detections_2],
        [detection.ids for detection in detections_2],
        board,
        image_size,
        None,
        None,
    )

    print("Intrinsic reprojection error cam1:", ret1)
    print("Intrinsic reprojection error cam2:", ret2)

    chessboard_corners = board.getChessboardCorners()
    stereo_objpoints = []
    stereo_imgpoints_1 = []
    stereo_imgpoints_2 = []

    for pair in kept_pairs:
        obj_pts, img_pts_1, img_pts_2 = stereo_points_from_pair(pair, chessboard_corners)
        stereo_objpoints.append(obj_pts)
        stereo_imgpoints_1.append(img_pts_1)
        stereo_imgpoints_2.append(img_pts_2)

    print("\nRunning refined stereo calibration...")
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
        flags=cv2.CALIB_FIX_INTRINSIC,
    )

    print("\nRefined stereo reprojection error:", stereo_ret)
    print("Translation T:\n", T)
    print("Rotation R:\n", R)

    np.savez(
        args.output,
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
        marker_length_m=MARKER_LENGTH_M,
        intrinsic_reprojection_error_cam1=ret1,
        intrinsic_reprojection_error_cam2=ret2,
        stereo_reprojection_error=stereo_ret,
        top_pairs=args.top_pairs,
        keep_ratio=-1.0 if args.keep_ratio is None else args.keep_ratio,
        max_pairs=-1 if args.max_pairs is None else args.max_pairs,
        kept_pair_ids=np.array([pair.pair_name for pair in kept_pairs]),
        kept_pair_scores=np.array([pair.score for pair in kept_pairs], dtype=np.float64),
        kept_pair_common_corners=np.array([len(pair.common_ids) for pair in kept_pairs], dtype=np.int32),
        kept_pair_coverage_cam1=np.array([pair.cam1.coverage for pair in kept_pairs], dtype=np.float64),
        kept_pair_coverage_cam2=np.array([pair.cam2.coverage for pair in kept_pairs], dtype=np.float64),
    )

    print(f"\nSaved refined calibration to: {args.output}")


if __name__ == "__main__":
    main()
