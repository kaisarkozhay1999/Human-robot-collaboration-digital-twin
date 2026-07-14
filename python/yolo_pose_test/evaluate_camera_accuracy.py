import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "stereo_calibration_charuco_refined.npz"
DEFAULT_IMAGE_DIR = SCRIPT_DIR / "charuco_pairs"
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR / "camera_accuracy"
ARUCO_DICT = cv2.aruco.DICT_4X4_50


@dataclass
class Calibration:
    path: Path
    image_size: tuple[int, int]
    K1: np.ndarray
    dist1: np.ndarray
    K2: np.ndarray
    dist2: np.ndarray
    R: np.ndarray
    T: np.ndarray
    F: np.ndarray
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    intrinsic_error_cam1_px: float | None
    intrinsic_error_cam2_px: float | None
    stereo_error_px: float | None
    kept_pair_ids: set[str] | None


@dataclass
class Detection:
    path: Path
    image_size: tuple[int, int]
    corners: np.ndarray
    ids: np.ndarray


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate stereo camera accuracy from paired ChArUco images. "
            "Reports calibration reprojection errors, stereo reprojection error, "
            "3D board-corner reconstruction error, pairwise distance error, and "
            "optional measured-vs-estimated distance errors."
        )
    )
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--image-dir", default=str(DEFAULT_IMAGE_DIR))
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--min-common-corners", type=int, default=12)
    parser.add_argument(
        "--use-calibration-kept-pairs",
        action="store_true",
        help="Evaluate only the pair IDs stored as kept_pair_ids in the calibration file.",
    )
    parser.add_argument(
        "--pair-ids",
        default=None,
        help="Optional comma-separated pair IDs to evaluate, for example 0001,0008,0010.",
    )
    parser.add_argument(
        "--distance-csv",
        default=None,
        help=(
            "Optional CSV with columns measured_m and estimated_m. "
            "Use this for tape-measured real-world distance trials."
        ),
    )
    return parser.parse_args()


def load_calibration(path: Path) -> Calibration:
    data = np.load(path)
    image_size = (int(data["image_width"]), int(data["image_height"]))

    def optional_float(key: str):
        return float(data[key]) if key in data.files else None

    return Calibration(
        path=path,
        image_size=image_size,
        K1=data["K1"].astype(np.float64),
        dist1=data["dist1"].astype(np.float64),
        K2=data["K2"].astype(np.float64),
        dist2=data["dist2"].astype(np.float64),
        R=data["R"].astype(np.float64),
        T=data["T"].reshape(3, 1).astype(np.float64),
        F=data["F"].astype(np.float64),
        squares_x=int(data["squares_x"]),
        squares_y=int(data["squares_y"]),
        square_length_m=float(data["square_length_m"]),
        marker_length_m=float(data["marker_length_m"]),
        intrinsic_error_cam1_px=optional_float("intrinsic_reprojection_error_cam1"),
        intrinsic_error_cam2_px=optional_float("intrinsic_reprojection_error_cam2"),
        stereo_error_px=optional_float("stereo_reprojection_error"),
        kept_pair_ids=set(str(x) for x in data["kept_pair_ids"].tolist()) if "kept_pair_ids" in data.files else None,
    )


def scale_camera_matrix(K: np.ndarray, from_size: tuple[int, int], to_size: tuple[int, int]) -> np.ndarray:
    if from_size == to_size:
        return K.copy()

    sx = to_size[0] / from_size[0]
    sy = to_size[1] / from_size[1]
    scaled = K.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


def make_board(calibration: Calibration):
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard(
        (calibration.squares_x, calibration.squares_y),
        calibration.square_length_m,
        calibration.marker_length_m,
        dictionary,
    )
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    return board, detector


def load_image_pairs(image_dir: Path):
    cam1 = {
        path.stem.replace("cam1_", ""): path
        for path in sorted(image_dir.glob("cam1_*.png"))
    }
    cam2 = {
        path.stem.replace("cam2_", ""): path
        for path in sorted(image_dir.glob("cam2_*.png"))
    }
    pair_ids = sorted(set(cam1) & set(cam2))
    return [(pair_id, cam1[pair_id], cam2[pair_id]) for pair_id in pair_ids]


def detect_charuco(path: Path, board, detector) -> Detection | None:
    image = cv2.imread(str(path))
    if image is None:
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

    return Detection(
        path=path,
        image_size=image_size,
        corners=charuco_corners.astype(np.float64),
        ids=charuco_ids.flatten().astype(np.int32),
    )


def common_corner_data(cam1: Detection, cam2: Detection, board, min_common: int):
    common_ids = np.intersect1d(cam1.ids, cam2.ids)
    if len(common_ids) < min_common:
        return None

    chessboard = board.getChessboardCorners().astype(np.float64)
    object_points = []
    image_points_1 = []
    image_points_2 = []

    for corner_id in common_ids:
        idx1 = np.where(cam1.ids == corner_id)[0][0]
        idx2 = np.where(cam2.ids == corner_id)[0][0]
        object_points.append(chessboard[int(corner_id)])
        image_points_1.append(cam1.corners[idx1][0])
        image_points_2.append(cam2.corners[idx2][0])

    return (
        common_ids.astype(np.int32),
        np.asarray(object_points, dtype=np.float64),
        np.asarray(image_points_1, dtype=np.float64),
        np.asarray(image_points_2, dtype=np.float64),
    )


def triangulate_points(image_points_1, image_points_2, K1, dist1, K2, dist2, R, T):
    pts1 = image_points_1.reshape(-1, 1, 2).astype(np.float64)
    pts2 = image_points_2.reshape(-1, 1, 2).astype(np.float64)
    und1 = cv2.undistortPoints(pts1, K1, dist1).reshape(-1, 2).T
    und2 = cv2.undistortPoints(pts2, K2, dist2).reshape(-1, 2).T

    P1 = np.hstack((np.eye(3), np.zeros((3, 1))))
    P2 = np.hstack((R, T.reshape(3, 1)))
    hom = cv2.triangulatePoints(P1, P2, und1, und2)
    points_3d = (hom[:3] / hom[3]).T
    return points_3d.astype(np.float64)


def rigid_align(source: np.ndarray, target: np.ndarray):
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    source_zero = source - source_centroid
    target_zero = target - target_centroid
    H = source_zero.T @ target_zero
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = target_centroid - R @ source_centroid
    aligned = (R @ source.T).T + t
    return aligned, R, t


def reprojection_errors(points_3d, image_points_1, image_points_2, K1, dist1, K2, dist2, R, T):
    zero_rvec = np.zeros((3, 1), dtype=np.float64)
    zero_tvec = np.zeros((3, 1), dtype=np.float64)
    proj1, _ = cv2.projectPoints(points_3d, zero_rvec, zero_tvec, K1, dist1)
    rvec2, _ = cv2.Rodrigues(R)
    proj2, _ = cv2.projectPoints(points_3d, rvec2, T.reshape(3, 1), K2, dist2)
    err1 = np.linalg.norm(proj1.reshape(-1, 2) - image_points_1, axis=1)
    err2 = np.linalg.norm(proj2.reshape(-1, 2) - image_points_2, axis=1)
    return err1, err2


def epipolar_errors(image_points_1, image_points_2, F):
    x1 = np.hstack((image_points_1, np.ones((len(image_points_1), 1), dtype=np.float64)))
    x2 = np.hstack((image_points_2, np.ones((len(image_points_2), 1), dtype=np.float64)))
    lines2 = (F @ x1.T).T
    lines1 = (F.T @ x2.T).T
    numerator = np.abs(np.sum(x2 * lines2, axis=1))
    d2 = numerator / np.maximum(np.linalg.norm(lines2[:, :2], axis=1), 1e-12)
    d1 = numerator / np.maximum(np.linalg.norm(lines1[:, :2], axis=1), 1e-12)
    return 0.5 * (d1 + d2)


def pairwise_distance_errors(object_points, reconstructed_points):
    errors = []
    signed_errors = []
    for i in range(len(object_points)):
        for j in range(i + 1, len(object_points)):
            true_d = float(np.linalg.norm(object_points[i] - object_points[j]))
            if true_d <= 1e-9:
                continue
            recon_d = float(np.linalg.norm(reconstructed_points[i] - reconstructed_points[j]))
            signed = recon_d - true_d
            signed_errors.append(signed)
            errors.append(abs(signed))
    return np.asarray(errors, dtype=np.float64), np.asarray(signed_errors, dtype=np.float64)


def stats(values):
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {
            "n": 0,
            "mean": None,
            "rmse": None,
            "min": None,
            "median": None,
            "p90": None,
            "p95": None,
            "max": None,
        }

    return {
        "n": int(arr.size),
        "mean": float(np.mean(arr)),
        "rmse": float(math.sqrt(np.mean(arr * arr))),
        "min": float(np.min(arr)),
        "median": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def fmt(value, decimals=6):
    if value is None:
        return ""
    return f"{value:.{decimals}f}"


def evaluate_distance_csv(path: Path):
    if path is None:
        return None

    errors = []
    signed = []
    rows = []
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        required = {"measured_m", "estimated_m"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"{path} is missing required columns: {sorted(missing)}")
        for row in reader:
            measured = float(row["measured_m"])
            estimated = float(row["estimated_m"])
            error = abs(estimated - measured)
            errors.append(error)
            signed.append(estimated - measured)
            rows.append(
                {
                    "trial_id": row.get("trial_id", str(len(rows) + 1)),
                    "measured_m": measured,
                    "estimated_m": estimated,
                    "signed_error_m": estimated - measured,
                    "abs_error_m": error,
                }
            )

    return {
        "path": str(path),
        "rows": rows,
        "absolute_error_m": stats(errors),
        "signed_error_m": stats(signed),
    }


def write_pair_csv(path: Path, pair_rows):
    fieldnames = [
        "pair_id",
        "common_corners",
        "frame_width",
        "frame_height",
        "corner_error_mean_m",
        "corner_error_rmse_m",
        "corner_error_p95_m",
        "corner_error_max_m",
        "pairwise_distance_error_mean_m",
        "pairwise_distance_error_rmse_m",
        "pairwise_distance_error_p95_m",
        "pairwise_distance_error_max_m",
        "reprojection_cam1_mean_px",
        "reprojection_cam2_mean_px",
        "epipolar_mean_px",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in pair_rows:
            writer.writerow(row)


def write_markdown(path: Path, summary):
    lines = []
    lines.append("# Stereo Camera Accuracy Report")
    lines.append("")
    lines.append(f"Generated: {summary['generated_at']}")
    lines.append("")
    lines.append("## Inputs")
    lines.append("")
    lines.append(f"- Calibration: `{summary['calibration']['path']}`")
    lines.append(f"- Image folder: `{summary['image_dir']}`")
    lines.append(f"- Calibration image size: {summary['calibration']['image_size'][0]}x{summary['calibration']['image_size'][1]}")
    lines.append(f"- Board: {summary['board']['squares_x']} x {summary['board']['squares_y']} ChArUco")
    lines.append(f"- Square length: {summary['board']['square_length_m']:.6f} m")
    lines.append(f"- Marker length: {summary['board']['marker_length_m']:.6f} m")
    lines.append("")
    lines.append("## Calibration Reprojection Error")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("| --- | ---: |")
    lines.append(f"| Cam1 intrinsic reprojection error | {fmt(summary['calibration']['intrinsic_error_cam1_px'])} px |")
    lines.append(f"| Cam2 intrinsic reprojection error | {fmt(summary['calibration']['intrinsic_error_cam2_px'])} px |")
    lines.append(f"| Stereo reprojection error | {fmt(summary['calibration']['stereo_error_px'])} px |")
    lines.append("")
    lines.append("## ChArUco 3D Reconstruction Accuracy")
    lines.append("")
    lines.append("This section triangulates common ChArUco board corners from both cameras, rigidly aligns the reconstructed 3D corner cloud to the known board geometry, and reports the Euclidean residual in meters.")
    lines.append("")
    for title, key in [
        ("Rigid-aligned board corner error", "corner_error_m"),
        ("Pairwise board distance error", "pairwise_distance_abs_error_m"),
        ("Cam1 reprojection error from triangulated points", "reprojection_cam1_px"),
        ("Cam2 reprojection error from triangulated points", "reprojection_cam2_px"),
    ]:
        s = summary[key]
        unit = "m" if key.endswith("_m") else "px"
        lines.append(f"### {title}")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("| --- | ---: |")
        lines.append(f"| Samples | {s['n']} |")
        lines.append(f"| Mean | {fmt(s['mean'])} {unit} |")
        lines.append(f"| RMSE | {fmt(s['rmse'])} {unit} |")
        lines.append(f"| Median | {fmt(s['median'])} {unit} |")
        lines.append(f"| P95 | {fmt(s['p95'])} {unit} |")
        lines.append(f"| Max | {fmt(s['max'])} {unit} |")
        lines.append("")
    if summary.get("distance_csv"):
        s = summary["distance_csv"]["absolute_error_m"]
        lines.append("## Tape-Measured Distance Trials")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("| --- | ---: |")
        lines.append(f"| Samples | {s['n']} |")
        lines.append(f"| Mean absolute distance error | {fmt(s['mean'])} m |")
        lines.append(f"| RMSE absolute distance error | {fmt(s['rmse'])} m |")
        lines.append(f"| Median absolute distance error | {fmt(s['median'])} m |")
        lines.append(f"| P95 absolute distance error | {fmt(s['p95'])} m |")
        lines.append(f"| Max absolute distance error | {fmt(s['max'])} m |")
        lines.append("")
    lines.append("## Journal-Ready Wording")
    lines.append("")
    corner = summary["corner_error_m"]
    pairwise = summary["pairwise_distance_abs_error_m"]
    lines.append(
        "Stereo camera accuracy was evaluated using a calibrated ChArUco board. "
        "Common board corners detected in both camera views were triangulated using the stereo calibration, "
        "then compared with the known board geometry. "
        f"The rigid-aligned 3D corner reconstruction produced a mean error of {corner['mean']:.4f} m "
        f"and an RMSE of {corner['rmse']:.4f} m over {corner['n']} reconstructed board corners. "
        f"Pairwise board-distance consistency produced a mean absolute distance error of {pairwise['mean']:.4f} m "
        f"and an RMSE of {pairwise['rmse']:.4f} m."
    )
    lines.append("")
    lines.append("## Interpretation")
    lines.append("")
    lines.append("- Intrinsic reprojection error is a pixel-space measure of each camera's lens calibration quality.")
    lines.append("- Stereo reprojection error is a pixel-space measure of the two-camera calibration consistency.")
    lines.append("- Rigid-aligned board corner error is a metric 3D reconstruction error in meters.")
    lines.append("- Pairwise board distance error checks whether reconstructed 3D distances match known board distances.")
    lines.append("- For safety-distance claims, the meter-level 3D errors are more directly interpretable than pixel errors.")
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    calibration_path = Path(args.calibration)
    image_dir = Path(args.image_dir)
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)

    calibration = load_calibration(calibration_path)
    board, detector = make_board(calibration)
    pairs = load_image_pairs(image_dir)
    selected_pair_ids = None
    if args.use_calibration_kept_pairs:
        if not calibration.kept_pair_ids:
            raise RuntimeError("--use-calibration-kept-pairs was requested, but the calibration has no kept_pair_ids")
        selected_pair_ids = calibration.kept_pair_ids
    if args.pair_ids:
        explicit_ids = {part.strip() for part in args.pair_ids.split(",") if part.strip()}
        selected_pair_ids = explicit_ids if selected_pair_ids is None else selected_pair_ids & explicit_ids
    if selected_pair_ids is not None:
        pairs = [pair for pair in pairs if pair[0] in selected_pair_ids]
    if not pairs:
        raise RuntimeError(f"No selected cam1_*.png/cam2_*.png pairs found in {image_dir}")

    all_corner_errors = []
    all_pairwise_abs_errors = []
    all_pairwise_signed_errors = []
    all_reproj1 = []
    all_reproj2 = []
    all_epipolar = []
    pair_rows = []
    rejected = []
    frame_size = None

    for pair_id, cam1_path, cam2_path in pairs:
        cam1 = detect_charuco(cam1_path, board, detector)
        cam2 = detect_charuco(cam2_path, board, detector)
        if cam1 is None or cam2 is None:
            rejected.append({"pair_id": pair_id, "reason": "missing_charuco_detection"})
            continue
        if cam1.image_size != cam2.image_size:
            rejected.append({"pair_id": pair_id, "reason": "image_size_mismatch"})
            continue
        frame_size = cam1.image_size

        common = common_corner_data(cam1, cam2, board, args.min_common_corners)
        if common is None:
            rejected.append({"pair_id": pair_id, "reason": "too_few_common_corners"})
            continue

        _, object_points, image_points_1, image_points_2 = common
        K1 = scale_camera_matrix(calibration.K1, calibration.image_size, frame_size)
        K2 = scale_camera_matrix(calibration.K2, calibration.image_size, frame_size)

        points_3d = triangulate_points(
            image_points_1,
            image_points_2,
            K1,
            calibration.dist1,
            K2,
            calibration.dist2,
            calibration.R,
            calibration.T,
        )
        finite = np.isfinite(points_3d).all(axis=1)
        object_points = object_points[finite]
        image_points_1 = image_points_1[finite]
        image_points_2 = image_points_2[finite]
        points_3d = points_3d[finite]
        if len(points_3d) < args.min_common_corners:
            rejected.append({"pair_id": pair_id, "reason": "too_few_finite_triangulated_points"})
            continue

        aligned, _, _ = rigid_align(object_points, points_3d)
        corner_errors = np.linalg.norm(aligned - points_3d, axis=1)
        pairwise_abs, pairwise_signed = pairwise_distance_errors(object_points, points_3d)
        reproj1, reproj2 = reprojection_errors(
            points_3d,
            image_points_1,
            image_points_2,
            K1,
            calibration.dist1,
            K2,
            calibration.dist2,
            calibration.R,
            calibration.T,
        )
        epipolar = epipolar_errors(image_points_1, image_points_2, calibration.F)

        all_corner_errors.extend(corner_errors.tolist())
        all_pairwise_abs_errors.extend(pairwise_abs.tolist())
        all_pairwise_signed_errors.extend(pairwise_signed.tolist())
        all_reproj1.extend(reproj1.tolist())
        all_reproj2.extend(reproj2.tolist())
        all_epipolar.extend(epipolar.tolist())

        corner_stats = stats(corner_errors)
        pairwise_stats = stats(pairwise_abs)
        pair_rows.append(
            {
                "pair_id": pair_id,
                "common_corners": int(len(points_3d)),
                "frame_width": int(frame_size[0]),
                "frame_height": int(frame_size[1]),
                "corner_error_mean_m": fmt(corner_stats["mean"]),
                "corner_error_rmse_m": fmt(corner_stats["rmse"]),
                "corner_error_p95_m": fmt(corner_stats["p95"]),
                "corner_error_max_m": fmt(corner_stats["max"]),
                "pairwise_distance_error_mean_m": fmt(pairwise_stats["mean"]),
                "pairwise_distance_error_rmse_m": fmt(pairwise_stats["rmse"]),
                "pairwise_distance_error_p95_m": fmt(pairwise_stats["p95"]),
                "pairwise_distance_error_max_m": fmt(pairwise_stats["max"]),
                "reprojection_cam1_mean_px": fmt(float(np.mean(reproj1))),
                "reprojection_cam2_mean_px": fmt(float(np.mean(reproj2))),
                "epipolar_mean_px": fmt(float(np.mean(epipolar))),
            }
        )

    distance_csv_result = evaluate_distance_csv(Path(args.distance_csv)) if args.distance_csv else None
    summary = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "calibration": {
            "path": str(calibration_path),
            "image_size": list(calibration.image_size),
            "intrinsic_error_cam1_px": calibration.intrinsic_error_cam1_px,
            "intrinsic_error_cam2_px": calibration.intrinsic_error_cam2_px,
            "stereo_error_px": calibration.stereo_error_px,
            "baseline_m": float(np.linalg.norm(calibration.T)),
        },
        "board": {
            "squares_x": calibration.squares_x,
            "squares_y": calibration.squares_y,
            "square_length_m": calibration.square_length_m,
            "marker_length_m": calibration.marker_length_m,
        },
        "image_dir": str(image_dir),
        "selected_pair_ids": sorted(selected_pair_ids) if selected_pair_ids is not None else None,
        "image_pairs_total": len(pairs),
        "image_pairs_used": len(pair_rows),
        "image_pairs_rejected": rejected,
        "frame_size": list(frame_size) if frame_size else None,
        "corner_error_m": stats(all_corner_errors),
        "pairwise_distance_abs_error_m": stats(all_pairwise_abs_errors),
        "pairwise_distance_signed_error_m": stats(all_pairwise_signed_errors),
        "reprojection_cam1_px": stats(all_reproj1),
        "reprojection_cam2_px": stats(all_reproj2),
        "epipolar_error_px": stats(all_epipolar),
        "distance_csv": distance_csv_result,
    }

    summary_path = output_dir / "camera_accuracy_summary.json"
    pair_csv_path = output_dir / "camera_accuracy_pairs.csv"
    report_path = output_dir / "camera_accuracy_report.md"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_pair_csv(pair_csv_path, pair_rows)
    write_markdown(report_path, summary)

    print(f"Used {len(pair_rows)} / {len(pairs)} stereo pairs")
    print(f"Output: {output_dir}")
    print("")
    print("Calibration reprojection:")
    print(f"  Cam1 intrinsic: {fmt(calibration.intrinsic_error_cam1_px)} px")
    print(f"  Cam2 intrinsic: {fmt(calibration.intrinsic_error_cam2_px)} px")
    print(f"  Stereo:         {fmt(calibration.stereo_error_px)} px")
    print("")
    print("3D ChArUco board reconstruction:")
    corner = summary["corner_error_m"]
    pairwise = summary["pairwise_distance_abs_error_m"]
    print(f"  Corner mean/RMSE/P95:   {fmt(corner['mean'])} / {fmt(corner['rmse'])} / {fmt(corner['p95'])} m")
    print(f"  Distance mean/RMSE/P95: {fmt(pairwise['mean'])} / {fmt(pairwise['rmse'])} / {fmt(pairwise['p95'])} m")
    print("")
    print(f"Wrote {summary_path}")
    print(f"Wrote {pair_csv_path}")
    print(f"Wrote {report_path}")


if __name__ == "__main__":
    main()
