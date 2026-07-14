import cv2
import os

IMAGE_PATH = r"charuco_pairs\cam1_0001.png"

ARUCO_DICT = cv2.aruco.DICT_4X4_50
SQUARES_X = 12
SQUARES_Y = 8
SQUARE_LENGTH_M = 0.048
MARKER_LENGTH_M = 0.036

img = cv2.imread(IMAGE_PATH)
if img is None:
    raise RuntimeError("Could not read image")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y),
    SQUARE_LENGTH_M,
    MARKER_LENGTH_M,
    aruco_dict
)

detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

corners, ids, _ = detector.detectMarkers(gray)

print("Markers detected:", 0 if ids is None else len(ids))

vis = img.copy()

if ids is not None and len(ids) > 0:
    cv2.aruco.drawDetectedMarkers(vis, corners, ids)

    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )

    if charuco_corners is not None and charuco_ids is not None:
        print("ChArUco corners detected:", len(charuco_ids))
        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids, (0, 255, 0))
    else:
        print("ChArUco corners detected: 0")

cv2.imshow("check", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()