"""
Solve camera intrinsics (focal length, optical center, distortion) from
a folder of ChArUco board photos captured by charuco_capture.py.

Usage:
    python charuco_calibrate.py

Reads images from charuco_images/, writes camera_calibration.json.
"""

import os
import glob
import json
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "images")
OUTPUT_FILE = os.path.join(BASE_DIR, "camera_calibration.json")

SQUARES_X = 4
SQUARES_Y = 5
SQUARE_LENGTH_MM = 50.0
MARKER_LENGTH_MM = 37.0
ARUCO_DICT = cv2.aruco.DICT_4X4_50

MIN_CORNERS_PER_IMAGE = 6


def main():
    images = sorted(glob.glob(os.path.join(IMAGE_DIR, "*.jpg")))
    if not images:
        print(f"No images found in {IMAGE_DIR}. Run charuco_capture.py first.")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH_MM, MARKER_LENGTH_MM, dictionary)
    detector = cv2.aruco.CharucoDetector(board)

    all_obj_points = []
    all_img_points = []
    image_size = None
    used = 0

    for path in images:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

        if charuco_corners is None or len(charuco_corners) < MIN_CORNERS_PER_IMAGE:
            print(f"Skipping {os.path.basename(path)}: only "
                  f"{0 if charuco_corners is None else len(charuco_corners)} corners found")
            continue

        obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
        if obj_points is None or len(obj_points) < MIN_CORNERS_PER_IMAGE:
            continue

        all_obj_points.append(obj_points)
        all_img_points.append(img_points)
        used += 1

    print(f"Using {used} / {len(images)} images for calibration.")
    if used < 10:
        print("Warning: fewer than 10 usable images. Results may be unreliable - "
              "capture more with better board coverage of the frame and varied distances.")
    if used == 0:
        print("No usable images. Aborting.")
        return

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        all_obj_points, all_img_points, image_size, None, None
    )

    print(f"Reprojection error (RMS): {ret:.4f} px")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs.ravel())

    result = {
        "image_size": image_size,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.ravel().tolist(),
        "reprojection_error_px": ret,
        "num_images_used": used,
        "board": {
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_length_mm": SQUARE_LENGTH_MM,
            "marker_length_mm": MARKER_LENGTH_MM,
            "dictionary": "DICT_4X4_50",
        },
    }
    with open(OUTPUT_FILE, "w") as f:
        json.dump(result, f, indent=2)
    print(f"Saved: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
