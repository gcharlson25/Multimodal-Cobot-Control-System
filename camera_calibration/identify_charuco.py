"""
Identify which ArUco dictionary a ChArUco board was generated from.

Usage:
    python identify_charuco.py [path/to/board_photo.jpg]
"""

import sys
import os
import cv2

DEFAULT_INPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_photo.jpg")

DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_INPUT
    img = cv2.imread(path)
    if img is None:
        print(f"Could not read image: {path}")
        sys.exit(1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    results = []
    for name, dict_id in DICTS.items():
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)
        n = 0 if ids is None else len(ids)
        results.append((name, n, None if ids is None else sorted(ids.flatten().tolist())))

    results.sort(key=lambda r: r[1], reverse=True)

    print(f"{'Dictionary':<22} {'Markers found':<14} IDs")
    for name, n, ids in results:
        print(f"{name:<22} {n:<14} {ids if ids else ''}")

    best = results[0]
    print()
    if best[1] == 0:
        print("No dictionary matched any markers. Try a clearer, well-lit, flat-on photo.")
    else:
        print(f"Best match: {best[0]} ({best[1]} markers detected)")
        print("If a second dictionary found a similar number of markers, take another photo"
              " (straighter angle, better lighting) to disambiguate.")


if __name__ == "__main__":
    main()
