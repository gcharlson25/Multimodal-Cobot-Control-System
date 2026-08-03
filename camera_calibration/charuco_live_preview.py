"""
Live preview: shows the RealSense camera feed with detected ArUco marker
IDs and ChArUco board corners overlaid in real time.

Useful for demoing detection - hold the board up and point at a marker
on screen to show its ID number.

Usage:
    python charuco_live_preview.py

ESC to quit.
"""

import cv2
import numpy as np
import pyrealsense2 as rs

SQUARES_X = 4
SQUARES_Y = 5
SQUARE_LENGTH_MM = 50.0
MARKER_LENGTH_MM = 37.0
ARUCO_DICT = cv2.aruco.DICT_4X4_50


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH_MM, MARKER_LENGTH_MM, dictionary)
    detector = cv2.aruco.CharucoDetector(board)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    cv2.namedWindow("Charuco Live Detection", cv2.WINDOW_NORMAL)
    print("Showing live marker/corner detection. ESC to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            display = image.copy()

            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

            if marker_ids is not None and len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)

            if charuco_corners is not None and len(charuco_corners) > 0:
                cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids, (0, 255, 0))

            n_markers = 0 if marker_ids is None else len(marker_ids)
            n_corners = 0 if charuco_corners is None else len(charuco_corners)
            cv2.putText(display, f"Markers: {n_markers}   Board corners: {n_corners}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("Charuco Live Detection", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
