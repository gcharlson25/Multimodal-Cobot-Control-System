"""
Demo: measure real-world distances using the calibrated camera + RealSense depth.

1. Accuracy check: automatically measures the distance between adjacent
   ChArUco board corners (known to be exactly SQUARE_LENGTH_MM apart) and
   compares against that known value - shows measurement error live.

2. Click-to-measure: click two points anywhere in the frame, and it reports
   the real-world distance between them in mm, using the calibrated
   intrinsics and the depth at each point.

Usage:
    python measure_demo.py

ESC to quit. Click two points to measure between them; click again to
start a new measurement.
"""

import os
import json
import cv2
import numpy as np
import pyrealsense2 as rs

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(BASE_DIR, "camera_calibration.json")

SQUARES_X = 4
SQUARES_Y = 5
SQUARE_LENGTH_MM = 50.0
MARKER_LENGTH_MM = 37.0
ARUCO_DICT = cv2.aruco.DICT_4X4_50

click_points = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(click_points) >= 2:
            click_points.clear()
        click_points.append((x, y))


def deproject(u, v, depth_mm, camera_matrix):
    fx = camera_matrix[0][0]
    fy = camera_matrix[1][1]
    cx = camera_matrix[0][2]
    cy = camera_matrix[1][2]
    z = depth_mm
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])


def get_depth_mm(depth_frame, u, v):
    d = depth_frame.get_distance(u, v)
    return d * 1000.0 if d > 0 else None


def main():
    with open(CALIBRATION_FILE, "r") as f:
        calib = json.load(f)
    camera_matrix = calib["camera_matrix"]
    dist_coeffs = np.array(calib["dist_coeffs"])
    print(f"Loaded calibration: fx={camera_matrix[0][0]:.1f} fy={camera_matrix[1][1]:.1f} "
          f"cx={camera_matrix[0][2]:.1f} cy={camera_matrix[1][2]:.1f}")

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH_MM, MARKER_LENGTH_MM, dictionary)
    detector = cv2.aruco.CharucoDetector(board)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    cv2.namedWindow("Measure Demo", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Measure Demo", on_mouse)
    print("Green text = auto accuracy check against known board squares.")
    print("Click two points anywhere to measure real-world distance between them. ESC to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            display = image.copy()

            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

            if charuco_corners is not None and len(charuco_corners) >= 2:
                cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids, (0, 255, 0))

                ids_flat = charuco_ids.flatten()
                board_cols = SQUARES_X - 1
                errors = []
                for i in range(len(ids_flat)):
                    for j in range(i + 1, len(ids_flat)):
                        id_a, id_b = ids_flat[i], ids_flat[j]
                        row_a, col_a = divmod(id_a, board_cols)
                        row_b, col_b = divmod(id_b, board_cols)
                        # only compare corners that are exactly one square apart (horiz or vert neighbor)
                        if (row_a == row_b and abs(col_a - col_b) == 1) or (col_a == col_b and abs(row_a - row_b) == 1):
                            u_a, v_a = charuco_corners[i][0]
                            u_b, v_b = charuco_corners[j][0]
                            d_a = get_depth_mm(depth_frame, int(u_a), int(v_a))
                            d_b = get_depth_mm(depth_frame, int(u_b), int(v_b))
                            if d_a and d_b:
                                p_a = deproject(u_a, v_a, d_a, camera_matrix)
                                p_b = deproject(u_b, v_b, d_b, camera_matrix)
                                measured = np.linalg.norm(p_a - p_b)
                                err = measured - SQUARE_LENGTH_MM
                                errors.append((measured, err))

                if errors:
                    measured_vals = [e[0] for e in errors]
                    err_vals = [e[1] for e in errors]
                    avg_measured = sum(measured_vals) / len(measured_vals)
                    avg_err = sum(err_vals) / len(err_vals)
                    pct_err = (avg_err / SQUARE_LENGTH_MM) * 100
                    cv2.putText(display, f"Known square: {SQUARE_LENGTH_MM:.1f}mm  Measured avg: {avg_measured:.1f}mm "
                                          f"(err {avg_err:+.1f}mm, {pct_err:+.1f}%) n={len(errors)}",
                                (10, display.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            if len(click_points) == 1:
                cv2.circle(display, click_points[0], 2, (0, 0, 255), -1)
            elif len(click_points) == 2:
                for pt in click_points:
                    cv2.circle(display, pt, 2, (0, 0, 255), -1)
                cv2.line(display, click_points[0], click_points[1], (0, 0, 255), 2)

                u1, v1 = click_points[0]
                u2, v2 = click_points[1]
                d1 = get_depth_mm(depth_frame, u1, v1)
                d2 = get_depth_mm(depth_frame, u2, v2)
                if d1 and d2:
                    p1 = deproject(u1, v1, d1, camera_matrix)
                    p2 = deproject(u2, v2, d2, camera_matrix)
                    dist = np.linalg.norm(p1 - p2)
                    cv2.putText(display, f"Distance: {dist:.1f} mm",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    cv2.putText(display, "No depth at click point(s)",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.putText(display, "Click two points to measure. ESC to quit.",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("Measure Demo", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
