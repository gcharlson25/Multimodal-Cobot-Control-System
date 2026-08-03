"""
Grab a still photo of the ChArUco board from the RealSense camera.

Usage:
    python capture_board_photo.py [output_path] [countdown_seconds]

Get the board positioned in frame with both hands during the countdown;
the frame is saved automatically when it hits zero. Press ESC any time
to cancel.
"""

import sys
import os
import time
import cv2
import numpy as np
import pyrealsense2 as rs

DEFAULT_OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_photo.jpg")
DEFAULT_COUNTDOWN = 5


def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT
    countdown_seconds = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_COUNTDOWN

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    cv2.namedWindow("Board Photo", cv2.WINDOW_NORMAL)
    print(f"Get the board in frame. Auto-capturing in {countdown_seconds:.0f} seconds... (ESC to cancel)")

    start = time.time()
    saved = False

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            display = image.copy()

            remaining = countdown_seconds - (time.time() - start)
            if remaining > 0:
                cv2.putText(display, f"{remaining:.1f}", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 255), 3)
                cv2.imshow("Board Photo", display)
            else:
                cv2.imwrite(output_path, image)
                print(f"Saved: {output_path}")
                saved = True
                break

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("Cancelled.")
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    if not saved:
        sys.exit(1)


if __name__ == "__main__":
    main()
