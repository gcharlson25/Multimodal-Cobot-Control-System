"""
Capture a burst of ChArUco board photos for camera calibration.

Move the board from close to the camera to far away (and around/tilted)
while this runs. Frames are saved automatically at a fixed interval -
no keypress needed.

Usage:
    python charuco_capture.py [duration_seconds] [interval_seconds] [lead_in_seconds]

Defaults: 8 second lead-in, then a 90 second burst, one frame every 0.5s (~180 images).
Press ESC to stop early.
"""

import sys
import os
import time
import cv2
import numpy as np
import pyrealsense2 as rs

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "images")

DEFAULT_DURATION = 90.0
DEFAULT_INTERVAL = 0.5
DEFAULT_LEAD_IN = 8.0


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_INTERVAL
    lead_in = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_LEAD_IN

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    cv2.namedWindow("Charuco Capture", cv2.WINDOW_NORMAL)
    print(f"Get the board ready - capturing starts in {lead_in:.0f}s.")

    lead_start = time.time()
    try:
        while time.time() - lead_start < lead_in:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            display = image.copy()
            remaining = lead_in - (time.time() - lead_start)
            cv2.putText(display, f"Starting in {remaining:.1f}s", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            cv2.imshow("Charuco Capture", display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("Cancelled.")
                return

        print(f"Capturing for {duration:.0f}s, one frame every {interval:.1f}s.")
        print("Move the board close -> far, and around the frame. ESC to stop early.")

        start = time.time()
        last_capture = 0.0
        count = 0

        while True:
            elapsed = time.time() - start
            if elapsed >= duration:
                break

            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            display = image.copy()

            remaining = duration - elapsed
            cv2.putText(display, f"{remaining:.0f}s left  |  saved: {count}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Charuco Capture", display)

            if elapsed - last_capture >= interval:
                filename = os.path.join(OUTPUT_DIR, f"charuco_{count:04d}.jpg")
                cv2.imwrite(filename, image)
                count += 1
                last_capture = elapsed

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("Stopped early.")
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    print(f"Saved {count} images to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
