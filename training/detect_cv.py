import os
import pyrealsense2 as rs
import numpy as np
import cv2
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

save_dir = os.path.join(PROJECT_ROOT, 'screw_images')

cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Detection', 1280, 960)

cv2.namedWindow('Debug', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Debug', 640, 480)

min_radius = 5
max_radius = 30
param1 = 50
param2 = 30
min_dist = 30
dp = 1.5
blur_size = 9

print("Controls:")
print("  S - save frame")
print("  Q - quit")
print("  1/2 - decrease/increase param1 (edge sensitivity)")
print("  3/4 - decrease/increase param2 (circle threshold)")
print("  5/6 - decrease/increase min radius")
print("  7/8 - decrease/increase max radius")
print("  9/0 - decrease/increase blur")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())
        annotated = color.copy()

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (blur_size, blur_size), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=min_dist,
            param1=param1,
            param2=param2,
            minRadius=min_radius,
            maxRadius=max_radius
        )

        count = 0
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0, :]:
                x, y, r = int(c[0]), int(c[1]), int(c[2])
                cv2.circle(annotated, (x, y), r, (0, 255, 0), 2)
                cv2.circle(annotated, (x, y), 2, (0, 0, 255), 3)
                cv2.putText(annotated, f"r={r}", (x - 20, y - r - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                count += 1

        info = (f"Screws: {count} | p1={param1} p2={param2} "
                f"r=[{min_radius}-{max_radius}] blur={blur_size}")
        cv2.putText(annotated, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow('Detection', annotated)
        cv2.imshow('Debug', blurred)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            filename = f'{save_dir}/{int(time.time())}.jpg'
            cv2.imwrite(filename, color)
            print(f'Saved: {filename}')
        elif key == ord('q'):
            break
        elif key == ord('1'):
            param1 = max(10, param1 - 5)
        elif key == ord('2'):
            param1 += 5
        elif key == ord('3'):
            param2 = max(5, param2 - 5)
        elif key == ord('4'):
            param2 += 5
        elif key == ord('5'):
            min_radius = max(1, min_radius - 2)
        elif key == ord('6'):
            min_radius += 2
        elif key == ord('7'):
            max_radius = max(5, max_radius - 5)
        elif key == ord('8'):
            max_radius += 5
        elif key == ord('9'):
            blur_size = max(3, blur_size - 2)
        elif key == ord('0'):
            blur_size = min(31, blur_size + 2)

        if key != 255:
            print(f"p1={param1} p2={param2} r=[{min_radius}-{max_radius}] blur={blur_size}")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
