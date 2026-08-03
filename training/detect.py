import os
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

model = YOLO(os.path.join(PROJECT_ROOT, 'runs', 'detect', 'train', 'weights', 'best.pt'))

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

save_dir = os.path.join(PROJECT_ROOT, 'screw_images')

cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Detection', 1280, 960)

print("Press S to save a frame, Q to quit")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())
        
        results = model(color, verbose=False, conf=0.05)
        annotated = results[0].plot()

        cv2.imshow('Detection', annotated)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            filename = f'{save_dir}/{int(time.time())}.jpg'
            cv2.imwrite(filename, color)
            print(f'Saved: {filename}')
        elif key == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()