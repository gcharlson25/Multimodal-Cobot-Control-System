import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)

spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 20)

temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal.set_option(rs.option.filter_smooth_delta, 20)

hole_filling = rs.hole_filling_filter()

alpha = 0.4
cv2.namedWindow('Depth Overlay', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Depth Overlay', 1280, 960)

print("Controls:")
print("  A/D - decrease/increase overlay opacity")
print("  Q   - quit")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_frame = hole_filling.process(depth_frame)
        depth_data = np.asanyarray(depth_frame.get_data()).astype(np.float32)

        valid_mask = depth_data > 0
        if not np.any(valid_mask):
            cv2.imshow('Depth Overlay', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        avg_depth = np.mean(depth_data[valid_mask])

        spread = avg_depth * 0.5 if avg_depth > 0 else 1.0
        normalized = (depth_data - avg_depth) / spread
        normalized = np.clip(normalized, -1.0, 1.0)

        overlay = np.zeros_like(color_image)

        close_strength = np.clip(-normalized, 0, 1)
        far_strength = np.clip(normalized, 0, 1)
        near_avg = 1.0 - np.abs(normalized)

        overlay[:, :, 0] = ((far_strength * 255) + (near_avg * 100)).clip(0, 255).astype(np.uint8)
        overlay[:, :, 2] = ((close_strength * 255) + (near_avg * 100)).clip(0, 255).astype(np.uint8)
        overlay[:, :, 1] = (near_avg * 40).clip(0, 255).astype(np.uint8)

        overlay[~valid_mask] = 0

        blended = cv2.addWeighted(color_image, 1.0, overlay, alpha, 0)

        info = f"Avg depth: {avg_depth:.0f}mm | Opacity: {alpha:.2f}"
        cv2.putText(blended, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Depth Overlay', blended)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):
            alpha = max(0.0, alpha - 0.05)
        elif key == ord('d'):
            alpha = min(1.0, alpha + 0.05)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
