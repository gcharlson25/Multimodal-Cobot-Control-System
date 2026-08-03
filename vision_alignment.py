
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys
import time
import json
import socket
import struct
from ultralytics import YOLO

HOST = "127.0.0.1"
PORT = 9100

SPEED = 250
STEP = 1

ALIGN_SPEED = 100
ALIGN_TOLERANCE = 2
DEPTH_PROBE_STEP = 80.0
MAX_DELTA_Z = 100.0
RECOVERY_STEP = 1.0
RECOVERY_MAX_STEPS = 20

KP = 0.12
KI = 0.03
KD = 0.015
I_WINDUP_PX = 100.0
MAX_STEP_MM = 5.0
ALIGN_SAMPLE_FRAMES = 3
ALIGN_SAMPLE_FRAMES_NEAR = 8
ALIGN_NEAR_PX = 6

PIXEL_X_TO_ROBOT_DIR = 1
PIXEL_Y_TO_ROBOT_DIR = 1

CAMERA_HORIZ_OFFSET = 91.0   
CAMERA_VERT_OFFSET = 95.0   

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR = os.path.join(BASE_DIR, "mounted_screw")
CALIBRATION_FILE = os.path.join(SAVE_DIR, "calibration.json")
MODEL_PATH = os.path.join(BASE_DIR, "runs", "detect", "head_detect", "weights", "best.pt")

CAMERA_CALIBRATION_FILE = os.path.join(BASE_DIR, "camera_calibration", "camera_calibration.json")

VISION_COMMAND_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision_command.json")
VOICE_KEY_MAP = {"calibrate": ord('c'), "align": ord('t')}
SQUARES_X = 4
SQUARES_Y = 5
SQUARE_LENGTH_MM = 50.0
MARKER_LENGTH_MM = 37.0
ARUCO_DICT = cv2.aruco.DICT_4X4_50

CONTROLS = {
    ord('w'): [0, -STEP, 0, 0, 0, 0],
    ord('s'): [0, STEP, 0, 0, 0, 0],
    ord('d'): [-STEP, 0, 0, 0, 0, 0],
    ord('a'): [STEP, 0, 0, 0, 0, 0],
    ord('e'): [0, 0, STEP, 0, 0, 0],
    ord('q'): [0, 0, -STEP, 0, 0, 0],
}

model = YOLO(MODEL_PATH)
hole_fill = rs.hole_filling_filter()

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


def load_camera_calibration():
    if os.path.exists(CAMERA_CALIBRATION_FILE):
        with open(CAMERA_CALIBRATION_FILE, "r") as f:
            calib = json.load(f)
        print(f"Loaded camera calibration: fx={calib['camera_matrix'][0][0]:.1f}")
        return calib["camera_matrix"]
    print("WARNING: no camera_calibration.json found - click-to-measure disabled.")
    return None


def send_msg(sock, msg):
    data = json.dumps(msg).encode("utf-8")
    sock.sendall(struct.pack("!I", len(data)) + data)


def recv_msg(sock):
    raw = _recv_exact(sock, 4)
    if raw is None:
        return None
    length = struct.unpack("!I", raw)[0]
    data = _recv_exact(sock, length)
    if data is None:
        return None
    return json.loads(data.decode("utf-8"))


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def detect_screw(image):
    results = model(image, verbose=False, conf=0.3)
    boxes = results[0].boxes
    if len(boxes) == 0:
        return None
    cx, cy = image.shape[1] // 2, image.shape[0] // 2
    best = None
    best_dist = float("inf")
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        bx, by = int((x1 + x2) / 2), int((y1 + y2) / 2)
        dist = (bx - cx) ** 2 + (by - cy) ** 2
        if dist < best_dist:
            best_dist = dist
            best = (bx, by, int(x2 - x1), int(y2 - y1))
    return best


def compute_calibration_z(depth_mm):
    inner = depth_mm ** 2 - CAMERA_HORIZ_OFFSET ** 2
    if inner < 0:
        return None
    return -CAMERA_VERT_OFFSET + np.sqrt(inner)


def load_calibration():
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
            cal_pos = data.get("robot_pos")
            return (data["target_x"], data["target_y"]), data.get("calibration_z"), cal_pos
    return None, None, None


def save_calibration(x, y, cal_z, robot_pos=None):
    os.makedirs(SAVE_DIR, exist_ok=True)
    with open(CALIBRATION_FILE, "w") as f:
        json.dump({"target_x": x, "target_y": y, "calibration_z": cal_z, "robot_pos": robot_pos}, f)
    dist_str = f"{cal_z:.1f} mm" if cal_z is not None else "None"
    print(f"Calibration saved: target pixel = ({x}, {y}), calibration dist = {dist_str}")


def get_depth_at_screw(depth_data, screw):
    sx, sy, sw, sh = screw
    h, w = depth_data.shape
    r = max(sw, sh) // 4
    samples = []
    for angle_deg in range(0, 360, 20):
        rad = np.radians(angle_deg)
        for frac in [0.0, 0.5]:
            px = int(sx + r * frac * np.cos(rad))
            py = int(sy + r * frac * np.sin(rad))
            if 0 <= px < w and 0 <= py < h:
                val = depth_data[py, px]
                if val > 0:
                    samples.append(float(val))
    return float(np.median(samples)) if samples else None


def _sample_depth_median(pipeline, align, n_frames=5):
    samples = []
    for _ in range(n_frames):
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue
        image = np.asanyarray(color_frame.get_data())
        filled = hole_fill.process(depth_frame)
        depth_data = np.asanyarray(filled.get_data())
        screw = detect_screw(image)
        if screw is None:
            continue
        d = get_depth_at_screw(depth_data, screw)
        if d is not None and 100 <= d <= 400:
            samples.append(d)
    return float(np.median(samples)) if samples else None


def setup_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    return pipeline, align


def send_robot_command(sock, command):
    send_msg(sock, command)
    reply = recv_msg(sock)
    return reply


def recover_depth_by_climbing(sock, pipeline, align):
    climbed = 0.0
    for _ in range(RECOVERY_MAX_STEPS):
        d = _sample_depth_median(pipeline, align, n_frames=3)
        if d is not None:
            return d, climbed
        send_robot_command(sock, {"command": "move", "move": [0, 0, RECOVERY_STEP, 0, 0, 0], "speed": ALIGN_SPEED, "blocking": True})
        time.sleep(0.1)
        climbed += RECOVERY_STEP
    return None, climbed


def recover_screw_by_climbing(sock, pipeline, align):
    for _ in range(RECOVERY_MAX_STEPS):
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        if color_frame:
            image = np.asanyarray(color_frame.get_data())
            screw = detect_screw(image)
            if screw is not None:
                return image, screw
        send_robot_command(sock, {"command": "move", "move": [0, 0, RECOVERY_STEP, 0, 0, 0], "speed": ALIGN_SPEED, "blocking": True})
        time.sleep(0.1)
    return None, None


def auto_align(sock, pipeline, align, target, calibration_z, cal_robot_pos=None):
    target_x, target_y = target
    print(f"Auto-aligning to target pixel ({target_x}, {target_y})...")
    print("Press ESC to cancel")

    if cal_robot_pos is not None:
        reply = send_robot_command(sock, {"command": "get_position"})
        if not reply or reply.get("status") != "ok":
            print("Z failed: could not read robot position")
            return False, None
        delta_z = reply["position"][2] - cal_robot_pos[2]
    else:
        d = _sample_depth_median(pipeline, align)
        climbed = 0.0
        if d is None:
            print("Z probe failed: no depth reading, climbing to recover...")
            d, climbed = recover_depth_by_climbing(sock, pipeline, align)
            if d is None:
                print("Z probe failed: no depth reading after recovery attempts")
                return False, None

        X = compute_calibration_z(d)
        if X is None:
            print(f"Z probe failed: depth {d:.1f} too small for geometry")
            return False, None

        if climbed > 0:
            send_robot_command(sock, {"command": "move", "move": [0, 0, -climbed, 0, 0, 0], "speed": ALIGN_SPEED, "blocking": True})
            time.sleep(0.1)
            X -= climbed

        delta_z = X - calibration_z

    if abs(delta_z) > MAX_DELTA_Z:
        print(f"Z rejected: delta_z {delta_z:.1f} exceeds {MAX_DELTA_Z} mm threshold")
        return False, None

    send_robot_command(sock, {"command": "move", "move": [0, 0, -delta_z, 0, 0, 0], "speed": ALIGN_SPEED, "blocking": True})
    time.sleep(0.1)

    integral_x = 0.0
    integral_y = 0.0
    prev_err_x = None
    prev_err_y = None

    while True:
        if prev_err_x is not None and max(abs(prev_err_x), abs(prev_err_y)) <= ALIGN_NEAR_PX:
            n_frames = ALIGN_SAMPLE_FRAMES_NEAR
        else:
            n_frames = ALIGN_SAMPLE_FRAMES
        centers = []
        image = None
        screw = None
        for _ in range(n_frames):
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            det = detect_screw(image)
            if det is not None:
                screw = det
                centers.append((det[0], det[1]))
        if image is None:
            continue

        display = image.copy()
        cv2.drawMarker(display, (target_x, target_y), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)

        if not centers:
            screw = None
        if screw is None:
            cv2.putText(display, "NO SCREW DETECTED - recovering",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("Teleop", display)
            if cv2.waitKey(1) & 0xFF == 27:
                print("Auto-align cancelled")
                return False, delta_z

            image, screw = recover_screw_by_climbing(sock, pipeline, align)
            if screw is None:
                if cv2.waitKey(100) & 0xFF == 27:
                    print("Auto-align cancelled")
                    return False, delta_z
                continue
            centers = [(screw[0], screw[1])]

        sx, sy, sw, sh = screw
        cv2.rectangle(display, (sx - sw // 2, sy - sh // 2), (sx + sw // 2, sy + sh // 2), (0, 255, 0), 2)
        cv2.circle(display, (sx, sy), 3, (0, 255, 0), -1)
        cv2.line(display, (sx, sy), (target_x, target_y), (255, 0, 0), 2)

        avg_x = sum(c[0] for c in centers) / len(centers)
        avg_y = sum(c[1] for c in centers) / len(centers)
        err_x = avg_x - target_x
        err_y = avg_y - target_y

        cv2.putText(display, f"Error: ({err_x:.1f}, {err_y:.1f}) px",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        dz_text = f"dZ: {delta_z:.1f} mm"
        (tw, _), _ = cv2.getTextSize(dz_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.putText(display, dz_text, (display.shape[1] - tw - 10, display.shape[0] - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        cv2.imshow("Teleop", display)

        if abs(err_x) <= ALIGN_TOLERANCE and abs(err_y) <= ALIGN_TOLERANCE:
            print(f"Aligned! Final error: ({err_x:.1f}, {err_y:.1f}) px")
            cv2.waitKey(500)
            return True, delta_z

        move = [0, 0, 0, 0, 0, 0]
        if abs(err_x) > ALIGN_TOLERANCE:
            if err_x * integral_x < 0:
                integral_x = 0.0
            integral_x = max(-I_WINDUP_PX, min(I_WINDUP_PX, integral_x + err_x))
            deriv_x = (err_x - prev_err_x) if prev_err_x is not None else 0.0
            step_x = KP * err_x + KI * integral_x + KD * deriv_x
            move[1] = max(-MAX_STEP_MM, min(MAX_STEP_MM, step_x)) * PIXEL_X_TO_ROBOT_DIR
        if abs(err_y) > ALIGN_TOLERANCE:
            if err_y * integral_y < 0:
                integral_y = 0.0
            integral_y = max(-I_WINDUP_PX, min(I_WINDUP_PX, integral_y + err_y))
            deriv_y = (err_y - prev_err_y) if prev_err_y is not None else 0.0
            step_y = KP * err_y + KI * integral_y + KD * deriv_y
            move[0] = max(-MAX_STEP_MM, min(MAX_STEP_MM, step_y)) * PIXEL_Y_TO_ROBOT_DIR

        prev_err_x = err_x
        prev_err_y = err_y

        send_robot_command(sock, {"command": "move", "move": move, "speed": ALIGN_SPEED, "blocking": True})
        time.sleep(0.25)

        for _ in range(2):
            pipeline.wait_for_frames()

        if cv2.waitKey(1) & 0xFF == 27:
            print("Auto-align cancelled")
            return False, delta_z


def main():
    print(f"Connecting to robot client at {HOST}:{PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
    except ConnectionRefusedError:
        print("ERROR: Could not connect. Start robot_client.py first.")
        sys.exit(1)
    print("Connected to robot client.")

    send_robot_command(sock, {"command": "setup"})

    pipeline, align = setup_camera()
    cv2.namedWindow("Teleop", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Teleop", 1280, 960)
    cv2.setMouseCallback("Teleop", on_mouse)

    camera_matrix = load_camera_calibration()
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    charuco_board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH_MM, MARKER_LENGTH_MM, dictionary)
    charuco_detector = cv2.aruco.CharucoDetector(charuco_board)

    target, calibration_z, cal_robot_pos = load_calibration()
    current_depth_mm = None
    robot_pos = None
    last_pos_time = 0
    last_delta_z = None

    print("Teleop ready!")
    print("  W = backward, S = forward")
    print("  A = left,     D = right")
    print("  Q = down,     E = up")
    print("  C = calibrate (align tool over screw, then press C)")
    print("  T = auto-align to calibrated target")
    print("  Click two points = measure real-world distance")
    print("  ESC = quit")
    if target:
        print(f"  Loaded calibration: target pixel = ({target[0]}, {target[1]}), calibration dist = {calibration_z}")
    else:
        print("  No calibration found. Align over a screw and press C first.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            if depth_frame:
                filled = hole_fill.process(depth_frame)
                depth_data = np.asanyarray(filled.get_data())
            else:
                depth_data = None
            display = image.copy()

            current_depth_mm = None
            screw = detect_screw(image)
            if screw:
                sx, sy, sw, sh = screw
                cv2.rectangle(display, (sx - sw // 2, sy - sh // 2), (sx + sw // 2, sy + sh // 2), (0, 255, 0), 2)
                cv2.circle(display, (sx, sy), 3, (0, 255, 0), -1)
                if depth_data is not None:
                    depth_mm = get_depth_at_screw(depth_data, screw)
                    if depth_mm is not None and 100 <= depth_mm <= 400:
                        current_depth_mm = depth_mm
                        depth_label = f"Depth: {depth_mm:.0f} mm"
                        depth_color = (0, 255, 255)
                    else:
                        depth_label = "Depth: --- mm"
                        depth_color = (0, 128, 255)
                    cv2.putText(display, depth_label, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, depth_color, 2)

            charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(image)

            if marker_ids is not None and len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
                n_corners = 0 if charuco_corners is None else len(charuco_corners)
                cv2.putText(display, f"Markers: {len(marker_ids)}   Board corners: {n_corners}",
                            (10, display.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            if charuco_corners is not None and len(charuco_corners) >= 2:
                cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids, (0, 255, 0))

                if camera_matrix is not None and depth_frame:
                    ids_flat = charuco_ids.flatten()
                    board_cols = SQUARES_X - 1
                    errors = []
                    for i in range(len(ids_flat)):
                        for j in range(i + 1, len(ids_flat)):
                            id_a, id_b = ids_flat[i], ids_flat[j]
                            row_a, col_a = divmod(id_a, board_cols)
                            row_b, col_b = divmod(id_b, board_cols)
                            if (row_a == row_b and abs(col_a - col_b) == 1) or (col_a == col_b and abs(row_a - row_b) == 1):
                                u_a, v_a = charuco_corners[i][0]
                                u_b, v_b = charuco_corners[j][0]
                                d_a = get_depth_mm(depth_frame, int(u_a), int(v_a))
                                d_b = get_depth_mm(depth_frame, int(u_b), int(v_b))
                                if d_a and d_b:
                                    p_a = deproject(u_a, v_a, d_a, camera_matrix)
                                    p_b = deproject(u_b, v_b, d_b, camera_matrix)
                                    errors.append(np.linalg.norm(p_a - p_b) - SQUARE_LENGTH_MM)

                    if errors:
                        avg_err = sum(errors) / len(errors)
                        pct_err = (avg_err / SQUARE_LENGTH_MM) * 100
                        cv2.putText(display, f"Board check: {SQUARE_LENGTH_MM:.0f}mm squares, "
                                             f"err {avg_err:+.1f}mm ({pct_err:+.1f}%) n={len(errors)}",
                                    (10, display.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            if len(click_points) == 1:
                cv2.circle(display, click_points[0], 2, (0, 0, 255), -1)
            elif len(click_points) == 2:
                for pt in click_points:
                    cv2.circle(display, pt, 2, (0, 0, 255), -1)
                cv2.line(display, click_points[0], click_points[1], (0, 0, 255), 2)

                if camera_matrix is not None and depth_frame:
                    u1, v1 = click_points[0]
                    u2, v2 = click_points[1]
                    d1 = get_depth_mm(depth_frame, u1, v1)
                    d2 = get_depth_mm(depth_frame, u2, v2)
                    if d1 and d2:
                        p1 = deproject(u1, v1, d1, camera_matrix)
                        p2 = deproject(u2, v2, d2, camera_matrix)
                        dist = np.linalg.norm(p1 - p2)
                        cv2.putText(display, f"Distance: {dist:.1f} mm",
                                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    else:
                        cv2.putText(display, "No depth at click point(s)",
                                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    cv2.putText(display, "No camera calibration - measure disabled",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            now = time.time()
            if now - last_pos_time >= 0.2:
                reply = send_robot_command(sock, {"command": "get_position"})
                if reply and reply.get("status") == "ok":
                    robot_pos = reply["position"]
                last_pos_time = now

            if robot_pos is not None:
                x, y, z = robot_pos[0], robot_pos[1], robot_pos[2]
                pos_lines = [f"X: {x:.1f}", f"Y: {y:.1f}", f"Z: {z:.1f}"]
                for i, line in enumerate(pos_lines):
                    (tw, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.putText(display, line, (display.shape[1] - tw - 10, 30 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if target:
                cv2.drawMarker(display, (target[0], target[1]), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)

            if cal_robot_pos is not None:
                cal_lines = [f"Cal X: {cal_robot_pos[0]:.1f}",
                             f"Cal Y: {cal_robot_pos[1]:.1f}",
                             f"Cal Z: {cal_robot_pos[2]:.1f}"]
                for i, line in enumerate(cal_lines):
                    (tw, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.putText(display, line, (display.shape[1] - tw - 10, display.shape[0] - 135 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

            if last_delta_z is not None:
                dz_text = f"dZ: {last_delta_z:.1f} mm"
                (tw, _), _ = cv2.getTextSize(dz_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.putText(display, dz_text, (display.shape[1] - tw - 10, display.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

            cv2.putText(display, "WASD/QE: move  C: calib  T: align  Click x2: measure  ESC: quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

            cv2.imshow("Teleop", display)

            key = cv2.waitKey(1) & 0xFF

            if os.path.exists(VISION_COMMAND_FILE):
                try:
                    with open(VISION_COMMAND_FILE, "r") as f:
                        action = json.load(f).get("action")
                    os.remove(VISION_COMMAND_FILE)
                    if action in VOICE_KEY_MAP:
                        key = VOICE_KEY_MAP[action]
                        print(f"Voice command received: {action}")
                except Exception as e:
                    print(f"Bad vision command file: {e}")

            if key == 27:
                break

            if key == ord('c'):
                if screw is None:
                    print("No screw detected! Move closer and try again.")
                elif robot_pos is None:
                    print("No robot position yet - wait a moment, then calibrate.")
                else:
                    sx, sy, sw, sh = screw
                    target = (sx, sy)
                    calibration_z = compute_calibration_z(current_depth_mm) if current_depth_mm is not None else None
                    cal_robot_pos = robot_pos[:3]
                    save_calibration(sx, sy, calibration_z, cal_robot_pos)

            if key == ord('t'):
                if target and (cal_robot_pos is not None or calibration_z is not None):
                    _, last_delta_z = auto_align(sock, pipeline, align, target, calibration_z, cal_robot_pos)
                else:
                    print("No calibration! Align over a screw and press C first.")

            if key in CONTROLS:
                move = CONTROLS[key]
                print(f"Moving: {move}")
                send_robot_command(sock, {"command": "move", "move": move, "speed": SPEED, "blocking": False})
    finally:
        send_robot_command(sock, {"command": "shutdown"})
        sock.close()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
