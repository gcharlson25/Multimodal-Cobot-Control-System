import ctypes
import json
import math
import os
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from ctypes import wintypes
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import keyboard
from PIL import Image, ImageTk

HOST = "127.0.0.1"
PORT = 9100

PANEL_WIDTH = 480
CAMERA_WINDOW_TITLE = "Teleop"
CAMERA_REFRESH_MS = 200
CAMERA_MAX_SIDE = 900
CAMERA_DISCOVERY_POLL_S = 0.05

PW_CLIENTONLY = 1
PW_RENDERFULLCONTENT = 2
SWP_NOSIZE = 0x0001
SWP_NOZORDER = 0x0004
SWP_NOACTIVATE = 0x0010
SM_XVIRTUALSCREEN = 76
SM_YVIRTUALSCREEN = 77
SM_CXVIRTUALSCREEN = 78
SM_CYVIRTUALSCREEN = 79
SPI_GETWORKAREA = 0x0030
WM_LBUTTONDOWN = 0x0201
WM_LBUTTONUP = 0x0202
MK_LBUTTON = 0x0001
WM_CHAR = 0x0102

user32 = ctypes.windll.user32 if sys.platform == "win32" else None
gdi32 = ctypes.windll.gdi32 if sys.platform == "win32" else None

if user32:
    user32.FindWindowW.restype = wintypes.HWND
    user32.FindWindowW.argtypes = [wintypes.LPCWSTR, wintypes.LPCWSTR]
    user32.FindWindowExW.restype = wintypes.HWND
    user32.FindWindowExW.argtypes = [wintypes.HWND, wintypes.HWND, wintypes.LPCWSTR, wintypes.LPCWSTR]
    user32.GetWindowDC.restype = wintypes.HDC
    user32.GetWindowDC.argtypes = [wintypes.HWND]
    user32.ReleaseDC.restype = ctypes.c_int
    user32.ReleaseDC.argtypes = [wintypes.HWND, wintypes.HDC]
    user32.GetClientRect.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.RECT)]
    user32.IsWindow.argtypes = [wintypes.HWND]
    user32.PrintWindow.argtypes = [wintypes.HWND, wintypes.HDC, wintypes.UINT]
    user32.SetWindowPos.argtypes = [wintypes.HWND, wintypes.HWND, ctypes.c_int, ctypes.c_int,
                                     ctypes.c_int, ctypes.c_int, wintypes.UINT]
    user32.SystemParametersInfoW.argtypes = [wintypes.UINT, wintypes.UINT, ctypes.c_void_p, wintypes.UINT]
    user32.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
if gdi32:
    gdi32.CreateCompatibleDC.restype = wintypes.HDC
    gdi32.CreateCompatibleDC.argtypes = [wintypes.HDC]
    gdi32.CreateCompatibleBitmap.restype = wintypes.HBITMAP
    gdi32.CreateCompatibleBitmap.argtypes = [wintypes.HDC, ctypes.c_int, ctypes.c_int]
    gdi32.SelectObject.restype = wintypes.HGDIOBJ
    gdi32.SelectObject.argtypes = [wintypes.HDC, wintypes.HGDIOBJ]
    gdi32.DeleteObject.argtypes = [wintypes.HGDIOBJ]
    gdi32.DeleteDC.argtypes = [wintypes.HDC]
    gdi32.GetDIBits.argtypes = [wintypes.HDC, wintypes.HBITMAP, wintypes.UINT, wintypes.UINT,
                                 ctypes.c_void_p, ctypes.c_void_p, wintypes.UINT]


class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [
        ("biSize", ctypes.c_uint32),
        ("biWidth", ctypes.c_int32),
        ("biHeight", ctypes.c_int32),
        ("biPlanes", ctypes.c_uint16),
        ("biBitCount", ctypes.c_uint16),
        ("biCompression", ctypes.c_uint32),
        ("biSizeImage", ctypes.c_uint32),
        ("biXPelsPerMeter", ctypes.c_int32),
        ("biYPelsPerMeter", ctypes.c_int32),
        ("biClrUsed", ctypes.c_uint32),
        ("biClrImportant", ctypes.c_uint32),
    ]

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VISION_COMMAND_FILE = os.path.join(BASE_DIR, "vision_command.json")

ABS = 0
INCR = 1

JOG_SPEED = 250
HOME_SPEED = 100
DEFAULT_STEP_MM = 10

HOME_JOINT_DEG = [-92.059, 58.642, 135.473, -35.255, -93.233, -40.562]

JOG_DIRECTIONS = {
    "Forward":  ("y", 1),
    "Backward": ("y", -1),
    "Left":     ("x", -1),
    "Right":    ("x", 1),
    "Up":       ("z", 1),
    "Down":     ("z", -1),
}

PROCS = [
    ("Robot", ["py", "-3.7", "robot_client.py"]),
    ("Vision", ["py", "-3.12", "vision_alignment.py"]),
    ("Voice", ["py", "-3.14", "voice_control.py"]),
]


def _send_msg(sock, msg):
    data = json.dumps(msg).encode("utf-8")
    sock.sendall(struct.pack("!I", len(data)) + data)


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _recv_msg(sock):
    raw = _recv_exact(sock, 4)
    if raw is None:
        return None
    length = struct.unpack("!I", raw)[0]
    data = _recv_exact(sock, length)
    if data is None:
        return None
    return json.loads(data.decode("utf-8"))


def port_open(host, port, timeout=0.5):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def get_virtual_screen_bounds():
    x = user32.GetSystemMetrics(SM_XVIRTUALSCREEN)
    y = user32.GetSystemMetrics(SM_YVIRTUALSCREEN)
    w = user32.GetSystemMetrics(SM_CXVIRTUALSCREEN)
    h = user32.GetSystemMetrics(SM_CYVIRTUALSCREEN)
    return x, y, w, h


def get_work_area():
    rect = wintypes.RECT()
    user32.SystemParametersInfoW(SPI_GETWORKAREA, 0, ctypes.byref(rect), 0)
    return rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top


def find_window(title):
    hwnd = user32.FindWindowW(None, title)
    return hwnd or None


def find_child_window(parent_hwnd, class_name):
    hwnd = user32.FindWindowExW(parent_hwnd, None, class_name, None)
    return hwnd or None


def is_window(hwnd):
    return bool(hwnd) and bool(user32.IsWindow(hwnd))


def move_window_offscreen(hwnd):
    vx, vy, vw, vh = get_virtual_screen_bounds()
    user32.SetWindowPos(hwnd, 0, vx + vw + 100, vy, 0, 0, SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE)


def send_click(hwnd, x, y):
    lparam = (y << 16) | (x & 0xFFFF)
    user32.PostMessageW(hwnd, WM_LBUTTONDOWN, MK_LBUTTON, lparam)
    user32.PostMessageW(hwnd, WM_LBUTTONUP, 0, lparam)


def send_key(hwnd, char_code):
    user32.PostMessageW(hwnd, WM_CHAR, char_code, 0)


def capture_window(hwnd):
    rect = wintypes.RECT()
    if not user32.GetClientRect(hwnd, ctypes.byref(rect)):
        return None
    width, height = rect.right - rect.left, rect.bottom - rect.top
    if width <= 0 or height <= 0:
        return None

    hwnd_dc = user32.GetWindowDC(hwnd)
    if not hwnd_dc:
        return None
    mem_dc = gdi32.CreateCompatibleDC(hwnd_dc)
    bitmap = gdi32.CreateCompatibleBitmap(hwnd_dc, width, height)
    old_obj = gdi32.SelectObject(mem_dc, bitmap)
    try:
        ok = user32.PrintWindow(hwnd, mem_dc, PW_CLIENTONLY | PW_RENDERFULLCONTENT)
        if not ok:
            return None

        bmi = BITMAPINFOHEADER()
        bmi.biSize = ctypes.sizeof(BITMAPINFOHEADER)
        bmi.biWidth = width
        bmi.biHeight = -height
        bmi.biPlanes = 1
        bmi.biBitCount = 24
        bmi.biCompression = 0

        stride = ((width * 3 + 3) // 4) * 4
        buffer = ctypes.create_string_buffer(stride * height)
        rows = gdi32.GetDIBits(mem_dc, bitmap, 0, height, buffer, ctypes.byref(bmi), 0)
        if rows == 0:
            return None
        return Image.frombuffer("RGB", (width, height), buffer.raw, "raw", "BGR", stride, 1)
    finally:
        gdi32.SelectObject(mem_dc, old_obj)
        gdi32.DeleteObject(bitmap)
        gdi32.DeleteDC(mem_dc)
        user32.ReleaseDC(hwnd, hwnd_dc)


class RobotConnection:
    def __init__(self, log_fn):
        self._sock = None
        self._lock = threading.Lock()
        self._log = log_fn

    def call(self, msg, timeout=30.0):
        with self._lock:
            if self._sock is None:
                self._connect(timeout)
            try:
                self._sock.settimeout(timeout)
                _send_msg(self._sock, msg)
                reply = _recv_msg(self._sock)
                if reply is None:
                    raise ConnectionError("robot client closed the connection")
                return reply
            except OSError:
                self._disconnect()
                raise

    def _connect(self, timeout):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((HOST, PORT))
        _send_msg(sock, {"command": "setup"})
        _recv_msg(sock)
        self._sock = sock
        self._log("Connected to robot client.")

    def _disconnect(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def close(self):
        with self._lock:
            self._disconnect()


def jog_move(direction_name, distance_mm):
    axis, sign = JOG_DIRECTIONS[direction_name]
    value = distance_mm * sign
    if axis == "x":
        value = -value
    move = [0, 0, 0, 0, 0, 0]
    move[{"x": 0, "y": 1, "z": 2}[axis]] = value
    return {"command": "move", "move": move, "speed": JOG_SPEED, "blocking": False}


class ControlPanel:
    def __init__(self, root):
        self.root = root
        root.title("JAKA Robot Control Panel")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.procs = {}
        self.status_vars = {name: tk.StringVar(value="Not started") for name, _ in PROCS}
        self.log_queue = queue.Queue()
        self.robot = RobotConnection(self.log)

        self._camera_hwnd = None
        self._camera_click_hwnd = None
        self._camera_photo = None
        self._camera_native_size = None
        self.camera_queue = queue.Queue()

        self._build_ui()
        self._position_window()
        self.root.unbind_class("TButton", "<space>")
        self.root.unbind_class("Button", "<space>")
        for key in ("w", "a", "s", "d", "q", "e", "c", "t"):
            self.root.bind_all(f"<KeyPress-{key}>", self._on_key_forward)
        self.root.bind_all("<Escape>", self._on_key_forward)
        self.root.after(150, self._drain_log_queue)
        self.root.after(500, self._poll_process_status)
        self.root.after(150, self._drain_camera_queue)
        threading.Thread(target=self._camera_worker, daemon=True).start()

    def _position_window(self):
        if not user32:
            return
        x, y, w, h = get_work_area()
        self.root.geometry(f"{w}x{880}+{x}+{y}")

    def _locate_camera_window(self):
        if not user32:
            return
        for _ in range(400):
            hwnd = find_window(CAMERA_WINDOW_TITLE)
            if hwnd:
                move_window_offscreen(hwnd)
                self._camera_hwnd = hwnd
                self._camera_click_hwnd = find_child_window(hwnd, "HighGUI class") or hwnd
                self.log("Camera feed connected.")
                return
            time.sleep(CAMERA_DISCOVERY_POLL_S)
        self.log("Could not find the camera window - it may not have opened yet.")

    def _camera_worker(self):
        warned = False
        while True:
            hwnd = self._camera_hwnd
            img = None
            if hwnd is not None:
                try:
                    if is_window(hwnd):
                        img = capture_window(hwnd)
                        if img is not None:
                            self._camera_native_size = img.size
                            img = img.copy()
                            img.thumbnail((CAMERA_MAX_SIDE, CAMERA_MAX_SIDE))
                    else:
                        self._camera_hwnd = None
                        self._camera_click_hwnd = None
                except Exception as exc:
                    if not warned:
                        self.log(f"Camera preview error: {exc!r}")
                        warned = True
                    self._camera_hwnd = None
                    self._camera_click_hwnd = None
                    img = None
            self.camera_queue.put(img)
            time.sleep(CAMERA_REFRESH_MS / 1000.0)

    def _drain_camera_queue(self):
        got_update = False
        latest = None
        while True:
            try:
                latest = self.camera_queue.get_nowait()
                got_update = True
            except queue.Empty:
                break
        if got_update:
            if latest is not None:
                label_w = self.camera_label.winfo_width()
                label_h = self.camera_label.winfo_height()
                if label_w > 10 and label_h > 10:
                    latest = latest.resize((label_w, label_h))
                self._camera_photo = ImageTk.PhotoImage(latest)
                self.camera_label.configure(image=self._camera_photo, text="")
            else:
                self.camera_label.configure(image="", text="Camera not started")
        self.root.after(150, self._drain_camera_queue)

    def _on_camera_click(self, event):
        click_hwnd = self._camera_click_hwnd
        native_size = self._camera_native_size
        if click_hwnd is None or native_size is None or not is_window(click_hwnd):
            return
        label_w = self.camera_label.winfo_width()
        label_h = self.camera_label.winfo_height()
        if label_w <= 0 or label_h <= 0:
            return
        native_w, native_h = native_size
        x = int(event.x * native_w / label_w)
        y = int(event.y * native_h / label_h)
        send_click(click_hwnd, x, y)

    def _on_key_forward(self, event):
        click_hwnd = self._camera_click_hwnd
        if click_hwnd is None or not is_window(click_hwnd):
            return
        if event.keysym == "Escape":
            send_key(click_hwnd, 27)
        else:
            send_key(click_hwnd, ord(event.keysym.lower()))

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        container = ttk.Frame(self.root)
        container.pack(fill="both", expand=True)

        left = ttk.Frame(container, width=PANEL_WIDTH)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        right = ttk.Frame(container)
        right.pack(side="left", fill="both", expand=True, padx=(4, 8), pady=8)

        camera_frame = ttk.LabelFrame(right, text="Camera")
        camera_frame.pack(fill="both", expand=True)
        self.camera_label = tk.Label(camera_frame, bg="black", fg="white",
                                      text="Camera not started", compound="center",
                                      font=("TkDefaultFont", 14))
        self.camera_label.pack(fill="both", expand=True, padx=2, pady=2)
        self.camera_label.bind("<Button-1>", self._on_camera_click)

        system = ttk.LabelFrame(left, text="System")
        system.pack(fill="x", **pad)
        ttk.Button(system, text="Start System", command=self.start_system).grid(
            row=0, column=0, rowspan=len(PROCS), padx=4, pady=4)
        ttk.Button(system, text="Stop System", command=self.stop_system).grid(
            row=0, column=1, rowspan=len(PROCS), padx=4, pady=4)
        for i, (name, _) in enumerate(PROCS):
            ttk.Label(system, text=f"{name}:").grid(row=i, column=2, sticky="e", padx=(16, 2), pady=2)
            ttk.Label(system, textvariable=self.status_vars[name]).grid(row=i, column=3, sticky="w", pady=2)

        move = ttk.LabelFrame(left, text="Move Robot (mm)")
        move.pack(fill="x", **pad)
        self.step_var = tk.StringVar(value=str(DEFAULT_STEP_MM))
        ttk.Label(move, text="Step size:").grid(row=0, column=0, sticky="e")
        ttk.Spinbox(move, from_=1, to=100, width=5, textvariable=self.step_var).grid(row=0, column=1, sticky="w")

        grid = ttk.Frame(move)
        grid.grid(row=1, column=0, columnspan=4, pady=6)
        self.jog_buttons = {}
        positions = {
            "Forward": (0, 1), "Backward": (2, 1),
            "Left": (1, 0), "Right": (1, 2),
            "Up": (0, 3), "Down": (2, 3),
        }
        for label, (r, c) in positions.items():
            btn = ttk.Button(grid, text=label, width=10,
                              command=lambda d=label: self.do_jog(d))
            btn.grid(row=r, column=c, padx=4, pady=4)
            self.jog_buttons[label] = btn

        vision = ttk.LabelFrame(left, text="Vision")
        vision.pack(fill="x", **pad)
        ttk.Style().configure("Wrap.TButton", wraplength=120)
        ttk.Button(vision, text="Calibrate", command=self.do_calibrate).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(vision, text="Align", command=self.do_align).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(vision, text="Unfasten then Fasten after Alignment", style="Wrap.TButton",
                   command=self.do_unfasten_current).grid(row=0, column=2, padx=4, pady=4)
        ttk.Label(vision, text="Position the tool over the screw, press Calibrate once, "
                               "then Align to re-use it on any screw.",
                  foreground="#555", wraplength=440).grid(row=1, column=0, columnspan=3, sticky="w", padx=4)

        voice = ttk.LabelFrame(left, text="Voice")
        voice.pack(fill="x", **pad)
        self.ptt_button = tk.Button(voice, text="Hold to Talk", bg="#2980b9", fg="white",
                                     font=("TkDefaultFont", 10, "bold"))
        self.ptt_button.grid(row=0, column=0, padx=4, pady=4, ipadx=8, ipady=4)
        self.ptt_button.bind("<ButtonPress-1>", self._on_ptt_press)
        self.ptt_button.bind("<ButtonRelease-1>", self._on_ptt_release)
        ttk.Label(voice, text="Press and hold, speak, then release - same as holding the spacebar.",
                  foreground="#555", wraplength=340).grid(row=0, column=1, sticky="w", padx=8)

        screws = ttk.LabelFrame(left, text="Screws")
        screws.pack(fill="x", **pad)
        ttk.Label(screws, text="Screw:").grid(row=0, column=0, sticky="e")
        self.screw_var = tk.StringVar(value="1")
        ttk.Combobox(screws, textvariable=self.screw_var, state="readonly", width=5,
                     values=["1", "2", "3", "4", "5"]).grid(row=0, column=1, sticky="w")
        ttk.Button(screws, text="Fasten", command=lambda: self.do_screw("fasten")).grid(row=0, column=2, padx=6)
        ttk.Button(screws, text="Unfasten", command=lambda: self.do_screw("unfasten")).grid(row=0, column=3, padx=6)

        safety = ttk.LabelFrame(left, text="Safety")
        safety.pack(fill="x", **pad)
        stop_btn = tk.Button(safety, text="CANCEL NEXT MOVE", command=self.do_abort,
                              bg="#c0392b", fg="white", font=("TkDefaultFont", 10, "bold"))
        stop_btn.grid(row=0, column=0, padx=4, pady=4, ipadx=8, ipady=4)
        ttk.Button(safety, text="Go Home", command=self.do_home).grid(row=0, column=1, padx=8)

        log_frame = ttk.LabelFrame(left, text="Log")
        log_frame.pack(fill="both", expand=True, **pad)
        self.log_text = ScrolledText(log_frame, height=10, state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True)

    def log(self, message):
        self.log_queue.put(message)

    def _drain_log_queue(self):
        drained = False
        while True:
            try:
                message = self.log_queue.get_nowait()
            except queue.Empty:
                break
            drained = True
            self.log_text.configure(state="normal")
            self.log_text.insert("end", message + "\n")
            self.log_text.configure(state="disabled")
        if drained:
            self.log_text.see("end")
        self.root.after(150, self._drain_log_queue)

    def run_async(self, fn, buttons=()):
        for b in buttons:
            b.configure(state="disabled")

        def worker():
            try:
                result = fn()
                error = None
            except Exception as exc:
                result = None
                error = exc

            def finish():
                for b in buttons:
                    b.configure(state="normal")
                if error is not None:
                    self.log(f"ERROR: {error}")
                elif isinstance(result, dict) and result.get("status") == "error":
                    self.log(f"Rejected: {result.get('message')}")
                elif result is not None:
                    self.log(f"OK: {result}")

            self.root.after(0, finish)

        threading.Thread(target=worker, daemon=True).start()

    def do_jog(self, direction_name):
        try:
            distance = float(self.step_var.get())
        except ValueError:
            messagebox.showerror("Invalid step size", "Step size must be a number.")
            return
        msg = jog_move(direction_name, distance)
        self.log(f"Jog {direction_name} {distance}mm")
        self.run_async(lambda: self.robot.call(msg), buttons=[self.jog_buttons[direction_name]])

    def do_calibrate(self):
        self._write_vision_command("calibrate")

    def do_align(self):
        self._write_vision_command("align")

    def _write_vision_command(self, action):
        try:
            with open(VISION_COMMAND_FILE, "w") as f:
                json.dump({"action": action}, f)
            self.log(f"Vision command sent: {action}")
        except OSError as exc:
            self.log(f"ERROR writing vision command: {exc}")

    def do_screw(self, action):
        selection = self.screw_var.get()
        screw_number = int(selection)
        payload = {"function": "screw_operation", "action": action, "screw_number": screw_number}
        self.log(f"Screw operation: {payload}")
        self.run_async(lambda: self.robot.call({"command": "execute", "payload": payload}, timeout=60.0))

    def do_unfasten_current(self):
        payload = {"function": "unfasten_after_align", "args": []}
        self.log("Unfasten at current (aligned) position")
        self.run_async(lambda: self.robot.call({"command": "execute", "payload": payload}, timeout=60.0))

    def do_abort(self):
        payload = {"function": "motion_abort", "args": []}
        self.log("ABORT requested")
        self.run_async(lambda: self.robot.call({"command": "execute", "payload": payload}))

    def _on_ptt_press(self, _event):
        voice_proc = self.procs.get("Voice")
        if voice_proc is None or voice_proc.poll() is not None:
            self.log("Push-to-talk: Voice isn't running - start the system first.")
            return
        self.log("Push-to-talk: recording...")
        keyboard.press("space")

    def _on_ptt_release(self, _event):
        keyboard.release("space")

    def do_home(self):
        joint_pos = [math.radians(x) for x in HOME_JOINT_DEG]
        payload = {"function": "joint_move", "args": [joint_pos, ABS, True, HOME_SPEED]}
        self.log("Go Home requested")
        self.run_async(lambda: self.robot.call({"command": "execute", "payload": payload}, timeout=60.0))

    def start_system(self):
        if any(p.poll() is None for p in self.procs.values()):
            messagebox.showinfo("Already running", "The system is already running.")
            return
        threading.Thread(target=self._start_sequence, daemon=True).start()

    def _start_sequence(self):
        name, cmd = PROCS[0]
        self._launch(name, cmd)
        self.log("Waiting for robot client to come up...")
        for _ in range(30):
            if port_open(HOST, PORT):
                break
            if self.procs[name].poll() is not None:
                self.log("Robot client exited before it started listening - aborting startup.")
                return
            time.sleep(0.5)
        else:
            self.log("Robot client did not start listening in time - aborting startup.")
            return

        for name, cmd in PROCS[1:]:
            self._launch(name, cmd)
            if name == "Vision":
                threading.Thread(target=self._locate_camera_window, daemon=True).start()

    def _launch(self, name, cmd):
        self.log(f"Starting {name} ({' '.join(cmd)})...")
        self.status_vars[name].set("Starting...")
        child_env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        kwargs = dict(
            cwd=BASE_DIR,
            env=child_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            encoding="utf-8",
            errors="replace",
        )
        if sys.platform == "win32":
            kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
        proc = subprocess.Popen(cmd, **kwargs)
        self.procs[name] = proc
        threading.Thread(target=self._reader, args=(name, proc), daemon=True).start()
        self.status_vars[name].set("Running")

    def _reader(self, name, proc):
        for line in proc.stdout:
            self.log(f"[{name}] {line.rstrip()}")
        proc.stdout.close()

    def stop_system(self):
        if not self.procs:
            return
        self.log("Stopping system...")
        self._camera_hwnd = None
        self._camera_click_hwnd = None
        self.robot.close()
        for name, proc in self.procs.items():
            if proc.poll() is None:
                proc.terminate()
        threading.Thread(target=self._wait_for_stop, daemon=True).start()

    def _wait_for_stop(self):
        for name, proc in self.procs.items():
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        self.log("System stopped.")

    def _poll_process_status(self):
        for name, proc in self.procs.items():
            if proc.poll() is not None and not self.status_vars[name].get().startswith("Stopped"):
                self.status_vars[name].set(f"Stopped (exit code {proc.returncode})")
        self.root.after(500, self._poll_process_status)

    def on_close(self):
        running = any(p.poll() is None for p in self.procs.values())
        if running and not messagebox.askyesno("Quit", "The system is still running. Stop it and quit?"):
            return
        self.robot.close()
        for proc in self.procs.values():
            if proc.poll() is None:
                proc.terminate()
        self.root.destroy()


def main():
    root = tk.Tk()
    ControlPanel(root)
    root.mainloop()


if __name__ == "__main__":
    main()

