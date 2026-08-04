# Multimodal Cobot Control System

A voice- and vision-guided control system for a JAKA collaborative robot arm, combining **local speech recognition**, a **large language model (LLM)** command interpreter, **computer vision-guided closed-loop alignment**, and a **hardware-enforced safety layer** to let an operator perform precision assembly tasks — hands-free, in natural language — without manually teach-pending every motion.

### 🎥 Demo Video

[![Watch the demo](https://img.youtube.com/vi/-4sJVdZdFJw/maxresdefault.jpg)](https://youtu.be/-4sJVdZdFJw)

*Click the thumbnail above to watch the full system in action — voice commands, vision-guided alignment, automated fastening, and live safety-rejection of an unsafe command.*

---

## Overview

This system lets an operator control a JAKA Zu5 collaborative robot arm by voice — jog it around, trigger a vision-guided alignment routine, and fasten or unfasten screws on a workpiece — while an LLM handles anything outside the pre-defined vocabulary (draw shapes, perform gestures, execute freeform motion requests) by generating robot code on the fly. Every command, whether from a deterministic parser or an LLM, passes through a single, hardware-enforced safety chokepoint before it's allowed to move real hardware.

It was built as a from-scratch exploration of **multimodal human-robot interaction**: fusing natural language understanding, real-time computer vision, and classical control theory (PID) into one coherent system running across three cooperating processes.

**Core capabilities:**
- 🎙️ **Voice control** — hold-to-talk speech input, transcribed locally, routed through a two-tier command interpreter
- 👁️ **Vision-guided alignment** — a custom-trained object detection model and a closed-loop PID controller visually servo the robot's tool onto a target with sub-millimeter precision
- 🤖 **LLM-driven freeform motion** — natural language requests ("do a dance," "draw a circle") are translated into robot motion by GPT-4o, with every generated command safety-checked before execution
- 🔩 **Automated assembly** — taught fastening/unfastening routines execute screw-driving operations by voice, including a "fasten the currently-aligned screw" mode chained directly off the vision system
- 🛡️ **Safety-critical command verification** — joint-angle envelopes, per-move distance caps, and command-count limits are enforced in code, not just prompted for

---

## System Architecture

The JAKA robot SDK, the RealSense camera SDK, and the speech/LLM stack each require **mutually incompatible Python runtimes** — so this is architected as three cooperating processes communicating over a custom TCP protocol, not a single monolithic script.

```
                    ┌──────────────────────────┐
                    │   robot_client.py         │   Python 3.7
                    │   (sole owner of the      │◄──────────────────┐
                    │   robot connection)        │                   │
                    │                            │                   │
                    │  • JAKA SDK (jkrc)         │                   │
                    │  • Safety envelope check   │                   │
                    │  • threading.Lock-serial-  │                   │
                    │    ized command execution  │                   │
                    └────────────▲───────────────┘                   │
                                 │ TCP (localhost:9100)               │
                     length-prefixed JSON frames                     │
                                 │                                    │
              ┌──────────────────┴───────────────┐    ┌───────────────┴────────────────┐
              │  vision_alignment.py              │    │  voice_control.py                │
              │  (Python 3.12)                     │    │  (Python 3.14)                    │
              │                                     │    │                                    │
              │  • RealSense depth camera           │    │  • Whisper (local speech-to-text) │
              │  • YOLOv8 screw detection            │    │  • Keyword command parser          │
              │  • ChArUco camera calibration        │    │  • llm_command_generator.py       │
              │  • PID closed-loop alignment          │    │    (GPT-4o freeform fallback)     │
              │  • WASD teleop + click-to-measure     │    │                                    │
              └──────────────────┬─────────────────┘    └───────────────┬──────────────────┘
                                 │                                       │
                                 │        vision_command.json            │
                                 └────────── (file-based trigger) ◄──────┘
                    voice-issued "calibrate"/"align" become a synthetic
                    keypress inside the vision loop — same tested code
                    path as the keyboard controls
```

**Why three processes, not one:** the JAKA vendor SDK only loads on Python 3.7; `pyrealsense2` targets 3.12; the Whisper/LLM stack runs on 3.14. Rather than fight binary compatibility, the system embraces it — one process per runtime, coordinated over sockets.

**Why a single "gatekeeper" process:** an earlier design had voice and vision *each* independently connect to the robot — a real hazard, since two uncoordinated processes could issue conflicting motion simultaneously. The current design routes every command through one process (`robot_client.py`), serialized by a `threading.Lock`, so concurrent motion is structurally impossible rather than merely unlikely.

---

## Machine Learning & Computer Vision

### Object Detection
A **YOLOv8** model (Ultralytics), trained on a custom dataset of mounted screw images (labeled via Roboflow, exported in YOLO format), detects screw heads in the live camera feed for vision-guided alignment. Detection runs at ~30fps against the RealSense color stream.

### Camera Calibration
Full **ChArUco (ArUco + chessboard) camera calibration** pipeline built from scratch — board detection, marker-dictionary identification, and intrinsic calibration (`cv2.aruco.CharucoBoard` / `CharucoDetector`) — producing a camera matrix and distortion coefficients used for real-world pixel-to-millimeter measurement. Achieved **0.24px reprojection error** and validated to **~1% real-world measurement accuracy** against the board's own known dimensions.

### Closed-Loop Visual Servoing (PID Control)
XY alignment uses a tuned **PID controller** (proportional-integral, derivative available) closing the loop on live detection error every iteration — not a one-shot open-loop move. Gains were tuned empirically using custom-built CSV telemetry logging of every control-loop iteration, which also diagnosed a false "instability" as a target-selection bug in the detector rather than a controller problem. Z-axis correction uses the robot's own encoder position relative to a calibrated reference pose (more reliable than the depth sensor at close range), while XY correction remains fully vision-driven.

### LLM-Based Command Generation
Freeform voice commands are sent to **GPT-4o** via the OpenAI Chat Completions API with a system prompt documenting the robot's full motion API. The model's generated code is never executed directly against hardware — it's run against a proxy object that *records* intended SDK calls (via Python's `__getattr__` hook) instead of executing them, producing an inspectable list of intended actions that passes through a function whitelist before any real motion occurs.

---

## Safety Systems

Safety is enforced in code at the one architectural chokepoint every command passes through — never left to trusting an LLM's compliance with prompt instructions alone.

| Layer | What it does |
|---|---|
| **Joint-angle envelope** | Empirically measured safe range per joint; any generated motion outside it is rejected before execution |
| **Per-move linear distance cap** | Caps any single linear move to a safe maximum, preventing large erroneous jogs |
| **LLM command-count cap** | Rejects abnormally large command sequences (guards against runaway/looping LLM generations) |
| **Proxy-verified LLM execution** | GPT-generated code is recorded and whitelist-filtered before touching the robot; unknown/hallucinated function calls are dropped |
| **Concurrency lock** | A single `threading.Lock` in the robot process serializes all commands from all clients |
| **External human-detection safety system** *(separate hardware, not included in this repository)* | A dedicated external device using machine learning to detect people near the workspace, drawing bounding boxes and applying two-zone deceleration — full stop within close range, automatic slow-down at a further range — reducing reaction time compared to a manual e-stop. Integrates with the JAKA controller as a standalone add-on. *(Wiring/integration diagram to be added.)* |

---

## Hardware Requirements

| Component | Used for |
|---|---|
| **JAKA Zu5** collaborative robot arm | The controlled robot; requires the vendor Python SDK (`jkrc`, vendored in `out/`) |
| **Intel RealSense D435if** depth camera | Screw detection, ChArUco calibration, depth-based measurement |
| Microphone | Voice command input |
| Custom screwdriver end-effector | Digital I/O–controlled fastening/unfastening tool |
| *(Optional, external)* ML-based human-detection safety device | Separate hardware unit; see Safety Systems above |

---

## Technical Stack

**Languages:** Python (three interpreter versions — 3.7 / 3.12 / 3.14 — run concurrently to satisfy incompatible native dependencies)

**Machine Learning / Computer Vision:**
- Ultralytics YOLOv8 — object detection
- OpenCV (`cv2.aruco`) — ChArUco camera calibration & marker detection
- Intel RealSense SDK (`pyrealsense2`) — depth sensing
- NumPy — geometric/control-loop computation

**Natural Language / LLM:**
- OpenAI Whisper — local speech-to-text inference
- OpenAI GPT-4o (Chat Completions API) — natural-language-to-robot-code generation

**Robotics & Control:**
- JAKA Zu5 vendor SDK (`jkrc`)
- Custom PID closed-loop controller

**Systems / Infrastructure:**
- Custom TCP socket protocol (length-prefixed JSON framing) for inter-process communication
- `threading` — concurrency-safe command serialization
- `sounddevice`, `keyboard` — audio I/O and push-to-talk input
- Windows batch orchestration for multi-interpreter process startup
- Git/GitHub version control

---

## Installation & Setup

This project requires **three separate Python installations** (3.7, 3.12, 3.14) due to native-dependency constraints described above. Install each via the [Windows Python launcher](https://docs.python.org/3/using/windows.html) (`py -3.7`, `py -3.12`, `py -3.14`).

```bash
# Python 3.7 — robot control (no pip packages needed beyond the vendored JAKA SDK)
py -3.7 -m pip install -r requirements-robot.txt

# Python 3.12 — vision / camera / detection
py -3.12 -m pip install -r requirements-vision.txt

# Python 3.14 — voice / LLM
py -3.14 -m pip install -r requirements-voice.txt
```

**Environment variables:** create a `.env` file in the repo root:
```
OPENAI_API_KEY=your-key-here
```

**Camera calibration:** run the ChArUco calibration pipeline in `camera_calibration/` once with your own camera before first use — `identify_charuco.py` to confirm your board's marker dictionary, then `charuco_capture.py` + `charuco_calibrate.py` to produce `camera_calibration.json`.

**Trained model:** the production screw-detection weights are included directly in this repo (`runs/detect/head_detect/weights/best.pt`, ~6MB) — no separate download needed to run the system as-is.

**Hardware sanity checks:** before running the full system, verify each hardware component independently with the scripts in `hardware_tests/`:
```bash
py -3.7  hardware_tests/test_robot_movement.py   # robot connects + makes one small move
py -3.12 hardware_tests/test_camera_feed.py      # RealSense RGB + depth stream
py -3.14 hardware_tests/test_microphone.py       # records 5s, reports peak volume
```

---

## Usage

Launch the full system with one command:
```bash
./start_system.bat
```
This starts all three processes in dependency order (robot client → vision → voice), each under its required Python interpreter.

### Voice Command Reference

**Keyword tier (instant, no network required):**
| Say | Action |
|---|---|
| "move right 20" / "move up 30" | Directional jog (default 10mm) |
| "left 10 and forward 20" | Chained commands, executed in order |
| "go near screw one" | Move to a taught approach pose |
| "calibrate" | Save current screw position as the alignment target |
| "align" | Run closed-loop PID alignment to the calibrated target |
| "fasten screw two" / "unfasten screw three" | Run the taught fastening/unfastening routine |
| "unfasten the screw" | Unfasten at the current (aligned) position — no number needed |
| "loosen" / "tighten" | Synonyms for unfasten/fasten |
| "quit" | Exit the voice process cleanly |

**LLM tier (anything else):** free-form requests like "draw a square," "do a dance," "draw a circle with radius 60," or "spin the screwdriver" are interpreted by GPT-4o and safety-checked before execution.

**Keyboard controls (vision window):** `W`/`A`/`S`/`D`/`Q`/`E` jog the robot; `C` calibrates; `T` runs alignment; click two points to measure real-world distance between them; `ESC` quits.

---

## Project Structure

```
├── robot_client.py           # Python 3.7 — sole robot connection, command dispatch, safety enforcement
├── vision_alignment.py       # Python 3.12 — camera, YOLO detection, PID alignment, teleop
├── voice_control.py          # Python 3.14 — speech capture, command routing
├── llm_command_generator.py  # GPT-4o integration, proxy-verified code generation
├── screw_operations.py       # Taught fasten/unfasten motion routines
├── start_system.bat          # Launches all three processes in order
├── __common.py                # JAKA SDK environment bootstrap
├── requirements-robot.txt
├── requirements-vision.txt
├── requirements-voice.txt
├── camera_calibration/        # ChArUco calibration capture/solve scripts + calibration data
├── training/                  # YOLO training & detection-experiment scripts
├── hardware_tests/            # Standalone connectivity checks (robot / camera / mic)
├── out/                       # Vendored JAKA SDK binaries
└── mounted_screw_dataset/,    # Training datasets (gitignored — see Datasets below)
    screw_dataset/, runs/
```

---

## Datasets & Training

The screw-detection model was trained on images labeled and exported via **Roboflow** in YOLOv8 format. The raw training datasets are not committed to this repository (large binary image sets don't belong in git) — the **trained model weights are included directly** (see Installation above), so no dataset download is required to *run* the system. If you want to retrain or extend the detector:

- **Screw head detection dataset:** *[Add your Roboflow dataset link here]*
- **Screw detection dataset (early iteration):** *[Add your Roboflow dataset link here]*

Retrain with `training/train_head.py` once the dataset is downloaded into `mounted_screw_dataset/`.

---

## Known Limitations

- The object detection model is trained on a limited set of screw types/viewpoints; detection can be less stable on unfamiliar screws or extreme viewing angles.
- Depth-sensor accuracy degrades on reflective/specular surfaces (a known limitation of stereo depth cameras) — Z-axis alignment mitigates this by using the robot's own encoder position instead of depth for the final correction.
- Per-move safety caps bound individual commands but not yet cumulative drift across a long sequence of small legal moves (a full Cartesian workspace-boundary check is a planned improvement).
- Chaining vision-guided alignment directly into the fastening motion is supported, but its reliability is currently bounded by an empirically-derived (rather than fully hand-eye calibrated) camera-to-tool offset.

## Future Work

- Full hand-eye calibration to replace the empirically-tuned camera-to-tool offset
- Cartesian workspace-boundary safety checks (beyond per-move caps)
- Expanded/retrained detection model across more screw types and viewpoints
- Extension toward general-purpose vision-guided pick-and-place
