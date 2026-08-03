import re
import time
import os
import json
import socket
import struct
import numpy as np
import sounddevice as sd
import whisper
import keyboard

from llm_command_generator import ask_llm, _backend_name

HOST = "127.0.0.1"
PORT = 9100

VISION_COMMAND_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision_command.json")

SAMPLE_RATE = 16000
MAX_RECORD_SECONDS = 10
MIN_RECORD_SECONDS = 0.3
PTT_KEY = "space"
STOP_WORD = "over"

DIRECTION_MAP = {
    "right":     ("x",  1),
    "left":      ("x", -1),
    "forward":   ("y",  1),
    "backwards": ("y", -1),
    "backward":  ("y", -1),
    "back":      ("y", -1),
    "up":        ("z",  1),
    "down":      ("z", -1),
}

SCREW_NUMBER_WORDS = {
    "one": 1, "1": 1,
    "two": 2, "to": 2, "too": 2, "2": 2,
    "three": 3, "3": 3,
    "four": 4, "for": 4, "4": 4,
    "five": 5, "5": 5,
}

_sock = None   
model = None   

def _send_msg(sock, msg):
    data = json.dumps(msg).encode("utf-8")
    sock.sendall(struct.pack("!I", len(data)) + data)


def _recv_msg(sock):
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


def connect_robot():
    """Connect to robot_client.py and trigger robot setup. Call once at startup."""
    global _sock
    print(f"Connecting to robot client at {HOST}:{PORT}...")
    _sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _sock.connect((HOST, PORT))
    _send_msg(_sock, {"command": "setup"})
    reply = _recv_msg(_sock)
    print(f"Robot client connected: {reply}")


def _send_payload(payload):
    """Send a command payload to the robot client and wait for the reply."""
    _send_msg(_sock, {"command": "execute", "payload": payload})
    return _recv_msg(_sock)


def execute_screw_command(action, screw_number):
    """Fasten/unfasten a screw (1-5) or all screws (0) via screw_operations.py."""
    cmd = {"function": "screw_operation", "action": action, "screw_number": screw_number}
    print(f"Screw operation: {cmd}")
    _send_payload(cmd)
    print("Command sent!\n")


def execute_finetuned_command(cmd):
    """Send a positional-args command from llm_finetuned to the robot client."""
    try:
        func = cmd["function"]
        args = cmd.get("args", [])

        if func == "circular_move" and "args" not in cmd:
            print(f"Sending: {cmd}")
            _send_payload(cmd)
            print("Command sent!\n")
            return

        PASS_THROUGH = {"linear_move", "joint_move", "set_digital_output",
                        "motion_abort", "execute_script", "unfasten_after_align"}
        if func in PASS_THROUGH:
            payload = cmd if func == "execute_script" else {"function": func, "args": args}
            print(f"Sending: {payload}")
            _send_payload(payload)
            print("Command sent!\n")
        else:
            print(f"Unknown function: {func}")
    except Exception as e:
        print(f"Error in execute_finetuned_command: {e}")


def execute_command(parsed):
    try:
        axis_map = {"x": 0, "y": 1, "z": 2}
        move = [0, 0, 0, 0, 0, 0]
        value = parsed["distance"] * parsed["direction"]
        if parsed["axis"] == "x":
            value = -value
        move[axis_map[parsed["axis"]]] = value
        print(f"Moving: {move}")
        _send_payload(move)
        print("Command sent!\n")
    except Exception as e:
        print(f"Error in execute_command: {e}")


def send_vision_command(action):
    with open(VISION_COMMAND_FILE, "w") as f:
        json.dump({"action": action}, f)
    print(f"Vision command sent: {action}")


def record_push_to_talk():
    chunk_duration = 0.03
    chunk_size = int(SAMPLE_RATE * chunk_duration)
    max_chunks = int(MAX_RECORD_SECONDS / chunk_duration)
    min_chunks = int(MIN_RECORD_SECONDS / chunk_duration)

    print(f"Hold [{PTT_KEY.upper()}] and speak...")
    keyboard.wait(PTT_KEY)
    print("Recording... release to finish.")

    chunks = []
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='float32',
                        device=1, blocksize=chunk_size) as stream:
        while keyboard.is_pressed(PTT_KEY):
            chunk, _ = stream.read(chunk_size)
            chunks.append(chunk.copy())
            if len(chunks) >= max_chunks:
                print("Max duration reached, processing...")
                break

    if len(chunks) < min_chunks:
        print("Too short - ignored.")
        return None

    return np.concatenate(chunks).flatten()

def transcribe(audio):
    print("Transcribing...")
    result = model.transcribe(audio, fp16=False)
    return result["text"].strip()


def parse_command(text):
    words = re.findall(r"[\w']+", text.lower())
    axis, direction = None, None
    for word in words:
        if word in DIRECTION_MAP:
            axis, direction = DIRECTION_MAP[word]
            break
    if axis is None:
        return None
    match = re.search(r'\b(\d+(?:\.\d+)?)\b', text)
    distance = int(float(match.group(1))) if match else 10
    return {"axis": axis, "distance": distance, "direction": direction}

def parse_screw_command(text):
    t = text.lower()
    if re.search(r'\b(unfasten|loosen|unscrew)\b', t):
        action = "unfasten"
    elif re.search(r'\b(fasten|tighten)\b', t):
        action = "fasten"
    else:
        return None
    if re.search(r'\ball\b', t):
        return {"action": action, "screw_number": 0}
    for word, n in SCREW_NUMBER_WORDS.items():
        if re.search(r'\b' + word + r'\b', t):
            return {"action": action, "screw_number": n}
    return None

def main():
    global model

    connect_robot()

    print("Loading Whisper model...")
    model = whisper.load_model("small")
    print("Ready!")

    while True:
        audio = record_push_to_talk()
        if audio is None:
            continue
        command = transcribe(audio)
        print(f"You said: {command}")
        if "quit" in command.lower() or "program over" in command.lower():
            print("Ending program.")
            break
        if "calibrate" in command.lower():
            send_vision_command("calibrate")
            continue
        if "align" in command.lower():
            send_vision_command("align")
            continue
        command = re.sub(r'\b' + STOP_WORD + r'\b\.?\s*$', '', command, flags=re.IGNORECASE).strip()
        if not command:
            continue
        sub_commands = [s.strip().strip('.,') for s in re.split(r'\band\b|\bthen\b|,', command, flags=re.IGNORECASE) if s.strip().strip('.,')]
        parsed_parts = []
        for s in sub_commands:
            screw = parse_screw_command(s)
            if screw is not None:
                parsed_parts.append(("screw", screw))
            else:
                move = parse_command(s)
                parsed_parts.append(("move", move) if move is not None else None)

        if sub_commands and all(p is not None for p in parsed_parts):
            for kind, parsed in parsed_parts:
                print(f"Parsed: {kind} {parsed}")
                if kind == "screw":
                    execute_screw_command(parsed["action"], parsed["screw_number"])
                else:
                    execute_command(parsed)
        else:
            print("Sending to LLM...")
            print()
            tool_results = ask_llm(command)
            if not tool_results:
                print("Invalid instruction.\n")
                continue
            print(f"{_backend_name}: {tool_results}")
            print()
            for tool_result in tool_results:
                execute_finetuned_command(tool_result)


if __name__ == "__main__":
    main()
