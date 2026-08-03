
import __common
__common.init_env()
import jkrc

import json
import math
import socket
import struct
import threading

from screw_operations import (
    fastenScrew1, fastenScrew2, fastenScrew3, fastenScrew4, fastenScrew5,
    unfastenScrew1, unfastenScrew2, unfastenScrew3, unfastenScrew4, unfastenScrew5,
    fastenAll, unfastenAll, unfastenAfterAlign,
    IO_TOOL, GO, DIRECTION, OFF,
)

HOST = "127.0.0.1"
PORT = 9100

ABS = 0
INCR = 1

TCP = [0, 0, 0, 0, 0, 0]
USR = [0, 0, 0, 0, 0, 0]

JOINT_LIMITS_DEG = [
    (-140, -15),   
    (  30,  90),   
    (  50, 150),   
    ( -65,  55),   
    (-200, -45),   
    ( -90,  10),   
]
JOINT_LIMITS = [(math.radians(lo), math.radians(hi)) for lo, hi in JOINT_LIMITS_DEG]

MAX_LINEAR_MM = 150.0

cobot = None
cmd_lock = threading.Lock()


def check_linear_limits(move, mode):
    if mode != INCR:
        return None
    dist = math.sqrt(move[0] ** 2 + move[1] ** 2 + move[2] ** 2)
    if dist > MAX_LINEAR_MM:
        return "linear move of {:.1f} mm exceeds the {:.0f} mm per-move limit".format(dist, MAX_LINEAR_MM)
    return None


def check_joint_limits(joint_pos, mode):
    """Validate a joint move target (radians) against the safe envelope.

    Returns None if safe, otherwise a human-readable reason string.
    Incremental moves are validated against live current joints + delta.
    """
    if mode == INCR:
        err, current = cobot.get_joint_position()
        if err != 0:
            return "could not read current joints to validate incremental move"
        targets = [c + d for c, d in zip(current, joint_pos)]
    else:
        targets = list(joint_pos)
    for i, (t, (lo, hi)) in enumerate(zip(targets, JOINT_LIMITS)):
        if not (lo <= t <= hi):
            return ("joint {} target {:.1f} deg is outside safe range [{:.0f}, {:.0f}] deg"
                    .format(i + 1, math.degrees(t), math.degrees(lo), math.degrees(hi)))
    return None


class _LoggingCobot:
    """Wraps cobot: enforces joint safety limits and prints any nonzero errcode."""
    def __init__(self, cobot):
        self._cobot = cobot
    def __getattr__(self, name):
        attr = getattr(self._cobot, name)
        if not callable(attr):
            return attr
        def wrapper(*args, **kwargs):
            if name == "joint_move" and len(args) >= 2:
                reason = check_joint_limits(args[0], args[1])
                if reason is not None:
                    raise RuntimeError("SAFETY REJECTED joint_move: {}".format(reason))
            if name in ("linear_move", "linear_move_extend") and len(args) >= 2:
                reason = check_linear_limits(args[0], args[1])
                if reason is not None:
                    raise RuntimeError("SAFETY REJECTED {}: {}".format(name, reason))
            result = attr(*args, **kwargs)
            if isinstance(result, tuple) and len(result) >= 1 and isinstance(result[0], int) and result[0] != 0:
                print("WARNING: cobot.{}{} returned errcode {}".format(name, args, result[0]))
            return result
        return wrapper


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


def setup_robot():
    global cobot
    if cobot is not None:
        return cobot
    cobot = jkrc.RC("192.168.10.200")
    cobot.login()
    cobot.power_on()
    cobot.enable_robot()
    cobot.set_payload(mass=0.5, centroid=[0, 0, 20])
    cobot.set_tool_data(5, TCP, "tool_teleop")
    cobot.set_tool_id(5)
    cobot.set_user_frame_data(6, USR, "user_teleop")
    cobot.set_user_frame_id(6)
    cobot.set_digital_output(IO_TOOL, DIRECTION, OFF)
    cobot.set_digital_output(IO_TOOL, GO, OFF)
    print("Robot ready.")
    return cobot


def execute_move(cobot, command):
    try:
        if isinstance(command, list):
            reason = check_linear_limits(command, INCR)
            if reason is not None:
                print("SAFETY REJECTED move: {}".format(reason))
                return
            print("Executing move: {}".format(command))
            cobot.linear_move(command, INCR, False, 500)
            print("Move complete\n")
            return

        func = command.get("function", "linear_move")

        if func == "execute_script":
            script = command.get("script", "")
            print("Executing script:\n{}".format(script))
            ns = {
                "cobot": _LoggingCobot(cobot),
                "math": math,
                "ABS": ABS, "INCR": INCR,
                "IO_CABINET": 0, "IO_TOOL": 1, "IO_EXTEND": 2,
            }

            err, pos = cobot.get_tcp_position()
            if err == 0:
                x, y, z, rx, ry, rz = pos
                ns.update({
                    "x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz,
                    "current_position": list(pos),
                    "current_pos": list(pos),
                    "current_tcp_position": list(pos),
                })

            exec(script, ns)
            print("Script complete")
            return

        if "args" in command:
            args = command["args"]
            if func == "linear_move":
                end_pos, mode, _, speed = args[0], args[1], args[2], args[3]
                reason = check_linear_limits(end_pos, mode)
                if reason is not None:
                    print("SAFETY REJECTED linear_move: {}".format(reason))
                    return
                print("Executing linear_move: {} mode={} speed={}".format(end_pos, mode, speed))
                cobot.linear_move(end_pos, mode, True, speed)
                print("Move complete\n")
            elif func == "joint_move":
                joint_pos, mode, _, speed = args[0], args[1], args[2], args[3]
                reason = check_joint_limits(joint_pos, mode)
                if reason is not None:
                    print("SAFETY REJECTED joint_move: {}".format(reason))
                    return
                print("Executing joint_move: {} speed={}".format(joint_pos, speed))
                cobot.joint_move(joint_pos, mode, True, speed)
                print("Move complete\n")
            elif func == "set_digital_output":
                iotype, index, value = args[0], args[1], args[2]
                print("set_digital_output: iotype={} index={} value={}".format(iotype, index, value))
                cobot.set_digital_output(iotype, index, value)
            elif func == "motion_abort":
                print("Aborting motion!")
                cobot.motion_abort()
                print("Motion aborted")
            elif func == "unfasten_after_align":
                print("Unfastening at current (aligned) position")
                unfastenAfterAlign(cobot)
                print("Unfasten complete")
            else:
                print("Unknown function: {}".format(func))
            return

        if func == "circular_move":
            ret = cobot.get_tcp_position()
            if ret[0] != 0:
                print("Error getting TCP position: {}".format(ret[0]))
                return
            pos = list(ret[1])
            R           = command["radius_mm"]
            if R < 20:
                print("Radius {}mm too small, skipping.".format(R))
                return
            plane       = command.get("plane", "xy") or "xy"
            num_circles = int(command.get("circles", 1))
            speed       = command.get("speed", 50)

            mid_pos = pos[:]
            end_pos = pos[:]
            if plane == "xy":
                mid_pos[0] = pos[0]+R; mid_pos[1] = pos[1]+R
                end_pos[1] = pos[1]+2*R
            elif plane == "xz":
                mid_pos[0] = pos[0]+R; mid_pos[2] = pos[2]+R
                end_pos[2] = pos[2]+2*R
            elif plane == "yz":
                mid_pos[1] = pos[1]+R; mid_pos[2] = pos[2]+R
                end_pos[2] = pos[2]+2*R

            mid_rev = pos[:]
            end_rev = pos[:]
            if plane == "xy":
                mid_rev[0] = pos[0]-R; mid_rev[1] = pos[1]+R
            elif plane == "xz":
                mid_rev[0] = pos[0]-R; mid_rev[2] = pos[2]+R
            elif plane == "yz":
                mid_rev[1] = pos[1]-R; mid_rev[2] = pos[2]+R

            print("Executing circular_move: R={}mm plane={} x{}".format(R, plane, num_circles))
            logging_cobot = _LoggingCobot(cobot)
            for _ in range(num_circles):
                for arc_end, arc_mid in [(end_pos, mid_pos), (end_rev, mid_rev)]:
                    logging_cobot.circular_move(arc_end, arc_mid, ABS, True, speed, 200, 0.1)
            print("Move complete\n")

        elif func == "motion_abort":
            print("Aborting motion!")
            cobot.motion_abort()
            print("Motion aborted")

        elif func == "screw_operation":
            action = command.get("action", "fasten")
            screw_number = int(command.get("screw_number", 0))
            screw_funcs = {
                "fasten":   [fastenScrew1, fastenScrew2, fastenScrew3, fastenScrew4, fastenScrew5],
                "unfasten": [unfastenScrew1, unfastenScrew2, unfastenScrew3, unfastenScrew4, unfastenScrew5],
            }
            if screw_number == 0:
                fastenAll(cobot) if action == "fasten" else unfastenAll(cobot)
            elif 1 <= screw_number <= 5:
                screw_funcs[action][screw_number - 1](cobot)
            else:
                print("Invalid screw number: {}".format(screw_number))

        else:
            print("Unknown function: {}".format(func))

    except Exception as e:
        print("Error in execute_move: {}".format(e))


def handle_command(msg):
    cmd = msg.get("command")

    if cmd == "setup":
        setup_robot()
        return {"status": "ok"}

    elif cmd == "move":
        move = msg["move"]
        speed = msg.get("speed", 250)
        blocking = msg.get("blocking", False)
        reason = check_linear_limits(move, INCR)
        if reason is not None:
            print("SAFETY REJECTED move: {}".format(reason))
            return {"status": "error", "message": reason}
        cobot.linear_move(move, INCR, blocking, speed)
        return {"status": "ok"}

    elif cmd == "get_position":
        ret, pos = cobot.get_tcp_position()
        if ret == 0:
            return {"status": "ok", "position": list(pos)}
        return {"status": "error", "message": "get_tcp_position failed"}

    elif cmd == "execute":
        execute_move(cobot, msg.get("payload"))
        return {"status": "ok"}

    elif cmd == "shutdown":
        print("Client requested shutdown (disconnecting that client only).")
        return {"status": "ok"}

    else:
        print("Unknown command: {}".format(cmd))
        return {"status": "error", "message": "unknown command"}


def client_thread(conn, addr):
    print("Client connected from {}\n".format(addr))
    try:
        while True:
            msg = recv_msg(conn)
            if msg is None:
                print("Client {} disconnected.".format(addr))
                break

            with cmd_lock:
                reply = handle_command(msg)
            send_msg(conn, reply)

            if msg.get("command") == "shutdown":
                break
    except Exception as e:
        print("Client {} error: {}".format(addr, e))
    finally:
        conn.close()


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)
    print("Robot client listening on {}:{}...".format(HOST, PORT))
    print("Waiting for vision server and voice control to connect.")

    try:
        while True:
            conn, addr = server.accept()
            t = threading.Thread(target=client_thread, args=(conn, addr))
            t.daemon = True
            t.start()
    finally:
        server.close()
        print("Robot client stopped.")


if __name__ == "__main__":
    main()
