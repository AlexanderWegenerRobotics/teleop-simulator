#!/usr/bin/env python3
"""
Avatar Transmission Test
- Drives avatar through state machine: IDLE -> HOMING -> AWAITING -> ENGAGED
- Sends circle trajectory to both arms and sine wave to head
- Receives and prints state feedback
"""

import socket
import struct
import time
import math
import threading

# ── Config ────────────────────────────────────────────────────────────────────

AVATAR_IP   = "10.112.24.173"
#AVATAR_IP = "127.0.0.1"

# Avatar
AVATAR_SEND_PORT    = 7000   # script sends to avatar's receive_port
AVATAR_RECEIVE_PORT = 8000   # script receives on avatar's send_port

# Arm left
ARM_LEFT_SEND_PORT    = 7001
ARM_LEFT_RECEIVE_PORT = 8001

# Arm right
ARM_RIGHT_SEND_PORT    = 7002
ARM_RIGHT_RECEIVE_PORT = 8002

# Head
HEAD_SEND_PORT    = 7003
HEAD_RECEIVE_PORT = 8003

# ── SysState ──────────────────────────────────────────────────────────────────

class SysState:
    OFFLINE  = 0
    IDLE     = 1
    HOMING   = 2
    AWAITING = 3
    ENGAGED  = 4
    PAUSED   = 5
    FAULT    = 6
    STOP     = 7

STATE_NAMES = {v: k for k, v in vars(SysState).items() if not k.startswith('_')}

# ── Message formats (must match #pragma pack(1) structs in common.hpp) ────────

# AvatarCommandMsg: SysState(u8), session_id(u8), timestamp_ns(u64)
AVATAR_CMD_FMT  = '<BBQ'
AVATAR_CMD_SIZE = struct.calcsize(AVATAR_CMD_FMT)

# AvatarStateMsg: SysState(u8), timestamp_ns(u64)
AVATAR_STATE_FMT  = '<BQ'
AVATAR_STATE_SIZE = struct.calcsize(AVATAR_STATE_FMT)

# ArmCommandMsg: device_id(u8), SysState(u8), position(3f), quaternion(4f), gripper(f), timestamp_ns(u64)
ARM_CMD_FMT  = '<BB3f4ffQ'
ARM_CMD_SIZE = struct.calcsize(ARM_CMD_FMT)

# ArmStateMsg: device_id(u8), SysState(u8), position(3f), quaternion(4f), tau_ext(7f), timestamp_ns(u64)
ARM_STATE_FMT  = '<BB3f4f7fQ'
ARM_STATE_SIZE = struct.calcsize(ARM_STATE_FMT)

# HeadCommandMsg: device_id(u8), SysState(u8), pan(f), tilt(f), timestamp_ns(u64)
HEAD_CMD_FMT  = '<BBffQ'
HEAD_CMD_SIZE = struct.calcsize(HEAD_CMD_FMT)

# HeadStateMsg: device_id(u8), SysState(u8), pan(f), tilt(f), timestamp_ns(u64)
HEAD_STATE_FMT  = '<BBffQ'
HEAD_STATE_SIZE = struct.calcsize(HEAD_STATE_FMT)

# ── Helpers ───────────────────────────────────────────────────────────────────

def now_ns():
    return int(time.time() * 1e9)

def make_socket(receive_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", receive_port))
    sock.settimeout(0.01)
    return sock

# ── Senders ───────────────────────────────────────────────────────────────────

def send_avatar_cmd(sock, state, session_id=0):
    msg = struct.pack(AVATAR_CMD_FMT, state, session_id, now_ns())
    sock.sendto(msg, (AVATAR_IP, AVATAR_SEND_PORT))

def send_arm_cmd(sock, port, device_id, state, position, quaternion, gripper=0.0):
    # quaternion: w x y z
    msg = struct.pack(ARM_CMD_FMT,
        device_id, state,
        position[0], position[1], position[2],
        quaternion[0], quaternion[1], quaternion[2], quaternion[3],
        gripper, now_ns())
    sock.sendto(msg, (AVATAR_IP, port))

def send_head_cmd(sock, port, device_id, state, pan, tilt):
    msg = struct.pack(HEAD_CMD_FMT, device_id, state, pan, tilt, now_ns())
    sock.sendto(msg, (AVATAR_IP, port))

# ── Receivers (run in background threads) ─────────────────────────────────────

latest = {
    "avatar_state": None,
    "arm_left_state": None,
    "arm_right_state": None,
    "head_state": None,
}

def recv_avatar_state(sock):
    while running:
        try:
            data, _ = sock.recvfrom(AVATAR_STATE_SIZE)
            if len(data) == AVATAR_STATE_SIZE:
                state, ts = struct.unpack(AVATAR_STATE_FMT, data)
                latest["avatar_state"] = state
        except socket.timeout:
            pass

def recv_arm_state(sock, key):
    while running:
        try:
            data, _ = sock.recvfrom(ARM_STATE_SIZE)
            if len(data) == ARM_STATE_SIZE:
                fields = struct.unpack(ARM_STATE_FMT, data)
                latest[key] = {
                    "device_id": fields[0],
                    "state":     fields[1],
                    "pos":       fields[2:5],
                    "quat":      fields[5:9],
                    "tau_ext":   fields[9:16],
                }
        except socket.timeout:
            pass

def recv_head_state(sock):
    while running:
        try:
            data, _ = sock.recvfrom(HEAD_STATE_SIZE)
            if len(data) == HEAD_STATE_SIZE:
                dev_id, state, pan, tilt, ts = struct.unpack(HEAD_STATE_FMT, data)
                latest["head_state"] = {"state": state, "pan": pan, "tilt": tilt}
        except socket.timeout:
            pass

def print_status():
    av  = latest["avatar_state"]
    al  = latest["arm_left_state"]
    ar  = latest["arm_right_state"]
    hd  = latest["head_state"]

    av_str = STATE_NAMES.get(av, "?") if av is not None else "—"
    al_str = STATE_NAMES.get(al["state"], "?") if al else "—"
    ar_str = STATE_NAMES.get(ar["state"], "?") if ar else "—"
    hd_str = STATE_NAMES.get(hd["state"], "?") if hd else "—"

    al_pos = f"{al['pos']}" if al else "—"
    ar_pos = f"{ar['pos']}" if ar else "—"
    hd_pos = f"pan={hd['pan']:.3f} tilt={hd['tilt']:.3f}" if hd else "—"

    print(f"\r  avatar:{av_str:10}  L:{al_str:10} {al_pos}  R:{ar_str:10} {ar_pos}  head:{hd_str:10} {hd_pos}   ", end="")

# ── Circle trajectory ─────────────────────────────────────────────────────────

def circle_pose(t, radius=0.05, center=(0.0, 0.0, 0.0), freq=0.2):
    """Returns position delta as circle in XZ plane, identity-ish quaternion."""
    x = center[0] + radius * math.cos(2 * math.pi * freq * t)
    y = center[1]
    z = center[2] + radius * math.sin(2 * math.pi * freq * t)
    return (x, y, z), (1.0, 0.0, 0.0, 0.0)  # w x y z identity rotation

# ── Main ──────────────────────────────────────────────────────────────────────

running = True

def main():
    global running

    # sockets
    avatar_sock    = make_socket(AVATAR_RECEIVE_PORT)
    arm_left_sock  = make_socket(ARM_LEFT_RECEIVE_PORT)
    arm_right_sock = make_socket(ARM_RIGHT_RECEIVE_PORT)
    head_sock      = make_socket(HEAD_RECEIVE_PORT)

    # receiver threads
    threads = [
        threading.Thread(target=recv_avatar_state,  args=(avatar_sock,),            daemon=True),
        threading.Thread(target=recv_arm_state,     args=(arm_left_sock,  "arm_left_state"),  daemon=True),
        threading.Thread(target=recv_arm_state,     args=(arm_right_sock, "arm_right_state"), daemon=True),
        threading.Thread(target=recv_head_state,    args=(head_sock,),               daemon=True),
    ]
    for t in threads:
        t.start()

    print("=" * 60)
    print("  Avatar Transmission Test")
    print("=" * 60)

    try:
        # ── Step 1: Request HOMING ────────────────────────────────
        print("\n[1] Requesting HOMING...")
        for _ in range(20):
            send_avatar_cmd(avatar_sock, SysState.HOMING)
            time.sleep(0.1)

        # ── Step 2: Wait for AWAITING ─────────────────────────────
        print("[2] Waiting for all devices to reach AWAITING...")
        timeout = time.time() + 30.0
        while time.time() < timeout:
            print_status()
            av = latest["avatar_state"]
            if av == SysState.AWAITING:
                break
            send_avatar_cmd(avatar_sock, SysState.HOMING)
            time.sleep(0.1)
        else:
            print("\n[WARN] Timed out waiting for AWAITING — continuing anyway")

        time.sleep(0.5)

        # ── Step 3: Request ENGAGED ───────────────────────────────
        print("\n[3] Requesting ENGAGED...")
        for _ in range(20):
            send_avatar_cmd(avatar_sock, SysState.ENGAGED)
            time.sleep(0.1)

        # ── Step 4: Circle trajectory ─────────────────────────────
        print("[4] Running circle trajectory for 20 seconds...")
        print("    (Ctrl+C to stop early)\n")

        freq    = 200   # Hz
        dt      = 1.0 / freq
        t_start = time.time()
        t_end   = t_start + 20.0

        while time.time() < t_end:
            t = time.time() - t_start

            # left arm — circle in XZ
            pos_l, quat_l = circle_pose(t, radius=0.05, center=(0.0, 0.0, 0.0), freq=0.2)
            send_arm_cmd(arm_left_sock, ARM_LEFT_SEND_PORT, 0, SysState.ENGAGED, pos_l, quat_l)

            # right arm — circle offset by pi (opposite phase)
            pos_r, quat_r = circle_pose(t + 2.5, radius=0.05, center=(0.0, 0.0, 0.0), freq=0.2)
            send_arm_cmd(arm_right_sock, ARM_RIGHT_SEND_PORT, 1, SysState.ENGAGED, pos_r, quat_r)

            # head — slow sine on pan, small tilt
            pan  = 0.3 * math.sin(2 * math.pi * 0.1 * t)
            tilt = 0.1 * math.sin(2 * math.pi * 0.05 * t)
            send_head_cmd(head_sock, HEAD_SEND_PORT, 0, SysState.ENGAGED, pan, tilt)

            print_status()
            time.sleep(dt)

        # ── Step 5: Return to IDLE ────────────────────────────────
        print("\n\n[5] Requesting IDLE...")
        for _ in range(20):
            send_avatar_cmd(avatar_sock, SysState.IDLE)
            time.sleep(0.1)

        print("\n[DONE]")

    except KeyboardInterrupt:
        print("\n\n[Interrupted] Sending IDLE...")
        for _ in range(10):
            send_avatar_cmd(avatar_sock, SysState.IDLE)
            time.sleep(0.05)
    finally:
        running = False
        avatar_sock.close()
        arm_left_sock.close()
        arm_right_sock.close()
        head_sock.close()

if __name__ == "__main__":
    main()
