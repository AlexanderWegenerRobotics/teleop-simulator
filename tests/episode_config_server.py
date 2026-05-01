"""
episode_config_server.py
------------------------
Lightweight UDP server that runs alongside the Avatar process.
When Avatar resets and needs a new episode configuration, it sends a
request here and we reply with randomised pick/place poses and task mode.

Later: replace sample_pick_pose() / sample_place_pose() with object
detection output while keeping the same UDP interface.

Protocol (msgpack over UDP):
  Request  <- Avatar: {"type": "request_episode_config"}
  Response -> Avatar: {
      "pick_x":  float,  # world frame, metres
      "pick_y":  float,
      "pick_z":  float,
      "place_x": float,
      "place_y": float,
      "place_z": float,
      "mode":    int     # 0 = unimanual, 1 = bimanual
  }
"""

import socket
import random
import logging
import msgpack

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("episode_config_server")

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 9100          # Avatar sends requests here

# Workspace bounds (world frame, metres) — tune to your table / arm reach
PICK_X_RANGE  = (0.55, 0.90)
PICK_Y_RANGE  = (-0.05, 0.35)
PICK_Z        = 0.62          # table surface + half box height, fixed

PLACE_X_RANGE = (0.65, 0.90)
PLACE_Y_RANGE = (-0.05, 0.35)
PLACE_Z       = 0.5  # same height as pick

MIN_PICK_PLACE_DIST = 0.10    # metres — avoid degenerate episodes

# Task mode distribution
# 0 = unimanual (left arm only)
# 1 = bimanual  (left picks, handover, right places)
MODE_WEIGHTS = {0: 0.5, 1: 0.5}


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------

def sample_pick_pose():
    x = random.uniform(*PICK_X_RANGE)
    y = random.uniform(*PICK_Y_RANGE)
    return x, y, PICK_Z


def sample_place_pose(pick_x, pick_y):
    """Sample a place pose that is far enough from the pick pose."""
    for _ in range(100):
        x = random.uniform(*PLACE_X_RANGE)
        y = random.uniform(*PLACE_Y_RANGE)
        dist = ((x - pick_x) ** 2 + (y - pick_y) ** 2) ** 0.5
        if dist >= MIN_PICK_PLACE_DIST:
            return x, y, PLACE_Z
    log.warning("Could not sample place pose satisfying min distance, sampling independently")
    x = random.uniform(*PLACE_X_RANGE)
    y = random.uniform(*PLACE_Y_RANGE)
    return x, y, PLACE_Z


def sample_mode():
    modes = list(MODE_WEIGHTS.keys())
    weights = list(MODE_WEIGHTS.values())
    return random.choices(modes, weights=weights, k=1)[0]


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------

def run():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_HOST, LISTEN_PORT))
    log.info(f"Listening on {LISTEN_HOST}:{LISTEN_PORT}")

    while True:
        try:
            raw, addr = sock.recvfrom(4096)
            msg = msgpack.unpackb(raw, raw=False)

            if msg.get("type") != "request_episode_config":
                log.warning(f"Unknown message type: {msg.get('type')}")
                continue

            pick_x, pick_y, pick_z     = sample_pick_pose()
            place_x, place_y, place_z  = sample_place_pose(pick_x, pick_y)
            mode                       = sample_mode()

            response = {
                "pick_x":  pick_x,
                "pick_y":  pick_y,
                "pick_z":  pick_z,
                "place_x": place_x,
                "place_y": place_y,
                "place_z": place_z,
                "mode":    mode,
            }

            sock.sendto(msgpack.packb(response), addr)

            mode_str = "unimanual" if mode == 0 else "bimanual"
            log.info(
                f"Episode config sent to {addr} | "
                f"pick=({pick_x:.3f},{pick_y:.3f},{pick_z:.3f}) "
                f"place=({place_x:.3f},{place_y:.3f},{place_z:.3f}) "
                f"mode={mode_str}"
            )

        except Exception as e:
            log.error(f"Error: {e}")


if __name__ == "__main__":
    run()