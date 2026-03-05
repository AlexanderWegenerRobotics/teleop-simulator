#!/usr/bin/env python3
"""
Simulate a stream receiver sending feedback to the StreamQualityController.

Usage:
    python test_stream_quality.py                          # interactive mode
    python test_stream_quality.py --scenario degrade       # auto scenario
    python test_stream_quality.py --scenario recover
    python test_stream_quality.py --scenario flap
    python test_stream_quality.py --loss 0.2 --jitter 40   # fixed values
"""

import argparse
import socket
import struct
import time
import sys

FEEDBACK_FORMAT = '<QfffIII'
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FORMAT)

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 5005
REPORT_INTERVAL = 0.5


def make_feedback(loss: float, jitter: float, rtt: float = 10.0,
                  packets_rx: int = 1000, packets_lost: int = 0,
                  frames_rx: int = 500) -> bytes:
    timestamp_ns = int(time.time() * 1e9)
    packets_lost = int(packets_rx * loss)
    return struct.pack(FEEDBACK_FORMAT,
                       timestamp_ns, loss, jitter, rtt,
                       packets_rx, packets_lost, frames_rx)


def send_report(sock: socket.socket, addr: tuple, loss: float, jitter: float):
    msg = make_feedback(loss, jitter)
    sock.sendto(msg, addr)
    print(f"  sent: loss={loss:.1%}  jitter={jitter:.1f}ms")


def run_scenario_degrade(sock, addr):
    """Gradually degrade from perfect to critical conditions."""
    print("\n--- Scenario: DEGRADE ---")
    print("Phase 1: NORMAL (good conditions)")
    for _ in range(6):
        send_report(sock, addr, 0.01, 5.0)
        time.sleep(REPORT_INTERVAL)

    print("Phase 2: Elevated loss → should trigger DEGRADED")
    for _ in range(8):
        send_report(sock, addr, 0.08, 15.0)
        time.sleep(REPORT_INTERVAL)

    print("Phase 3: Severe loss → should trigger CRITICAL")
    for _ in range(8):
        send_report(sock, addr, 0.25, 60.0)
        time.sleep(REPORT_INTERVAL)

    print("Phase 4: Hold critical")
    for _ in range(6):
        send_report(sock, addr, 0.30, 80.0)
        time.sleep(REPORT_INTERVAL)

    print("--- Scenario complete ---\n")


def run_scenario_recover(sock, addr):
    """Start in bad conditions, gradually recover."""
    print("\n--- Scenario: RECOVER ---")
    print("Phase 1: Bad conditions → DEGRADED/CRITICAL")
    for _ in range(8):
        send_report(sock, addr, 0.20, 55.0)
        time.sleep(REPORT_INTERVAL)

    print("Phase 2: Improving → should recover to DEGRADED")
    for _ in range(10):
        send_report(sock, addr, 0.07, 20.0)
        time.sleep(REPORT_INTERVAL)

    print("Phase 3: Good conditions → should recover to NORMAL")
    for _ in range(10):
        send_report(sock, addr, 0.01, 5.0)
        time.sleep(REPORT_INTERVAL)

    print("--- Scenario complete ---\n")


def run_scenario_flap(sock, addr):
    """Alternate between good and bad to test hysteresis."""
    print("\n--- Scenario: FLAP (hysteresis test) ---")
    for cycle in range(5):
        print(f"Cycle {cycle + 1}: bad")
        for _ in range(4):
            send_report(sock, addr, 0.12, 35.0)
            time.sleep(REPORT_INTERVAL)

        print(f"Cycle {cycle + 1}: good")
        for _ in range(4):
            send_report(sock, addr, 0.02, 8.0)
            time.sleep(REPORT_INTERVAL)

    print("--- Scenario complete ---\n")


def run_scenario_stale(sock, addr):
    """Connect then go silent to test STALE → NO_RECEIVER transitions."""
    print("\n--- Scenario: STALE ---")
    print("Phase 1: Establish connection")
    for _ in range(6):
        send_report(sock, addr, 0.01, 5.0)
        time.sleep(REPORT_INTERVAL)

    print("Phase 2: Stop sending (wait 15s for STALE → NO_RECEIVER)")
    time.sleep(15)

    print("Phase 3: Resume sending")
    for _ in range(6):
        send_report(sock, addr, 0.01, 5.0)
        time.sleep(REPORT_INTERVAL)

    print("--- Scenario complete ---\n")


def run_interactive(sock, addr):
    """Manual control: type 'loss jitter' values or preset names."""
    print("\n--- Interactive Mode ---")
    print("Commands:")
    print("  <loss> <jitter>   e.g. '0.1 30' sends 10% loss, 30ms jitter")
    print("  good              sends loss=0.01, jitter=5")
    print("  bad               sends loss=0.15, jitter=40")
    print("  terrible          sends loss=0.30, jitter=80")
    print("  stop              pause sending (test STALE)")
    print("  quit              exit")
    print()

    presets = {
        'good':     (0.01, 5.0),
        'bad':      (0.15, 40.0),
        'terrible': (0.30, 80.0),
    }

    loss, jitter = 0.01, 5.0
    paused = False

    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if cmd == 'quit':
            break
        elif cmd == 'stop':
            paused = True
            print("  paused (not sending)")
            continue
        elif cmd in presets:
            loss, jitter = presets[cmd]
            paused = False
        elif cmd:
            try:
                parts = cmd.split()
                loss = float(parts[0])
                jitter = float(parts[1]) if len(parts) > 1 else 10.0
                paused = False
            except (ValueError, IndexError):
                print("  invalid input")
                continue

        if not paused:
            for _ in range(3):
                send_report(sock, addr, loss, jitter)
                time.sleep(REPORT_INTERVAL)


SCENARIOS = {
    'degrade': run_scenario_degrade,
    'recover': run_scenario_recover,
    'flap':    run_scenario_flap,
    'stale':   run_scenario_stale,
}


def main():
    parser = argparse.ArgumentParser(description='Test stream quality controller')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--scenario', choices=list(SCENARIOS.keys()),
                        help='Run a predefined scenario')
    parser.add_argument('--loss', type=float, help='Fixed loss rate (0.0-1.0)')
    parser.add_argument('--jitter', type=float, default=10.0, help='Fixed jitter in ms')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Duration in seconds for fixed mode')
    args = parser.parse_args()

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Sending feedback to {args.host}:{args.port}")

    if args.scenario:
        SCENARIOS[args.scenario](sock, addr)
    elif args.loss is not None:
        print(f"Fixed mode: loss={args.loss:.1%} jitter={args.jitter:.1f}ms "
              f"duration={args.duration:.0f}s")
        end_time = time.time() + args.duration
        while time.time() < end_time:
            send_report(sock, addr, args.loss, args.jitter)
            time.sleep(REPORT_INTERVAL)
    else:
        run_interactive(sock, addr)

    sock.close()
    print("Done.")


if __name__ == '__main__':
    main()