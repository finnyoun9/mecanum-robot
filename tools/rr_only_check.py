#!/usr/bin/env python3
"""Isolated RR-only spin check over the existing rtos_drive UART link.

Commands only RR (others held at 0) via CMD_VEL_CTRL, using the same
legacy Kp=100/Ki=300 tune as drive_check.py --tune-pid, and reports all
four wheels' encoder deltas so a genuine RR-specific fault (vs. a shared
wiring/power issue) is visible: if RR still lags while FL/FR/RL stay at
exactly 0 (as commanded), the fault is isolated to RR.

    python3 tools/rr_only_check.py --spin 2.5

Chassis must be lifted.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import crc16_modbus, encode_frame, open_serial  # noqa: E402
from drive_check import Link, pid_frame, vel_frame, CMD_HEARTBEAT, EDGES_PER_WHEEL_REV, WHEEL_NAMES  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--spin", type=float, default=2.5)
    ap.add_argument("--spin-seconds", type=float, default=4.0)
    ap.add_argument("--kp", type=float, default=100.0)
    ap.add_argument("--ki", type=float, default=300.0)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--integral-limit", type=float, default=1.0)
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    link = Link(fd)
    seq = 0

    def heartbeat():
        nonlocal seq
        link.write(encode_frame(CMD_HEARTBEAT, b"", seq)); seq += 1

    print(f"rr_only_check on {args.port} @ {args.baud}")
    print("phase 1: passive 3s")
    t_end = time.monotonic() + 3.0
    next_hb = time.monotonic()
    while time.monotonic() < t_end:
        if time.monotonic() >= next_hb:
            heartbeat(); next_hb = time.monotonic() + 1.0
        link.pump(0.05)

    print(f"phase 2: PI tune (Kp={args.kp} Ki={args.ki} Kd={args.kd} "
          f"integral_limit={args.integral_limit}) on all 4 wheels")
    for motor in range(4):
        link.write(pid_frame(motor, args.kp, args.ki, args.kd,
                              args.integral_limit, seq)); seq += 1
    time.sleep(0.2)
    link.pump(0.2)

    target = args.spin
    spin_s = args.spin_seconds
    print(f"phase 3: RR only at +{target} rad/s for {spin_s}s "
          f"(FL/FR/RL held at 0) — CHASSIS MUST BE LIFTED")
    counts_before = link.odom[-1][1]
    t_start = time.monotonic()
    t_end = t_start + spin_s
    next_cmd = t_start
    while time.monotonic() < t_end:
        if time.monotonic() >= next_cmd:
            link.write(vel_frame(0, 0, 0, target, seq)); seq += 1
            next_cmd += 0.05
        link.pump(0.01)
    counts_after = link.odom[-1][1]
    actual_dt = link.odom[-1][0] - t_start

    print("phase 4: zero for 2s")
    t_end = time.monotonic() + 2.0
    next_cmd = time.monotonic()
    while time.monotonic() < t_end:
        if time.monotonic() >= next_cmd:
            link.write(vel_frame(0, 0, 0, 0, seq)); seq += 1
            next_cmd += 0.05
        link.pump(0.02)
    counts_coast = link.odom[-1][1]
    link.write(vel_frame(0, 0, 0, 0, seq)); seq += 1
    link.pump(0.3)
    fd.close()

    print(f"\n{'wheel':6} {'edges':>8} {'measured rad/s':>15} {'coast edges':>12}")
    for i, name in enumerate(WHEEL_NAMES):
        delta = counts_after[i] - counts_before[i]
        coast = counts_coast[i] - counts_after[i]
        measured = (delta / EDGES_PER_WHEEL_REV) * 6.2831853 / actual_dt
        print(f"{name:6} {delta:8d} {measured:15.2f} {coast:12d}"
              + ("  <- commanded" if name == "RR" else "  (should be ~0)"))
    print(f"\nCRC failures: {link.crc_fail}, error_flags last: "
          f"{link.odom[-1][2]:#04x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
