#!/usr/bin/env python3
"""Per-wheel isolated speed calibration: command one wheel at a time
(others held at 0) and report each wheel's actual measured speed at the
same target and PID gains, to find wheel-to-wheel mismatch that shows up
as unwanted strafe under a nominally pure-translation command.

Chassis MUST be lifted before running.

    python3 tools/wheel_calibration.py --spin 2.5 --kp 15 --ki 35
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import crc16_modbus, encode_frame, open_serial, CMD_HEARTBEAT  # noqa: E402
from drive_check import Link, pid_frame, vel_frame, EDGES_PER_WHEEL_REV, WHEEL_NAMES  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--spin", type=float, default=2.5)
    ap.add_argument("--spin-seconds", type=float, default=4.0)
    ap.add_argument("--kp", type=float, default=15.0)
    ap.add_argument("--ki", type=float, default=35.0)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--integral-limit", type=float, default=10.0)
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    link = Link(fd)
    seq = 0

    def heartbeat():
        nonlocal seq
        link.write(encode_frame(CMD_HEARTBEAT, b"", seq)); seq += 1

    print(f"wheel_calibration on {args.port} @ {args.baud}")
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

    results = []
    for idx, name in enumerate(WHEEL_NAMES):
        print(f"\nphase 3.{idx}: {name} only at +{args.spin} rad/s for "
              f"{args.spin_seconds}s (others held at 0) — CHASSIS MUST BE LIFTED")
        targets = [0.0, 0.0, 0.0, 0.0]
        targets[idx] = args.spin
        counts_before = link.odom[-1][1] if link.odom else (0, 0, 0, 0)
        t_start = time.monotonic()
        t_end = t_start + args.spin_seconds
        next_cmd = t_start
        while time.monotonic() < t_end:
            if time.monotonic() >= next_cmd:
                link.write(vel_frame(*targets, seq)); seq += 1
                next_cmd += 0.05
            link.pump(0.01)
        counts_after = link.odom[-1][1]
        actual_dt = link.odom[-1][0] - t_start

        # zero + brief settle before the next wheel
        t_end2 = time.monotonic() + 1.5
        next_cmd = time.monotonic()
        while time.monotonic() < t_end2:
            if time.monotonic() >= next_cmd:
                link.write(vel_frame(0, 0, 0, 0, seq)); seq += 1
                next_cmd += 0.05
            link.pump(0.02)

        delta = counts_after[idx] - counts_before[idx]
        measured = (delta / EDGES_PER_WHEEL_REV) * 6.2831853 / actual_dt
        ratio = measured / args.spin if args.spin else 0.0
        # cross-talk: did any other wheel move while "off"?
        cross = [counts_after[j] - counts_before[j] for j in range(4) if j != idx]
        results.append((name, delta, measured, ratio, max(abs(c) for c in cross)))
        print(f"  {name}: {delta} edges, {measured:.2f} rad/s "
              f"(target {args.spin:.2f}, ratio {ratio:.2f}), "
              f"max cross-talk on other wheels: {max(abs(c) for c in cross)} edges")

    link.write(vel_frame(0, 0, 0, 0, seq)); seq += 1
    link.pump(0.3)
    fd.close()

    print(f"\n{'wheel':6} {'measured rad/s':>15} {'ratio to target':>16} {'cross-talk':>11}")
    speeds = [r[2] for r in results]
    for name, delta, measured, ratio, cross in results:
        print(f"{name:6} {measured:15.2f} {ratio:16.2f} {cross:11d}")

    spread = (max(speeds) - min(speeds)) / (sum(speeds) / len(speeds)) * 100 if speeds else 0
    print(f"\nwheel-to-wheel spread: {spread:.1f}% "
          f"(max {max(speeds):.2f} vs min {min(speeds):.2f} rad/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
