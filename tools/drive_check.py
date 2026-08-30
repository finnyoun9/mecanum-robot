#!/usr/bin/env python3
"""End-to-end drive check: Pi 5 -> STM32 FreeRTOS (rtos_drive target).

 Passive mode (default): verifies the 50 Hz odometry stream, heartbeat
 ACKs and that encoder counts stay still with no motion command.

 Motion mode (--spin RAD_S): preserves the firmware's active PI gains by
 default, streams CMD_VEL_CTRL at 20 Hz for a few seconds, and reports per-wheel
 encoder-derived speed from the returned odometry frames — proving the
 full chain Pi -> UART DMA -> CtrlTask PI -> TB6612 -> encoders -> odom.

   python3 tools/drive_check.py                  # passive, safe
   python3 tools/drive_check.py --spin 5.0       # LIFT THE CHASSIS FIRST

 Stdlib only; run on the Pi host (not in the ROS2 container).
 Exit 0 = all checks passed.
"""

import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import crc16_modbus, encode_frame, open_serial  # noqa: E402

CMD_VEL_CTRL = 0x10
CMD_EMERGENCY_STOP = 0x11
CMD_PID_TUNE = 0x12
CMD_HEARTBEAT = 0x1F
CMD_ODOM = 0x20
CMD_ACK = 0x2E

EDGES_PER_WHEEL_REV = 448
WHEEL_NAMES = ("FL", "FR", "RL", "RR")


def vel_frame(w1, w2, w3, w4, seq):
    return encode_frame(CMD_VEL_CTRL, struct.pack("<4f", w1, w2, w3, w4), seq)


def pid_frame(motor_id, kp, ki, kd, integral_limit, seq):
    payload = struct.pack("<B4f", motor_id, kp, ki, kd, integral_limit)
    return encode_frame(CMD_PID_TUNE, payload, seq)


def parse_odom(payload):
    if len(payload) < 48:
        return None
    counts = struct.unpack_from("<4i", payload, 0)
    error_flags = payload[47]
    return counts, error_flags


class Link:
    def __init__(self, fd):
        self.fd = fd
        self.buf = bytearray()
        self.odom = []          # (t, counts[4], error_flags)
        self.acks = 0
        self.crc_fail = 0
        self.valid = 0

    def _scan(self):
        i = 0
        while i < len(self.buf) - 1:
            if self.buf[i] == 0xA5 and self.buf[i + 1] == 0x5A:
                if len(self.buf) < i + 5:
                    break
                length = self.buf[i + 2]
                total = 7 + length
                if len(self.buf) < i + total:
                    break
                frame = bytes(self.buf[i:i + total])
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                if crc16_modbus(frame[2:-2]) == crc_recv:
                    self.valid += 1
                    cmd = frame[4]
                    if cmd == CMD_ODOM:
                        parsed = parse_odom(frame[5:5 + length])
                        if parsed:
                            self.odom.append((time.monotonic(),) + parsed)
                    elif cmd == CMD_ACK:
                        self.acks += 1
                else:
                    self.crc_fail += 1
                del self.buf[:i + total]
                i = 0
                continue
            i += 1

    def pump(self, duration=0.0):
        end = time.monotonic() + duration
        while True:
            now = time.monotonic()
            if duration and now >= end:
                break
            chunk = self.fd.read(256)
            if chunk:
                self.buf.extend(chunk)
                self._scan()
            if not duration:
                return
            time.sleep(0.005)

    def write(self, data):
        self.fd.write(data)


def odom_rate(link, window=2.0):
    if len(link.odom) < 2:
        return 0.0
    t0 = link.odom[-1][0] - window
    samples = [o for o in link.odom if o[0] >= t0]
    return len(samples) / window if samples else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--spin", type=float, default=0.0,
                    help="wheel target rad/s for the motion phase (0 = passive)")
    ap.add_argument("--spin-seconds", type=float, default=3.0)
    ap.add_argument("--tune-pid", action="store_true",
                    help="replace all wheel gains with the legacy Kp=100 Ki=300 test values")
    args = ap.parse_args()

    try:
        fd = open_serial(args.port, args.baud)
    except OSError as e:
        print(f"FAIL: cannot open {args.port}: {e}", file=sys.stderr)
        return 1
    link = Link(fd)
    seq = 0

    def heartbeat():
        nonlocal seq
        link.write(encode_frame(CMD_HEARTBEAT, b"", seq)); seq += 1

    print(f"drive_check on {args.port} @ {args.baud}")
    print("phase 1: passive 3s — expect ~50 Hz ODOM, ACKs, static encoders")
    t_end = time.monotonic() + 3.0
    next_hb = time.monotonic()
    while time.monotonic() < t_end:
        if time.monotonic() >= next_hb:
            heartbeat(); next_hb = time.monotonic() + 1.0
        link.pump(0.05)

    rate = odom_rate(link)
    first_counts = link.odom[0][1] if link.odom else None
    last_counts = link.odom[-1][1] if link.odom else None
    drift = tuple(abs(b - a) for a, b in
                  zip(first_counts, last_counts)) if first_counts else (999,) * 4
    print(f"  ODOM rate: {rate:.1f} Hz (expect ~50)")
    print(f"  ACKs: {link.acks} (3 heartbeats sent), CRC failures: {link.crc_fail}")
    print(f"  encoder drift while stationary: {drift}")

    ok = rate > 30 and link.acks >= 2 and link.crc_fail == 0 and max(drift) < 50
    if not ok:
        print("RESULT: PASSIVE CHECK FAIL")
        return 1

    if args.spin <= 0.0:
        print("\npassive mode done. Re-run with --spin RAD_S (chassis lifted!) "
              "for the motion phase.")
        print("RESULT: PASSIVE OK")
        return 0

    if args.tune_pid:
        print("\nphase 2: explicit legacy PI tune (Kp=100 Ki=300) on all 4 wheels")
        for motor in range(4):
            link.write(pid_frame(motor, 100.0, 300.0, 0.0, 1.0, seq)); seq += 1
        time.sleep(0.2)
        link.pump(0.2)
    else:
        print("\nphase 2: preserve active firmware PI gains")

    target = args.spin
    spin_s = args.spin_seconds
    print(f"phase 3: stream CMD_VEL_CTRL at 20 Hz, all wheels +{target} rad/s "
          f"for {spin_s}s (CHASSIS MUST BE LIFTED)")
    counts_before = link.odom[-1][1]
    t_start = time.monotonic()
    t_end = t_start + spin_s
    next_cmd = t_start
    while time.monotonic() < t_end:
        if time.monotonic() >= next_cmd:
            link.write(vel_frame(target, target, target, target, seq)); seq += 1
            next_cmd += 0.05
        link.pump(0.01)

    counts_after = link.odom[-1][1]
    t_after = link.odom[-1][0]
    actual_dt = t_after - t_start
    print(f"phase 4: zero targets for 2s, verify wheels stop")
    t_end = time.monotonic() + 2.0
    next_cmd = time.monotonic()
    while time.monotonic() < t_end:
        if time.monotonic() >= next_cmd:
            link.write(vel_frame(0, 0, 0, 0, seq)); seq += 1
            next_cmd += 0.05
        link.pump(0.02)
    counts_coast = link.odom[-1][1]

    print("phase 5: final zero command (normal, restartable stop)")
    link.write(vel_frame(0, 0, 0, 0, seq)); seq += 1
    link.pump(0.3)
    fd.close()

    print("\n=== results ===")
    print(f"{'wheel':6} {'edges':>8} {'measured rad/s':>15} {'target':>8} "
          f"{'coast edges':>12}")
    all_match = True
    for i, name in enumerate(WHEEL_NAMES):
        delta = counts_after[i] - counts_before[i]
        coast = counts_coast[i] - counts_after[i]
        measured = (delta / EDGES_PER_WHEEL_REV) * 6.2831853 / actual_dt
        ratio = measured / target if target else 0.0
        flag = "ok" if 0.5 < ratio < 1.5 and delta > 0 else "CHECK"
        if flag != "ok":
            all_match = False
        print(f"{name:6} {delta:8d} {measured:15.2f} {target:8.2f} "
              f"{coast:12d}  {flag}")
    print(f"\nCRC failures: {link.crc_fail}, error_flags last: "
          f"{link.odom[-1][2]:#04x}")

    if all_match and max(tuple(abs(counts_coast[i] - counts_after[i])
                               for i in range(4))) < 100:
        print("RESULT: DRIVE LOOP OK")
        return 0
    print("RESULT: DRIVE LOOP CHECK — inspect table above")
    return 1


if __name__ == "__main__":
    sys.exit(main())
