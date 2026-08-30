#!/usr/bin/env python3
"""Live IMU watch: Raspberry Pi 5 <-> STM32 USART1, CMD_ODOM_FEEDBACK.

Passively reads the 50 Hz odom stream and prints the fused quaternion
(imu_q, w-x-y-z) and gyro (imu_gyro, rad/s) fields from odom_feedback_t
(shared/protocol.h). Sends CMD_HEARTBEAT at 1 Hz to keep the link alive;
never sends CMD_VEL_CTRL — safe to run with the chassis on the ground.

    python3 tools/imu_watch.py                 # /dev/ttyAMA0 @ 921600
    python3 tools/imu_watch.py --duration 10

Exit 0 = at least one ODOM frame with a non-identity-zero quaternion seen,
1 = no valid frames, or quaternion stayed all-zero for the whole window.
"""

import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import crc16_modbus, encode_frame, open_serial  # noqa: E402

CMD_HEARTBEAT = 0x1F
CMD_ODOM = 0x20

ERR_TOF_TIMEOUT = 0x04

ODOM_FMT = "<4iH4f3fBB"  # encoder_counts[4], tof_mm, imu_q[4], imu_gyro[3], battery_pct, error_flags
ODOM_LEN = struct.calcsize(ODOM_FMT)


def parse_odom(payload):
    if len(payload) < ODOM_LEN:
        return None
    (c0, c1, c2, c3, tof_mm, qw, qx, qy, qz, gx, gy, gz,
     battery_pct, error_flags) = struct.unpack_from(ODOM_FMT, payload, 0)
    return {
        "q": (qw, qx, qy, qz),
        "gyro": (gx, gy, gz),
        "battery_pct": battery_pct,
        "error_flags": error_flags,
    }


class Link:
    def __init__(self, fd):
        self.fd = fd
        self.buf = bytearray()
        self.frames = []
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
                            self.frames.append((time.monotonic(), parsed))
                else:
                    self.crc_fail += 1
                del self.buf[:i + total]
                i = 0
                continue
            i += 1

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            chunk = self.fd.read(256)
            if chunk:
                self.buf.extend(chunk)
                self._scan()
            time.sleep(0.005)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--duration", type=float, default=10.0,
                     help="test window in seconds (0 = until Ctrl-C)")
    ap.add_argument("--print-every", type=int, default=25,
                     help="print every Nth ODOM frame (default 25 -> ~2 Hz at 50 Hz stream)")
    args = ap.parse_args()

    try:
        fd = open_serial(args.port, args.baud)
    except OSError as e:
        print(f"FAIL: cannot open {args.port}: {e}", file=sys.stderr)
        return 1

    print(f"imu_watch on {args.port} @ {args.baud} — watching imu_q / imu_gyro")
    link = Link(fd)
    seq = 0
    start = time.monotonic()
    next_hb = start
    nonzero_seen = False
    count = 0
    try:
        while True:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break
            if now >= next_hb:
                fd.write(encode_frame(CMD_HEARTBEAT, b"", seq))
                seq += 1
                next_hb = now + 1.0
            link.pump(0.02)
            while link.frames:
                _, d = link.frames.pop(0)
                count += 1
                if any(v != 0.0 for v in d["q"][1:]) or d["q"][0] not in (0.0, 1.0):
                    nonzero_seen = True
                if count % args.print_every == 0:
                    qw, qx, qy, qz = d["q"]
                    gx, gy, gz = d["gyro"]
                    print(f"  q=({qw:+.3f},{qx:+.3f},{qy:+.3f},{qz:+.3f}) "
                          f"gyro=({gx:+.3f},{gy:+.3f},{gz:+.3f}) "
                          f"err={d['error_flags']:#04x} batt={d['battery_pct']}%")
    except KeyboardInterrupt:
        pass
    finally:
        fd.close()

    elapsed = time.monotonic() - start
    print(f"\n--- {elapsed:.1f}s, {count} ODOM frames, "
          f"{link.crc_fail} CRC failures ---")
    if count == 0:
        print("RESULT: NO ODOM FRAMES")
        return 1
    if not nonzero_seen:
        print("RESULT: IMU STUCK AT ZERO (quaternion never left all-zero/identity)")
        return 1
    print("RESULT: IMU LIVE (quaternion varying)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
