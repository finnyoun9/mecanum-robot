#!/usr/bin/env python3
"""Verify MPU6050 data carried in STM32 odometry frames.

Sends heartbeat only, so the chassis remains stopped. Rotate or tilt the IMU
during the test: a healthy sensor produces a unit quaternion that changes and
non-zero angular velocity. An uninitialised/failed MPU6050 stays at the
firmware defaults (identity quaternion and exactly-zero gyro).

Run on the Pi host:
    python3 tools/imu_watch.py --duration 15
"""

import argparse
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import crc16_modbus, encode_frame, open_serial


CMD_HEARTBEAT = 0x1F
CMD_ODOM = 0x20
ODOM_SIZE = 48


def quaternion_to_euler(q):
    """Return roll, pitch, yaw in degrees for a (w, x, y, z) quaternion."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class OdomParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_failures = 0

    def feed(self, data):
        self.buffer.extend(data)
        samples = []
        while len(self.buffer) >= 5:
            sync = self.buffer.find(b"\xA5\x5A")
            if sync < 0:
                del self.buffer[:-1]
                break
            if sync:
                del self.buffer[:sync]
            if len(self.buffer) < 5:
                break
            payload_len = self.buffer[2]
            frame_len = payload_len + 7
            if len(self.buffer) < frame_len:
                break
            frame = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]
            if crc16_modbus(frame[2:-2]) != struct.unpack("<H", frame[-2:])[0]:
                self.crc_failures += 1
                continue
            if frame[4] != CMD_ODOM or payload_len < ODOM_SIZE:
                continue
            payload = frame[5:5 + payload_len]
            q = struct.unpack_from("<4f", payload, 18)
            gyro = struct.unpack_from("<3f", payload, 34)
            samples.append((q, gyro, payload[47]))
        return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()

    try:
        serial = open_serial(args.port, args.baud)
    except OSError as exc:
        print(f"FAIL: cannot open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"IMU watch on {args.port} @ {args.baud}; rotate/tilt the sensor now")
    decoder = OdomParser()
    start = time.monotonic()
    next_heartbeat = start
    next_print = start
    seq = 0
    sample_count = 0
    motion_seen = False
    norm_ok = True
    last_sample = None

    try:
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_heartbeat:
                serial.write(encode_frame(CMD_HEARTBEAT, b"", seq))
                seq = (seq + 1) & 0xFF
                next_heartbeat = now + 1.0

            for q, gyro, errors in decoder.feed(serial.read(256)):
                sample_count += 1
                last_sample = (q, gyro, errors)
                q_norm = math.sqrt(sum(value * value for value in q))
                norm_ok = norm_ok and math.isfinite(q_norm) and 0.95 <= q_norm <= 1.05
                if max(abs(value) for value in gyro) > 1e-5 or \
                        max(abs(q[i] - (1.0 if i == 0 else 0.0)) for i in range(4)) > 1e-5:
                    motion_seen = True

            if last_sample is not None and now >= next_print:
                q, gyro, errors = last_sample
                roll, pitch, yaw = quaternion_to_euler(q)
                print(f"rpy {roll:+7.1f} {pitch:+7.1f} {yaw:+7.1f} deg  "
                      f"gyro {gyro[0]:+7.3f} {gyro[1]:+7.3f} {gyro[2]:+7.3f} rad/s  "
                      f"err=0x{errors:02x}")
                next_print = now + 0.5
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        serial.close()

    print(f"samples={sample_count}, crc_failures={decoder.crc_failures}")
    if sample_count == 0:
        print("RESULT: FAIL — no odometry frames received")
        return 1
    if decoder.crc_failures:
        print("RESULT: FAIL — corrupt serial frames received")
        return 1
    if not norm_ok:
        print("RESULT: FAIL — quaternion norm left the 0.95..1.05 range")
        return 1
    if not motion_seen:
        print("RESULT: FAIL — IMU stayed at identity quaternion and zero gyro")
        return 1
    print("RESULT: IMU DATA OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
