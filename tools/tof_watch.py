#!/usr/bin/env python3
"""Verify VL53L0X distance data carried in STM32 odometry frames.

Sends heartbeat only, so motors remain stopped. Move a flat target between
roughly 50 mm and 1500 mm in front of the sensor during the test.
"""

import argparse
import sys
import time

from link_check import CMD_HEARTBEAT, Parser, encode_frame, open_serial


ERR_TOF_TIMEOUT = 0x04


def main():
    args_parser = argparse.ArgumentParser(description=__doc__)
    args_parser.add_argument("--port", default="/dev/ttyAMA0")
    args_parser.add_argument("--baud", type=int, default=921600)
    args_parser.add_argument("--duration", type=float, default=15.0)
    args = args_parser.parse_args()

    try:
        serial = open_serial(args.port, args.baud)
    except OSError as exc:
        print(f"FAIL: cannot open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"ToF watch on {args.port} @ {args.baud}; move a target now")
    decoder = Parser()
    start = time.monotonic()
    next_heartbeat = start
    next_print = start
    sequence = 0
    seen_distances = set()
    try:
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_heartbeat:
                serial.write(encode_frame(CMD_HEARTBEAT, b"", sequence))
                sequence = (sequence + 1) & 0xFF
                next_heartbeat = now + 1.0

            for byte in serial.read(256):
                decoder.push(byte)
            if decoder.last_tof_mm is not None:
                if decoder.last_tof_mm > 0:
                    seen_distances.add(decoder.last_tof_mm)
                if now >= next_print:
                    print(f"distance={decoder.last_tof_mm:4d} mm  "
                          f"error_flags=0x{decoder.last_error_flags:02x}")
                    next_print = now + 0.5
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        serial.close()

    print(f"odom={decoder.odom}, crc_failures={decoder.invalid}, "
          f"distinct_nonzero_ranges={len(seen_distances)}")
    if decoder.odom == 0:
        print("RESULT: FAIL — no odometry frames received")
        return 1
    if decoder.invalid:
        print("RESULT: FAIL — corrupt serial frames received")
        return 1
    if decoder.last_error_flags is not None and \
            (decoder.last_error_flags & ERR_TOF_TIMEOUT):
        print("RESULT: FAIL — firmware reports TOF_TIMEOUT")
        return 1
    if not seen_distances:
        print("RESULT: FAIL — no non-zero VL53L0X range received")
        return 1
    print("RESULT: TOF DATA OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
