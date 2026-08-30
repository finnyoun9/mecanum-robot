#!/usr/bin/env python3
"""Release a latched stall fault (ERR_MOTOR_FAULT) without a reflash/reboot.

Sends CMD_CLEAR_MOTOR_FAULT once and reports error_flags before/after by
reading a few ODOM frames off the passive link. Only clears a stall
latch -- inspect the wheel that stalled first (per robot_clear_motor_fault()'s
own contract: this exists for after the mechanical/electrical cause has
actually been dealt with, not as a blind unstick button). Does not touch
K9/ToF/comm-watchdog stops -- those still need their own recovery path
(ToF auto-clears on distance; K9/watchdog need a fresh motion command).

    python3 tools/clear_motor_fault.py
"""

import argparse
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from link_check import (  # noqa: E402
    CMD_HEARTBEAT, Parser, encode_frame, open_serial,
)

CMD_CLEAR_MOTOR_FAULT = 0x13


def read_error_flags(fd, seq, window=1.0):
    parser = Parser()
    end = time.monotonic() + window
    next_hb = time.monotonic()
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_hb:
            fd.write(encode_frame(CMD_HEARTBEAT, b"", seq))
            seq += 1
            next_hb = now + 0.5
        chunk = fd.read(256)
        for b in chunk:
            parser.push(b)
        time.sleep(0.01)
    return parser.last_error_flags, seq


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    seq = 0

    before, seq = read_error_flags(fd, seq)
    print(f"error_flags before: {before:#04x}" if before is not None
          else "error_flags before: (no ODOM frame received)")

    fd.write(encode_frame(CMD_CLEAR_MOTOR_FAULT, b"", seq))
    seq += 1
    print("sent CMD_CLEAR_MOTOR_FAULT")
    time.sleep(0.2)

    after, seq = read_error_flags(fd, seq)
    fd.close()
    print(f"error_flags after:  {after:#04x}" if after is not None
          else "error_flags after:  (no ODOM frame received)")

    if after == 0x00:
        print("RESULT: CLEARED")
        return 0
    print("RESULT: STILL FAULTED — inspect the wheel before retrying")
    return 1


if __name__ == "__main__":
    sys.exit(main())
