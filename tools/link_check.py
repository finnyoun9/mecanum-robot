#!/usr/bin/env python3
"""Real-hardware UART link check: Raspberry Pi 5 <-> STM32 USART1.

 Talks to the `uart_link_probe` firmware target (firmware/Core/HW/
 uart_link_probe_main.c): the probe streams CMD_ODOM_FEEDBACK frames at
 ~20 Hz (zeroed sensors, identity quaternion q.w = 1.0 as a transport
 marker) and replies CMD_ACK to every CMD_HEARTBEAT. It never configures
 any motor pin, so it is safe to run with the chassis on the ground.

 Stdlib only (termios) — run directly on the Pi host, not in the ROS2
 container:

     python3 tools/link_check.py                 # /dev/ttyAMA0 @ 921600
     python3 tools/link_check.py --duration 30

 Exit code 0 = link healthy (odom frames flowing + heartbeat acked),
 1 = no valid frames / no acks within the test window.
"""

import argparse
import struct
import sys
import termios
import time
from collections import deque

SYNC0, SYNC1 = 0xA5, 0x5A
MAX_PAYLOAD = 64
OVERHEAD = 7  # sync0 sync1 len seq cmd crc16

CMD_HEARTBEAT = 0x1F
CMD_ODOM = 0x20
CMD_ACK = 0x2E

# CRC-16/MODBUS, poly 0x8005 reflected (0xA001), init 0xFFFF — matches
# proto_crc16() in shared/protocol.c.
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc


def encode_frame(cmd: int, payload: bytes, seq: int) -> bytes:
    body = bytes([len(payload), seq & 0xFF, cmd]) + payload
    crc = crc16_modbus(body)
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc)


def open_serial(port: str, baud: int):
    fd = open(port, "r+b", buffering=0)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag: raw
    attrs[1] = 0  # oflag: raw
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1
    attrs[3] = 0  # lflag: non-canonical
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1  # 100 ms read timeout
    baud_const = getattr(termios, f"B{baud}")
    # attrs indices: 0 iflag, 1 oflag, 2 cflag, 3 lflag, 4 ispeed, 5 ospeed, 6 cc
    attrs[4] = baud_const
    attrs[5] = baud_const
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


class Parser:
    """Byte-at-a-time frame parser, mirroring parser_push() in the probe."""

    def __init__(self):
        self.state = "SYNC0"
        self.buf = bytearray()
        self.expected = 0
        self.valid = 0
        self.invalid = 0
        self.odom = 0
        self.acks = 0
        self.other = 0
        self.odom_times = deque(maxlen=200)
        self.last_odom_qw = None

    def push(self, b: int):
        if self.state == "SYNC0":
            if b == SYNC0:
                self.buf = bytearray([b])
                self.state = "SYNC1"
        elif self.state == "SYNC1":
            if b == SYNC1:
                self.buf.append(b)
                self.state = "HEADER"
            elif b == SYNC0:
                self.buf = bytearray([b])
            else:
                self.state = "SYNC0"
        elif self.state == "HEADER":
            self.buf.append(b)
            if len(self.buf) == 5:
                length = self.buf[2]
                if length > MAX_PAYLOAD:
                    self.invalid += 1
                    self.state = "SYNC0"
                else:
                    self.expected = OVERHEAD + length
                    self.state = "BODY"
        elif self.state == "BODY":
            self.buf.append(b)
            if len(self.buf) == self.expected:
                self._finish()
                self.state = "SYNC0"

    def _finish(self):
        frame = bytes(self.buf)
        payload_len = frame[2]
        crc_recv = struct.unpack("<H", frame[-2:])[0]
        if crc16_modbus(frame[2:-2]) != crc_recv:
            self.invalid += 1
            return
        self.valid += 1
        cmd = frame[4]
        payload = frame[5:5 + payload_len]
        if cmd == CMD_ODOM:
            self.odom += 1
            self.odom_times.append(time.monotonic())
            # odom_feedback_t: int32[4] + u16 + float q[4] ...
            if len(payload) >= 22:
                self.last_odom_qw = struct.unpack_from("<f", payload, 18)[0]
        elif cmd == CMD_ACK:
            self.acks += 1
        else:
            self.other += 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--duration", type=float, default=15.0,
                    help="test window in seconds (0 = until Ctrl-C)")
    args = ap.parse_args()

    try:
        fd = open_serial(args.port, args.baud)
    except OSError as e:
        print(f"FAIL: cannot open {args.port}: {e}", file=sys.stderr)
        print("(check: serial enabled in raspi-config, user in 'dialout', "
              "no other process holding the port)", file=sys.stderr)
        return 1

    print(f"link_check on {args.port} @ {args.baud} — "
          f"expecting ~20 Hz ODOM + ACK to 1 Hz heartbeat")
    parser = Parser()
    start = time.monotonic()
    next_hb = start
    seq = 0
    try:
        while True:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break
            if now >= next_hb:
                fd.write(encode_frame(CMD_HEARTBEAT, b"", seq))
                seq += 1
                next_hb = now + 1.0
            chunk = fd.read(256)
            for b in chunk:
                parser.push(b)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        fd.close()

    elapsed = time.monotonic() - start
    rate = parser.odom / elapsed if elapsed > 0 else 0.0
    print(f"\n--- {elapsed:.1f}s ---")
    print(f"valid frames : {parser.valid}  (CRC failures: {parser.invalid})")
    print(f"ODOM frames  : {parser.odom}  ({rate:.1f} Hz, expect ~20)")
    print(f"ACK frames   : {parser.acks}  (sent {seq} heartbeats)")
    print(f"other cmds   : {parser.other}")
    if parser.last_odom_qw is not None:
        print(f"odom q.w     : {parser.last_odom_qw:.3f} "
              f"(probe marker = 1.0)")

    ok = parser.odom > 0 and parser.acks > 0 and parser.invalid == 0
    if rate > 0 and not (15.0 <= rate <= 25.0):
        print("WARNING: odom rate outside expected 15-25 Hz band")
    print("RESULT: LINK OK" if ok else "RESULT: LINK FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
