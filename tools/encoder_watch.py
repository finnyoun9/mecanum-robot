#!/usr/bin/env python3
"""Hand-turn encoder check: watch odometry counts live.

Streams heartbeats only (no motion commands -> motors stay coasted),
prints per-wheel encoder deltas once a second. Turn each wheel by hand
when asked; every turned wheel should show a steadily changing count
(positive for forward), dead encoder wiring stays 0.

Stdlib only; run on the Pi host.
"""
import os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import crc16_modbus, encode_frame, open_serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
fd = open_serial(PORT, 921600)
buf = bytearray()
counts = [0, 0, 0, 0]
seq = 0
names = ("FL", "FR", "RL", "RR")

def scan():
    global buf
    i = 0
    while i < len(buf) - 1:
        if buf[i] == 0xA5 and buf[i+1] == 0x5A and len(buf) >= i + 5:
            length = buf[i+2]
            total = 7 + length
            if len(buf) < i + total:
                break
            frame = bytes(buf[i:i+total])
            crc = struct.unpack("<H", frame[-2:])[0]
            if crc16_modbus(frame[2:-2]) == crc and frame[4] == 0x20 and length >= 18:
                c = struct.unpack_from("<4i", frame, 5)
                for j in range(4):
                    counts[j] = c[j]
            del buf[:i+total]
            i = 0
            continue
        i += 1

print("encoder watch: turn each wheel slowly FORWARD when asked (Ctrl-C to stop)")
start = time.monotonic()
last = time.monotonic()
prev = [0,0,0,0]
phase = 0
labels = ["turn FL now", "turn FR now", "turn RL now", "turn RR now",
          "turn ALL four forward together"]
phase_end = start + 8
while True:
    now = time.monotonic()
    if now >= phase_end and phase < len(labels):
        print(f"\n>>> {labels[phase]} <<<")
        phase += 1
        phase_end = now + 8
    if now - last >= 1.0:
        d = [counts[j]-prev[j] for j in range(4)]
        print(f"  FL {d[0]:+6d}  FR {d[1]:+6d}  RL {d[2]:+6d}  RR {d[3]:+6d}   "
              f"(total: {counts[0]} {counts[1]} {counts[2]} {counts[3]})")
        prev = list(counts)
        last = now
    fd.write(encode_frame(0x1F, b"", seq)); seq += 1
    chunk = fd.read(256)
    if chunk:
        buf.extend(chunk)
        scan()
    time.sleep(0.02)
