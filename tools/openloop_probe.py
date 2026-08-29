#!/usr/bin/env python3
"""Open-loop channel-strength probe — no encoder/channel pairing needed.

Why not the closed loop: the PI on index i drives channel i but reads
encoder i. After swapping only the motor output wires that pairing is
broken, so a closed-loop run would see measurement ~0, wind the integral to
integral_max and slam the channel to full duty. Useless as a comparison and
rough on the hardware.

Instead we make the loop open by construction: Ki=0 (no integral, so no
windup) and a fixed Kp, with the setpoint chosen so Kp*setpoint saturates
the output clamp. Every driven channel then gets the SAME duty regardless
of what its encoder reports, and we compare wheels by the encoder that is
physically on each motor.

Channels not under test get Kp=Ki=0, which pins their output to 0.

  python3 openloop_probe.py --drive 2 --duty 0.5   # A channel (RL slot)
  python3 openloop_probe.py --drive 3 --duty 0.5   # B channel (RR slot)
"""
import argparse, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_check import encode_frame, open_serial
from drive_check import (Link, vel_frame, pid_frame, WHEEL_NAMES,
                         EDGES_PER_WHEEL_REV, CMD_HEARTBEAT,
                         CMD_EMERGENCY_STOP, odom_rate)

PID_OUT_MAX = 1000.0   # firmware robot_control.h PID_OUT_MAX

ap = argparse.ArgumentParser()
ap.add_argument("--drive", type=int, required=True,
                help="channel index to drive: 0=FL 1=FR 2=RL-slot 3=RR-slot")
ap.add_argument("--duty", type=float, default=0.5,
                help="fraction of full duty, 0..1")
ap.add_argument("--seconds", type=float, default=3.0)
ap.add_argument("--port", default="/dev/ttyAMA0")
ap.add_argument("--baud", type=int, default=921600)
a = ap.parse_args()

if not 0 <= a.drive < 4:
    sys.exit("--drive must be 0..3")
if not 0.0 < a.duty <= 1.0:
    sys.exit("--duty must be in (0, 1]")

# Kp chosen so Kp * setpoint = duty * PID_OUT_MAX exactly, with Ki=Kd=0.
# The measurement subtracts from error, so use a setpoint far above any
# reachable speed to keep the term dominated by the setpoint: at sp=1000
# rad/s a real 6 rad/s reading perturbs the duty by 0.6%.
SETPOINT = 1000.0
kp = (a.duty * PID_OUT_MAX) / SETPOINT

fd = open_serial(a.port, a.baud)
link = Link(fd)
seq = 0
def send(f):
    global seq
    link.write(f); seq += 1

print(f"open-loop probe: channel {a.drive} ({WHEEL_NAMES[a.drive]} slot) "
      f"at {a.duty*100:.0f}% duty for {a.seconds}s")
print(f"  Kp={kp:.4f} Ki=0 setpoint={SETPOINT} -> duty {a.duty*PID_OUT_MAX:.0f}/1000")

send(encode_frame(CMD_HEARTBEAT, b"", seq))
t_end = time.monotonic() + 1.5
while time.monotonic() < t_end:
    link.pump(0.05)
if not link.odom:
    sys.exit("FAIL: no odom — link down?")
print(f"  odom {odom_rate(link):.1f} Hz, crc_fail={link.crc_fail}")

# Only the channel under test gets gain; the rest are pinned to zero output.
for m in range(4):
    if m == a.drive:
        send(pid_frame(m, kp, 0.0, 0.0, 0.0, seq))
    else:
        send(pid_frame(m, 0.0, 0.0, 0.0, 0.0, seq))
time.sleep(0.2); link.pump(0.2)

targets = [SETPOINT if i == a.drive else 0.0 for i in range(4)]
before = link.odom[-1][1]
t0 = time.monotonic(); t_end = t0 + a.seconds; nxt = t0
while time.monotonic() < t_end:
    if time.monotonic() >= nxt:
        send(vel_frame(*targets, seq)); nxt += 0.05
    link.pump(0.01)
after, t_after = link.odom[-1][1], link.odom[-1][0]
dt = t_after - t0

t_end = time.monotonic() + 1.5
nxt = time.monotonic()
while time.monotonic() < t_end:
    if time.monotonic() >= nxt:
        send(vel_frame(0, 0, 0, 0, seq)); nxt += 0.05
    link.pump(0.02)
send(encode_frame(CMD_EMERGENCY_STOP, b"", seq))
link.pump(0.3)
fd.close()

print(f"\n{'encoder':9}{'edges':>8}{'rad/s':>9}   note")
for i, n in enumerate(WHEEL_NAMES):
    d = after[i] - before[i]
    meas = (d / EDGES_PER_WHEEL_REV) * 6.2831853 / dt
    tag = "<-- encoder on the motor being driven?" if abs(d) > 20 else ""
    print(f"{n:9}{d:8d}{meas:9.2f}   {tag}")
print(f"\ncrc_fail={link.crc_fail}, error_flags={link.odom[-1][2]:#04x}, dt={dt:.2f}s")
print("NOTE: read by ENCODER position. After a motor-wire swap the moving "
      "wheel's encoder is the one that counts, not the channel index.")
