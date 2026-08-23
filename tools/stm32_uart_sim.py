#!/usr/bin/env python3
"""
stm32_uart_sim.py — Protocol-accurate STM32 UART simulator.

Speaks the exact binary protocol in shared/protocol.h over a pseudo-terminal
(pty), so the full ROS2 bringup stack can run end-to-end WITHOUT a real
STM32. The ROS2 side (SerialProtocol in mcr_bringup) opens the slave end as
a serial device; this script owns the master end and plays the firmware:

  * parses incoming frames (CMD_VEL_CTRL / CMD_EMERGENCY_STOP /
    CMD_HEARTBEAT / CMD_PID_TUNE)
  * applies a measured deadband and speed limit before integrating modeled
    wheel angular velocities into encoder counts
  * publishes CMD_ODOM_FEEDBACK at 50 Hz with a virtual IMU (yaw integrates
    the wheel-derived omega) so the robot_localization EKF can rotate the
    robot on angular commands
  * replies ACK to heartbeats

Usage:
  tools/stm32_uart_sim.py                # prints the pty path, e.g. /dev/pts/3
  ros2 launch mcr_bringup robot.launch.py serial_device:=/dev/pts/3
"""

import fcntl
import math
import os
import pty
import struct
import sys
import time

# --- Protocol constants (shared/protocol.h) ---
PROTO_SYNC0 = 0xA5
PROTO_SYNC1 = 0x5A
PROTO_MAX_PAYLOAD = 64

CMD_VEL_CTRL = 0x10
CMD_EMERGENCY_STOP = 0x11
CMD_PID_TUNE = 0x12
CMD_HEARTBEAT = 0x1F
CMD_ODOM_FEEDBACK = 0x20
CMD_ACK = 0x2E

# Firmware constants — must match firmware/Core/Inc/encoder.h.
# Measured 2026-08-23 by hand-turning a wheel 10 revolutions: 224 edges/rev
# at 1x, doubled to 448 when the firmware moved to both-edge decoding.
# (Previously 11 * 4 * 34 = 1496, derived from datasheet-typical constants
# that were wrong on both the gear ratio and the edge-counting scheme.)
EDGES_PER_WHEEL_REV = 448

ODOM_HZ = 50          # odometry publish rate
SIM_DT = 0.02         # sim step (20 ms)

# Chassis-lifted measurements from the 2026-08-23 duty sweep at the DP100
# bench-supply voltage, not battery-supply limits. The 20%-80% linear region
# extrapolates to 4.27 wheel rev/s at full duty. Re-measure these after moving
# to the 3S battery because motor speed scales approximately with supply
# voltage.
MAX_WHEEL_RPS = 4.27
MAX_WHEEL_RAD_S = MAX_WHEEL_RPS * 2.0 * math.pi

# 5% duty did not move any wheel; 10% did, at 0.32 wheel rev/s. The exact
# static-friction threshold was not measured between those points, so this is
# a conservative deadband derived from the lowest confirmed moving point, not
# a directly measured command-speed threshold.
MIN_START_WHEEL_RPS = 0.32
MIN_START_WHEEL_RAD_S = MIN_START_WHEEL_RPS * 2.0 * math.pi

# The outer wheel diameter was measured as 60 mm. Keep this consistent with
# mcr_hardware_interface.cpp instead of the former 0.0325 m estimate.
WHEEL_RADIUS_M = 0.030


# --- CRC-16/MODBUS (same table + algorithm as shared/protocol.c) ---
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


_CRC_TABLE = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ 0xA001 if (_c & 1) else (_c >> 1)
    _CRC_TABLE.append(_c)


def encode_frame(cmd: int, payload: bytes = b"", seq: int = 0) -> bytes:
    body = bytes([len(payload), seq & 0xFF, cmd]) + payload
    crc = crc16(body)
    return bytes([PROTO_SYNC0, PROTO_SYNC1]) + body + struct.pack("<H", crc)


class Stm32Sim:
    def __init__(self, master_fd: int):
        self.master = master_fd
        self.rx_buf = b""
        self.commanded_w = [0.0, 0.0, 0.0, 0.0]  # requested wheel speed (rad/s)
        self.counts = [0, 0, 0, 0]            # accumulated encoder edges
        self.count_residual = [0.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0                        # virtual IMU heading (rad)
        self.gyro_z = 0.0
        self.seq_tx = 0
        self.stopped = False
        self.last_t = time.monotonic()

    # ---- RX: parse + handle commands ---------------------------------
    def feed(self, data: bytes):
        self.rx_buf += data
        while True:
            frame_len = self.try_parse()
            if frame_len is None:
                break
            self.rx_buf = self.rx_buf[frame_len:]

    def try_parse(self):
        """Try to decode one frame at the front of rx_buf. Returns its total
        length, or None if incomplete/no valid frame at head."""
        buf = self.rx_buf
        if len(buf) < 7:
            return None
        if buf[0] != PROTO_SYNC0 or buf[1] != PROTO_SYNC1:
            # scan for sync
            i = buf.find(bytes([PROTO_SYNC0, PROTO_SYNC1]), 1)
            if i == -1:
                self.rx_buf = b""
                return None
            self.rx_buf = buf[i:]
            return None
        pay_len = buf[2]
        if pay_len > PROTO_MAX_PAYLOAD:
            self.rx_buf = b""
            return None
        total = 7 + pay_len
        if len(buf) < total:
            return None
        # CRC over [LEN..PAYLOAD_END] i.e. bytes 2 .. 2+3+pay_len
        crc_calc = crc16(buf[2:5 + pay_len])
        crc_recv = struct.unpack("<H", buf[5 + pay_len:7 + pay_len])[0]
        if crc_calc != crc_recv:
            self.rx_buf = buf[1:]  # drop one byte, resync
            return None
        seq, cmd = buf[3], buf[4]
        payload = buf[5:5 + pay_len]
        self.handle(cmd, payload, seq)
        return total

    def handle(self, cmd: int, payload: bytes, seq: int):
        if cmd == CMD_VEL_CTRL and len(payload) == 16:
            self.commanded_w = list(struct.unpack("<4f", payload))
            self.stopped = False
            print("[sim] VEL seq=%d w=[%.3f %.3f %.3f %.3f]" %
                  (seq, *self.commanded_w), flush=True)
        elif cmd == CMD_EMERGENCY_STOP:
            self.commanded_w = [0.0] * 4
            self.stopped = True
        elif cmd == CMD_HEARTBEAT:
            self._send(CMD_ACK, b"")
        elif cmd == CMD_PID_TUNE:
            pass  # gains accepted silently

    # ---- TX: odometry + ack ------------------------------------------
    def _send(self, cmd: int, payload: bytes):
        frame = encode_frame(cmd, payload, self.seq_tx)
        self.seq_tx = (self.seq_tx + 1) & 0xFF
        try:
            os.write(self.master, frame)
        except OSError:
            pass

    @staticmethod
    def modeled_wheel_speed(command_w: float) -> float:
        """Map a requested wheel speed to the measured bench-supply envelope.

        This is intentionally a static model: no time constant was measured,
        so adding one would be an unsupported estimate.
        """
        magnitude = abs(command_w)
        if magnitude < MIN_START_WHEEL_RAD_S:
            return 0.0
        return math.copysign(min(magnitude, MAX_WHEEL_RAD_S), command_w)

    def send_odometry(self):
        # Integrate modeled wheel speeds -> encoder edges. Keep fractional
        # edges so the 50 Hz simulation does not introduce a low-speed bias.
        modeled_w = [self.modeled_wheel_speed(w) for w in self.commanded_w]
        for i in range(4):
            edges = (modeled_w[i] / (2.0 * math.pi) *
                     EDGES_PER_WHEEL_REV * SIM_DT + self.count_residual[i])
            delta = math.trunc(edges)
            self.count_residual[i] = edges - delta
            self.counts[i] += delta

        # Wheel-derived twist (mecanum forward kin). lx/ly remain estimated;
        # see mcr_hardware_interface.cpp and the closed-loop roadmap.
        r, l = WHEEL_RADIUS_M, 0.10 + 0.12
        w1, w2, w3, w4 = modeled_w
        omega = (-w1 + w2 - w3 + w4) * r / (4.0 * l)
        vx = (w1 + w2 + w3 + w4) * r * 0.25

        # Virtual IMU: yaw integrates the (wheel-derived) omega; gyro_z = omega
        self.yaw += omega * SIM_DT
        self.gyro_z = omega

        q = _quat_from_yaw(self.yaw)
        payload = struct.pack(
            "<4iH4f3fBB",
            *self.counts,          # encoder_counts[4]
            250,                   # tof_distance_mm (25 cm, no obstacle)
            *q,                    # imu_q[4] w,x,y,z
            0.0, 0.0, self.gyro_z,  # imu_gyro[3]
            100,                   # battery_pct
            0,                     # error_flags
        )
        self._send(CMD_ODOM_FEEDBACK, payload)

    def tick(self):
        now = time.monotonic()
        dt = now - self.last_t
        if dt >= SIM_DT:
            self.last_t = now
            self.send_odometry()


def _quat_from_yaw(yaw: float):
    """ZYX -> quaternion (w,x,y,z) for pure yaw rotation."""
    cy = _cospi(yaw / 2.0)
    sy = _sinpi(yaw / 2.0)
    return (cy, 0.0, 0.0, sy)


def _cospi(a):
    return _cos(a * 3.141592653589793)


def _sinpi(a):
    return _sin(a * 3.141592653589793)


def _cos(a):
    x = a % (2.0 * 3.141592653589793)
    # Taylor-ish via math module for clarity
    import math
    return math.cos(x)


def _sin(a):
    import math
    return math.sin(a % (2.0 * 3.141592653589793))


def main():
    import argparse
    ap = argparse.ArgumentParser(description="STM32 UART simulator (protocol-accurate)")
    ap.add_argument("--pty-file", default=None,
                    help="also write the pty slave path to this file (useful for scripting)")
    args = ap.parse_args()

    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    os.close(slave)
    print(f"[sim] STM32 sim on {slave_path}")
    print(f"[sim] launch with: serial_device:={slave_path}")
    sys.stdout.flush()
    if args.pty_file:
        with open(args.pty_file, "w") as f:
            f.write(slave_path + "\n")

    sim = Stm32Sim(master)
    fcntl.fcntl(master, fcntl.F_SETFL, os.O_NONBLOCK)
    print("[sim] running (Ctrl-C to stop)", flush=True)

    try:
        while True:
            # Non-blocking RX drain. Reading a pty master returns b'' when the
            # slave end is not open (e.g. bringup hasn't connected yet), so
            # treat empty reads as "no traffic" rather than an error.
            try:
                while True:
                    data = os.read(master, 512)
                    if not data:
                        break
                    sim.feed(data)
            except BlockingIOError:
                pass
            except OSError:
                pass
            sim.tick()
            time.sleep(SIM_DT)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[sim] stopped", flush=True)


if __name__ == "__main__":
    main()
