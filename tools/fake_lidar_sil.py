#!/usr/bin/env python3
"""
Fake LD06 LiDAR for SIL — 让 slam_toolbox 在没有真实雷达时也能建图。

原理：假设机器人在一个 6m x 4m 的矩形房间里（含一根中央柱子），
根据 TF (odom->base_link) 拿到当前位姿，用射线求交算出每束激光的距离，
发布成 /scan (sensor_msgs/LaserScan)，规格对齐 LD06：
  - 360 束 / 圈, 角分辨率 1°
  - 测距 0.02 ~ 12 m
  - 10 Hz

这样 SLAM 链路 (scan + odom -> map) 就完整了，可以验证：
  * TF 树是否正确 (map->odom->base_link->laser)
  * slam_toolbox 参数是否合理
  * 里程计漂移对建图质量的影响（正是 M5 要解决的地图扭曲问题）

用法（容器内）：
  python3 /tools/fake_lidar_sil.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


# ---- 房间几何：线段列表 [(x1,y1,x2,y2), ...] 单位 m，odom 坐标系 ----
# 机器人从 (0,0) 出发，房间以起点为中心偏置一点，避免贴墙起步
ROOM_W = 6.0
ROOM_H = 4.0
X0, Y0 = -1.5, -2.0          # 房间左下角
WALLS = [
    (X0,           Y0,           X0 + ROOM_W, Y0),            # 下墙
    (X0 + ROOM_W,  Y0,           X0 + ROOM_W, Y0 + ROOM_H),   # 右墙
    (X0 + ROOM_W,  Y0 + ROOM_H,  X0,          Y0 + ROOM_H),   # 上墙
    (X0,           Y0 + ROOM_H,  X0,          Y0),            # 左墙
    # 中央柱子（0.4m 见方），给 SLAM 一个明显的回环特征
    (1.3, -0.2, 1.7, -0.2),
    (1.7, -0.2, 1.7,  0.2),
    (1.7,  0.2, 1.3,  0.2),
    (1.3,  0.2, 1.3, -0.2),
]

ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
N_BEAMS = 360
RANGE_MIN = 0.02
RANGE_MAX = 12.0
RATE_HZ = 10.0
NOISE_SIGMA = 0.008          # 8mm 测距噪声，接近 LD06 实际水平


def ray_segment_dist(px, py, dx, dy, x1, y1, x2, y2):
    """射线 (px,py)+t*(dx,dy) 与线段 (x1,y1)-(x2,y2) 求交，返回 t>0 或 None。"""
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return None
    # 解 t (射线参数) 和 u (线段参数, 0..1)
    t = ((x1 - px) * sy - (y1 - py) * sx) / denom
    u = ((x1 - px) * dy - (y1 - py) * dx) / denom
    if t > 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar_sil')

        self.declare_parameter('frame_id', 'laser_link')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('noise', NOISE_SIGMA)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.noise = float(self.get_parameter('noise').value)

        # 传感器数据用 BEST_EFFORT，和真实雷达驱动一致
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(LaserScan, 'scan', qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.angles = [
            ANGLE_MIN + i * (ANGLE_MAX - ANGLE_MIN) / N_BEAMS
            for i in range(N_BEAMS)
        ]
        self._warned = False
        self._rng_state = 12345

        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(
            f'fake LD06 up: {ROOM_W}x{ROOM_H}m room + center pillar, '
            f'{N_BEAMS} beams @ {RATE_HZ}Hz -> /scan'
        )

    def _noise(self):
        """轻量 LCG 随机数，避免依赖 numpy。返回近似正态噪声。"""
        s = 0.0
        for _ in range(3):
            self._rng_state = (1103515245 * self._rng_state + 12345) & 0x7FFFFFFF
            s += self._rng_state / 0x7FFFFFFF - 0.5
        return s * self.noise * 2.0

    def get_pose(self):
        """从 TF 拿 odom->base_link，返回 (x, y, yaw)。拿不到返回 None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # 只取 yaw
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return t.x, t.y, yaw

    def tick(self):
        pose = self.get_pose()
        if pose is None:
            if not self._warned:
                self.get_logger().warn(
                    f'waiting for TF {self.odom_frame}->{self.base_frame} '
                    '(EKF/odom not up yet?)'
                )
                self._warned = True
            return
        if self._warned:
            self.get_logger().info('TF acquired, publishing scans')
            self._warned = False

        px, py, yaw = pose
        ranges = []
        for a in self.angles:
            wa = yaw + a                      # 世界坐标系下的射线方向
            dx, dy = math.cos(wa), math.sin(wa)
            best = float('inf')
            for (x1, y1, x2, y2) in WALLS:
                t = ray_segment_dist(px, py, dx, dy, x1, y1, x2, y2)
                if t is not None and t < best:
                    best = t
            if best == float('inf') or best > RANGE_MAX:
                ranges.append(float('inf'))   # 无回波
            else:
                r = best + self._noise()
                ranges.append(max(RANGE_MIN, r))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = (ANGLE_MAX - ANGLE_MIN) / N_BEAMS
        msg.time_increment = 1.0 / RATE_HZ / N_BEAMS
        msg.scan_time = 1.0 / RATE_HZ
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
