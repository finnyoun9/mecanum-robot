#!/usr/bin/env python3
"""
陀螺仪零偏 (gyro bias) 测量 —— M5 地图质量优化的第一步。

背景：
  SIL 测试证明 slam_toolbox 参数和 TF 链是健康的（地图不扭曲、
  map->odom 修正仅 5.6cm/0.21°）。真车上出现地图扭曲，主要嫌疑是
  IMU 零偏：车静止时陀螺仪 z 轴仍输出非零角速度，EKF 把它积分成
  持续转动，导致建图时墙线被"掰弯"。

原理：
  让车完全静止，采集 N 秒 /imu/data 的 angular_velocity，
  取均值即为零偏（真值应为 0），标准差反映噪声水平。

判读标准（MPU6050/ICM 类消费级 IMU）：
  |bias_z| < 0.002 rad/s (0.11°/s)  → 良好，不必补偿
  0.002 ~ 0.01 rad/s               → 需要补偿，10 分钟累积 1.1~5.7°
  > 0.01 rad/s (0.57°/s)           → 严重，必须补偿，建图必然扭曲

用法（车必须完全静止！）：
  python3 tools/gyro_bias_check.py                # 默认 60s
  python3 tools/gyro_bias_check.py --duration 120 # 采 120s 更准
  python3 tools/gyro_bias_check.py --topic /imu/data_raw
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class GyroBiasCheck(Node):
    def __init__(self, topic, duration):
        super().__init__('gyro_bias_check')
        self.duration = duration
        self.samples = {'x': [], 'y': [], 'z': []}
        self.accel = {'x': [], 'y': [], 'z': []}
        self.t_start = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.sub = self.create_subscription(Imu, topic, self.cb, qos)
        self.get_logger().info(
            f'采集 {topic} {duration}s —— 请确保小车完全静止、无人触碰、地面无震动'
        )
        self.timer = self.create_timer(5.0, self.progress)

    def cb(self, msg):
        if self.t_start is None:
            self.t_start = self.get_clock().now()
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.samples['x'].append(g.x)
        self.samples['y'].append(g.y)
        self.samples['z'].append(g.z)
        self.accel['x'].append(a.x)
        self.accel['y'].append(a.y)
        self.accel['z'].append(a.z)

    def progress(self):
        n = len(self.samples['z'])
        if n == 0:
            self.get_logger().warn('还没收到 IMU 数据，检查话题名和节点是否在跑')
            return
        elapsed = (self.get_clock().now() - self.t_start).nanoseconds / 1e9
        self.get_logger().info(f'  已采 {n} 帧 / {elapsed:.0f}s')
        if elapsed >= self.duration:
            raise KeyboardInterrupt

    def report(self):
        n = len(self.samples['z'])
        if n < 10:
            print(f'\n❌ 样本太少 ({n} 帧)，无法评估。检查 IMU 话题是否发布。')
            return 1

        def stats(vals):
            m = sum(vals) / len(vals)
            var = sum((v - m) ** 2 for v in vals) / len(vals)
            return m, math.sqrt(var)

        print('\n' + '=' * 62)
        print(f'陀螺仪零偏报告  (样本 {n} 帧)')
        print('=' * 62)

        print('\n【角速度零偏】理想值 0，静止时的均值就是零偏')
        worst = 0.0
        for ax in ('x', 'y', 'z'):
            m, sd = stats(self.samples[ax])
            deg = math.degrees(m)
            print(f'  gyro_{ax}: bias = {m:+.6f} rad/s ({deg:+.4f} °/s)'
                  f'   噪声 σ = {sd:.6f}')
            worst = max(worst, abs(m))

        mz, sdz = stats(self.samples['z'])
        print('\n【z 轴影响推算】z 轴决定航向，直接影响建图')
        for t, label in ((60, '1 分钟'), (600, '10 分钟')):
            print(f'  {label}静止累积航向漂移: {math.degrees(abs(mz) * t):.2f}°')

        print('\n【加速度计】静止时应只有重力 ~9.81 m/s² 在 z 轴')
        for ax in ('x', 'y', 'z'):
            m, sd = stats(self.accel[ax])
            print(f'  accel_{ax}: {m:+.4f} m/s²   σ = {sd:.4f}')
        gmag = math.sqrt(sum(
            (sum(self.accel[a]) / n) ** 2 for a in ('x', 'y', 'z')
        ))
        print(f'  合成重力模长: {gmag:.4f} m/s²  (标准 9.807, 偏差 '
              f'{abs(gmag - 9.807) / 9.807 * 100:.2f}%)')

        print('\n' + '=' * 62)
        if abs(mz) < 0.002:
            print('✅ 判定：z 轴零偏良好 (< 0.002 rad/s)，不是地图扭曲的主因。')
            print('   → 下一步查 EKF 融合配置 / 轮式里程计标定 / scan 时间戳同步')
            rc = 0
        elif abs(mz) < 0.01:
            print('⚠️  判定：z 轴零偏偏大，需要补偿。')
            print(f'   → 建议在固件或 EKF 前置节点里减去 {mz:+.6f} rad/s')
            print('   → 或启用 robot_localization 的 imu bias 估计')
            rc = 0
        else:
            print('❌ 判定：z 轴零偏严重，建图必然扭曲！')
            print(f'   → 必须补偿 {mz:+.6f} rad/s ({math.degrees(mz):+.3f} °/s)')
            print('   → 优先在 STM32 固件上电时做静止零偏标定')
            rc = 1
        print('=' * 62)

        print('\n【建议的补偿常量】(贴进固件或 EKF 前置节点)')
        for ax in ('x', 'y', 'z'):
            m, _ = stats(self.samples[ax])
            print(f'  GYRO_BIAS_{ax.upper()} = {m:+.6f}f;  // rad/s')
        return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/imu/data',
                    help='IMU 话题 (默认 /imu/data)')
    ap.add_argument('--duration', type=float, default=60.0,
                    help='采集秒数 (默认 60)')
    args = ap.parse_args()

    rclpy.init()
    node = GyroBiasCheck(args.topic, args.duration)
    rc = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rc = node.report()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
