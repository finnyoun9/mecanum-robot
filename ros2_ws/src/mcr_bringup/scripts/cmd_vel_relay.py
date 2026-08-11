#!/usr/bin/env python3
"""
cmd_vel_relay.py — Forward /cmd_vel (Twist) to the mecanum_drive_controller's
reference interface.

The Jazzy mecanum_drive_controller (ros2_controllers 4.40.x) accepts velocity
commands only on /mecanum_drive_controller/reference (geometry_msgs/TwistStamped)
and has no /cmd_vel subscriber of its own. Nav2, teleop, and users all publish
geometry_msgs/Twist on /cmd_vel, so this tiny node bridges the two.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.pub = self.create_publisher(
            TwistStamped, 'mecanum_drive_controller/reference', 10)

    def _on_cmd_vel(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
