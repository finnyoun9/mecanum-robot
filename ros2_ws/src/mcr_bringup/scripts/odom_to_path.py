#!/usr/bin/env python3
"""
odom_to_path.py — Publish a nav_msgs/Path trail from an Odometry topic.

RViz2's Path display only accepts nav_msgs/Path, so this node bridges an
Odometry topic (e.g. /odometry/filtered from robot_localization) into a Path
of accumulated poses for visualizing the robot's traveled trajectory.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


MAX_POSES = 2000  # ~40s of trail at the 50 Hz /odometry/filtered rate


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.sub = self.create_subscription(
            Odometry, 'odometry/filtered', self._on_odom, 10)
        self.pub = self.create_publisher(Path, 'path', 10)
        self.path = Path()

    def _on_odom(self, odom: Odometry):
        p = PoseStamped()
        p.header = odom.header
        p.pose = odom.pose.pose
        self.path.header = odom.header
        self.path.poses.append(p)
        # poses grows without bound otherwise, and every publish re-sends
        # the whole list -- at 50 Hz that's O(session length) work per
        # callback and eventually pins a CPU core (measured 87% after
        # ~20 min). A rolling window is plenty for a visualized trail.
        if len(self.path.poses) > MAX_POSES:
            del self.path.poses[:-MAX_POSES]
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
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
