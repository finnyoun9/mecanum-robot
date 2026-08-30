from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mcr_perception",
            executable="udp_detection_bridge",
            name="udp_detection_bridge",
            output="screen",
        ),
    ])
