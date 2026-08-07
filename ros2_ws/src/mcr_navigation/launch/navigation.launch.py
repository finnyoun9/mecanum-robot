"""
navigation.launch.py — Launch SLAM + Nav2 for the Mecanum Robot.

Modes:
  - SLAM mode:    slam:=true  (default) — run slam_toolbox for mapping
  - Localize mode: slam:=false map:=/path/to/map.yaml — AMCL with saved map

Usage:
  ros2 launch mcr_navigation navigation.launch.py
  ros2 launch mcr_navigation navigation.launch.py slam:=false map:=./maps/my_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav  = get_package_share_directory('mcr_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # --- Launch args ---
    slam_arg = DeclareLaunchArgument('slam', default_value='true',
        description='Run SLAM (true) or localization mode (false)')

    map_arg = DeclareLaunchArgument('map', default_value='',
        description='Path to map YAML for localization mode')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # --- SLAM Toolbox ---
    slam_toolbox_node = Node(
        condition=IfCondition(LaunchConfiguration('slam')),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_nav, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # --- Nav2 Bringup ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map':  LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
            'autostart': 'true',
        }.items(),
    )

    # --- RViz2 ---
    rviz_config = os.path.join(pkg_nav, 'config', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        slam_arg,
        map_arg,
        use_sim_time_arg,
        slam_toolbox_node,
        nav2_bringup,
        rviz_node,
    ])
