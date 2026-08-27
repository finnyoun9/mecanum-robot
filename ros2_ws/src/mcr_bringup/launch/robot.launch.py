"""
robot.launch.py — Launch the Mecanum Robot hardware stack.

Launches:
  1. robot_state_publisher (URDF → TF)
  2. controller_manager (ros2_control)
  3. mecanum_drive_controller + joint_state_broadcaster + imu_broadcaster
  4. (Optional) laser driver, camera driver
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Paths ---
    pkg_description = get_package_share_directory('mcr_description')
    pkg_bringup     = get_package_share_directory('mcr_bringup')

    urdf_xacro = os.path.join(pkg_description, 'urdf', 'mcr.urdf.xacro')

    # --- Launch args ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_dev   = LaunchConfiguration('serial_device', default='/dev/ttyAMA0')

    # Jazzy requires an explicit ParameterValue for substitution-based string
    # params (robot_description is URDF, which is not valid YAML).
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_xacro,
            ' serial_device:=', serial_dev
        ]),
        value_type=str,
    )

    # --- Nodes ---

    # Robot State Publisher (URDF → TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # Controller Manager (ros2_control)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
            },
            os.path.join(pkg_bringup, 'config', 'controllers.yaml'),
        ],
        remappings=[
            ('~/motors_cmd', '/mecanum_drive_controller/motors_cmd'),
        ],
    )

    # Start the controllers. The spawner executables block until the
    # controller_manager service is available, then spawn and exit — so they
    # can run in parallel instead of being chained on process exit (the old
    # OnProcessExit(controller_manager) chain never fired because
    # ros2_control_node is a persistent process that never exits).
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
        output='screen',
    )

    imu_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        output='screen',
    )

    # robot_localization EKF — fuses wheel odom + IMU into a corrected
    # odom -> base_footprint TF and /odometry/filtered topic. Started at the
    # same time as the spawners; it simply waits for its input topics to
    # appear before publishing anything.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # Bridge /cmd_vel (Twist) -> mecanum_drive_controller/reference
    # (TwistStamped). The Jazzy mecanum_drive_controller has no /cmd_vel
    # subscriber, so users/Nav2 publish Twist on /cmd_vel as usual and this
    # node forwards it to the controller's reference interface.
    cmd_vel_relay = Node(
        package='mcr_bringup',
        executable='cmd_vel_relay',
        output='screen',
    )

    # Accumulates /odometry/filtered poses into a nav_msgs/Path so RViz2 can
    # draw the traveled trajectory (its Path display only takes nav_msgs/Path).
    odom_to_path = Node(
        package='mcr_bringup',
        executable='odom_to_path',
        output='screen',
    )

    # --- LD06 LiDAR driver (ldlidar_stl_ros2 submodule, /dev/ttyUSB0 via CH340) ---
    # 2026-08-27 verified on the Pi: 230400 8N1, 47-byte frames, CRC-8 poly
    # 0x4D, ~10 Hz. frame_id matches the laser_link in mcr.urdf.xacro so the
    # base_footprint -> base_link -> laser_link TF chain (from robot_state_pub)
    # is the sole source of that transform — do NOT add the static base_laser
    # publisher that the upstream ld06.launch.py ships, it would conflict.
    laser_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': 'scan',
            'frame_id': 'laser_link',
            'port_name': '/dev/ttyUSB0',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),

        robot_state_pub,
        controller_manager,

        # Spawners run in parallel; each waits for controller_manager itself.
        joint_state_spawner,
        mecanum_controller_spawner,
        imu_spawner,

        # EKF waits for its inputs, so it can start alongside the spawners.
        ekf_node,

        cmd_vel_relay,
        odom_to_path,

        # LiDAR — starts as soon as /dev/ttyUSB0 is available.
        laser_node,
    ])
