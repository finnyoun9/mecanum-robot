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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Paths ---
    pkg_description = get_package_share_directory('mcr_description')
    pkg_bringup     = get_package_share_directory('mcr_bringup')

    urdf_xacro = os.path.join(pkg_description, 'urdf', 'mcr.urdf.xacro')
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_xacro
    ])

    # --- Launch args ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_dev   = LaunchConfiguration('serial_device', default='/dev/ttyAMA0')

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

    # Spawn controllers after controller_manager is up
    def spawn_controllers(event, controller_names):
        """Delay-spawn controllers after CM is ready."""
        from launch_ros.actions import LoadJointStateBroadcaster, LoadController
        actions = []
        for name in controller_names:
            actions.append(LoadController(
                controller_name=name,
            ))
        return actions

    # Start the controllers sequentially
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

    # --- Laser driver (LD19/LD06) — uncomment when wired ---
    # laser_node = Node(
    #     package='ldlidar_stl_ros2',
    #     executable='ldlidar_stl_ros2_node',
    #     name='ldlidar',
    #     output='screen',
    #     parameters=[{
    #         'product_name': 'LDLiDAR_LD19',
    #         'topic_scan': 'scan',
    #         'frame_id': 'laser_link',
    #         'port_name': '/dev/ttyUSB0',
    #     }],
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),

        robot_state_pub,
        controller_manager,

        # Register event handler to spawn controllers after CM starts
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[joint_state_spawner],
            )
        ),

        # Spawn mecanum controller after joint state broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[mecanum_controller_spawner, imu_spawner],
            )
        ),
    ])
