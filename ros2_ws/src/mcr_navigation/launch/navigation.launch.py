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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
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

    # Bringing up Nav2 is separable from bringing up SLAM: mapping only needs
    # /scan plus the odom->base_footprint TF, while Nav2 pulls in the whole
    # lifecycle/BT stack. Keeping a switch lets you verify (and debug) the
    # mapping half on its own — which is also the only way to run SLAM here
    # today, because nav2_bringup evaluates `slam` through a PythonExpression
    # and 'true' is not a Python literal ('True' is), so including it raises
    # NameError: name 'true' is not defined.
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true',
        description='Bring up the Nav2 stack (true) or SLAM/RViz only (false)')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
        description='Start RViz2 (needs a display; set false when headless)')

    # --- SLAM Toolbox ---
    # async_slam_toolbox_node is a LifecycleNode: started as a plain Node it
    # comes up 'unconfigured' and simply sits there — it logs "Node using
    # stack size ..." and then nothing, never subscribing to /scan and never
    # publishing /map. (Its parameters also read back as "not set" in that
    # state, which looks like the YAML failed to load but is just an artifact
    # of the node not being configured yet.) Declaring it as a LifecycleNode
    # and emitting the configure/activate transitions below is what actually
    # starts mapping. Nav2's own bringup drives these transitions via its
    # lifecycle_manager, so this only matters on the nav2:=false path — but
    # it must be driven either way.
    slam_toolbox_node = LifecycleNode(
        condition=IfCondition(LaunchConfiguration('slam')),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(pkg_nav, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # unconfigured -> inactive. This has to wait for OnProcessStart: a bare
    # EmitEvent in the launch description fires while the node process is
    # still coming up, before its lifecycle services exist, and the
    # transition request is silently dropped (the node just stays
    # 'unconfigured' with no 'Configuring' line in the log).
    slam_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # inactive -> active, once configuring has actually finished. Chaining on
    # the state-transition event (rather than a fixed sleep) matters here:
    # configure loads the Ceres solver plugin, which took ~10 s on the Pi.
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # --- Nav2 Bringup ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('nav2')),
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
        condition=IfCondition(LaunchConfiguration('rviz')),
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
        nav2_arg,
        rviz_arg,
        slam_toolbox_node,
        slam_configure,
        slam_activate,
        nav2_bringup,
        rviz_node,
    ])
