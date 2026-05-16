#!/usr/bin/env python3
"""
MorphoNav Nav2 launch — Gazebo simulation.

Brings up:
  - Gazebo + URDF (use_sim:=true, drive_mode arg)
  - laser_filter (/scan → /scan_filtered)
  - slam_toolbox in mapping mode (sim params)
  - Nav2 stack: planner, controller, smoother, behaviors, waypoint_follower,
                velocity_smoother, bt_navigator, lifecycle_manager
  - RViz with rviz_nav2.rviz

map_server + amcl are intentionally NOT brought up; slam_toolbox owns /map and
map→odom while mapping. Switch to localization mode in a follow-up launch by
loading a saved map with map_server + slam_toolbox `mode: localization`.

Usage:
  ros2 launch morphonav_bringup nav2_sim.launch.py
  ros2 launch morphonav_bringup nav2_sim.launch.py drive_mode:=awd
  ros2 launch morphonav_bringup nav2_sim.launch.py world:=test_track
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_pkg = get_package_share_directory('morphonav_bringup')
    sim_pkg     = get_package_share_directory('simulations')
    desc_pkg    = get_package_share_directory('morphonav_description')
    lidar_pkg   = get_package_share_directory('lidar_slam')

    drive_mode = LaunchConfiguration('drive_mode')
    world_arg  = LaunchConfiguration('world')

    nav2_params_file = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    rviz_config      = os.path.join(bringup_pkg, 'rviz',   'rviz_nav2.rviz')
    urdf_file        = os.path.join(desc_pkg,    'urdf',   'morphonav.urdf.xacro')
    slam_params      = os.path.join(sim_pkg,     'config', 'slam_params_sim.yaml')
    filter_params    = os.path.join(lidar_pkg,   'config', 'lidar_filter_params.yaml')

    world_path = PathJoinSubstitution([sim_pkg, 'worlds', [world_arg, '.world']])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' use_sim:=true',
            ' drive_mode:=', drive_mode,
        ]),
        value_type=str
    )

    # Inject use_sim_time:=true into every Nav2 node section of the shared YAML.
    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': 'true'},
        convert_types=True,
    )

    nav2_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('drive_mode', default_value='rwd',
            description="Drive mode: 'rwd' (rear pair) or 'awd' (all four wheels)"),
        DeclareLaunchArgument('world', default_value='open_world',
            description='Gazebo .world basename (open_world or test_track)'),

        # ── Gazebo ──────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'verbose': 'false'}.items(),
        ),

        # ── Robot State Publisher ────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # ── Spawn robot (delayed so /spawn_entity service is up first) ──────
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'morphonav',
                        '-x', '0', '-y', '0', '-z', '0.05',
                    ],
                ),
            ],
        ),

        # ── Laser filter: /scan → /scan_filtered ─────────────────────────────
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            output='screen',
            parameters=[filter_params, {'use_sim_time': True}],
            remappings=[
                ('scan',          '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
        ),

        # ── SLAM Toolbox (mapping) ───────────────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': True}],
        ),

        # ── Nav2 nodes ───────────────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': nav2_lifecycle_nodes,
            }],
        ),

        # ── RViz ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
