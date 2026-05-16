#!/usr/bin/env python3
"""
MorphoNav Gazebo Simulation Launch File
Starts: Gazebo + Robot + laser_filter + SLAM Toolbox + RViz

Usage:
  ros2 launch simulations gazebo_slam_launch.py
  ros2 launch simulations gazebo_slam_launch.py world:=test_track
  ros2 launch simulations gazebo_slam_launch.py drive_mode:=awd
  ros2 launch simulations gazebo_slam_launch.py launch_teleop:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg      = get_package_share_directory('simulations')
    desc_pkg = get_package_share_directory('morphonav_description')

    world_arg     = LaunchConfiguration('world')
    launch_teleop = LaunchConfiguration('launch_teleop')
    drive_mode    = LaunchConfiguration('drive_mode')

    urdf_file     = os.path.join(desc_pkg, 'urdf',   'morphonav.urdf.xacro')
    slam_params   = os.path.join(pkg, 'config', 'slam_params_sim.yaml')
    filter_params = os.path.join(pkg, 'config', 'lidar_filter_params.yaml')
    rviz_config   = os.path.join(pkg, 'config', 'morphonav_slam.rviz')

    world_path = PathJoinSubstitution([pkg, 'worlds', [world_arg, '.world']])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' use_sim:=true',
            ' drive_mode:=', drive_mode,
        ]),
        value_type=str
    )

    return LaunchDescription([

        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('world', default_value='open_world',
            description='Gazebo .world file basename (open_world or test_track)'),
        DeclareLaunchArgument('launch_teleop', default_value='false',
            description='Start keyboard teleop in an xterm alongside the simulation'),
        DeclareLaunchArgument('drive_mode', default_value='rwd',
            description="Drive mode: 'rwd' (rear pair) or 'awd' (all four wheels)"),

        # ── 1. Gazebo ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'verbose': 'false'}.items(),
        ),

        # ── 2. Robot State Publisher ─────────────────────────────────────────
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

        # ── 3. Spawn robot ───────────────────────────────────────────────────
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

        # ── 4. Laser filter (/scan → /scan_filtered) ─────────────────────────
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

        # ── 5. SLAM Toolbox ──────────────────────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': True}],
        ),

        # ── 6. RViz ─────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),

        LogInfo(
            msg='Drive with: ros2 run simulations diff_drive_teleop  '
                '(or set launch_teleop:=true)'
        ),

        # ── 7. Teleop (optional) ─────────────────────────────────────────────
        Node(
            package='simulations',
            executable='diff_drive_teleop',
            name='twist_teleop',
            output='screen',
            prefix='xterm -e',
            condition=IfCondition(launch_teleop),
        ),

        # ── 8. Trajectory Publisher ──────────────────────────────────────────
        Node(
            package='simulations',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_poses': 0,
            }],
        ),
    ])
