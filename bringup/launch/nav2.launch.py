#!/usr/bin/env python3
"""
MorphoNav Nav2 launch — real robot.

Brings up:
  - imu_udp_node (ESP32 UDP/JSON IMU) -> /imu/data_raw (+ /imu/data_udp)
  - robot_state_publisher (URDF with use_sim:=false, drive_mode arg)
  - host-side imu_filter chain -> /imu/data
  - rplidar -> /scan
  - laser_filter -> /scan_filtered
  - slam_toolbox in mapping mode -> /map + /odom + TF(map->odom->base_footprint)
  - Nav2 stack (planner, controller, smoother, behaviors, waypoint_follower,
                velocity_smoother, bt_navigator, lifecycle_manager)
  - RViz with rviz_nav2.rviz

map_server + amcl are intentionally NOT brought up; slam_toolbox owns /map and
map->odom while mapping. Save a map with `ros2 run morphonav_bringup save_map.sh`.

Usage:
  ros2 launch morphonav_bringup nav2.launch.py
  ros2 launch morphonav_bringup nav2.launch.py drive_mode:=awd
  ros2 launch morphonav_bringup nav2.launch.py udp_port:=8888
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_pkg  = get_package_share_directory('morphonav_bringup')
    lidar_pkg    = get_package_share_directory('lidar_slam')
    desc_pkg     = get_package_share_directory('morphonav_description')
    imu_pkg      = get_package_share_directory('morphonav_imu')
    nav2_bt_pkg  = get_package_share_directory('nav2_bt_navigator')

    # Simple BT tree: ComputePathToPose → FollowPath, no Spin/BackUp recovery.
    # Spin and BackUp track progress via odom→base_footprint TF change, which
    # never changes with our static identity TF (no wheel encoders). Using the
    # full recovery tree causes silent 60-second abort loops.
    simple_bt_xml = os.path.join(
        nav2_bt_pkg, 'behavior_trees',
        'navigate_w_replanning_only_if_goal_is_updated.xml'
    )

    drive_mode = LaunchConfiguration('drive_mode')
    udp_port   = LaunchConfiguration('udp_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_rviz   = LaunchConfiguration('use_rviz')

    nav2_params_file = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    rviz_config      = os.path.join(bringup_pkg, 'rviz',   'rviz_nav2.rviz')
    slam_params      = os.path.join(lidar_pkg,   'config', 'slam_params.yaml')
    filter_params    = os.path.join(lidar_pkg,   'config', 'lidar_filter_params.yaml')

    description_launch = os.path.join(desc_pkg, 'launch', 'description.launch.py')
    imu_filter_launch  = os.path.join(imu_pkg,  'launch', 'imu_filter.launch.py')

    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': 'false'},
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
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Launch RViz2 (set false on RPi, true on laptop)'),
        DeclareLaunchArgument('drive_mode', default_value='rwd',
            description="Drive mode: 'rwd' (rear pair) or 'awd' (all four wheels)"),
        DeclareLaunchArgument('udp_port',   default_value='8888',
            description='UDP port the ESP32 streams IMU JSON to'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
            description='Serial port for RPLidar (USB0 when IMU is UDP-only)'),

        # ── 1. ESP32 UDP IMU ─────────────────────────────────────────────────
        Node(
            package='morphonav_imu',
            executable='imu_udp_node',
            name='imu_udp_node',
            output='screen',
            parameters=[{
                'port': udp_port,
                'frame_id': 'imu_link',
                'raw_topic': '/imu/data_raw',
                'fused_topic': '/imu/data_udp',
                'publish_fused': True,
            }],
        ),

        # ── 2. URDF / robot_state_publisher ──────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'use_sim':      'false',
                'use_sim_time': 'false',
                'drive_mode':   drive_mode,
            }.items(),
        ),

        # ── 3+4. Host-side Madgwick + cov_setter -> /imu/data ────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_launch),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # ── 5. RPLidar -> /scan ──────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # ── 6. Laser filter -> /scan_filtered ────────────────────────────────
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            output='screen',
            parameters=[filter_params],
            remappings=[
                ('scan',          '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
        ),

        # ── 6b. Static odom->base_footprint — slam_toolbox needs this as input;
        #        no wheel encoders so it stays identity; slam_toolbox updates
        #        map->odom to track the real position instead.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
            output='screen',
        ),

        # ── 7. SLAM Toolbox (mapping) ────────────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),

        # ── 8. Nav2 nodes ────────────────────────────────────────────────────
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
            parameters=[nav2_params, {
                'default_nav_to_pose_bt_xml': simple_bt_xml,
            }],
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
                'use_sim_time': False,
                'autostart': True,
                'node_names': nav2_lifecycle_nodes,
            }],
        ),

        # ── 9. RViz ──────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
