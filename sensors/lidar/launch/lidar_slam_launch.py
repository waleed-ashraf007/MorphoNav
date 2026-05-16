"""LiDAR-only SLAM: RPLIDAR -> range filter -> slam_toolbox -> RViz.

No wheel encoders on this platform, so odom->base_footprint is a static
identity. slam_toolbox runs in pure scan-matching mode and publishes
map->odom; the URDF (via robot_state_publisher) supplies base_footprint
down to laser_frame, so the whole model moves in RViz as the map updates.

Launch:
  ros2 launch lidar_slam lidar_slam_launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lidar_slam')
    slam_params   = os.path.join(pkg, 'config', 'slam_params.yaml')
    filter_params = os.path.join(pkg, 'config', 'lidar_filter_params.yaml')
    rviz_cfg      = os.path.join(pkg, 'config', 'lidar_slam.rviz')

    description_launch = os.path.join(
        get_package_share_directory('morphonav_description'),
        'launch', 'description.launch.py'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim': 'false', 'use_sim_time': 'false'}.items(),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'base_footprint',
            ],
            output='screen',
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port':     '/dev/ttyUSB0',
                'serial_baudrate': 115200,        # A1/A2
                'frame_id':        'laser_frame', # MUST match URDF
                'angle_compensate': True,
                'scan_mode':       'Standard',
            }],
            output='screen',
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_params],
            remappings=[
                ('scan',          '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
            output='screen',
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),

    ])
