"""Minimal demo: ESP32 IMU over WiFi/UDP -> URDF tilts in RViz.

No lidar, no GPS, no SLAM, no EKF. Just enough to verify the IMU stream is
landing on the host and the URDF responds to motion.

Pipeline:
  ESP32 (main_udp.cpp) --UDP/JSON--> imu_udp_node --/imu/data_raw-->
  imu_filter_madgwick (use_mag=False, publish_tf=True, fixed_frame=odom)
  --TF odom->base_footprint--> robot_state_publisher --URDF--> RViz

The IMU message's frame_id is overridden to 'base_footprint' so Madgwick
publishes odom->base_footprint directly. The whole URDF rotates under that.

Launch:
  ros2 launch morphonav_bringup imu_demo.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_port = LaunchConfiguration('udp_port')

    description_launch = os.path.join(
        get_package_share_directory('morphonav_description'),
        'launch', 'description.launch.py'
    )
    rviz_cfg = os.path.join(
        get_package_share_directory('morphonav_bringup'),
        'rviz', 'imu_demo.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='8888'),

        Node(
            package='morphonav_imu',
            executable='imu_udp_node',
            name='imu_udp_node',
            output='screen',
            parameters=[{
                'port': udp_port,
                'frame_id': 'base_footprint',
                'raw_topic': '/imu/data_raw',
                'publish_fused': False,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim': 'false', 'use_sim_time': 'false'}.items(),
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag':     False,
                'publish_tf':  True,
                'fixed_frame': 'odom',
                'world_frame': 'enu',
                'gain':        0.1,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
