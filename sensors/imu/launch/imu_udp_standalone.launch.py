"""Standalone IMU-only path (Parts 1-6 of the original spec).

ESP32 (UDP/JSON) -> imu_udp_node -> imu_odom_node -> RViz.

Publishes:
  /imu/data_raw  — raw accel+gyro (no orientation)
  /imu/data_udp  — accel+gyro + on-device fused quaternion  (this is /imu)
  /imu/odom      — integrated odometry
  TF: odom -> base_link

Run as:
  ros2 launch morphonav_imu imu_udp_standalone.launch.py

DO NOT chain this with the SLAM/EKF fusion launch — both publish odom->base_*.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_port = LaunchConfiguration('udp_port')

    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='8888'),

        Node(
            package='morphonav_imu',
            executable='imu_udp_node',
            name='imu_udp_node',
            output='screen',
            parameters=[{
                'port': udp_port,
                'frame_id': 'imu_link',
                'raw_topic': '/imu/data_raw',
                'fused_topic': '/imu',
                'publish_fused': True,
            }],
        ),

        Node(
            package='morphonav_imu',
            executable='imu_odom_node',
            name='imu_odom_node',
            output='screen',
            parameters=[{
                'input_topic': '/imu',
                'odom_topic':  '/imu/odom',
                'odom_frame':  'odom',
                'child_frame': 'base_link',
                'publish_tf':  True,
            }],
        ),
    ])
