"""P1 standalone smoke test — MPU9250 + NEO-7M -> ESP32 (micro-ROS) -> RViz.

Launches:
  1. micro_ros_agent       — serial bridge to the ESP32 firmware
  2. description           — robot_state_publisher with morphonav URDF (real-hw mode)
  3. imu_filter            — Madgwick + covariance setter -> /imu/data
  4. imu_gps_to_tf         — drives world -> base_footprint from /imu/data + /gps/fix
  5. rviz2                 — sensor_test.rviz, fixed frame = world, RobotModel display

Usage:
    ros2 launch morphonav_bringup sensor_test.launch.py
    ros2 launch morphonav_bringup sensor_test.launch.py serial_dev:=/dev/ttyUSB1

What you should see:
    Physically rotate the ESP32+IMU 90 degrees -> the whole MorphoNav URDF in
    RViz rotates 90 degrees about the `world` frame.

This launch is smoke-test-only. Do NOT run it alongside the EKF or any
launch that publishes odom -> base_footprint — they will fight on the TF tree.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_dev   = LaunchConfiguration('serial_dev')
    serial_baud  = LaunchConfiguration('serial_baud')
    rviz_config  = PathJoinSubstitution([
        FindPackageShare('morphonav_bringup'), 'rviz', 'sensor_test.rviz'
    ])

    description_launch = os.path.join(
        get_package_share_directory('morphonav_description'),
        'launch', 'description.launch.py'
    )
    imu_filter_launch = os.path.join(
        get_package_share_directory('morphonav_imu'),
        'launch', 'imu_filter.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_dev',  default_value='/dev/ttyUSB0',
                              description='ESP32 USB-serial device'),
        DeclareLaunchArgument('serial_baud', default_value='115200',
                              description='micro-ROS agent baud rate'),

        # 1. micro-ROS agent (serial bridge to ESP32 firmware)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', serial_dev, '-b', serial_baud],
        ),

        # 2. URDF / robot_state_publisher (real-hardware mode, no Gazebo plugins)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim': 'false', 'use_sim_time': 'false'}.items(),
        ),

        # 3. IMU pipeline: Madgwick orientation + covariance injection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_launch),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # 4. Smoke-test TF bridge: world -> base_footprint from /imu/data + /gps/fix
        Node(
            package='morphonav_imu',
            executable='imu_gps_to_tf',
            name='imu_gps_to_tf',
            output='screen',
            parameters=[{
                'world_frame':     'world',
                'child_frame':     'base_footprint',
                'publish_rate_hz': 30.0,
            }],
        ),

        # 5. RViz with the sensor_test layout (fixed frame = world, RobotModel display)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
