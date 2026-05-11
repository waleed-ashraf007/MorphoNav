"""fusion_udp.launch.py — UDP-transport variant of fusion.launch.py.

Same node graph as fusion.launch.py except the IMU comes from the ESP32 over
WiFi/UDP instead of micro-ROS over serial. We keep the existing host-side
Madgwick + cov_setter chain by routing imu_udp_node -> /imu/data_raw, so the
EKF inputs and TF tree are identical to the serial path.

TF tree:  map -> odom -> base_footprint -> base_link -> {laser_frame, imu_link, gps_link}
          (slam_toolbox owns map->odom->base_footprint, EKFs do NOT publish TF)

Nodes launched:
  1. imu_udp_node          — UDP/8888 -> /imu/data_raw + /imu/data_udp
  2. robot_state_publisher — URDF (base_footprint -> base_link -> sensors)
  3. imu_filter_madgwick   — /imu/data_raw -> /imu/data_madgwick
  4. imu_cov_setter        — /imu/data_madgwick -> /imu/data
  5. rplidar_node          — /scan
  6. laser_filter          — /scan -> /scan_filtered
  7. slam_toolbox          — /scan_filtered -> /map + /odom + TF(map->odom->bf)
  8. ekf_local             — /imu/data + /odom -> /odometry/filtered
  9. (optional) navsat_transform + ekf_global + gps_fix_health, gated on enable_gps
 10. rviz2                 — fusion.rviz

Launch args:
  udp_port      (8888)         UDP port the ESP32 streams to
  enable_gps    (false)        If true, expects /gps/fix from another source
                               (e.g., a separately-launched micro_ros_agent
                               configured for GPS only, or a NMEA driver)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_port   = LaunchConfiguration('udp_port')
    enable_gps = LaunchConfiguration('enable_gps')

    bringup_share = get_package_share_directory('morphonav_bringup')
    lidar_share   = get_package_share_directory('lidar_slam')

    ekf_local_cfg  = os.path.join(bringup_share, 'config', 'ekf_local.yaml')
    ekf_global_cfg = os.path.join(bringup_share, 'config', 'ekf_global.yaml')
    navsat_cfg     = os.path.join(bringup_share, 'config', 'navsat_transform.yaml')
    slam_cfg       = os.path.join(lidar_share,   'config', 'slam_params.yaml')
    filter_cfg     = os.path.join(lidar_share,   'config', 'lidar_filter_params.yaml')
    rviz_cfg       = os.path.join(bringup_share, 'rviz',   'fusion.rviz')

    description_launch = os.path.join(
        get_package_share_directory('morphonav_description'),
        'launch', 'description.launch.py'
    )
    imu_filter_launch = os.path.join(
        get_package_share_directory('morphonav_imu'),
        'launch', 'imu_filter.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('udp_port',   default_value='8888'),
        DeclareLaunchArgument('enable_gps', default_value='false'),

        # 1. ESP32 UDP/JSON -> /imu/data_raw (+ /imu/data_udp)
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

        # 2. URDF / robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim': 'false', 'use_sim_time': 'false'}.items(),
        ),

        # 3+4. Existing host-side IMU chain — kept intact so the EKF input
        # /imu/data is identical to the serial path.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_launch),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # 5. RPLidar -> /scan
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),

        # 6. Laser filter -> /scan_filtered
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_cfg],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
            output='screen',
        ),

        # 7. SLAM Toolbox -- map->odom->base_footprint TF + /odom + /map
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_cfg],
            output='screen',
        ),

        # 8. EKF local: /imu/data + /odom -> /odometry/filtered (publish_tf=false)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_cfg],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),

        # 9. GPS-dependent fusion -- only when /gps/fix is being published
        GroupAction(
            condition=IfCondition(enable_gps),
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform',
                    output='screen',
                    parameters=[navsat_cfg],
                    remappings=[
                        ('imu/data',          '/imu/data'),
                        ('gps/fix',           '/gps/fix'),
                        ('odometry/filtered', '/odometry/filtered'),
                        ('odometry/gps',      '/odometry/gps'),
                    ],
                ),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_global',
                    output='screen',
                    parameters=[ekf_global_cfg],
                    remappings=[('odometry/filtered', '/odometry/global')],
                ),
                Node(
                    package='morphonav_gps',
                    executable='fix_health',
                    name='gps_fix_health',
                    output='screen',
                ),
            ],
        ),

        # 10. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
