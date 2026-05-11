"""fusion.launch.py — Full IMU+LiDAR+GPS fusion for MorphoNav.

TF tree:  map -> odom -> base_footprint -> base_link -> {laser_frame, imu_link, gps_link}
          (slam_toolbox owns map->odom and odom->base_footprint)

Nodes launched:
  1. micro_ros_agent       — serial bridge to ESP32 (/imu/data_raw, /imu/mag, /gps/fix)
  2. robot_state_publisher — URDF joints (base_footprint->base_link->sensors)
  3. imu_filter_madgwick   — /imu/data_raw + /imu/mag -> /imu/data_madgwick
  4. imu_cov_setter        — /imu/data_madgwick -> /imu/data  (with covariances)
  5. rplidar_node          — /scan
  6. laser_filter          — /scan -> /scan_filtered
  7. slam_toolbox          — /scan_filtered -> map + TF(map->odom->base_footprint)
  8. ekf_local             — /imu/data + /odom -> /odometry/filtered  (no TF)
  9. navsat_transform      — /gps/fix + /odometry/filtered -> /odometry/gps
 10. ekf_global            — /odometry/filtered + /odometry/gps -> /odometry/global
 11. gps_fix_health        — GPS quality logger
 12. rviz2                 — fusion.rviz

NOTE: imu_gps_to_tf is intentionally NOT launched here — it conflicts with
      slam_toolbox's odom->base_footprint TF.
NOTE: The static odom->base_footprint TF from lidar_slam_launch.py is also
      intentionally skipped — slam_toolbox publishes that transform dynamically.
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
    serial_dev  = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')

    bringup_share = get_package_share_directory('morphonav_bringup')
    lidar_share   = get_package_share_directory('lidar_slam')

    ekf_local_cfg      = os.path.join(bringup_share, 'config', 'ekf_local.yaml')
    ekf_global_cfg     = os.path.join(bringup_share, 'config', 'ekf_global.yaml')
    navsat_cfg         = os.path.join(bringup_share, 'config', 'navsat_transform.yaml')
    slam_cfg           = os.path.join(lidar_share,   'config', 'slam_params.yaml')
    filter_cfg         = os.path.join(lidar_share,   'config', 'lidar_filter_params.yaml')
    rviz_cfg           = os.path.join(bringup_share, 'rviz',   'fusion.rviz')

    description_launch = os.path.join(
        get_package_share_directory('morphonav_description'),
        'launch', 'description.launch.py'
    )
    imu_filter_launch = os.path.join(
        get_package_share_directory('morphonav_imu'),
        'launch', 'imu_filter.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_dev',  default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),

        # 1. micro-ROS serial bridge to ESP32
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', serial_dev, '-b', serial_baud],
        ),

        # 2. URDF / robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim': 'false', 'use_sim_time': 'false'}.items(),
        ),

        # 3+4. IMU pipeline: Madgwick + covariance injection -> /imu/data
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
                'serial_port': '/dev/ttyUSB1',   # adjust if lidar is on USB0
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

        # 7. SLAM Toolbox — publishes map->odom->base_footprint TF + /odom + /map
        #    (NO static odom->base_footprint here — slam_toolbox owns it dynamically)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_cfg],
            output='screen',
        ),

        # 8. EKF local: /imu/data + /odom -> /odometry/filtered  (publish_tf=false)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_cfg],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),

        # 9. navsat_transform: /gps/fix + /odometry/filtered -> /odometry/gps
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_cfg],
            remappings=[
                ('imu/data',           '/imu/data'),
                ('gps/fix',            '/gps/fix'),
                ('odometry/filtered',  '/odometry/filtered'),
                ('odometry/gps',       '/odometry/gps'),
            ],
        ),

        # 10. EKF global: /odometry/filtered + /odometry/gps -> /odometry/global
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_cfg],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),

        # 11. GPS fix quality logger
        Node(
            package='morphonav_gps',
            executable='fix_health',
            name='gps_fix_health',
            output='screen',
        ),

        # 12. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
