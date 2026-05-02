import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('lidar_slam')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')
    filter_params = os.path.join(pkg, 'config', 'lidar_filter_params.yaml')

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'scan_mode': 'Standard',
            }],
            output='screen'
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_params],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'base_footprint'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'laser_frame',
                '--z', '0.1'
            ],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen'
        ),

    ])
