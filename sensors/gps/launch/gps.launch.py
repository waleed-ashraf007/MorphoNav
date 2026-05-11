"""GPS host-side launch.

NavSatFix is published directly by the ESP32 micro-ROS bridge. This launch
exists so higher-level launches (bringup_real, gps_outdoor) can include a
single GPS entry point. Currently it only starts the fix-health logger.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='morphonav_gps',
            executable='fix_health',
            name='gps_fix_health',
            output='screen',
        ),
    ])
