"""Launch posture_manager + posture_udp_bridge.

Usage:
  ros2 launch morphonav_posture posture.launch.py
  ros2 launch morphonav_posture posture.launch.py target_ip:=192.168.1.55 target_port:=9999
  ros2 launch morphonav_posture posture.launch.py uphill_threshold:=8.0 hysteresis:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('uphill_threshold',   default_value='10.0'),
        DeclareLaunchArgument('downhill_threshold', default_value='10.0'),
        DeclareLaunchArgument('hysteresis',         default_value='3.0'),
        DeclareLaunchArgument('smoothing_alpha',    default_value='0.2'),
        DeclareLaunchArgument('target_ip',          default_value='192.168.1.100',
            description='IP of servo ESP32'),
        DeclareLaunchArgument('target_port',        default_value='9999',
            description='UDP port of servo ESP32'),

        Node(
            package='morphonav_posture',
            executable='posture_manager',
            name='posture_manager',
            output='screen',
            parameters=[{
                'uphill_threshold':   LaunchConfiguration('uphill_threshold'),
                'downhill_threshold': LaunchConfiguration('downhill_threshold'),
                'hysteresis':         LaunchConfiguration('hysteresis'),
                'smoothing_alpha':    LaunchConfiguration('smoothing_alpha'),
            }],
        ),

        Node(
            package='morphonav_posture',
            executable='posture_udp_bridge',
            name='posture_udp_bridge',
            output='screen',
            parameters=[{
                'target_ip':   LaunchConfiguration('target_ip'),
                'target_port': LaunchConfiguration('target_port'),
            }],
        ),
    ])
