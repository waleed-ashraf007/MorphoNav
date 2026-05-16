"""Robot state publisher launch helper.

Included by every higher-level launch (sensor_test, slam, navigation, gazebo).
Resolves the URDF via xacro with `use_sim` and `drive_mode` flags.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim      = LaunchConfiguration('use_sim')
    drive_mode   = LaunchConfiguration('drive_mode')

    xacro_path = PathJoinSubstitution([
        FindPackageShare('morphonav_description'),
        'urdf', 'morphonav.urdf.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_sim:=',    use_sim,
        ' drive_mode:=', drive_mode,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_sim',      default_value='false',
                              description='Emit Gazebo plugins in the URDF (sim only)'),
        DeclareLaunchArgument('drive_mode',   default_value='rwd',
                              description="Drive mode: 'rwd' or 'awd'"),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_sim),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
