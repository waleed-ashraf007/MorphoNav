"""Robot state publisher launch helper.

Included by every higher-level launch (sensor_test, slam, navigation, gazebo).
Resolves the URDF via xacro with a `use_sim` flag.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim      = LaunchConfiguration('use_sim')

    xacro_path = PathJoinSubstitution([
        FindPackageShare('morphonav_description'),
        'urdf', 'morphonav.urdf.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_path, ' use_sim:=', use_sim
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_sim',      default_value='false',
                              description='Emit Gazebo plugins in the URDF (sim only)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),

        # joint_state_publisher only on the real robot — Gazebo provides joint
        # states via libgazebo_ros_joint_state_publisher.so when use_sim:=true.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            condition=__import__('launch.conditions', fromlist=['UnlessCondition']).UnlessCondition(use_sim),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
