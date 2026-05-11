"""IMU orientation pipeline.

Composes:
  imu_filter_madgwick  /imu/data_raw + /imu/mag -> /imu/data_madgwick
  imu_cov_setter       /imu/data_madgwick       -> /imu/data  (with covariances)

Output topic /imu/data carries an orientation-bearing sensor_msgs/Imu with
non-zero covariances, ready for robot_localization.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mag      = LaunchConfiguration('use_mag')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_mag',      default_value='false'),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_sim_time':   use_sim_time,
                'use_mag':        use_mag,
                'world_frame':    'enu',
                'publish_tf':     False,
                'gain':           0.1,
                'mag_bias_x':     0.0,
                'mag_bias_y':     0.0,
                'mag_bias_z':     0.0,
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/mag',      '/imu/mag'),
                ('/imu/data',     '/imu/data_madgwick'),
            ],
        ),

        Node(
            package='morphonav_imu',
            executable='imu_cov_setter',
            name='imu_cov_setter',
            output='screen',
            parameters=[{
                'use_sim_time':   use_sim_time,
                'input_topic':    '/imu/data_madgwick',
                'output_topic':   '/imu/data',
            }],
        ),
    ])
