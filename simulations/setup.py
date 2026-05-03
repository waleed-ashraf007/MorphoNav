import os
from glob import glob
from setuptools import setup

package_name = 'simulations'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required for ROS 2 to discover the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Config files (SLAM params, filter params, RViz config)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),

        # URDF / Xacro robot description
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Waleed Ashraf',
    maintainer_email='waleed@morphonav.local',
    description='MorphoNav Gazebo simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registers teleop as a ros2 run command
            'ackermann_teleop = simulations.ackermann_teleop:main',
        ],
    },
)
