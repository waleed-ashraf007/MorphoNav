from setuptools import setup
import os
from glob import glob

package_name = 'lidar_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],

    data_files=[
        # package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waleedashraf',
    maintainer_email='waleed@example.com',
    description='MorphoNav LiDAR SLAM',
    license='MIT',

    entry_points={
        'console_scripts': [],
    },
)
