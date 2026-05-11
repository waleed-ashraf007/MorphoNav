import os
from glob import glob
from setuptools import setup

package_name = 'morphonav_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Waleed Ashraf',
    maintainer_email='w.ashraf2002@gmail.com',
    description='MorphoNav IMU host-side stack',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_gps_to_tf  = morphonav_imu.imu_gps_to_tf:main',
            'imu_cov_setter = morphonav_imu.imu_cov_setter:main',
            'calibrate_mag  = morphonav_imu.calibrate_mag:main',
            'imu_udp_node   = morphonav_imu.imu_udp_node:main',
            'imu_odom_node  = morphonav_imu.imu_odom_node:main',
        ],
    },
)
