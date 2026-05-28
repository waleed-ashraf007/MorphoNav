import os
from glob import glob
from setuptools import setup

package_name = 'morphonav_posture'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Waleed Ashraf',
    maintainer_email='w.ashraf2002@gmail.com',
    description='MorphoNav pitch-based posture manager',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'posture_manager     = morphonav_posture.posture_manager:main',
            'posture_udp_bridge  = morphonav_posture.posture_udp_bridge:main',
        ],
    },
)
