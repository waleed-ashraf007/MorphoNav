import os
from glob import glob
from setuptools import setup

package_name = 'morphonav_bringup'

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
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Waleed Ashraf',
    maintainer_email='w.ashraf2002@gmail.com',
    description='MorphoNav top-level launch composition',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
