from setuptools import setup
import os
from glob import glob

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shen',
    maintainer_email='dev@example.com',
    description='Hikvision network camera ROS2 driver (RTSP).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hikvision_node = camera_driver.hikvision_node:main',
        ],
    },
)
