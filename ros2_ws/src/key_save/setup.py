from setuptools import setup
import os
from glob import glob

package_name = 'key_save'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shen',
    maintainer_email='dev@example.com',
    description='Press a key to save the latest camera + lidar frame to disk.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'key_save = key_save.key_save_node:main',
        ],
    },
)
