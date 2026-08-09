"""Setup script for the popeye package."""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'popeye'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        # Include video files (if any)
        (os.path.join('share', package_name, 'videos'), 
            glob('popeye/videos/*.mp4')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='POPEYE drone control system with ArUco marker detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MAV_manager_node = popeye.MAV_manager__node:main',
            'CAM_node = popeye.CAM__node:main',
            'ARUCO_node = popeye.ARUCO__node:main',
            'FSM_node = popeye.FSM__node:main',
        ],
    },
)
