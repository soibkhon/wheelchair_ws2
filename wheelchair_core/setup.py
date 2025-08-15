from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wheelchair_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
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
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 core control package for wheelchair with L2DB motor drivers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheelchair_core_node = wheelchair_core.wheelchair_core_node:main',
            'joystick_controller = wheelchair_core.joystick_controller_node:main',
            'position_controller = wheelchair_core.position_controller_node:main',
            'odometry_calibration = wheelchair_core.odometry_calibration_node:main'
        ],
    },
)
