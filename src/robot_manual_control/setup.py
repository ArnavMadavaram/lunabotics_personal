"""
setup.py for robot_manual_control

NOTE: ROS2 Python packages use setup.py + ament_python instead of
CMakeLists.txt.  This file is the build-system equivalent.
"""
import os
from glob import glob
from setuptools import setup

PACKAGE = 'robot_manual_control'

setup(
    name=PACKAGE,
    version='0.1.0',
    # nodes/ is treated as a Python package so entry_points can resolve it.
    packages=['nodes'],
    data_files=[
        # Required ament index marker
        (
            os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', PACKAGE)],
        ),
        # package.xml must be installed to share/<package>/
        (os.path.join('share', PACKAGE), ['package.xml']),
        # Launch files
        (os.path.join('share', PACKAGE, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Purdue Lunabotics Software Team',
    maintainer_email='arnavm@purdue.edu',
    description='Keyboard teleoperation node for Lunabotics robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run robot_manual_control manual_control
            f'manual_control = nodes.manual_control:main',
        ],
    },
)
