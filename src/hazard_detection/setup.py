import os
from glob import glob
from setuptools import setup

package_name = 'hazard_detection'

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
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnavm',
    maintainer_email='arnavm@purdue.edu',
    description='Crater and rock detection pipeline for Lunabotics autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_node = hazard_detection.hazard_node:main',
            'fake_lidar_publisher = hazard_detection.fake_lidar_publisher:main',
        ],
    },
)
