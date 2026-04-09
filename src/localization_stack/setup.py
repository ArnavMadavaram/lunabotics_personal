import os
from glob import glob
from setuptools import setup

package_name = 'localization_stack'

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
    maintainer='arnavm',
    maintainer_email='arnavm@purdue.edu',
    description='Localization stack: Point-LIO sim, EKF, UWB correction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_lio_sim       = localization_stack.point_lio_sim:main',
            'uwb_correction_node = localization_stack.uwb_correction_node:main',
        ],
    },
)
