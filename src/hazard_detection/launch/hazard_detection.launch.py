"""
Launch file for hazard_detection standalone test.

Starts:
  1. robot_state_publisher   — publishes TF tree from URDF (robot_bringup)
  2. joint_state_publisher   — drives continuous wheel joints
  3. fake_lidar_publisher    — simulated Unitree L2 point cloud
  4. hazard_node             — crater/rock detection pipeline
  5. rviz2                   — pre-configured visualisation

Usage:
  ros2 launch hazard_detection hazard_detection.launch.py
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_hazard  = get_package_share_directory('hazard_detection')

    xacro_file = os.path.join(pkg_bringup, 'urdf', 'lunabot.urdf.xacro')
    robot_description = subprocess.check_output(
        ['xacro', xacro_file], text=True)

    hazard_params = os.path.join(pkg_hazard, 'config', 'hazard_params.yaml')
    rviz_config   = os.path.join(pkg_hazard, 'rviz',   'hazard_detection.rviz')

    return LaunchDescription([

        # ── TF providers (from robot_bringup URDF) ───────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # ── Fake Unitree L2 sensor ────────────────────────────────────
        Node(
            package='hazard_detection',
            executable='fake_lidar_publisher',
            name='fake_lidar_publisher',
            output='screen',
        ),

        # ── Crater / rock detection pipeline ─────────────────────────
        Node(
            package='hazard_detection',
            executable='hazard_node',
            name='hazard_node',
            output='screen',
            parameters=[hazard_params],
        ),

        # ── RViz2 ─────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
