"""
Localization stack launch file.

Assumes robot_bringup is already running (robot_state_publisher,
joint_state_publisher, fake_lidar_publisher).  This file starts only
the localization-specific nodes:

  1. point_lio_sim        — map → odom TF @ 50 Hz
  2. ekf_node             — odom → base_link TF @ 20 Hz
  3. uwb_correction_node  — periodic absolute position fix

TF chain produced (combined with bringup):
  map → odom              ← point_lio_sim
  odom → base_link        ← ekf_node
  base_link → *_link      ← robot_state_publisher (from robot_bringup)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_loc = get_package_share_directory('localization_stack')

    ekf_params       = os.path.join(pkg_loc, 'config', 'ekf_params.yaml')
    point_lio_params = os.path.join(pkg_loc, 'config', 'point_lio_sim_params.yaml')
    uwb_params       = os.path.join(pkg_loc, 'config', 'uwb_params.yaml')

    return LaunchDescription([

        # ── 1. Fake LiDAR sensor (hazard_detection) ──────────────────
        Node(
            package='hazard_detection',
            executable='fake_lidar_publisher',
            name='fake_lidar_publisher',
            output='screen',
        ),

        # ── 3. Point-LIO simulator (map → odom TF) ───────────────────
        Node(
            package='localization_stack',
            executable='point_lio_sim',
            name='point_lio_sim',
            output='screen',
            parameters=[point_lio_params],
        ),

        # ── 4. EKF node (odom → base_link TF) ────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),

        # ── 5. UWB correction node ────────────────────────────────────
        Node(
            package='localization_stack',
            executable='uwb_correction_node',
            name='uwb_correction_node',
            output='screen',
            parameters=[uwb_params],
        ),
    ])
