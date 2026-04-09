"""
nav2.launch.py — Lunabotics full autonomy stack

Launch order:
  1. robot_state_publisher + joint_state_publisher  (robot_bringup URDF)
  2. fake_lidar_publisher                           (hazard_detection)
  3. hazard_node                                    (hazard_detection)
  4. point_lio_sim                                  (localization_stack)
  5. ekf_node                                       (robot_localization)
  6. uwb_correction_node                            (localization_stack)
  7. map_server + lifecycle_manager_map             (nav2_map_server)
  8. nav2 navigation stack                          (nav2_bringup)
  9. rviz2

Usage:
  ros2 launch nav2_config nav2.launch.py
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package directories ────────────────────────────────────────────────
    pkg_bringup   = get_package_share_directory('robot_bringup')
    pkg_hazard    = get_package_share_directory('hazard_detection')
    pkg_loc       = get_package_share_directory('localization_stack')
    pkg_nav2      = get_package_share_directory('nav2_config')
    pkg_nav2_bup  = get_package_share_directory('nav2_bringup')

    # ── File paths ─────────────────────────────────────────────────────────
    xacro_file        = os.path.join(pkg_bringup, 'urdf', 'lunabot.urdf.xacro')
    hazard_params     = os.path.join(pkg_hazard,  'config', 'hazard_params.yaml')
    point_lio_params  = os.path.join(pkg_loc,     'config', 'point_lio_sim_params.yaml')
    ekf_params        = os.path.join(pkg_loc,     'config', 'ekf_params.yaml')
    map_yaml          = os.path.join(pkg_nav2,    'maps',   'arena.yaml')
    nav2_params       = os.path.join(pkg_nav2,    'config', 'nav2_params.yaml')
    rviz_config       = os.path.join(pkg_nav2,    'rviz',   'nav2.rviz')
    nav2_launch       = os.path.join(pkg_nav2_bup, 'launch', 'navigation_launch.py')

    # ── URDF (processed once at launch time) ──────────────────────────────
    robot_description = subprocess.check_output(['xacro', xacro_file], text=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),

        # ── 1. Robot model / TF publishers ────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── 2. Fake LiDAR (simulated Unitree L2 point cloud) ──────────────
        Node(
            package='hazard_detection',
            executable='fake_lidar_publisher',
            name='fake_lidar_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── 3. Hazard detection (crater + rock pipeline) ──────────────────
        Node(
            package='hazard_detection',
            executable='hazard_node',
            name='hazard_node',
            output='screen',
            parameters=[hazard_params, {'use_sim_time': use_sim_time}],
        ),

        # ── 4. Point-LIO simulator (map → odom TF) ────────────────────────
        Node(
            package='localization_stack',
            executable='point_lio_sim',
            name='point_lio_sim',
            output='screen',
            parameters=[point_lio_params, {'use_sim_time': use_sim_time}],
        ),

        # ── 5. EKF (odom → base_link TF + /odometry/filtered) ────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),

        # ── 6. UWB absolute position correction ───────────────────────────
        Node(
            package='localization_stack',
            executable='uwb_correction_node',
            name='uwb_correction_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── 7. Map server (arena static map) ──────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        # ── 8. Nav2 navigation stack (planner, controller, BT, costmaps) ──
        # Delayed 2 s to let TF tree stabilise before costmaps initialise.
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'params_file': nav2_params,
                        'use_sim_time': 'false',
                    }.items(),
                ),
            ],
        ),

        # ── 9. RViz2 ──────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
