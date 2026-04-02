import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_config')
    global_config = os.path.join(pkg_dir, 'config', 'costmaps.yaml')
    fake_hazards_script = os.path.join(pkg_dir, 'scripts', 'fake_hazards.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    demo_mode = LaunchConfiguration('demo', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('demo', default_value='true',
                              description='Publish fake hazard data for visualization'),

        # TF tree: map -> odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ),

        # Global costmap node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            namespace='costmap',
            output='screen',
            arguments=['--ros-args',
                       '--params-file', global_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Auto-activate the costmap 3 seconds after launch
        # Auto-activate via service call (more reliable than lifecycle set)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call',
                         '/costmap/costmap/change_state',
                         'lifecycle_msgs/srv/ChangeState',
                         '{transition: {id: 1}}'],  # configure
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call',
                         '/costmap/costmap/change_state',
                         'lifecycle_msgs/srv/ChangeState',
                         '{transition: {id: 3}}'],  # activate
                    output='screen',
                ),
            ]
        ),

        # Fake hazard publisher for demo/presentation (rocks + craters)
        Node(
            package='nav2_config',
            executable='fake_hazards',
            name='fake_hazards',
            output='screen',
        ),
    ])
