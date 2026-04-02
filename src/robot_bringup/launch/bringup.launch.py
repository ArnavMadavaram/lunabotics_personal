import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'lunabot.urdf.xacro')
    teleop_config = os.path.join(pkg_dir, 'config', 'teleop.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process xacro → URDF string at launch time
    robot_description = subprocess.check_output(
        ['xacro', xacro_file], text=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (set true when running in Gazebo)',
        ),

        # 1. robot_state_publisher
        #    Reads robot_description and publishes /tf (fixed joints)
        #    and /robot_description latched topic.
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

        # 2. joint_state_publisher
        #    Publishes /joint_states for the continuous wheel joints so
        #    robot_state_publisher can broadcast their transforms.
        #    Replace with joint_state_publisher_gui for visual sliders in RViz.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # 3. teleop_twist_keyboard
        #    Reads keyboard input and publishes geometry_msgs/Twist on /cmd_vel.
        #    Must run in a terminal with a TTY — use prefix to open xterm.
        #    output='screen' alone won't capture keyboard input when launched
        #    as a subprocess; xterm gives it its own interactive terminal.
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
            parameters=[teleop_config],
        ),
    ])
