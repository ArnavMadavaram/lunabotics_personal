"""
manual_control.launch.py — Launch file for the keyboard teleoperation node.

Usage:
    ros2 launch robot_manual_control manual_control.launch.py

NOTE: The curses UI requires a real TTY.  If the terminal shows no output or
garbled text, run the node directly instead:
    ros2 run robot_manual_control manual_control
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='robot_manual_control',
            executable='manual_control',
            name='manual_control',
            output='screen',
            # emulate_tty=True lets curses detect terminal dimensions correctly
            # when launched from a standard terminal emulator.
            emulate_tty=True,
        ),
    ])
