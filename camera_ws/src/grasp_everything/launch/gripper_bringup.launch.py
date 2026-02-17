"""
Bringup launch for gripper control.

Starts:
- Gripper control node

Usage:
  ros2 launch grasp_everything gripper_bringup.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gripper_control = Node(
        package='gripper_control',
        executable='gripper_control',
        name='gripper_control',
        output='screen',
    )

    return LaunchDescription([
        gripper_control,
    ])
