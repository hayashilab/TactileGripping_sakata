from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='gripper_control',
            name='gripper_control',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='gripper_control',
            executable='gripper_utils',
            name='gripper_utils',
            output='screen',
            emulate_tty=True,
        ),
    ])