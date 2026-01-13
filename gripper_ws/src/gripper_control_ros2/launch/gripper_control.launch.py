from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control_ros2',
            executable='gripper_control_node',
            name='gripper_control',
            output='screen',
            parameters=[
                {'port_stepper': '/dev/ttyUSB_Gripper'},
                {'baudrate_stepper': 9600},
                {'speed_rpm': 3000},
                {'cmd_timeout': 10},
                {'max_mm': 130},
            ],
        ),
    ])
