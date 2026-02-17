#!/usr/bin/env python3
"""
Crush Detector Launch File

起動例:
    ros2 launch grasp_everything crush_detector.launch.py

引数例:
    ros2 launch grasp_everything crush_detector.launch.py \
        model_path:=/path/to/best.pt threshold:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # デフォルトモデルパス
    default_model_path = "runs/crush_detector/crush_resnet18_20260125_013100/best.pt"

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to the trained model weights (.pt file)'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='resnet18',
            description='Model architecture name (cnn, vgg16, resnet18, efficientnet)'
        ),
        DeclareLaunchArgument(
            'image_size',
            default_value='128',
            description='Input image size'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.5',
            description='Classification threshold (0-1)'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Device to run inference on (cuda, cpu, or empty for auto)'
        ),

        # Crush detector node
        Node(
            package='grasp_everything',
            executable='crush_detector',
            name='crush_detector_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'model_name': LaunchConfiguration('model_name'),
                'image_size': LaunchConfiguration('image_size'),
                'threshold': LaunchConfiguration('threshold'),
                'device': LaunchConfiguration('device'),
            }],
        ),
    ])
