"""
RealSenseカメラ + Softness Classifier を同時起動するlaunchファイル

Usage:
  ros2 launch clip_cam realsense_softness.launch.py
  ros2 launch clip_cam realsense_softness.launch.py inference_rate:=10.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージのソースディレクトリ（.venvがある場所）
    src_package_dir = '/home/hayashi/worksp/camera_ws/src/clip_cam'

    # === Launch Arguments ===

    # RealSense関連
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='false',
        description='Enable depth stream'
    )

    rgb_profile_arg = DeclareLaunchArgument(
        'rgb_camera.profile',
        default_value='640x480x30',
        description='RGB camera profile (WxHxFPS)'
    )

    # Softness Classifier関連
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='ViT-B-32',
        description='OpenCLIP model name'
    )

    pretrained_arg = DeclareLaunchArgument(
        'pretrained',
        default_value='openai',
        description='Pretrained weights'
    )

    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='30.0',
        description='Inference rate in Hz'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to use (cuda or cpu)'
    )

    # === RealSense Camera Launch ===
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'rgb_camera.profile': LaunchConfiguration('rgb_camera.profile'),
        }.items()
    )

    # === Softness Classifier Node ===
    softness_classifier_node = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'source {src_package_dir}/.venv/bin/activate && '
            f'source /home/hayashi/worksp/camera_ws/install/setup.bash && '
            f'python -m clip_cam.softness_classifier_node'
        ],
        name='softness_classifier',
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        enable_color_arg,
        enable_depth_arg,
        rgb_profile_arg,
        model_name_arg,
        pretrained_arg,
        inference_rate_arg,
        device_arg,
        # Nodes
        realsense_launch,
        softness_classifier_node,
    ])
