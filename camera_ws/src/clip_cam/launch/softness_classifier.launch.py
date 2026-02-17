"""
OpenCLIP softness classifier launch file
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # パッケージのソースディレクトリ（.venvがある場所）
    src_package_dir = '/home/hayashi/worksp/camera_ws/src/clip_cam'
    venv_python = os.path.join(src_package_dir, '.venv', 'bin', 'python')

    # Launch arguments
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
        default_value='5.0',
        description='Inference rate in Hz'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to use (cuda or cpu)'
    )

    # uv仮想環境のPythonで直接モジュールを実行
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
        model_name_arg,
        pretrained_arg,
        inference_rate_arg,
        device_arg,
        softness_classifier_node,
    ])
