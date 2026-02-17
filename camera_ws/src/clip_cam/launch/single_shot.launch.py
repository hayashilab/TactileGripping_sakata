"""
Single shot softness classifier launch file
1枚撮影 → 判定 → 結果表示 → 終了
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # パッケージのソースディレクトリ（.venvがある場所）
    src_package_dir = '/home/hayashi/worksp/camera_ws/src/clip_cam'

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

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to use (cuda or cpu)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='Input image topic'
    )

    save_image_arg = DeclareLaunchArgument(
        'save_image',
        default_value='true',
        description='Save result image'
    )

    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='/tmp/softness_result.png',
        description='Output image path'
    )

    # uv仮想環境のPythonで直接モジュールを実行
    single_shot_node = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'source {src_package_dir}/.venv/bin/activate && '
            f'source /home/hayashi/worksp/camera_ws/install/setup.bash && '
            f'python -m clip_cam.single_shot_classifier_node'
        ],
        name='single_shot_classifier',
        output='screen',
    )

    return LaunchDescription([
        model_name_arg,
        pretrained_arg,
        device_arg,
        image_topic_arg,
        save_image_arg,
        output_path_arg,
        single_shot_node,
    ])
