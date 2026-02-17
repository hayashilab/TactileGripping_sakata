"""
Bringup launch for data collection.

Starts:
- GelSight stream node
- Gripper control node
- RealSense camera

Usage:
  ros2 launch grasp_everything data_collection_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable RealSense color stream',
    )
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='false',
        description='Enable RealSense depth stream',
    )
    rgb_profile_arg = DeclareLaunchArgument(
        'rgb_camera.color_profile',
        default_value='640x480x30',
        description='RealSense RGB profile (widthxheightxfps)',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'rgb_camera.color_profile': LaunchConfiguration('rgb_camera.color_profile'),
        }.items(),
    )

    gelsight_stream = Node(
        package='grasp_everything',
        executable='stream_ros2',
        name='tactile_stream',
        output='screen',
    )

    gripper_control = Node(
        package='gripper_control',
        executable='gripper_control',
        name='gripper_control',
        output='screen',
    )

    return LaunchDescription([
        enable_color_arg,
        enable_depth_arg,
        rgb_profile_arg,
        gelsight_stream,
        gripper_control,
        realsense_launch,
    ])
