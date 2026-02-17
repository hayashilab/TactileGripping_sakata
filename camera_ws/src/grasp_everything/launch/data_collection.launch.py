"""
データ収集用 Launch ファイル

以下のノードを起動:
- RealSense camera
- GelSight stream (tactile)
- Gripper control
- Data collection CLI

Usage:
  ros2 launch grasp_everything data_collection.launch.py object_id:=S1 condition:=A
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    # Declare arguments
    object_id_arg = DeclareLaunchArgument(
        'object_id',
        default_value='S1',
        description='Object ID (S1-S5 for soft, H1-H5 for hard)'
    )
    
    condition_arg = DeclareLaunchArgument(
        'condition',
        default_value='A',
        description='Experiment condition (A, B, C, D)'
    )
    
    delta_step_arg = DeclareLaunchArgument(
        'delta_step_level',
        default_value='weak',
        description='Delta step level (weak, mid, strong)'
    )
    
    disturbance_arg = DeclareLaunchArgument(
        'disturbance',
        default_value='none',
        description='Disturbance type (none, pull)'
    )
    
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='data_raw',
        description='Data directory path'
    )
    
    grasp_target_arg = DeclareLaunchArgument(
        'grasp_target_mm',
        default_value='15.0',
        description='Target gripper position (mm)'
    )
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'pointcloud.enable': 'false',
            'enable_rgbd': 'false',
            'enable_sync': 'true',
            'align_depth.enable': 'false',
            'enable_color': 'true',
            'enable_depth': 'false',
            'rgb_camera.color_profile': '640x480x30',
        }.items()
    )
    
    # Data collection CLI node
    data_collection_cli = Node(
        package='grasp_everything',
        executable='data_collection_cli',
        name='data_collection_cli',
        output='screen',
        parameters=[{
            'object_id': LaunchConfiguration('object_id'),
            'condition': LaunchConfiguration('condition'),
            'delta_step_level': LaunchConfiguration('delta_step_level'),
            'disturbance': LaunchConfiguration('disturbance'),
            'data_dir': LaunchConfiguration('data_dir'),
            'grasp_target_mm': LaunchConfiguration('grasp_target_mm'),
        }],
    )
    
    return LaunchDescription([
        object_id_arg,
        condition_arg,
        delta_step_arg,
        disturbance_arg,
        data_dir_arg,
        grasp_target_arg,
        realsense_launch,
        data_collection_cli,
    ])
