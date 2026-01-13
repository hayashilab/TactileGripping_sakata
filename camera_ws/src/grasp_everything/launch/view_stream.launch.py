from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # grasp_everything 側
        ExecuteProcess(
            cmd=['ros2', 'run', 'grasp_everything', 'stream_ros2'],
            output='screen'
        ),
        # rqt_image_view 側（GUIなのでExecuteProcessで起動するのが無難）
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        ),
    ])
