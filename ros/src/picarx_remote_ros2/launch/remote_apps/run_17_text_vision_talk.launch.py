from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='run_17_text_vision_talk_app',
            name='run_17_text_vision_talk_app',
            output='screen',
        ),
    ])
