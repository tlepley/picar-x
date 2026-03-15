from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='run_8_stare_at_you_app',
            name='run_8_stare_at_you_app',
            output='screen',
        ),
    ])
