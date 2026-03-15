from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='run_10_bull_fight_app',
            name='run_10_bull_fight_app',
            output='screen',
        ),
    ])
