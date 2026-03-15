from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='run_11_video_car_app',
            name='run_11_video_car_app',
            output='screen',
        ),
    ])
