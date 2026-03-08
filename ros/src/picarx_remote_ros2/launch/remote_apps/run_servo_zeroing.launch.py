from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='run_servo_zeroing_app',
            name='run_servo_zeroing_app',
            output='screen',
        ),
    ])
