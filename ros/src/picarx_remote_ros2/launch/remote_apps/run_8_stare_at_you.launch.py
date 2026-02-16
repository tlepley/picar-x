from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='picarx_example_runner_node',
            name='picarx_example_runner',
            output='screen',
            parameters=[
                {
                    'example_script': '8.stare_at_you.py',
                    'pass_ros_args': False,
                }
            ],
        ),
    ])
