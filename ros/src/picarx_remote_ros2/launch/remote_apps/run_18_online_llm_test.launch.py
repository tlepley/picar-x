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
                    'example_script': '18.online_llm_test.py',
                    'pass_ros_args': False,
                }
            ],
        ),
    ])
