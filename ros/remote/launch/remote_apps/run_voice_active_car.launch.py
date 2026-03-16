from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        LogInfo(
            msg=(
                'run_voice_active_car.launch.py starts an interactive app without a usable stdin in ros2 launch. '
                'Use `ros2 run picarx_remote_ros2 picarx_voice_active_car_app` from a terminal instead.'
            )
        ),
        Node(
            package='picarx_remote_ros2',
            executable='picarx_voice_active_car_app',
            name='picarx_voice_active_car_app',
            output='screen',
        ),
    ])
