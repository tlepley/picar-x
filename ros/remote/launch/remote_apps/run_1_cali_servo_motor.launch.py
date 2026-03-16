from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'script',
                '-qec',
                'ros2 run picarx_remote_ros2 run_1_cali_servo_motor_app',
                '/dev/null',
            ],
            output='screen',
        ),
    ])
