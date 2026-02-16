from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_local_ros2',
            executable='picarx_driver_node',
            name='picarx_driver_node',
            output='screen',
            parameters=[
                {
                    'max_speed': 100.0,
                    'max_steering_deg': 30.0,
                    'steering_gain_deg_per_rad_s': 30.0,
                    'cmd_timeout_sec': 0.6,
                    'sensor_rate_hz': 10.0,
                    'config_path': '~/.config/picar-x/picar-x.conf',
                    'ultrasonic_trig_pin': 'D0',
                    'ultrasonic_echo_pin': 'D1',
                }
            ],
        ),
        Node(
            package='picarx_local_ros2',
            executable='picarx_safety_node',
            name='picarx_safety_node',
            output='screen',
            parameters=[
                {
                    'stop_distance_m': 0.2,
                    'distance_timeout_sec': 1.0,
                    'cmd_timeout_sec': 0.6,
                    'publish_rate_hz': 20.0,
                }
            ],
        ),
    ])
