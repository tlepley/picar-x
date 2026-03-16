from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enable_embodiment = LaunchConfiguration('enable_embodiment')
    enable_audio = LaunchConfiguration('enable_audio')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_embodiment',
            default_value='true',
            description='Whether to start the local embodiment node (LED/sound helpers).',
        ),
        DeclareLaunchArgument(
            'enable_audio',
            default_value='false',
            description='Whether the embodiment node should initialize and use local audio.',
        ),
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
                    'direction_servo_pin': 'P3',
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
        Node(
            package='picarx_local_ros2',
            executable='picarx_embodiment_node',
            name='picarx_embodiment_node',
            output='screen',
            condition=IfCondition(enable_embodiment),
            parameters=[
                {
                    'enable_audio': enable_audio,
                }
            ],
        ),
        Node(
            package='picarx_local_ros2',
            executable='picarx_camera_publisher_node',
            name='picarx_camera_publisher_node',
            output='screen',
            parameters=[
                {
                    'camera_source': '',
                    'camera_backend': 'auto',
                    'gstreamer_pipeline': '',
                    'camera_auto_exposure': True,
                    'camera_controls': 'exposure-value=1.5 awb-enable=true brightness=0.1 contrast=1.15',
                    'start_stream_on_launch': False,
                    'image_topic': '/picarx/camera/image_raw',
                    'frame_id': 'picarx_camera',
                    'publish_rate_hz': 10.0,
                    'width': 640,
                    'height': 480,
                    'fps': 30.0,
                }
            ],
        ),
    ])
