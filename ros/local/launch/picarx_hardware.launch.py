import asyncio
import select
import sys
import termios
import tty

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueCoroutine
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_key(timeout_sec: float = 0.2) -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if ready:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


async def _keyboard_shutdown_watcher(context) -> None:
    if not sys.stdin.isatty():
        print('Keyboard shutdown disabled: stdin is not a TTY.', flush=True)
        return

    print("Keyboard shutdown enabled: press 'q' to stop the local stack.", flush=True)
    while True:
        key = await asyncio.to_thread(_read_key)
        if key.lower() == 'q':
            print('Stopping local stack...', flush=True)
            await context.emit_event(Shutdown(reason="keyboard request: 'q'"))
            return


def generate_launch_description() -> LaunchDescription:
    enable_embodiment = LaunchConfiguration('enable_embodiment')
    enable_audio = LaunchConfiguration('enable_audio')
    enable_keyboard_shutdown = LaunchConfiguration('enable_keyboard_shutdown')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')

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
        DeclareLaunchArgument(
            'enable_keyboard_shutdown',
            default_value='true',
            description='Whether to enable q-based keyboard shutdown for the local stack.',
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='640',
            description='Camera stream width in pixels.',
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='480',
            description='Camera stream height in pixels.',
        ),
        DeclareLaunchArgument(
            'camera_fps',
            default_value='30.0',
            description='Camera capture framerate.',
        ),
        OpaqueCoroutine(
            coroutine=_keyboard_shutdown_watcher,
            condition=IfCondition(enable_keyboard_shutdown),
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
                    'width': ParameterValue(camera_width, value_type=int),
                    'height': ParameterValue(camera_height, value_type=int),
                    'fps': ParameterValue(camera_fps, value_type=float),
                }
            ],
        ),
    ])
