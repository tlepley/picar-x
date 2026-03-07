from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='picarx_remote_ros2',
            executable='picarx_video_viewer_node',
            name='picarx_video_viewer_node',
            output='screen',
            parameters=[
                {
                    'image_topic': '/picarx/camera/image_raw',
                    'window_name': 'PI-CAR-X Camera',
                    'display_width': 0,
                    'display_height': 0,
                    'start_camera_on_launch': True,
                    'stop_camera_on_exit': True,
                    'camera_start_service': '/picarx_camera_publisher_node/start',
                    'camera_stop_service': '/picarx_camera_publisher_node/stop',
                }
            ],
        ),
    ])
