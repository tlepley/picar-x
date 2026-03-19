from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    engine_path = LaunchConfiguration('engine_path')
    classes_path = LaunchConfiguration('classes_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'engine_path',
            default_value='',
            description='Path to a TensorRT engine (.engine) on the remote Jetson host.',
        ),
        DeclareLaunchArgument(
            'classes_path',
            default_value='',
            description='Optional text file with one class name per line.',
        ),
        LogInfo(msg='Native C++ video-car controls: arrows drive, space stop, +/- speed, p photo, v rec/pause, e rec stop, f detector, q/esc quit.'),
        Node(
            package='picarx_remote_jetson_ros2',
            executable='picarx_remote_yolo_video_car',
            name='picarx_remote_yolo_video_car',
            output='screen',
            parameters=[
                {
                    'engine_path': engine_path,
                    'classes_path': classes_path,
                    'yolo_variant': 'yolov8',
                    'image_topic': '/picarx/camera/image_raw',
                    'camera_start_service': '/picarx_camera_publisher_node/start',
                    'camera_stop_service': '/picarx_camera_publisher_node/stop',
                    'input_width': 640,
                    'input_height': 640,
                    'confidence_threshold': 0.35,
                    'score_threshold': 0.25,
                    'nms_threshold': 0.45,
                    'tracker_iou_threshold': 0.35,
                    'tracker_confirm_frames': 2,
                    'tracker_max_missed_frames': 3,
                    'tracker_smoothing_alpha': 0.65,
                    'tracker_confidence_alpha': 0.7,
                    'detect_every_n_frames': 2,
                    'base_speed': 20.0,
                    'max_speed': 60.0,
                    'turn_angle_deg': 28.0,
                    'show_fps': True,
                }
            ],
        ),
    ])
