from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        LogInfo(msg='run_7_computer_vision: click the OpenCV window to give it keyboard focus.'),
        LogInfo(msg='run_7 controls:'),
        LogInfo(msg='  q: take photo'),
        LogInfo(msg='  1: detect red'),
        LogInfo(msg='  2: detect orange'),
        LogInfo(msg='  3: detect yellow'),
        LogInfo(msg='  4: detect green'),
        LogInfo(msg='  5: detect blue'),
        LogInfo(msg='  6: detect purple'),
        LogInfo(msg='  0: disable color detection'),
        LogInfo(msg='  f: toggle face detection'),
        LogInfo(msg='  c: toggle cat detection'),
        LogInfo(msg='  r: toggle QR detection'),
        LogInfo(msg='  s: print current detection info'),
        LogInfo(msg='  h: print help in the app terminal'),
        LogInfo(msg='  ESC: quit'),
        Node(
            package='picarx_remote_ros2',
            executable='run_7_computer_vision_app',
            name='run_7_computer_vision_app',
            output='screen',
        ),
    ])
