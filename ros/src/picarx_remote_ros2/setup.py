from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'picarx_remote_ros2'


def collect_launch_data_files():
    launch_root = Path('launch')
    data = []
    for directory in sorted({p.parent for p in launch_root.rglob('*.py')}):
        files = sorted(str(p) for p in directory.glob('*.py'))
        if files:
            rel_dir = str(directory).replace('launch', 'launch', 1)
            data.append((f'share/{package_name}/{rel_dir}', files))
    return data


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/examples', glob('../../../example/*.py')),
        (f'share/{package_name}/sounds', glob('../../../sounds/*')),
        (f'share/{package_name}/musics', glob('../../../musics/*')),
    ] + collect_launch_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='picar-x user',
    maintainer_email='you@example.com',
    description='PI-CAR-X remote apps ROS 2 package',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'picarx_example_runner_node = picarx_remote_ros2.remote_apps.example_runner_node:main',
            'picarx_calibration_cli = picarx_remote_ros2.remote_apps.calibration_cli:main',
            'picarx_calibration_gui = picarx_remote_ros2.remote_apps.calibration_gui:main',
            'run_7_computer_vision_app = picarx_remote_ros2.remote_apps.computer_vision_app:main',
            'run_8_stare_at_you_app = picarx_remote_ros2.remote_apps.stare_at_you_app:main',
            'run_9_record_video_app = picarx_remote_ros2.remote_apps.record_video_app:main',
            'run_10_bull_fight_app = picarx_remote_ros2.remote_apps.bull_fight_app:main',
            'run_11_video_car_app = picarx_remote_ros2.remote_apps.video_car_app:main',
            'run_17_text_vision_talk_app = picarx_remote_ros2.remote_apps.text_vision_talk_app:main',
            'run_20_treasure_hunt_app = picarx_remote_ros2.remote_apps.treasure_hunt_app:main',
            'run_1_cali_grayscale_app = picarx_remote_ros2.remote_apps.grayscale_calibration_app:main',
            'run_1_cali_servo_motor_app = picarx_remote_ros2.remote_apps.servo_motor_calibration_app:main',
            'run_servo_zeroing_app = picarx_remote_ros2.remote_apps.servo_zeroing_cli:main',
            'picarx_video_viewer_node = picarx_remote_ros2.remote_apps.video_viewer_node:main',
            'run_3_keyboard_control_app = picarx_remote_ros2.remote_apps.keyboard_control_app:main',
            'picarx_voice_active_car_app = picarx_remote_ros2.remote_apps.voice_active_car_app:main',
        ],
    },
)
