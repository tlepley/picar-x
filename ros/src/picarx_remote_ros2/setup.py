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
            'picarx_video_viewer_node = picarx_remote_ros2.remote_apps.video_viewer_node:main',
        ],
    },
)
