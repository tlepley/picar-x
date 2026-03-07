from glob import glob
from setuptools import find_packages, setup

package_name = 'picarx_local_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='picar-x user',
    maintainer_email='you@example.com',
    description='PI-CAR-X local hardware ROS 2 package',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'picarx_node = picarx_local_ros2.picarx_node:main',
            'picarx_driver_node = picarx_local_ros2.driver_node:main',
            'picarx_safety_node = picarx_local_ros2.safety_node:main',
            'picarx_camera_control_cli = picarx_local_ros2.camera_control_cli:main',
        ],
    },
)
