from setuptools import setup

package_name = 'picarx_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/picarx.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='picar-x user',
    maintainer_email='you@example.com',
    description='ROS 2 Humble integration nodes for PI-CAR-X',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'picarx_node = picarx_ros2.picarx_node:main',
            'picarx_driver_node = picarx_ros2.driver_node:main',
            'picarx_safety_node = picarx_ros2.safety_node:main',
        ],
    },
)
