# PI-CAR-X ROS Fork

This repository is a fork of the SunFounder PI-CAR-X 2.1.x codebase.

The goal of this fork is to integrate PI-CAR-X into a ROS 2 environment while preserving the original Python project structure and examples. The original SunFounder examples are still kept in their original directories. They were not removed or rewritten in place. Instead, they were adapted for ROS usage through the ROS packages under [`ros/`](./ros).

## What This Fork Adds

- A ROS 2 local hardware package for the PI-CAR-X board
- A ROS 2 remote application package for external compute nodes such as an NVIDIA Jetson
- ROS-converted execution paths for the original examples
- Remote calibration and remote control workflows over ROS topics and services

The ROS workspace is documented in [`ros/README.md`](ros/README.md).

## Project Layout

- `example/`: original SunFounder example scripts, preserved
- `picarx/`: original Python library codebase
- `ros/src/picarx_local_ros2`: ROS 2 package for hardware-side nodes running on the PI-CAR-X board
- `ros/src/picarx_remote_ros2`: ROS 2 package for remote applications running on another machine

## Using The Original SunFounder Version

If you want the original non-ROS usage, keep using the repository as a standard SunFounder-style Python project.

Typical workflow:

```bash
cd ~/git/picar-x
python3 -m pip install -e .
python3 example/4.avoiding_obstacles.py
```

This mode is useful if you want to run the original examples directly on the PI-CAR-X without ROS.

## Using The ROS Version

If you want PI-CAR-X to operate as part of a ROS 2 system:

- Run `picarx_local_ros2` on the PI-CAR-X board
- Run `picarx_remote_ros2` on the remote machine that hosts the application logic

Typical hardware-side workflow on the PI-CAR-X:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_local_ros2
source install/setup.bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

Typical remote-side workflow on the Jetson:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
export ROS_DOMAIN_ID=99
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_remote_ros2
source install/setup.bash
ros2 launch picarx_remote_ros2 run_4_avoiding_obstacles.launch.py
```

Adjust `ROS_DOMAIN_ID` to match the PI-CAR-X side.

## Why The Original Examples Were Preserved

The original examples remain useful for:

- validating SunFounder-style direct hardware behavior
- comparing original behavior with ROS-based behavior
- debugging hardware independently from ROS

The ROS packages do not replace the original example files. They provide a ROS execution layer around them so the project can be used both as:

- the original SunFounder Python project
- a ROS 2-integrated robotics platform

## Upstream Reference

- SunFounder documentation: <https://docs.sunfounder.com/projects/picar-x-v20/en/latest/>
- Robot Hat documentation: <https://docs.sunfounder.com/projects/robot-hat-v4/en/latest/>
- SunFounder forum: <https://forum.sunfounder.com/>
- SunFounder website: <https://www.sunfounder.com/>

## License

This repository remains under the original upstream license terms unless stated otherwise in specific files.
