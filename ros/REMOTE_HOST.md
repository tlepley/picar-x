# Remote Host Setup

This guide is for the remote ROS machine that runs the application logic and controls the PI-CAR-X over ROS.

The remote host can be any machine that runs ROS 2 Humble correctly. In practice, Ubuntu is usually the best-integrated choice.

It covers:

- installing ROS 2 Humble on Ubuntu
- building the remote PI-CAR-X package
- configuring DDS domain settings
- setting up a Fast DDS Discovery Server for reliable multi-machine discovery
- verifying connectivity with the PI-CAR-X board
- viewing the camera stream remotely
- launching obstacle avoidance remotely

## Install ROS 2 Humble on Ubuntu

Use the standard Ubuntu + apt workflow for ROS 2 Humble.

Reference:

- <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

After ROS is installed:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

## Clone and build the remote package

Clone this repository on the remote host, then build the remote ROS package:

```bash
cd ~
git clone <YOUR_GIT_REMOTE> picar-x
cd ~/picar-x/ros
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_remote_ros2
source install/setup.bash
```

## Configure the DDS domain

The remote host must use the same ROS domain as the PI-CAR-X board:

```bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
```

## Start a Fast DDS Discovery Server

ROS 2 multicast discovery may not work reliably on every local network. For a two-machine setup, the recommended approach is to run a Fast DDS Discovery Server on the remote host.

Start the server on the remote host:

```bash
fastdds discovery --server-id 0
```

Then configure the remote ROS shell to use it:

```bash
cd ~/picar-x/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
```

The PI-CAR-X local shell must use the same `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, and `ROS_DISCOVERY_SERVER`.

## Verify connectivity with the PI-CAR-X board

Check domain-related environment:

```bash
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
```

Verify that the remote host sees the PI-CAR-X nodes:

```bash
ros2 node list
```

Expected nodes:

- `/picarx_driver_node`
- `/picarx_safety_node`

If those nodes are not visible, do not launch the remote app yet. The problem is DDS discovery or network configuration, not the application package.

## Verify calibration service visibility

The remote host should also see the calibration services published by the PI-CAR-X local stack:

```bash
ros2 service list | grep calibration
```

Expected services:

- `/picarx/calibration/get`
- `/picarx/calibration/save`
- `/picarx/calibration/load`
- `/picarx/calibration/reset`

Quick remote calibration check:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

## View the camera stream remotely

The PI-CAR-X camera stream is published on:

- `/picarx/camera/image_raw`

The local camera node also exposes stream control services:

- `/picarx_camera_publisher_node/start`
- `/picarx_camera_publisher_node/stop`

The simplest remote viewer is:

```bash
cd ~/picar-x/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
ros2 launch picarx_remote_ros2 view_video_stream.launch.py
```

By default, `view_video_stream.launch.py` requests camera start when the viewer launches and requests camera stop when it exits.

## Launch obstacle avoidance remotely

Once connectivity is confirmed:

```bash
cd ~/picar-x/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
ros2 launch picarx_remote_ros2 run_4_avoiding_obstacles.launch.py
```

## Notes

- Replace `<REMOTE_HOST_IP>` with the IPv4 address of the machine running the Discovery Server.
- Remote examples that depend on camera, STT, TTS, or LLM tooling also require those dependencies on the remote host.
- The optional tkinter calibration GUI requires `_tkinter` support in the active Python environment.
