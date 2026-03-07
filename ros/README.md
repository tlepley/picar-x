# ROS 2 split architecture for PI-CAR-X

This workspace is split into two ROS 2 packages:

- `ros/src/picarx_local_ros2`: hardware-side nodes running on the PI-CAR-X board
- `ros/src/picarx_remote_ros2`: application-side nodes running on a remote ROS host

The recommended reading order is:

1. [`LOCAL_PI_CAR_X.md`](LOCAL_PI_CAR_X.md)
2. [`REMOTE_HOST.md`](REMOTE_HOST.md)

## Quick Summary

The PI-CAR-X board owns the hardware:

- `picarx_local_ros2`
- publishes sensors
- publishes the camera stream on `/picarx/camera/image_raw`
- receives safe motion and camera commands
- stores calibration locally in `~/.config/picar-x/picar-x.conf`
- applies a default local camera tuning profile in `picarx_hardware.launch.py`

The remote machine owns the application logic:

- `picarx_remote_ros2`
- runs converted example applications
- calls calibration services over ROS
- should use the same `ROS_DOMAIN_ID` as the PI-CAR-X
- for multi-machine setups, should preferably use a Fast DDS Discovery Server instead of relying on multicast discovery

## Topic Contract

Remote apps publish commands and local hardware consumes them:

- `/cmd_vel_raw` -> filtered by safety -> `/picarx/cmd_vel`
- `/picarx/speed`
- `/picarx/steering`
- `/picarx/camera_pan`
- `/picarx/camera_tilt`
- `/picarx/stop_request`

Local hardware publishes sensors:

- `/picarx/distance`
- `/picarx/grayscale`
- `/picarx/calibration/state`
- `/picarx/camera/image_raw`

## Camera Stream

The camera stream is published by the local PI-CAR-X side on:

- `/picarx/camera/image_raw`

The local camera node also exposes stream control services:

- `/picarx_camera_publisher_node/start`
- `/picarx_camera_publisher_node/stop`

The default local hardware launch also applies a practical camera tuning profile for `libcamerasrc`:

- auto exposure enabled
- `exposure-value=1.5`
- `awb-enable=true`
- `brightness=0.1`
- `contrast=1.15`

The detailed explanation and tuning notes are documented in [`LOCAL_PI_CAR_X.md`](LOCAL_PI_CAR_X.md).

For a simple remote viewer, use:

```bash
ros2 launch picarx_remote_ros2 view_video_stream.launch.py
```

That viewer requests camera start on launch and camera stop on exit by default.

## Remote Calibration Tools

CLI:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

Optional tkinter GUI:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_gui
```

The GUI requires `_tkinter` in the active Python environment.

## Discovery

Two deployment modes are supported:

- local-only ROS on the PI-CAR-X: standard ROS 2 discovery is enough
- remote host + PI-CAR-X: prefer Fast DDS Discovery Server

The local-only workflow is documented in [`LOCAL_PI_CAR_X.md`](LOCAL_PI_CAR_X.md).

The remote multi-machine workflow, including Discovery Server setup, is documented in [`REMOTE_HOST.md`](REMOTE_HOST.md).
