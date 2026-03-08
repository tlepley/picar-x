# PI-CAR-X ROS 2 Interface Reference

This page documents the ROS 2 interface exposed by the project: nodes, topics,
services, and practical commands to test each interface.

It complements:

1. `LOCAL_PI_CAR_X.md` for the PI-CAR-X machine
2. `REMOTE_HOST.md` for the remote machine

## Environment reminder

Before running ROS commands, use the same environment in the relevant shells:

```bash
source ~/ros2_humble/install/setup.bash
cd ~/git/picar-x/ros
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
```

Note: when using a DDS Discovery Server, `ros2 service list` or `ros2 node info`
may appear incomplete. That does not prove that a service is missing.

## Main local launch

Command:

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

Expected nodes:

- `/picarx_driver_node`
- `/picarx_safety_node`
- `/picarx_embodiment_node`
- `/picarx_camera_publisher_node`

Verification:

```bash
ros2 node list
```

## Node `/picarx_driver_node`

Role:

- sole owner of the PI-CAR-X hardware
- applies drive and camera commands
- publishes sensor data
- exposes calibration services

### Subscribed topics

- `/picarx/cmd_vel` - `geometry_msgs/msg/Twist`
- `/picarx/stop` - `std_msgs/msg/Empty`
- `/picarx/speed` - `std_msgs/msg/Float32`
- `/picarx/steering` - `std_msgs/msg/Float32`
- `/picarx/camera_pan` - `std_msgs/msg/Float32`
- `/picarx/camera_tilt` - `std_msgs/msg/Float32`
- `/picarx/calibration/servo_offsets` - `std_msgs/msg/Float32MultiArray`
- `/picarx/calibration/motor_directions` - `std_msgs/msg/Int32MultiArray`

### Published topics

- `/picarx/distance` - `std_msgs/msg/Float32`
- `/picarx/grayscale` - `std_msgs/msg/Float32MultiArray`
- `/picarx/calibration/state` - `std_msgs/msg/Float32MultiArray`

### Published services

- `/picarx/calibration/save` - `std_srvs/srv/Trigger`
- `/picarx/calibration/load` - `std_srvs/srv/Trigger`
- `/picarx/calibration/reset` - `std_srvs/srv/Trigger`
- `/picarx/calibration/get` - `std_srvs/srv/Trigger`

### Test commands

Immediate stop:

```bash
ros2 topic pub --once /picarx/stop std_msgs/msg/Empty "{}"
```

Simple speed command:

```bash
ros2 topic pub --once /picarx/speed std_msgs/msg/Float32 "{data: 20.0}"
```

Steering command:

```bash
ros2 topic pub --once /picarx/steering std_msgs/msg/Float32 "{data: 10.0}"
```

Camera pan command:

```bash
ros2 topic pub --once /picarx/camera_pan std_msgs/msg/Float32 "{data: 20.0}"
```

Camera tilt command:

```bash
ros2 topic pub --once /picarx/camera_tilt std_msgs/msg/Float32 "{data: -10.0}"
```

Read distance:

```bash
ros2 topic echo /picarx/distance --once
```

Read calibration:

```bash
ros2 service call /picarx/calibration/get std_srvs/srv/Trigger "{}"
```

Save calibration:

```bash
ros2 service call /picarx/calibration/save std_srvs/srv/Trigger "{}"
```

Reset calibration:

```bash
ros2 service call /picarx/calibration/reset std_srvs/srv/Trigger "{}"
```

## Node `/picarx_safety_node`

Role:

- receives raw driving commands
- blocks forward motion when an obstacle is too close
- republishes a safe command to the driver

### Subscribed topics

- `/cmd_vel_raw` - `geometry_msgs/msg/Twist`
- `/picarx/distance` - `std_msgs/msg/Float32`
- `/picarx/stop_request` - `std_msgs/msg/Empty`

### Published topics

- `/picarx/cmd_vel` - `geometry_msgs/msg/Twist`
- `/picarx/stop` - `std_msgs/msg/Empty`

### Test commands

Raw velocity command:

```bash
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Stop request:

```bash
ros2 topic pub --once /picarx/stop_request std_msgs/msg/Empty "{}"
```

## Node `/picarx_embodiment_node`

Role:

- keeps PI-side `robot_hat` features off the remote machine
- drives the simple LED
- plays local sound effects

### Subscribed topics

- `/picarx/embodiment/led` - `std_msgs/msg/String`
- `/picarx/embodiment/sound` - `std_msgs/msg/String`

### LED commands

Supported values:

- `on`
- `off`
- `blink_once`
- `blink_fast`

Tests:

```bash
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'on'}"
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'off'}"
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'blink_once'}"
```

Note: this currently controls the simple `robot_hat.led.LED` device, not yet a
multicolor RGB LED interface.

### Sound commands

Supported values:

- `honking`
- `start engine`

Tests:

```bash
ros2 topic pub --once /picarx/embodiment/sound std_msgs/msg/String "{data: 'honking'}"
ros2 topic pub --once /picarx/embodiment/sound std_msgs/msg/String "{data: 'start engine'}"
```

If the node logs `Sound command received: ...` but no actual audio is heard,
the ROS path is working and the issue is on the local ALSA / speaker /
`robot_hat` audio side.

## Node `/picarx_camera_publisher_node`

Role:

- publishes the local camera stream
- exposes start/stop services for streaming

### Published topics

- `/picarx/camera/image_raw` - `sensor_msgs/msg/Image`

### Published services

- `/picarx_camera_publisher_node/start` - `std_srvs/srv/Trigger`
- `/picarx_camera_publisher_node/stop` - `std_srvs/srv/Trigger`

### Test commands

Start the stream:

```bash
ros2 service call /picarx_camera_publisher_node/start std_srvs/srv/Trigger "{}"
```

Stop the stream:

```bash
ros2 service call /picarx_camera_publisher_node/stop std_srvs/srv/Trigger "{}"
```

Check the image topic:

```bash
ros2 topic list | grep picarx/camera
ros2 topic hz /picarx/camera/image_raw
```

## Useful remote-side tools

These tools usually run on the Jetson or other remote machine.

### Calibration CLI

Command:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

Usage:

- reads `/picarx/calibration/state`
- calls `/picarx/calibration/get`
- can call `save`, `load`, and `reset`

### Calibration GUI

Command:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_gui
```

### Camera viewer

Command:

```bash
ros2 launch picarx_remote_ros2 view_video_stream.launch.py
```

This app uses:

- `/picarx/camera/image_raw`
- `/picarx_camera_publisher_node/start`
- `/picarx_camera_publisher_node/stop`

### Split voice-active-car app

Interactive command:

```bash
ros2 run picarx_remote_ros2 picarx_voice_active_car_app
```

Important:

- do not use `ros2 launch ... run_voice_active_car.launch.py` for interaction
- this app requires a real interactive terminal

Interfaces used:

- `/cmd_vel_raw`
- `/picarx/stop_request`
- `/picarx/speed`
- `/picarx/steering`
- `/picarx/camera_pan`
- `/picarx/camera_tilt`
- `/picarx/distance`
- `/picarx/grayscale`
- `/picarx/embodiment/led`
- `/picarx/embodiment/sound`

## Quick command summary

Launch local hardware:

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

Show nodes:

```bash
ros2 node list
```

Show calibration:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

Start the camera:

```bash
ros2 service call /picarx_camera_publisher_node/start std_srvs/srv/Trigger "{}"
```

Test the LED:

```bash
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'blink_once'}"
```

Test a sound:

```bash
ros2 topic pub --once /picarx/embodiment/sound std_msgs/msg/String "{data: 'honking'}"
```
