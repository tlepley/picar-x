# ROS 2 split architecture for PI-CAR-X

This workspace is now split into two explicit domains:

- `ros/src/picarx_local_ros2`: nodes that run on the PI-CAR-X board and access hardware.
- `ros/src/picarx_remote_ros2`: apps/examples that run remotely (Jetson/Ubuntu) and control the car through ROS topics.

The old mixed package `ros/src/picarx_ros2` has been removed from this workspace.

## Build

```bash
cd /home/gotcha/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_local_ros2 picarx_remote_ros2
source install/setup.bash
```

## Run on PI-CAR-X (local/hardware)

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

Or run hardware nodes directly:

```bash
ros2 run picarx_local_ros2 picarx_driver_node
ros2 run picarx_local_ros2 picarx_safety_node
```

## Run on Jetson (remote/apps)

Launch any converted example app:

```bash
ros2 launch picarx_remote_ros2 run_4_avoiding_obstacles.launch.py
```

Other examples follow the same pattern:

```bash
ros2 launch picarx_remote_ros2 run_6_line_tracking.launch.py
ros2 launch picarx_remote_ros2 run_11_video_car.launch.py
```

Generic runner command:

```bash
ros2 run picarx_remote_ros2 picarx_example_runner_node --ros-args \
  -p example_script:=4.avoiding_obstacles.py
```

## Topic contract

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

## Notes

- Keep `picarx` and required runtime dependencies installed in each machine environment as needed.
- Remote examples that use camera/STT/TTS/LLM still require those dependencies on the Jetson side.
