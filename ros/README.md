# ROS 2 Humble integration for PI-CAR-X

This folder contains a local ROS 2 workspace with a clean node architecture:

- `picarx_driver_node`: the **only** node that instantiates `picarx.Picarx` and touches hardware.
- `picarx_safety_node`: command filtering/safety logic without hardware access.

This avoids multiple `Picarx()` instances and concurrent GPIO/PWM access.

## Workspace layout

- `ros/src/picarx_ros2`: ROS 2 Python package (`ament_python`)

## Prerequisites

- ROS 2 Humble installed and sourced from:

```bash
source ~/ros2_humble/install/setup.bash
```

- User must be in hardware access groups required by PI-CAR-X control:

```bash
sudo usermod -aG gpio,i2c,spi,dialout $USER
```

Then log out/log back in (or reboot), and verify:

```bash
groups
```

- `picarx` package importable in the **same Python env** used by ROS:

```bash
python3 -c "import sys; print(sys.executable)"
cd /home/gotcha/git/picar-x
python3 -m pip install -e .
```

- `robot_hat` must be SunFounder-compatible (`Pin, ADC, PWM, Servo, fileDB`):

```bash
python3 -m pip install --no-cache-dir --force-reinstall \
  "git+https://github.com/sunfounder/robot-hat.git@2.5.x"
```

## Build

```bash
cd /home/gotcha/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_ros2
source install/setup.bash
```

`PYTHONNOUSERSITE=1` avoids user-site package conflicts (e.g. old `nose` with Python 3.13).

## Run

Run as normal user (no `sudo`) on this setup.

Start complete stack (recommended):

```bash
ros2 launch picarx_ros2 picarx.launch.py
```

Or run nodes separately:

```bash
ros2 run picarx_ros2 picarx_driver_node
ros2 run picarx_ros2 picarx_safety_node
```

Compatibility entry point (driver only):

```bash
ros2 run picarx_ros2 picarx_node
```

## Topic flow

Main command flow:

- Input command: `/cmd_vel_raw` (`geometry_msgs/msg/Twist`)
- Safety-filtered output: `/picarx/cmd_vel` (`geometry_msgs/msg/Twist`)
- Driver consumes `/picarx/cmd_vel`

Safety node also accepts:

- `/picarx/stop_request` (`std_msgs/msg/Empty`) -> publishes `/picarx/stop`

Driver node subscriptions:

- `/picarx/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/picarx/stop` (`std_msgs/msg/Empty`)
- `/picarx/speed` (`std_msgs/msg/Float32`)
- `/picarx/steering` (`std_msgs/msg/Float32`)
- `/picarx/camera_pan` (`std_msgs/msg/Float32`)
- `/picarx/camera_tilt` (`std_msgs/msg/Float32`)

Driver node publications:

- `/picarx/distance` (`std_msgs/msg/Float32`)
- `/picarx/grayscale` (`std_msgs/msg/Float32MultiArray`)

Driver parameter:

- `config_path` (default `~/.config/picar-x/picar-x.conf`): writable calibration/config file path used by `Picarx`.
- `ultrasonic_trig_pin` (default `D0`): ultrasonic trigger pin.
- `ultrasonic_echo_pin` (default `D1`): ultrasonic echo pin.

## Quick checks

Publish command (goes through safety):

```bash
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist \
  '{linear: {x: 40.0}, angular: {z: 0.2}}' -r 5
```

Emergency stop request:

```bash
ros2 topic pub /picarx/stop_request std_msgs/msg/Empty '{}' --once
```

Read sensors:

```bash
ros2 topic echo /picarx/distance
ros2 topic echo /picarx/grayscale
```

## Troubleshooting

`ros2 launch` missing:

- Build ROS with `ros2launch`, `launch`, and `launch_ros` included.

`ModuleNotFoundError: No module named picarx`:

- Install this repository in the active ROS Python:

```bash
cd /home/gotcha/git/picar-x
python3 -m pip install -e .
```

`ImportError` from `robot_hat` (e.g. missing `ADC`):

- Wrong package/version installed. Reinstall SunFounder `robot-hat` (commands above).

`PermissionError: /opt/picar-x/picar-x.conf`:

- Use user-writable config path (`config_path`), default already set to `~/.config/picar-x/picar-x.conf` in this ROS package.

Ultrasonic always `-1`:

- Means invalid reading/timeout at hardware layer (not ROS).
- With strict safety enabled, forward motion can be blocked if distance is considered unsafe.
- On this setup, the working mapping is `D0`/`D1` (not `D2`/`D3`).
- Test sensor directly:

```bash
python3 - <<'PY'
from picarx import Picarx
px = Picarx(ultrasonic_pins=['D0','D1'], config='/home/gotcha/.config/picar-x/picar-x.conf')
for i in range(20):
    print(i, px.get_distance())
PY
```
