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
- `/picarx_embodiment_node`

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

Note: with DDS discovery through a Discovery Server, `ros2 service list` or `ros2 node info` can still
look empty even when the services are reachable from a real ROS client. In that case, use the calibration
CLI below as the real connectivity check.

Quick remote calibration check:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

## Choose `ros2 launch` vs `ros2 run`

Use `ros2 launch` for remote applications that behave like autonomous ROS apps:

- they start, run their own loop, and do not require live keyboard input
- they may launch one or more supporting processes
- they fit the usual ROS "application launch" model

Use `ros2 run` for remote tools that are interactive:

- CLI tools
- GUI tools
- keyboard-driven apps
- apps that expect a real terminal or direct user input

In this project, this rule is important: `ros2 launch` is not a reliable way to
run a keyboard-interactive app on the remote side.

## Remote application list

### Use `ros2 run` for interactive remote tools

Calibration CLI:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

Calibration GUI:

```bash
ros2 run picarx_remote_ros2 picarx_calibration_gui
```

Grayscale calibration:

```bash
ros2 run picarx_remote_ros2 run_1_cali_grayscale_app
```

Servo and motor calibration:

```bash
ros2 run picarx_remote_ros2 run_1_cali_servo_motor_app
```

Keyboard control:

```bash
ros2 run picarx_remote_ros2 run_3_keyboard_control_app
```

Voice-active-car top-level app:

```bash
ros2 run picarx_remote_ros2 picarx_voice_active_car_app
```

### Use `ros2 launch` for autonomous or viewer-style remote apps

Camera stream viewer:

```bash
ros2 launch picarx_remote_ros2 view_video_stream.launch.py
```

Converted example launches that are currently best treated as ROS app launches:

```bash
ros2 launch picarx_remote_ros2 run_2_move.launch.py
ros2 launch picarx_remote_ros2 run_4_avoiding_obstacles.launch.py
ros2 launch picarx_remote_ros2 run_5_cliff_detection.launch.py
ros2 launch picarx_remote_ros2 run_6_line_tracking.launch.py
ros2 launch picarx_remote_ros2 run_7_computer_vision.launch.py
ros2 launch picarx_remote_ros2 run_8_stare_at_you.launch.py
ros2 launch picarx_remote_ros2 run_9_record_video.launch.py
ros2 launch picarx_remote_ros2 run_10_bull_fight.launch.py
ros2 launch picarx_remote_ros2 run_11_video_car.launch.py
ros2 launch picarx_remote_ros2 run_12_app_control.launch.py
ros2 launch picarx_remote_ros2 run_17_text_vision_talk.launch.py
ros2 launch picarx_remote_ros2 run_20_treasure_hunt.launch.py
ros2 launch picarx_remote_ros2 run_servo_zeroing.launch.py
```

Practical classification:

- `run_2_move.launch.py`: one-shot motion demo, expected to exit after the sequence
- `run_servo_zeroing.launch.py`: one-shot reset/zero command, expected to exit after the command is sent
- `run_4_avoiding_obstacles.launch.py`: autonomous loop
- `run_5_cliff_detection.launch.py`: autonomous loop
- `run_6_line_tracking.launch.py`: autonomous loop
- `run_7_computer_vision.launch.py`: interactive/visual example, but still currently exposed as an example launch
- `run_8_stare_at_you.launch.py`: autonomous vision behavior
- `run_9_record_video.launch.py`: viewer/recording style example
- `run_10_bull_fight.launch.py`: autonomous vision behavior
- `run_11_video_car.launch.py`: keyboard/video oriented example and not ideal as a launch-based workflow
- `run_12_app_control.launch.py`: special control-mode example, not a simple terminal tool
- `run_17_text_vision_talk.launch.py`: mixed vision/LLM style example, not a basic terminal-interactive tool
- `run_20_treasure_hunt.launch.py`: autonomous vision behavior

### Keep these launches out of the non-audio validation path

These launches are voice/audio-oriented or otherwise depend on audio stacks:

```bash
ros2 launch picarx_remote_ros2 run_13_sound_background_music.launch.py
ros2 launch picarx_remote_ros2 run_14_voice_promt_car.launch.py
ros2 launch picarx_remote_ros2 run_15_storytelling_robot.launch.py
ros2 launch picarx_remote_ros2 run_16_voice_controlled_car.launch.py
ros2 launch picarx_remote_ros2 run_18_online_llm_test.launch.py
ros2 launch picarx_remote_ros2 run_19_local_voice_chatbot.launch.py
ros2 launch picarx_remote_ros2 run_21_voice_active_car_doubao_cn.launch.py
ros2 launch picarx_remote_ros2 run_21_voice_active_car_gpt.launch.py
```

Examples with keyboard control are remote-interactive only and should be launched with:

```bash
ros2 run picarx_remote_ros2 run_3_keyboard_control_app
ros2 run picarx_remote_ros2 run_1_cali_grayscale_app
ros2 run picarx_remote_ros2 run_1_cali_servo_motor_app
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

## Install remote vision dependencies

Some remote applications use `vilib` for image processing and higher-level
vision logic on the remote side.

Install it in the active `ros-humble` Python environment:

```bash
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"
pyenv activate ros-humble
pip install vilib
```

This is mainly needed for examples such as:

- `run_7_computer_vision.launch.py`
- `run_8_stare_at_you.launch.py`
- `run_9_record_video.launch.py`
- `run_10_bull_fight.launch.py`
- `run_11_video_car.launch.py`
- `run_20_treasure_hunt.launch.py`

After installing `vilib`, rebuild and resource the remote workspace:

```bash
cd ~/picar-x/ros
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_remote_ros2
source install/setup.bash
```

## Launch the split voice-active-car example remotely

`run_voice_active_car.launch.py` now starts the Jetson-side top-level app directly.
Driving still goes over the existing ROS control topics, while LED and sound effects stay local on
the PI-CAR-X through `/picarx/embodiment/led` and `/picarx/embodiment/sound`.

The PI-CAR-X side must already be running `picarx_hardware.launch.py`, which now includes
`/picarx_embodiment_node`.

```bash
cd ~/picar-x/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
ros2 launch picarx_remote_ros2 run_voice_active_car.launch.py
```

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
- Remote examples that depend on camera, vision, STT, TTS, or LLM tooling also require those dependencies on the remote host.
- The optional tkinter calibration GUI requires `_tkinter` support in the active Python environment.
