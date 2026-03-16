# PI-CAR-X Local Setup

This guide is for the PI-CAR-X board itself.

It covers:

- ROS 2 Humble compilation on Raspberry Pi OS
- local PI-CAR-X ROS package compilation
- local validation of nodes, calibration, camera stream, and obstacle avoidance

## Why Raspberry Pi OS

Raspberry Pi OS is preferred on the PI-CAR-X board because it is the native OS for the platform and usually gives the least friction with the hardware stack.

## 1. ROS 2 compilation

### Build ROS 2 Humble on Raspberry Pi OS

ROS 2 Humble does not work well with the default Python 3.13+ found on newer Raspberry Pi OS images. Use Python 3.10 in a dedicated `pyenv` environment.

Install `pyenv`:

```bash
curl https://pyenv.run | bash
```

Install Python 3.10 and create the ROS Humble environment:

```bash
pyenv install 3.10.14
pyenv virtualenv 3.10.14 ros-humble
pyenv activate ros-humble
python --version
```

Install Python-side build dependencies:

```bash
pip install --upgrade pip setuptools wheel
pip install rosdep vcstool
pip install colcon-core colcon-common-extensions
pip install numpy lark-parser netifaces
pip uninstall -y empy
pip install empy==3.3.4
```

Useful local tool:

```bash
sudo apt install -y tmux
```

Create the ROS 2 source workspace:

```bash
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
```

Fetch ROS 2 Humble sources:

```bash
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```

Initialize `rosdep`:

```bash
sudo rosdep init
rosdep update
```

Install ROS dependencies:

```bash
cd ~/ros2_humble
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -y \
  --skip-keys="\
urdfdom_headers \
ignition-cmake2 \
ignition-math6 \
rti-connext-dds-6.0.1 \
fastcdr \
fastrtps"
```

Recommended Raspberry Pi 4 build setting:

```bash
export MAKEFLAGS="-j2"
```

Build a practical Humble subset:

```bash
cd ~/ros2_humble
colcon build \
  --merge-install \
  --packages-select \
    ros2cli \
    ros2multicast \
    ros2launch \
    ros2topic \
    ros2node \
    ros2run \
    ros2pkg \
    ros2service \
    ros2param \
    launch \
    launch_ros \
    demo_nodes_cpp \
    demo_nodes_py \
    rmw_fastrtps_cpp \
    rmw_fastrtps_dynamic_cpp \
    rmw_fastrtps_shared_cpp \
    sensor_msgs \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
```

Verify ROS itself:

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash
ros2 --help
ros2 topic list
ros2 node list
```

At each new shell:

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash
```

## 2. Local PI-CAR-X project compilation

### Video stream dependencies

In this project, the local camera stream currently depends on the Raspberry Pi camera stack (`libcamera`), GStreamer support for `libcamerasrc`, and OpenCV with GStreamer-enabled `VideoCapture`.

Install the required system packages on Raspberry Pi OS:

```bash
sudo apt update
sudo apt install -y \
  libcamera0 \
  libcamera-tools \
  gstreamer1.0-tools \
  gstreamer1.0-libcamera \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  python3-opencv
```

Verify that `libcamerasrc` is available:

```bash
gst-inspect-1.0 libcamerasrc
```

### Notes on the global video pipeline

The current video path is:

1. the Raspberry Pi camera sensor produces frames
2. `libcamera` controls the sensor locally on the PI-CAR-X
3. `libcamerasrc` exposes the camera feed inside a GStreamer pipeline
4. `picarx_local_ros2` opens that pipeline through `cv::VideoCapture`
5. frames are read into `cv::Mat`
6. the node converts each frame to `sensor_msgs/msg/Image`
7. ROS publishes the stream on `/picarx/camera/image_raw`
8. the remote host subscribes to `/picarx/camera/image_raw`

The default pipeline built by the local camera node is conceptually:

```text
libcamerasrc ... ! video/x-raw,width=...,height=...,framerate=... ! videoconvert ! video/x-raw,format=BGR ! appsink
```

This means:

- camera tuning issues such as brightness, contrast, exposure, or noise are local PI-side issues
- ROS is only used after the frame has already been captured and converted into a ROS image message


### Build the local PI-CAR-X package

The hardware launch now includes the C++ camera publisher directly inside `picarx_local_ros2`.
It is also advised to build the remote package too as it can be used for testing the setup
locally on the Raspberry Pi.

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash

cd ~/git/picar-x/ros
PYTHONNOUSERSITE=1 colcon build \
   --packages-select picarx_local_ros2 picarx_remote_ros2
source install/setup.bash
```

### Execute the local PI-CAR-X

In the setup below, the Discovery Server runs on the remote host at `<REMOTE_HOST_IP>`.
Replace this IP adress with the right one in your environment.

```bash
export REMOTE_HOST_IP=192.168.0.15
```

Set the PI-CAR-X environment before launching the local hardware stack.
Note that RMW_IMPLEMENTATION, ROS_DOMAIN_ID and ROS_DISCOVERY_SERVER must
be configured the same way on the local raspberry pi and remote computer. 

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash

cd ~/git/picar-x/ros
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=$REMOTE_HOST_IP:11811
```

Then launch the hardware nodes:

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

## 3. Local tests

### Launch and verify local nodes

In another shell, verify the nodes:

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash

cd ~/git/picar-x/ros
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=$REMOTE_HOST_IP:11811

ros2 node list
```

Expected nodes:
- `/picarx_camera_publisher_node`
- `/picarx_driver_node`
- `/picarx_safety_node`
- `/picarx_embodiment_node`

In case te node list command gives nothing, best is to restart the daemon and retry:
```bash
ros2 daemon stop
ros2 daemon start
sleep 2
```

### Test local calibration

If you also want to test the remote-style calibration tools from another shell on the same PI:

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash

cd ~/git/picar-x/ros
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=$REMOTE_HOST_IP:11811

ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

Important note: when ROS 2 is used with DDS discovery through a Discovery Server, `ros2 service list`
and `ros2 node info` may show no services even while the driver is alive and the calibration endpoints
are reachable from a real ROS client. In that case, the missing `ros2cli` output does not mean the
services are absent.

Calibration is stored locally in:

```bash
~/.config/picar-x/picar-x.conf
```

### Test the camera stream locally

You can stop and restart the local camera stream explicitly:

```bash
ros2 service call /picarx_camera_publisher_node/stop std_srvs/srv/Trigger
ros2 service call /picarx_camera_publisher_node/start std_srvs/srv/Trigger
```

You can also check that the camera node and stream are available, but
except the nodes, the topics and service may not well be listed with DDS:

```bash
ros2 node list
ros2 topic list | grep picarx/camera
ros2 topic hz /picarx/camera/image_raw
```

### Voice-active-car split support

The local hardware launch also starts the PI-side embodiment node used by the
split `voice_active_car` example:

- `/picarx_embodiment_node`

### Test local embodiment features

Quick node check:

```bash
ros2 node list | grep embodiment
```

Quick LED tests:

```bash
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'on'}"
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'off'}"
ros2 topic pub --once /picarx/embodiment/led std_msgs/msg/String "{data: 'blink_once'}"
```

Quick sound tests:

```bash
ros2 topic pub --once /picarx/embodiment/sound std_msgs/msg/String "{data: 'honking'}"
ros2 topic pub --once /picarx/embodiment/sound std_msgs/msg/String "{data: 'start engine'}"
```

If the node logs `Sound command received: ...` but no audio comes out, the ROS
path is working and the issue is in the local ALSA / speaker / `robot_hat`
audio setup on the PI-CAR-X.

### Test obstacle avoidance locally

If you want a local end-to-end test on the same PI-CAR-X machine:

```bash
pyenv activate ros-humble
source ~/ros2_humble/install/setup.bash

cd ~/git/picar-x/ros
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=$REMOTE_HOST_IP:11811

ros2 launch picarx_remote_ros2 run_4_avoiding_obstacles.launch.py
```

### Default camera tuning

The local camera publisher also exposes stream control services:

- `/picarx_camera_publisher_node/start`
- `/picarx_camera_publisher_node/stop`

The local hardware launch includes a default camera tuning profile for `libcamerasrc`.

It enables auto exposure and applies a conservative brightness and contrast correction that worked better than the raw defaults on this hardware:

- `camera_auto_exposure=True`
- `camera_controls='exposure-value=1.5 awb-enable=true brightness=0.1 contrast=1.15'`

This tuning is defined in:

- `ros/local/launch/picarx_hardware.launch.py`

It is intended as a practical compromise:

- brighter than the stock camera output
- less noisy than more aggressive exposure settings
- still dependent on the actual ambient light

If the image is still too dark, improve the real scene lighting first. Software tuning can only trade brightness against noise; it cannot compensate for a weak sensor in poor light.
