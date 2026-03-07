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

The local camera publisher is implemented in:

- `ros/src/picarx_camera_cpp/src/picarx_camera_publisher_node.cpp`

### Global video pipeline

The current video path is:

1. the Raspberry Pi camera sensor produces frames
2. `libcamera` controls the sensor locally on the PI-CAR-X
3. `libcamerasrc` exposes the camera feed inside a GStreamer pipeline
4. `picarx_camera_cpp` opens that pipeline through `cv::VideoCapture`
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

The local camera publisher also exposes stream control services:

- `/picarx_camera_publisher_node/start`
- `/picarx_camera_publisher_node/stop`

### Build the local PI-CAR-X package

The local hardware launch also starts the camera publisher from `picarx_camera_cpp`, so build both packages:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_camera_cpp picarx_local_ros2
source install/setup.bash
```


### Configure the DDS domain

Use the same domain as the remote ROS host:

```bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
```

This can also be added to your shell startup if that matches your deployment.

### Remote operation from another machine

If you run the application logic from another ROS host, local-only ROS usage remains unchanged, but multi-machine discovery should preferably use a Fast DDS Discovery Server instead of relying on multicast.

In the setup below, the Discovery Server runs on the remote host at `<REMOTE_HOST_IP>`.

Set the PI-CAR-X environment like this before launching the local hardware stack:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
```

Then launch the hardware nodes normally:

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

If you only work locally on the PI-CAR-X itself, `ROS_DISCOVERY_SERVER` is not required.

### Default camera tuning

The local hardware launch includes a default camera tuning profile for `libcamerasrc`.

It enables auto exposure and applies a conservative brightness and contrast correction that worked better than the raw defaults on this hardware:

- `camera_auto_exposure=True`
- `camera_controls='exposure-value=1.5 awb-enable=true brightness=0.1 contrast=1.15'`

This tuning is defined in:

- `ros/src/picarx_local_ros2/launch/picarx_hardware.launch.py`

It is intended as a practical compromise:

- brighter than the stock camera output
- less noisy than more aggressive exposure settings
- still dependent on the actual ambient light

If the image is still too dark, improve the real scene lighting first. Software tuning can only trade brightness against noise; it cannot compensate for a weak sensor in poor light.

## 3. Local tests

### Launch and verify local nodes

Launch the hardware stack:

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

In another shell, verify the nodes:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
ros2 node list
```

Expected nodes:

- `/picarx_driver_node`
- `/picarx_safety_node`

If you are using a Discovery Server for multi-machine operation, make sure the verification shell uses the same environment:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=<REMOTE_HOST_IP>:11811
ros2 daemon stop
ros2 daemon start
sleep 2
ros2 node list
```

### Test local calibration

Check calibration state:

```bash
ros2 service list | grep calibration
ros2 topic echo /picarx/calibration/state --once
```

If you also want to test the remote-style calibration tools from another shell on the same PI:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
ros2 run picarx_remote_ros2 picarx_calibration_cli --show
```

Calibration is stored locally in:

```bash
~/.config/picar-x/picar-x.conf
```

### Test the camera stream locally

Check that the camera node and stream are available:

```bash
ros2 node list
ros2 topic list | grep picarx/camera
ros2 topic hz /picarx/camera/image_raw
```

You can also stop and restart the local camera stream explicitly:

```bash
ros2 service call /picarx_camera_publisher_node/stop std_srvs/srv/Trigger
ros2 service call /picarx_camera_publisher_node/start std_srvs/srv/Trigger
```

### Test obstacle avoidance locally

If you want a local end-to-end test on the same PI-CAR-X machine:

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_remote_ros2
source install/setup.bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
ros2 launch picarx_remote_ros2 run_4_avoiding_obstacles.launch.py
```
