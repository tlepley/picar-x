# PI-CAR-X Local Setup

This guide is for the PI-CAR-X board itself.

It covers:

- initial ROS 2 Humble build on Raspberry Pi OS
- verification that the local ROS nodes are available
- DDS domain configuration
- local validation of calibration and obstacle avoidance
- remote operation preparation when a separate ROS host is used

## Why Raspberry Pi OS

Raspberry Pi OS is preferred on the PI-CAR-X board because it is the native OS for the platform and usually gives the least friction with the hardware stack.

## Build ROS 2 Humble on Raspberry Pi OS

ROS 2 Humble does not work well with the default Python 3.13 found on newer Raspberry Pi OS images. Use Python 3.10 in a dedicated `pyenv` environment.

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
  --packages-up-to \
    ros2launch \
    launch \
    launch_ros \
    ros2cli \
    ros2topic \
    ros2node \
    ros2run \
    ros2pkg \
    ros2service \
    ros2param \
    demo_nodes_cpp \
    demo_nodes_py \
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

## Build the local PI-CAR-X package

```bash
cd ~/git/picar-x/ros
source ~/ros2_humble/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select picarx_local_ros2
source install/setup.bash
```

## Configure the DDS domain

Use the same domain as the remote ROS host:

```bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
```

This can also be added to your shell startup if that matches your deployment.

## Remote operation from another machine

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

## Launch and verify local nodes

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

## Test local calibration

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

## Test obstacle avoidance locally

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
