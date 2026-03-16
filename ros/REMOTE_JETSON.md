# Remote Jetson App Setup

This guide documents the dedicated Jetson remote application:

- package: `picarx_remote_jetson_ros2`
- source directory: `ros/remote_jetson`
- executable: `picarx_remote_yolo_video_car`

It covers:

- ROS and DDS configuration
- Jetson and TensorRT prerequisites
- building the application
- where to store the model and label files
- preparing a YOLO TensorRT engine
- running the application

The application is a native C++ ROS 2 remote app for:

- remote video display
- keyboard driving
- YOLO object detection
- bounding-box overlay on the live stream

## Project file locations

Use these project-local directories to keep the model artifacts in a predictable place:

- TensorRT and model files: `ros/remote_jetson/assets/models/`
- label files: `ros/remote_jetson/assets/labels/`

Recommended files:

- `ros/remote_jetson/assets/models/yolov8n_coco_640x640.pt`
- `ros/remote_jetson/assets/models/yolov8n_coco_640x640.onnx`
- `ros/remote_jetson/assets/models/yolov8n_coco_640x640_fp16.engine`
- `ros/remote_jetson/assets/labels/yolov8n_coco_80.names`

These directories are intentionally ignored by Git so that large machine-specific artifacts are not committed by default.

Recommended naming convention:

- model family: `yolov8n`
- dataset: `coco`
- fixed input size: `640x640`
- engine precision: `fp16`
- label count or dataset suffix for names: `80`

Example:

- `yolov8n_coco_640x640.pt`
- `yolov8n_coco_640x640.onnx`
- `yolov8n_coco_640x640_fp16.engine`
- `yolov8n_coco_80.names`

It consumes the existing PI-CAR-X ROS interfaces:

- camera input: `/picarx/camera/image_raw`
- drive commands: `/picarx/speed`, `/picarx/steering`
- camera stream services:
  - `/picarx_camera_publisher_node/start`
  - `/picarx_camera_publisher_node/stop`

## 1. Target setup

This application is intended to run on a Jetson machine used as the remote app host.

Recommended target:

- Jetson Orin NX
- Ubuntu 22.04
- ROS 2 Humble
- Jetson Linux / JetPack 6.x

## 2. ROS environment

Before building or launching the app:

```bash
source /opt/ros/humble/setup.bash
cd ~/git/picar-x/ros
```

For a multi-machine setup, the remote host must use the same ROS discovery settings as the PI-CAR-X board.

Typical environment:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER=192.168.0.15:11811
```

If you do not use a Fast DDS Discovery Server, keep at least:

```bash
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
```

## 3. Verify the PI-CAR-X local stack

Before launching the Jetson app, the car-side local ROS stack must already be running.

On the PI-CAR-X board, the expected launch is:

```bash
ros2 launch picarx_local_ros2 picarx_hardware.launch.py
```

From the remote Jetson shell, verify that the main interfaces are visible:

```bash
ros2 topic list | grep picarx
ros2 service list | grep picarx_camera_publisher_node
```

Important topics and services:

- `/picarx/camera/image_raw`
- `/picarx/speed`
- `/picarx/steering`
- `/picarx_camera_publisher_node/start`
- `/picarx_camera_publisher_node/stop`

If these are missing, do not debug the app first. Fix ROS discovery or the local hardware launch.

## 4. TensorRT and YOLO prerequisites

This application expects a TensorRT engine file:

- `*.engine`

The current code is written for a YOLO-style model with standard detection output and was prepared first for YOLOv8-style export.

### Check TensorRT

On this project, TensorRT is not provided by the repository itself. It must already exist on the Jetson system.

Useful checks:

```bash
dpkg -l | grep -E 'tensorrt|nvinfer'
ls /usr/include/aarch64-linux-gnu/NvInferRuntime.h
/usr/src/tensorrt/bin/trtexec --help | head
```

Important note:

- if TensorRT development headers and runtime are already installed, you do not need to install `nvidia-jetpack`
- if `sudo apt install nvidia-jetpack` fails because of version conflicts, do not force it unless you intend to realign the full Jetson software stack

### Check Ultralytics

Useful check:

```bash
python3 -m pip show ultralytics
```

If needed:

```bash
python3 -m pip install -U ultralytics
```

## 5. Build the application

Build from the ROS workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/git/picar-x/ros
colcon build --packages-select picarx_remote_jetson_ros2
source install/setup.bash
```

The official interactive execution mode is `ros2 run`, not `ros2 launch`.

Reason:

- the application needs a real interactive terminal for reliable keyboard capture
- `ros2 launch` is not the right tool for keyboard-driven control apps

## 6. Prepare a YOLO model

The application does not download a model automatically. You must prepare a model yourself.

Recommended first model:

- `ros/remote_jetson/assets/models/yolov8n_coco_640x640.pt`

### Export to ONNX

Example:

```bash
cd ~/git/picar-x/ros/remote_jetson/assets/models
python3 - <<'PY'
from ultralytics import YOLO

model = YOLO("yolov8n_coco_640x640.pt")
model.export(format="onnx", imgsz=640, opset=17)
PY
```

This should produce:

- `yolov8n_coco_640x640.onnx`

### Check the ONNX input tensor name

The TensorRT conversion command needs the correct input tensor name.

Example:

```bash
cd ~/git/picar-x/ros/remote_jetson/assets/models
python3 - <<'PY'
import onnx

model = onnx.load("yolov8n_coco_640x640.onnx")
print(model.graph.input[0].name)
PY
```

### Convert ONNX to TensorRT engine

Use `trtexec` directly on the Jetson:

```bash
cd ~/git/picar-x/ros/remote_jetson/assets/models
/usr/src/tensorrt/bin/trtexec \
  --onnx=yolov8n_coco_640x640.onnx \
  --saveEngine=yolov8n_coco_640x640_fp16.engine \
  --fp16
```

Recommended first settings for Orin NX:
- FP16
- image size 640
- lightweight YOLO model

Because this ONNX export is static, you do not need `--shapes`.

After the conversion, you should have:

- `~/git/picar-x/ros/remote_jetson/assets/models/yolov8n_coco_640x640_fp16.engine`

## 7. Prepare class names

The application can display class labels if given a text file with one class name per line.

Example file:

- `ros/remote_jetson/assets/labels/yolov8n_coco_80.names`

For the standard pretrained `yolov8n.pt` model, the expected labels are the 80 classes from the COCO dataset.

### Download an official class list

Prefer downloading the class list from the official model provider or from the official dataset metadata instead of rewriting it by hand.

Important rule:

- the class file must match the exact training dataset and class order of the model you exported
- `coco.names` is correct only for a YOLO model trained on the standard 80-class COCO dataset

For a standard pretrained `yolov8n.pt` from Ultralytics, use the standard COCO class list.

One practical approach is to export the names directly from the model metadata with Python:

```bash
cd ~/git/picar-x/ros/remote_jetson/assets
python3 - <<'PY'
from ultralytics import YOLO

model = YOLO("models/yolov8n_coco_640x640.pt")
names = model.names
with open("labels/yolov8n_coco_80.names", "w", encoding="utf-8") as f:
    for i in range(len(names)):
        f.write(f"{names[i]}\n")
PY
```

This avoids any manual copy-paste error and guarantees that the generated file follows the class order embedded in the downloaded model.

If you prefer downloading a static file, use an official COCO class list and verify that:

- the file contains exactly 80 lines
- the class order matches the model

Useful checks:

```bash
wc -l ~/git/picar-x/ros/remote_jetson/assets/labels/yolov8n_coco_80.names
head ~/git/picar-x/ros/remote_jetson/assets/labels/yolov8n_coco_80.names
tail ~/git/picar-x/ros/remote_jetson/assets/labels/yolov8n_coco_80.names
```

Typical parameter usage:

```bash
classes_path:=$(pwd)/remote_jetson/assets/labels/yolov8n_coco_80.names
```

If `classes_path` is omitted or empty, the app still runs and falls back to generic labels such as `cls_0`.

## 8. Run the application

From the ROS workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/git/picar-x/ros
source install/setup.bash

ros2 run picarx_remote_jetson_ros2 picarx_remote_yolo_video_car --ros-args \
  -p engine_path:=$(pwd)/remote_jetson/assets/models/yolov8n_coco_640x640_fp16.engine \
  -p classes_path:=$(pwd)/remote_jetson/assets/labels/yolov8n_coco_80.names
```

This command assumes you run it from:

```bash
~/git/picar-x/ros
```

Important ROS parameters:

- `engine_path`: TensorRT engine file
- `classes_path`: optional class-name file

The launch currently sets these defaults:

- input size: `640x640`
- detector variant: `yolov8`
- detection every `2` frames
- base speed: `20`
- max speed: `60`

## 9. Keyboard controls

The application uses the interactive terminal as the primary keyboard input path.

Controls:

- Up arrow: drive forward
- Down arrow: drive backward
- Left arrow: drive with left steering
- Right arrow: drive with right steering
- Space: stop
- `+` or `=`: increase speed
- `-` or `_`: decrease speed
- `f`: enable or disable detector
- `h`: print help
- `x` or `Esc`: quit

Recommended usage:

- keep the terminal used for `ros2 run` focused when driving
- use the OpenCV window for the video display

## 10. Runtime behavior

At startup, the application:

- creates a C++ ROS node
- subscribes to `/picarx/camera/image_raw`
- publishes speed and steering commands
- optionally requests the camera stream start service
- opens an OpenCV window
- runs TensorRT inference on the received frames
- overlays detections on the displayed video

When the app exits, it:

- sends a stop command
- requests camera stream stop
- closes the OpenCV window

## 11. Troubleshooting

### The app builds but detections are wrong

Possible causes:

- the TensorRT engine output layout does not match the current YOLO post-processing path
- the engine was exported from a different YOLO family than expected
- the input shape used for export does not match the launch parameters

Start with:

- `yolov8n`
- `640x640`
- FP16

### The app launches but no image appears

Check:

- the PI-CAR-X local hardware launch is running
- DDS discovery is correct
- `/picarx/camera/image_raw` exists
- the camera start service exists

Useful commands:

```bash
ros2 topic hz /picarx/camera/image_raw
ros2 service list | grep picarx_camera_publisher_node
```

### The car does not move

Check:

- the OpenCV window has keyboard focus
- `/picarx/speed` and `/picarx/steering` are visible
- the local driver node is running

Useful commands:

```bash
ros2 topic echo /picarx/speed
ros2 topic echo /picarx/steering
```

### `nvidia-jetpack` installation fails

If TensorRT is already installed and `nvidia-jetpack` reports dependency conflicts, do not treat that as an application blocker.

For this project, what matters is:

- TensorRT runtime is installed
- TensorRT headers are installed
- `trtexec` is available
- the package `picarx_remote_jetson_ros2` builds

## 12. Relevant files

- package: [`ros/remote_jetson/package.xml`](./remote_jetson/package.xml)
- build config: [`ros/remote_jetson/CMakeLists.txt`](./remote_jetson/CMakeLists.txt)
- launch: [`ros/remote_jetson/launch/run_yolo_car.launch.py`](./remote_jetson/launch/run_yolo_car.launch.py)
- app source: [`ros/remote_jetson/src/picarx_remote_yolo_video_car.cpp`](./remote_jetson/src/picarx_remote_yolo_video_car.cpp)
- model directory: [`ros/remote_jetson/assets/models`](./remote_jetson/assets/models)
- label directory: [`ros/remote_jetson/assets/labels`](./remote_jetson/assets/labels)
