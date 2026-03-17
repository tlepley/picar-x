# `picarx_remote_jetson_ros2`

Native ROS2 C++ application for the remote workstation:

- low-latency video display
- keyboard teleoperation without Tkinter
- YOLO object detection through NVIDIA TensorRT
- direct compatibility with the existing local stack (`/picarx/camera/image_raw`, `/picarx/speed`, `/picarx/steering`)

## Supported Ultralytics Models

For this Jetson Orin NX project, the native C++ viewer targets the Ultralytics models that are most useful for real-time box overlays:

- supported: `YOLOv5 Detect` exported to TensorRT with raw detection outputs
- supported: `YOLOv8 Detect` exported to TensorRT with raw detection outputs
- supported: `YOLO11 Detect` exported to TensorRT with raw detection outputs
- theoretically compatible only if the output contract stays the same: other `Ultralytics Detect` models with axis-aligned boxes

Not supported in this native viewer today:

- `Segment`
- `Pose`
- `OBB`
- `Classify`
- TensorRT engines with integrated NMS/post-processing if the output shape no longer matches what the C++ parser expects

## Runtime Architecture

The native Jetson node is now split into three internal responsibilities:

- `DetectionPipeline`: TensorRT inference, YOLO decoding, tracking, hysteresis, and stable detection boxes
- `ViewerUi`: optional OpenCV display and keyboard input for debugging and manual teleoperation
- `VehicleController`: speed, steering, camera centering, and camera stream control

This separation is intentional:

- the viewer can be disabled for headless runs with `-p enable_viewer:=false`
- the detection pipeline can keep running without any display code
- the vehicle controller is isolated enough to become autonomous later without depending on the debug viewer

## Running

From `~/git/picar-x/ros`:

```bash
colcon build --packages-select picarx_remote_jetson_ros2
source install/setup.bash
ros2 run picarx_remote_jetson_ros2 picarx_remote_yolo_video_car --ros-args \
  -p engine_path:=$(pwd)/remote_jetson/assets/models/yolov8n_coco_640x640_fp16.engine \
  -p classes_path:=$(pwd)/remote_jetson/assets/labels/yolov8n_coco_80.names \
  -p model_task:=detect \
  -p yolo_variant:=yolov8
```

The official interactive mode is `ros2 run`, not `ros2 launch`, so the process keeps a real terminal for keyboard input.

## Jetson Orin NX Notes

- The node loads a TensorRT `.engine` file already optimized for the Jetson.
- The recommended path is to generate the engine directly on the Orin NX with `trtexec`.
- The recommended variants for this project are the smaller Detect models first (`n` or `s`).
- For a solid first performance profile:

```bash
cd ~/git/picar-x/ros/remote_jetson/assets/models
/usr/src/tensorrt/bin/trtexec --onnx=yolov8n_coco_640x640.onnx \
  --saveEngine=yolov8n_coco_640x640_fp16.engine \
  --fp16 \
```

- The C++ parser is designed for the standard Detect outputs of YOLOv5, YOLOv8, and YOLO11.
