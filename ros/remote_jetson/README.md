# `picarx_remote_jetson_ros2`

Application ROS2 native en C++ pour le poste distant:

- affichage video faible latence
- teleoperation clavier sans Tkinter
- detection d'objets YOLO via NVIDIA TensorRT
- compatibilite directe avec la stack locale existante (`/picarx/camera/image_raw`, `/picarx/speed`, `/picarx/steering`)

## Execution

Depuis `~/git/picar-x/ros`:

```bash
colcon build --packages-select picarx_remote_jetson_ros2
source install/setup.bash
ros2 run picarx_remote_jetson_ros2 picarx_remote_yolo_video_car --ros-args \
  -p engine_path:=$(pwd)/remote_jetson/assets/models/yolov8n_coco_640x640_fp16.engine \
  -p classes_path:=$(pwd)/remote_jetson/assets/labels/yolov8n_coco_80.names
```

Le mode interactif officiel est `ros2 run`, pas `ros2 launch`, afin de garder un vrai terminal pour le clavier.

## Notes Jetson Orin NX

- Le noeud charge un moteur TensorRT `.engine` deja optimise pour la Jetson.
- Le chemin recommande est de generer le moteur directement sur l'Orin NX avec `trtexec`.
- Pour un premier profil performant:

```bash
cd ~/git/picar-x/ros/remote_jetson/assets/models
/usr/src/tensorrt/bin/trtexec --onnx=yolov8n_coco_640x640.onnx \
  --saveEngine=yolov8n_coco_640x640_fp16.engine \
  --fp16 \
```

- Ce premier increment cible le format de sortie standard YOLOv8 exporte vers TensorRT.
