cmake_minimum_required(VERSION 3.10)
project(picarx_remote_jetson_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn highgui imgproc)

find_path(TENSORRT_INCLUDE_DIR NvInferRuntime.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include
  REQUIRED
)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  PATHS /usr/local/cuda/targets/aarch64-linux/include /usr/local/cuda/include
  REQUIRED
)
find_library(TENSORRT_LIB nvinfer REQUIRED)
find_library(TENSORRT_PLUGIN_LIB nvinfer_plugin REQUIRED)
find_library(CUDART_LIB cudart PATHS /usr/local/cuda/targets/aarch64-linux/lib REQUIRED)

add_executable(picarx_remote_yolo_video_car
  src/picarx_remote_yolo_video_car.cpp
  src/yolo_video_car_shared.cpp
  src/detection_pipeline.cpp
  src/media_capture.cpp
  src/vehicle_controller.cpp
  src/viewer_ui.cpp
)
target_compile_features(picarx_remote_yolo_video_car PUBLIC cxx_std_17)
target_include_directories(
  picarx_remote_yolo_video_car
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIR}
)
target_link_libraries(
  picarx_remote_yolo_video_car
  ${OpenCV_LIBS}
  ${TENSORRT_LIB}
  ${TENSORRT_PLUGIN_LIB}
  ${CUDART_LIB}
)
ament_target_dependencies(
  picarx_remote_yolo_video_car
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(
  TARGETS picarx_remote_yolo_video_car
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    resource/${PROJECT_NAME}
    package.xml
    README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
