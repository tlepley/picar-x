cmake_minimum_required(VERSION 3.10)
project(picarx_camera_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(picarx_camera_publisher_node
  src/picarx_camera_publisher_node.cpp
)

target_compile_features(picarx_camera_publisher_node PUBLIC cxx_std_17)
target_include_directories(picarx_camera_publisher_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(picarx_camera_publisher_node ${OpenCV_LIBS})
ament_target_dependencies(picarx_camera_publisher_node rclcpp sensor_msgs std_srvs)

install(TARGETS
  picarx_camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
