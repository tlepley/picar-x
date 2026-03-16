cmake_minimum_required(VERSION 3.10)
project(picarx_local_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

ament_python_install_package(${PROJECT_NAME})

function(configure_python_entrypoint launcher_name module_name)
  set(PICARX_LOCAL_PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
  set(PICARX_LOCAL_MODULE_NAME "${module_name}")
  configure_file(
    cmake/python_entrypoint.in
    "${CMAKE_CURRENT_BINARY_DIR}/${launcher_name}"
    @ONLY
  )
endfunction()

configure_python_entrypoint(picarx_node picarx_local_ros2.picarx_node)
configure_python_entrypoint(picarx_driver_node picarx_local_ros2.driver_node)
configure_python_entrypoint(picarx_safety_node picarx_local_ros2.safety_node)
configure_python_entrypoint(picarx_embodiment_node picarx_local_ros2.embodiment_node)
configure_python_entrypoint(picarx_camera_control_cli picarx_local_ros2.camera_control_cli)

add_executable(picarx_camera_publisher_node
  src/picarx_camera_publisher_node.cpp
)
target_compile_features(picarx_camera_publisher_node PUBLIC cxx_std_17)
target_include_directories(picarx_camera_publisher_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(picarx_camera_publisher_node ${OpenCV_LIBS})
ament_target_dependencies(picarx_camera_publisher_node rclcpp sensor_msgs std_srvs)

install(
  TARGETS picarx_camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    "${CMAKE_CURRENT_BINARY_DIR}/picarx_node"
    "${CMAKE_CURRENT_BINARY_DIR}/picarx_driver_node"
    "${CMAKE_CURRENT_BINARY_DIR}/picarx_safety_node"
    "${CMAKE_CURRENT_BINARY_DIR}/picarx_embodiment_node"
    "${CMAKE_CURRENT_BINARY_DIR}/picarx_camera_control_cli"
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    resource/${PROJECT_NAME}
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../sounds")
  install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../sounds/"
    DESTINATION share/${PROJECT_NAME}/sounds
  )
endif()

ament_package()
