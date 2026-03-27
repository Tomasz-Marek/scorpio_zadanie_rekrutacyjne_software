cmake_minimum_required(VERSION 3.8)
project(scorpio_zadanie_rekrutacyjne_software)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED) # Example of importing cv_bridge

include_directories(include)

# Echo node component
add_library(echoNode SHARED src/echo_node.cpp)
ament_target_dependencies(echoNode
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  # cv_bridge
)
rclcpp_components_register_node(echoNode PLUGIN "example::EchoNode" EXECUTABLE echo)

# Cube detector node component
add_library(cubeDetectorNode SHARED src/cube_detector_node.cpp)
ament_target_dependencies(cubeDetectorNode
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
)
rclcpp_components_register_node(cubeDetectorNode PLUGIN "recruitment_task::CubeDetectorNode" EXECUTABLE cube_detector)

# Ilmenite check node component
add_library(ilmeniteCheckNode SHARED src/ilmenite_check_node.cpp)
ament_target_dependencies(ilmeniteCheckNode
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
)
rclcpp_components_register_node(ilmeniteCheckNode PLUGIN "recruitment_task::IlmeniteCheckNode" EXECUTABLE ilmenite_check)


# Install libraries
install(TARGETS
  echoNode
  cubeDetectorNode
  ilmeniteCheckNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
