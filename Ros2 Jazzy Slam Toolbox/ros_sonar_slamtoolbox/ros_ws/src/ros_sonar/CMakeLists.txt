cmake_minimum_required(VERSION 3.8)
project(ros_sonar)

# Default to C++17 (required by ROS 2 generators)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# -----------------------------
# Message generation
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sonar.msg"
  "msg/SonarScan.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------
# Install Python nodes
# -----------------------------
install(
  DIRECTORY nodes/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# -----------------------------
# Install launch files
# -----------------------------
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# -----------------------------
# Install config files
# -----------------------------
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# -----------------------------
# Install resource index
# -----------------------------
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

