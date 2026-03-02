cmake_minimum_required(VERSION 3.16)
project(ur5e_rt_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

# Include directories
include_directories(
  include
)

# Custom controller executable
add_executable(custom_controller
  src/custom_controller.cpp
)

ament_target_dependencies(custom_controller
  rclcpp
  std_msgs
  sensor_msgs
  ur_msgs
  controller_interface
  hardware_interface
  realtime_tools
)

# Hand UDP receiver node
add_executable(hand_udp_receiver_node
  src/hand_udp_receiver_node.cpp
)

ament_target_dependencies(hand_udp_receiver_node
  rclcpp
  std_msgs
)

# Hand UDP sender node
add_executable(hand_udp_sender_node
  src/hand_udp_sender_node.cpp
)

ament_target_dependencies(hand_udp_sender_node
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  custom_controller
  hand_udp_receiver_node
  hand_udp_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts
install(PROGRAMS
  scripts/monitor_data_health_v2.py
  scripts/plot_ur_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
