cmake_minimum_required(VERSION 3.16)
project(ur5e_rt_controller VERSION 4.2.0 LANGUAGES CXX)

# Require C++20, no GNU extensions
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
  )
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

# ── custom_controller ──────────────────────────────────────────────────────────
add_executable(custom_controller
  src/custom_controller.cpp
)
target_include_directories(custom_controller PRIVATE include)
ament_target_dependencies(custom_controller
  rclcpp
  std_msgs
  sensor_msgs
  ur_msgs
  controller_interface
  hardware_interface
  realtime_tools
)

# ── hand_udp_receiver_node ─────────────────────────────────────────────────────
add_executable(hand_udp_receiver_node
  src/hand_udp_receiver_node.cpp
)
target_include_directories(hand_udp_receiver_node PRIVATE include)
ament_target_dependencies(hand_udp_receiver_node
  rclcpp
  std_msgs
)

# ── hand_udp_sender_node ───────────────────────────────────────────────────────
add_executable(hand_udp_sender_node
  src/hand_udp_sender_node.cpp
)
target_include_directories(hand_udp_sender_node PRIVATE include)
ament_target_dependencies(hand_udp_sender_node
  rclcpp
  std_msgs
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  custom_controller
  hand_udp_receiver_node
  hand_udp_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install all Python utility scripts
install(PROGRAMS
  scripts/monitor_data_health.py
  scripts/plot_ur_trajectory.py
  scripts/motion_editor_gui.py
  scripts/hand_udp_sender_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
