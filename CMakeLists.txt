cmake_minimum_required(VERSION 3.16)
project(ur5e_rt_controller VERSION 4.4.0 LANGUAGES CXX)

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
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

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
target_link_libraries(custom_controller pinocchio::pinocchio)

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

# ── mujoco_simulator_node (optional — requires MuJoCo 3.x C++ library) ────────
# Install MuJoCo:
#   wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
#   sudo tar -xzf mujoco-*.tar.gz -C /opt/
#   cmake -Dmujoco_DIR=/opt/mujoco-3.x.x/lib/cmake/mujoco ...
find_package(mujoco QUIET)
find_package(glfw3 QUIET)

if(mujoco_FOUND)
  message(STATUS "MuJoCo found — building mujoco_simulator_node")

  add_executable(mujoco_simulator_node
    src/mujoco_simulator_node.cpp
  )
  target_include_directories(mujoco_simulator_node PRIVATE include)
  ament_target_dependencies(mujoco_simulator_node
    rclcpp
    std_msgs
    sensor_msgs
    ament_index_cpp
  )
  target_link_libraries(mujoco_simulator_node mujoco::mujoco)

  if(glfw3_FOUND)
    message(STATUS "GLFW found — MuJoCo viewer enabled")
    target_compile_definitions(mujoco_simulator_node PRIVATE MUJOCO_HAVE_GLFW)
    target_link_libraries(mujoco_simulator_node glfw)
  else()
    message(STATUS "GLFW not found — MuJoCo viewer disabled (headless only)")
  endif()

  install(TARGETS mujoco_simulator_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "MuJoCo not found — mujoco_simulator_node will NOT be built.\n"
    "  Install MuJoCo 3.x and re-run cmake with:\n"
    "    -Dmujoco_DIR=/opt/mujoco-3.x.x/lib/cmake/mujoco"
  )
endif()

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

# MuJoCo model files (always installed, even without MuJoCo library)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
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
