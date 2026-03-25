cmake_minimum_required(VERSION 3.16)
project(ur5e_hand_driver VERSION 5.17.0 LANGUAGES CXX)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_inference REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Unified node: HandController (request-response polling) ──────────────────
# Note: HAS_ONNXRUNTIME, ONNX includes, and link libraries are propagated
# from rtc_inference via its INTERFACE target.
add_executable(hand_udp_node
  src/hand_udp_node.cpp
)
target_include_directories(hand_udp_node PRIVATE include)
ament_target_dependencies(hand_udp_node
  rclcpp
  sensor_msgs
  std_msgs
  rtc_base
  rtc_communication
  rtc_inference
  rtc_msgs
  ament_index_cpp
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  hand_udp_node
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

# ── Export headers for downstream packages ────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(rtc_base rtc_communication rtc_inference rtc_msgs)

ament_package()
