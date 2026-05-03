cmake_minimum_required(VERSION 3.16)
project(udp_hand_driver VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

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
find_package(rclcpp_lifecycle REQUIRED)
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
add_executable(udp_hand_node
  src/udp_hand_node.cpp
)
target_include_directories(udp_hand_node PRIVATE include)
ament_target_dependencies(udp_hand_node
  rclcpp
  rclcpp_lifecycle
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
  udp_hand_node
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ── Tier 1: Pure computation tests (no ROS2 node, no network) ──────────────
  ament_add_gtest(test_udp_hand_packets test/test_udp_hand_packets.cpp)
  target_include_directories(test_udp_hand_packets PRIVATE include)
  ament_target_dependencies(test_udp_hand_packets rtc_base)

  ament_add_gtest(test_udp_hand_codec test/test_udp_hand_codec.cpp)
  target_include_directories(test_udp_hand_codec PRIVATE include)
  ament_target_dependencies(test_udp_hand_codec rtc_base)

  ament_add_gtest(test_udp_hand_transport test/test_udp_hand_transport.cpp)
  target_include_directories(test_udp_hand_transport PRIVATE include)
  ament_target_dependencies(test_udp_hand_transport rtc_base rclcpp)

  ament_add_gtest(test_udp_hand_timing_profiler test/test_udp_hand_timing_profiler.cpp)
  target_include_directories(test_udp_hand_timing_profiler PRIVATE include)
  ament_target_dependencies(test_udp_hand_timing_profiler rtc_base)

  ament_add_gtest(test_udp_hand_sensor_processor test/test_udp_hand_sensor_processor.cpp)
  target_include_directories(test_udp_hand_sensor_processor PRIVATE include)
  ament_target_dependencies(test_udp_hand_sensor_processor rtc_base rclcpp)

  # ── Tier 2: Component integration tests (fake_hand mode) ──────────────────
  ament_add_gtest(test_udp_hand_controller test/test_udp_hand_controller.cpp)
  target_include_directories(test_udp_hand_controller PRIVATE include)
  ament_target_dependencies(test_udp_hand_controller rtc_base rtc_inference rclcpp)

  ament_add_gtest(test_udp_hand_failure_detector test/test_udp_hand_failure_detector.cpp)
  target_include_directories(test_udp_hand_failure_detector PRIVATE include)
  ament_target_dependencies(test_udp_hand_failure_detector rtc_base rtc_inference rclcpp)

  # ── Tier 3: FT inferencer stub tests ──────────────────────────────────────
  ament_add_gtest(test_fingertip_ft_inferencer test/test_fingertip_ft_inferencer.cpp)
  target_include_directories(test_fingertip_ft_inferencer PRIVATE include)
  ament_target_dependencies(test_fingertip_ft_inferencer rtc_base rtc_inference rclcpp)
endif()

ament_package()
