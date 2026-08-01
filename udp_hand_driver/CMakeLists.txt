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
    -Werror=switch  # enum switch 누락은 dead topic 을 낳는다 (#211)
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_inference REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Sensor protocol seam (version 1a/1b factory + decode) ────────────────────
# Small static lib so the node and every controller-using test share one
# definition of CreateSensorProtocol() (UdpHandController::Start() references it).
add_library(udp_hand_protocol STATIC
  src/protocol/sensor_protocol.cpp
)
target_include_directories(udp_hand_protocol PUBLIC include)
ament_target_dependencies(udp_hand_protocol rtc_base)

# ── Unified node: HandController (request-response polling) ──────────────────
# Note: HAS_ONNXRUNTIME, ONNX includes, and link libraries are propagated
# from rtc_inference via its INTERFACE target.
add_executable(udp_hand_node
  src/udp_hand_node.cpp
  src/udp_hand_node_lifecycle.cpp
  src/udp_hand_node_publish.cpp
  src/udp_hand_node_stats.cpp
)
target_include_directories(udp_hand_node PRIVATE include)
target_link_libraries(udp_hand_node udp_hand_protocol)
ament_target_dependencies(udp_hand_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  std_msgs
  rtc_base
  rtc_communication
  rtc_inference
  rtc_msgs
  ament_index_cpp
)

# ── RT tracing (opt-in) ──────────────────────────────────────────────────────
# udp_hand_node is the only target that compiles the header-only
# UdpHandController::RunCommCycle CommLoop tick, so the RTC_TRACE_SCOPE spans
# (hand_comm_tick → read/tail → ft_infer) live here. When rtc_base was built
# with -DRTC_ENABLE_TRACING=ON it exports rtc_base::rtc_tracing (lttng include
# path). Default OFF → macro is a no-op, no lttng dependency. The test targets
# include the same header but leave RTC_TRACING_ENABLED undefined, so they stay
# no-op. See rtc_base/CMakeLists.txt and docs/tracing.md.
option(RTC_ENABLE_TRACING "Compile rtc:* LTTng UST tracepoints (RT tracing)" OFF)
if(RTC_ENABLE_TRACING)
  target_link_libraries(udp_hand_node rtc_base::rtc_tracing)
  # Define set here (PRIVATE), not inherited from rtc_tracing — see the
  # interface-leak note in rtc_base/CMakeLists.txt. Keeps the opt-in local so an
  # ON rtc_base never silently instruments an OFF consumer.
  target_compile_definitions(udp_hand_node PRIVATE RTC_TRACING_ENABLED)
endif()

# ── Full-SIL loopback firmware simulator (device side) ───────────────────────
# Peer UDP server that answers the real udp_hand_node's requests with
# protocol-correct responses (no hardware). Core logic is the header-only
# FakeHandFirmware class; this executable is just the socket/param/log shell.
add_executable(fake_hand_firmware
  src/fake_hand_firmware_main.cpp
)
target_include_directories(fake_hand_firmware PRIVATE include)
ament_target_dependencies(fake_hand_firmware
  rclcpp
  rtc_base
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  udp_hand_node
  fake_hand_firmware
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

  ament_add_gtest(test_udp_hand_joint_command_reorder
    test/test_udp_hand_joint_command_reorder.cpp)
  target_include_directories(test_udp_hand_joint_command_reorder PRIVATE include)
  ament_target_dependencies(test_udp_hand_joint_command_reorder rtc_base)

  ament_add_gtest(test_sensor_protocol_1b test/test_sensor_protocol_1b.cpp)
  target_include_directories(test_sensor_protocol_1b PRIVATE include)
  target_link_libraries(test_sensor_protocol_1b udp_hand_protocol)
  ament_target_dependencies(test_sensor_protocol_1b rtc_base)

  ament_add_gtest(test_udp_hand_transport test/test_udp_hand_transport.cpp)
  target_include_directories(test_udp_hand_transport PRIVATE include)
  ament_target_dependencies(test_udp_hand_transport rtc_base rclcpp)

  ament_add_gtest(test_fake_hand_firmware test/test_fake_hand_firmware.cpp)
  target_include_directories(test_fake_hand_firmware PRIVATE include)
  ament_target_dependencies(test_fake_hand_firmware rtc_base)

  ament_add_gtest(test_udp_hand_timing_profiler test/test_udp_hand_timing_profiler.cpp)
  target_include_directories(test_udp_hand_timing_profiler PRIVATE include)
  ament_target_dependencies(test_udp_hand_timing_profiler rtc_base)

  ament_add_gtest(test_udp_hand_sensor_processor test/test_udp_hand_sensor_processor.cpp)
  target_include_directories(test_udp_hand_sensor_processor PRIVATE include)
  ament_target_dependencies(test_udp_hand_sensor_processor rtc_base rclcpp)

  # ── Tier 2: Component integration tests (fake_hand mode) ──────────────────
  ament_add_gtest(test_udp_hand_controller test/test_udp_hand_controller.cpp)
  target_include_directories(test_udp_hand_controller PRIVATE include)
  target_link_libraries(test_udp_hand_controller udp_hand_protocol)
  ament_target_dependencies(test_udp_hand_controller rtc_base rtc_inference rclcpp)

  ament_add_gtest(test_udp_hand_failure_detector test/test_udp_hand_failure_detector.cpp)
  target_include_directories(test_udp_hand_failure_detector PRIVATE include)
  target_link_libraries(test_udp_hand_failure_detector udp_hand_protocol)
  ament_target_dependencies(test_udp_hand_failure_detector rtc_base rtc_inference rclcpp)

  # ── Tier 2b: Node-level lane wiring (issue #345) ──────────────────────────
  # Compiles the node's non-main translation units directly: udp_hand_node.cpp
  # owns main(), so the executable cannot be linked into a gtest binary.
  ament_add_gtest(test_udp_hand_node_lanes
    test/test_udp_hand_node_lanes.cpp
    src/udp_hand_node_lifecycle.cpp
    src/udp_hand_node_publish.cpp
    src/udp_hand_node_stats.cpp)
  target_include_directories(test_udp_hand_node_lanes PRIVATE include)
  target_link_libraries(test_udp_hand_node_lanes udp_hand_protocol)
  ament_target_dependencies(test_udp_hand_node_lanes
    rtc_base rtc_inference rtc_msgs rclcpp rclcpp_lifecycle lifecycle_msgs
    sensor_msgs std_msgs ament_index_cpp)

  # Lifecycle behaviour in sil_mode=loopmodel: configure/activate bring up the
  # real CommLoop + NRT publish lane, so this binary talks DDS and gets a
  # dedicated ROS_DOMAIN_ID (the isolation idiom above rtc_controller_manager /
  # test_demo_wbc_tsid_path use; domains 44-52 are taken).
  ament_add_gtest(test_udp_hand_node_lifecycle
    test/test_udp_hand_node_lifecycle.cpp
    src/udp_hand_node_lifecycle.cpp
    src/udp_hand_node_publish.cpp
    src/udp_hand_node_stats.cpp
    ENV ROS_DOMAIN_ID=53)
  target_include_directories(test_udp_hand_node_lifecycle PRIVATE include)
  target_link_libraries(test_udp_hand_node_lifecycle udp_hand_protocol)
  ament_target_dependencies(test_udp_hand_node_lifecycle
    rtc_base rtc_inference rtc_msgs rclcpp rclcpp_lifecycle lifecycle_msgs
    sensor_msgs std_msgs ament_index_cpp)

  # ── Tier 3: FT inferencer stub tests ──────────────────────────────────────
  ament_add_gtest(test_fingertip_ft_inferencer test/test_fingertip_ft_inferencer.cpp)
  target_include_directories(test_fingertip_ft_inferencer PRIVATE include)
  ament_target_dependencies(test_fingertip_ft_inferencer rtc_base rtc_inference rclcpp)
endif()

ament_package()
