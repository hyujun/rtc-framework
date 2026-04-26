cmake_minimum_required(VERSION 3.16)
project(rtc_controller_manager VERSION 5.17.0 LANGUAGES CXX)

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

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rtc_urdf_bridge REQUIRED)

# RTC packages
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_msgs REQUIRED)

# ── Static library ────────────────────────────────────────────────────────────
add_library(rtc_controller_manager_lib STATIC
  src/rt_controller_node.cpp
  src/rt_controller_node_params.cpp
  src/rt_controller_node_topics.cpp
  src/rt_controller_node_callbacks.cpp
  src/rt_controller_node_rt_loop.cpp
  src/rt_controller_node_publish.cpp
  src/rt_controller_node_estop.cpp
  src/rt_controller_node_device_config.cpp
  src/rt_controller_main_impl.cpp
)
target_include_directories(rtc_controller_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_controller_manager_lib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  ament_index_cpp
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_communication
  rtc_msgs
  rtc_urdf_bridge
)
target_link_libraries(rtc_controller_manager_lib
  yaml-cpp
  pinocchio::pinocchio
)

# ── Install ───────────────────────────────────────────────────────────────────
# rtc_controller_manager is library-only (no executable). Robot-specific
# bringup packages (e.g. ur5e_bringup) link rtc_controller_manager_lib and
# expose their own executable via rtc::RtControllerMain(argc, argv, node_name).
# See agent_docs/design-principles.md.
install(TARGETS rtc_controller_manager_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Export library for external bringup packages ─────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  ament_index_cpp
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_communication
  rtc_msgs
  rtc_urdf_bridge
  pinocchio
  yaml-cpp
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_controller_timing_profiler
    test/test_controller_timing_profiler.cpp)
  if(TARGET test_controller_timing_profiler)
    target_include_directories(test_controller_timing_profiler PRIVATE include)
    ament_target_dependencies(test_controller_timing_profiler
      rtc_controller_interface rtc_base rtc_urdf_bridge)
    target_link_libraries(test_controller_timing_profiler yaml-cpp)
  endif()

  ament_add_gtest(test_controller_lifecycle
    test/test_controller_lifecycle.cpp)
  if(TARGET test_controller_lifecycle)
    target_include_directories(test_controller_lifecycle PRIVATE include)
    ament_target_dependencies(test_controller_lifecycle
      rclcpp rclcpp_lifecycle std_msgs sensor_msgs ament_index_cpp
      rtc_controller_interface rtc_controllers rtc_base rtc_communication
      rtc_msgs rtc_urdf_bridge)
    target_link_libraries(test_controller_lifecycle
      rtc_controller_manager_lib yaml-cpp pinocchio::pinocchio)
  endif()
endif()

ament_package()
