cmake_minimum_required(VERSION 3.16)
project(rtc_status_monitor VERSION 5.16.0 LANGUAGES CXX)

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

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rtc_base REQUIRED)

# Optional: UR dashboard msgs for auto-recovery features
find_package(ur_dashboard_msgs QUIET)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(rtc_status_monitor SHARED
  src/rtc_status_monitor.cpp
)
target_include_directories(rtc_status_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_status_monitor
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  controller_manager_msgs
  rtc_base
)

if(ur_dashboard_msgs_FOUND)
  target_compile_definitions(rtc_status_monitor PUBLIC HAVE_UR_DASHBOARD_MSGS)
  ament_target_dependencies(rtc_status_monitor ur_dashboard_msgs)
  message(STATUS "ur_dashboard_msgs found — auto-recovery enabled")
else()
  message(STATUS "ur_dashboard_msgs NOT found — auto-recovery via dashboard disabled")
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rtc_status_monitor
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

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  controller_manager_msgs
  rtc_base
)
if(ur_dashboard_msgs_FOUND)
  ament_export_dependencies(ur_dashboard_msgs)
endif()
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
