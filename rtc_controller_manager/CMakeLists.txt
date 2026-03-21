cmake_minimum_required(VERSION 3.16)
project(rtc_controller_manager VERSION 0.1.0 LANGUAGES CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)

# RTC packages
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_communication REQUIRED)
find_package(rtc_status_monitor REQUIRED)
find_package(rtc_msgs REQUIRED)

find_package(rtc_inference REQUIRED)
find_package(ur5e_hand_driver REQUIRED)
find_package(rtc_msgs REQUIRED)

# ── Static library ────────────────────────────────────────────────────────────
add_library(rtc_controller_manager_lib STATIC
  src/rt_controller_node.cpp
)
target_include_directories(rtc_controller_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtc_controller_manager_lib
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_communication
  rtc_status_monitor
  rtc_msgs
  rtc_inference
  ur5e_hand_driver
  rtc_msgs
)
target_link_libraries(rtc_controller_manager_lib
  yaml-cpp
  pinocchio::pinocchio
)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(rt_controller
  src/rt_controller_main.cpp
)
target_link_libraries(rt_controller
  rtc_controller_manager_lib
)
ament_target_dependencies(rt_controller
  rclcpp
  rtc_base
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rt_controller
  DESTINATION lib/${PROJECT_NAME}
)

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
  sensor_msgs
  std_msgs
  ament_index_cpp
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_communication
  rtc_status_monitor
  rtc_msgs
  rtc_inference
  ur5e_hand_driver
  pinocchio
  yaml-cpp
)
ament_export_include_directories(include)

ament_package()
