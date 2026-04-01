cmake_minimum_required(VERSION 3.16)
project(ur5e_bringup VERSION 5.17.0 LANGUAGES CXX)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_controllers REQUIRED)
find_package(rtc_base REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pre-find Python3 with NumPy so eigenpy's cmake detection succeeds.
set(Python3_FIND_VIRTUALENV STANDARD)
find_package(Python3 COMPONENTS Interpreter Development NumPy)

find_package(pinocchio REQUIRED)
find_package(urdf_pinocchio_bridge REQUIRED)
find_package(rtc_controller_manager REQUIRED)

# ── Static library: demo controllers ──────────────────────────────────────────
add_library(${PROJECT_NAME}_demo_controllers STATIC
  src/controllers/demo_joint_controller.cpp
  src/controllers/demo_task_controller.cpp
  src/controllers/controller_registration.cpp
)

target_include_directories(${PROJECT_NAME}_demo_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_demo_controllers
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_msgs
  pinocchio
  urdf_pinocchio_bridge
)

target_link_libraries(${PROJECT_NAME}_demo_controllers
  yaml-cpp
)

# ── UR5e-specific executable (links built-in + demo controllers) ──────────────
# --whole-archive prevents the linker from stripping static-init registration
# TUs, eliminating the need for ForceDemoControllerRegistration().
add_executable(ur5e_rt_controller
  src/ur5e_rt_controller_main.cpp
)
target_link_libraries(ur5e_rt_controller
  -Wl,--whole-archive
  ${PROJECT_NAME}_demo_controllers
  -Wl,--no-whole-archive
)
ament_target_dependencies(ur5e_rt_controller
  rclcpp
  rtc_base
  rtc_controller_manager
  rtc_controllers
)

# ── Install headers ───────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

# ── Install library ───────────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}_demo_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── Install executable ────────────────────────────────────────────────────────
install(TARGETS ur5e_rt_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config ────────────────────────────────────────────────────────────
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Install launch files ──────────────────────────────────────────────────────
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Install Python scripts ───────────────────────────────────────────────────
install(
  PROGRAMS scripts/demo_controller_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_controller_gui
)

install(
  PROGRAMS scripts/motion_editor_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_editor_gui
)

# ── Export ─────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  rtc_controller_interface
  rtc_controllers
  rtc_base
  rtc_msgs
  pinocchio
  urdf_pinocchio_bridge
  yaml-cpp
)
ament_export_include_directories(include)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
