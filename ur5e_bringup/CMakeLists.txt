cmake_minimum_required(VERSION 3.16)
project(ur5e_bringup VERSION 0.1.0 LANGUAGES CXX)

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
find_package(rtc_controller_interface REQUIRED)
find_package(rtc_base REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pinocchio (suppress noisy warnings)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Demo controllers (STATIC library) ────────────────────────────────────────
add_library(ur5e_bringup_controllers STATIC
  src/controllers/demo_joint_controller.cpp
  src/controllers/demo_task_controller.cpp
)
target_include_directories(ur5e_bringup_controllers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ur5e_bringup_controllers
  PUBLIC
    pinocchio::pinocchio
    Eigen3::Eigen
    yaml-cpp
)
ament_target_dependencies(ur5e_bringup_controllers
  PUBLIC
    rclcpp
    rtc_controller_interface
    rtc_base
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS ur5e_bringup_controllers
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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Export ─────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(ur5e_bringup_controllers)
ament_export_dependencies(
  rclcpp
  rtc_controller_interface
  rtc_base
  pinocchio
  Eigen3
  yaml-cpp
)

ament_package()
