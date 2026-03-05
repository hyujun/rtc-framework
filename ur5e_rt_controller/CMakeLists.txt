cmake_minimum_required(VERSION 3.16)
project(ur5e_rt_controller VERSION 5.0.0 LANGUAGES CXX)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ur5e_rt_base REQUIRED)

# ── custom_controller ──────────────────────────────────────────────────────────
add_executable(custom_controller
  src/custom_controller.cpp
)
target_include_directories(custom_controller PRIVATE include)
ament_target_dependencies(custom_controller
  rclcpp
  std_msgs
  sensor_msgs
  realtime_tools
  ur5e_rt_base
)
target_link_libraries(custom_controller pinocchio::pinocchio)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS custom_controller
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

ament_package()
