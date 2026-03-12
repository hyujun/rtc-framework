cmake_minimum_required(VERSION 3.16)
project(ur5e_hand_udp VERSION 5.2.2 LANGUAGES CXX)

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
find_package(ur5e_rt_base REQUIRED)

# ── Unified node: HandController (request-response polling) ──────────────────
add_executable(hand_udp_node
  src/hand_udp_node.cpp
)
target_include_directories(hand_udp_node PRIVATE include)
ament_target_dependencies(hand_udp_node
  rclcpp
  std_msgs
  ur5e_rt_base
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

ament_package()
