cmake_minimum_required(VERSION 3.16)
project(ur5e_hand_udp VERSION 1.0.0 LANGUAGES CXX)

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

# ── Shared library: HandUdpReceiver + HandUdpSender implementations ────────────
add_library(ur5e_hand_udp_lib STATIC
  src/hand_udp_receiver.cpp
  src/hand_udp_sender.cpp
)
target_include_directories(ur5e_hand_udp_lib PUBLIC include)
ament_target_dependencies(ur5e_hand_udp_lib ur5e_rt_base)

# ── hand_udp_receiver_node ─────────────────────────────────────────────────────
add_executable(hand_udp_receiver_node
  src/hand_udp_receiver_node.cpp
)
target_link_libraries(hand_udp_receiver_node ur5e_hand_udp_lib)
ament_target_dependencies(hand_udp_receiver_node
  rclcpp
  std_msgs
  ur5e_rt_base
)

# ── hand_udp_sender_node ───────────────────────────────────────────────────────
add_executable(hand_udp_sender_node
  src/hand_udp_sender_node.cpp
)
target_link_libraries(hand_udp_sender_node ur5e_hand_udp_lib)
ament_target_dependencies(hand_udp_sender_node
  rclcpp
  std_msgs
  ur5e_rt_base
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  hand_udp_receiver_node
  hand_udp_sender_node
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
