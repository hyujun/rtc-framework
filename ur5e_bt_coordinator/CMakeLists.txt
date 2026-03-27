cmake_minimum_required(VERSION 3.16)
project(ur5e_bt_coordinator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Library: all BT nodes + ROS bridge ─────────────────────────────────────
add_library(bt_nodes SHARED
  src/bt_ros_bridge.cpp
  src/nodes/move_to_pose.cpp
  src/nodes/move_to_joints.cpp
  src/nodes/grasp_control.cpp
  src/nodes/track_trajectory.cpp
  src/nodes/set_gains.cpp
  src/nodes/switch_controller.cpp
  src/nodes/compute_offset_pose.cpp
  src/nodes/compute_sweep_trajectory.cpp
  src/nodes/wait_duration.cpp
  src/nodes/is_force_above.cpp
  src/nodes/is_grasped.cpp
  src/nodes/is_object_detected.cpp
)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_nodes
  rclcpp behaviortree_cpp std_msgs geometry_msgs rtc_msgs tf2
)

# ── Executable: coordinator node ───────────────────────────────────────────
add_executable(bt_coordinator_node
  src/bt_coordinator_node.cpp
  src/main.cpp
)
target_include_directories(bt_coordinator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(bt_coordinator_node bt_nodes)
ament_target_dependencies(bt_coordinator_node
  rclcpp behaviortree_cpp std_msgs geometry_msgs rtc_msgs tf2 ament_index_cpp
)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS bt_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS bt_nodes
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
