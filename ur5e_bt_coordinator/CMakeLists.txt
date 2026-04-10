cmake_minimum_required(VERSION 3.16)
project(ur5e_bt_coordinator VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(shape_estimation_msgs REQUIRED)
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
  src/nodes/set_pose_z.cpp
  src/nodes/compute_sweep_trajectory.cpp
  src/nodes/compute_tilt_sequence.cpp
  src/nodes/get_current_pose.cpp
  src/nodes/wait_duration.cpp
  src/nodes/is_force_above.cpp
  src/nodes/is_grasp_phase.cpp
  src/nodes/is_grasped.cpp
  src/nodes/is_object_detected.cpp
  src/nodes/is_vision_target_ready.cpp
  src/nodes/move_finger.cpp
  src/nodes/flex_extend_finger.cpp
  src/nodes/set_hand_pose.cpp
  src/nodes/ur5e_hold_pose.cpp
  src/nodes/move_opposition.cpp
  src/nodes/trigger_shape_estimation.cpp
  src/nodes/wait_shape_result.cpp
  src/nodes/check_shape_type.cpp
)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_nodes
  rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs
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
  rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs ament_index_cpp
)

# ── Executable: offline tree validator ─────────────────────────────────────
add_executable(validate_tree
  src/validate_tree.cpp
)
target_include_directories(validate_tree PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(validate_tree bt_nodes)
ament_target_dependencies(validate_tree
  rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs ament_index_cpp
)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS bt_coordinator_node validate_tree
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
install(DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Tier 1: Pure computation tests (no ROS2 fixture needed)
  set(TIER1_TESTS
    test_bt_types
    test_bt_utils
    test_compute_offset_pose
    test_set_pose_z
    test_compute_sweep_trajectory
    test_wait_duration
  )
  foreach(test_name ${TIER1_TESTS})
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} bt_nodes)
    target_include_directories(${test_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    )
    ament_target_dependencies(${test_name}
      rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs
    )
  endforeach()

  # Tier 2: ROS2 test fixture tests (use test_helpers.hpp)
  set(TIER2_TESTS
    test_move_to_pose
    test_move_to_joints
    test_switch_controller
    test_set_gains
    test_grasp_control
    test_condition_nodes
    test_hand_nodes
    test_shape_nodes
  )
  foreach(test_name ${TIER2_TESTS})
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} bt_nodes)
    target_include_directories(${test_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    )
    ament_target_dependencies(${test_name}
      rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs
    )
  endforeach()

  # Tier 3: XML tree validation
  ament_add_gtest(test_tree_validation test/test_tree_validation.cpp)
  target_link_libraries(test_tree_validation bt_nodes)
  target_include_directories(test_tree_validation PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  )
  ament_target_dependencies(test_tree_validation
    rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs ament_index_cpp
  )
endif()

ament_package()
