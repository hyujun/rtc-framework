cmake_minimum_required(VERSION 3.16)
project(ur5e_bt_coordinator VERSION 5.17.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Coverage (gcov/lcov) ─────────────────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov code coverage" OFF)
if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtc_msgs REQUIRED)
find_package(shape_estimation_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# ── Library: all BT nodes + ROS bridge ─────────────────────────────────────
add_library(bt_nodes SHARED
  src/bt_ros_bridge.cpp
  src/bt_node_registration.cpp
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
  src/nodes/start_tof_collection.cpp
  src/nodes/stop_tof_collection.cpp
  src/nodes/process_search_data.cpp
)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_nodes
  rclcpp rclcpp_lifecycle behaviortree_cpp std_msgs std_srvs geometry_msgs sensor_msgs rtc_msgs shape_estimation_msgs tf2 tf2_ros tf2_msgs
)
target_link_libraries(bt_nodes Eigen3::Eigen)

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
  rclcpp rclcpp_lifecycle behaviortree_cpp std_msgs std_srvs geometry_msgs sensor_msgs rtc_msgs shape_estimation_msgs tf2 tf2_ros tf2_msgs ament_index_cpp
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
  # Every test here links bt_nodes, sees the same include roots, and needs the
  # same dependency floor. Only two things genuinely vary — extra ament deps and
  # extra sources — so a helper makes those the only thing a test declares.
  # Hand-rolling the whole block per target is what let the tiers drift apart.
  set(BT_TEST_DEPS
    rclcpp behaviortree_cpp std_msgs std_srvs geometry_msgs rtc_msgs shape_estimation_msgs
  )
  # Additional deps for tests that build a BtRosBridge / BtCoordinatorNode
  # themselves instead of reaching them through bt_nodes' registered nodes:
  # the LifecycleNode plus the tf/sensor types on the bridge's own surface.
  set(BT_BRIDGE_DEPS rclcpp_lifecycle sensor_msgs tf2 tf2_ros tf2_msgs)

  # add_bt_gtest(<name> [EXTRA_DEPS ...] [EXTRA_SOURCES ...])
  #   Sources default to test/<name>.cpp.
  function(add_bt_gtest test_name)
    cmake_parse_arguments(ARG "" "" "EXTRA_DEPS;EXTRA_SOURCES" ${ARGN})
    ament_add_gtest(${test_name} test/${test_name}.cpp ${ARG_EXTRA_SOURCES})
    target_link_libraries(${test_name} bt_nodes)
    target_include_directories(${test_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    )
    ament_target_dependencies(${test_name} ${BT_TEST_DEPS} ${ARG_EXTRA_DEPS})
  endfunction()

  set(TIER1_TESTS
    test_bt_types
    test_bt_utils
    test_topic_naming
    test_finger_resolver
    test_node_registration
    test_compute_offset_pose
    test_set_pose_z
    test_compute_sweep_trajectory
    test_compute_tilt_sequence
    test_wait_duration
  )
  foreach(test_name ${TIER1_TESTS})
    add_bt_gtest(${test_name})
  endforeach()

  # Tier 2 inject: DDS-free state-inject tests (use inject_fixture.hpp) —
  # cached-state consumers driven through the BridgeStateInjector handler seam,
  # no cross-node pub/sub delivery (issue #154).
  set(TIER2_INJECT
    test_move_to_pose
    test_move_to_joints
    test_condition_nodes
    test_hand_nodes
    test_shape_nodes
    test_data_nodes
  )
  # Tier 2 e2e: real-DDS fixture tests (use test_helpers.hpp) — the
  # service/rebind path itself is the test subject, so these keep the mock
  # controllers + background spin.
  set(TIER2_E2E
    test_switch_controller
    test_set_gains
    test_service_singlethread
    test_grasp_control
  )
  foreach(test_name ${TIER2_INJECT} ${TIER2_E2E})
    add_bt_gtest(${test_name})
  endforeach()

  # Tier 3: XML tree validation — ament_index_cpp resolves the tree share dir.
  add_bt_gtest(test_tree_validation EXTRA_DEPS ament_index_cpp)

  # Rewire-gap regression (issue #158) — drives the real BtCoordinatorNode
  # lifecycle + tick/step paths, so it compiles the node TU in (bt_coordinator_node
  # is an executable, not a library) and needs ament_index_cpp for LoadTree.
  # DDS-free: the active-controller transition is injected via the bridge
  # handler seam, so this stays out of the e2e tier.
  add_bt_gtest(test_rewire_gate
    EXTRA_DEPS ${BT_BRIDGE_DEPS} ament_index_cpp
    EXTRA_SOURCES src/bt_coordinator_node.cpp
  )

  # on_configure safety-net coverage — capability gating + finger_map range
  # check, exercised at the RegisterBtNodes / BtRosBridge level (bt_nodes).
  add_bt_gtest(test_lifecycle_config EXTRA_DEPS ${BT_BRIDGE_DEPS})
endif()

ament_package()
