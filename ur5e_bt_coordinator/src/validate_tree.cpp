/// Offline BT tree validation tool.
///
/// Validates a BehaviorTree XML file against the registered node types
/// without launching a ROS2 node or connecting to any topics.
///
/// Usage:
///   ros2 run ur5e_bt_coordinator validate_tree <tree_file.xml>
///   ros2 run ur5e_bt_coordinator validate_tree trees/pick_and_place.xml
///
/// Exit codes:
///   0 = valid, 1 = invalid, 2 = file not found

#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_tilt_sequence.hpp"
#include "ur5e_bt_coordinator/action_nodes/get_current_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"
#include "ur5e_bt_coordinator/action_nodes/trigger_shape_estimation.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_shape_result.hpp"
#include "ur5e_bt_coordinator/condition_nodes/check_shape_type.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_vision_target_ready.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <filesystem>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: validate_tree <tree_file.xml>\n"
              << "\n"
              << "Validates a BehaviorTree XML file against registered node types.\n"
              << "The file can be:\n"
              << "  - An absolute path\n"
              << "  - A relative path (resolved from package share/trees/)\n"
              << "\n"
              << "Exit codes: 0=valid, 1=invalid, 2=file not found\n";
    return 1;
  }

  std::string input = argv[1];
  std::filesystem::path tree_path;

  if (std::filesystem::path(input).is_absolute()) {
    tree_path = input;
  } else {
    try {
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("ur5e_bt_coordinator");
      tree_path = std::filesystem::path(pkg_share) / "trees" / input;
    } catch (...) {
      // If package not found, try relative to CWD
      tree_path = input;
    }
  }

  if (!std::filesystem::exists(tree_path)) {
    std::cerr << "[ERROR] File not found: " << tree_path << "\n";
    return 2;
  }

  std::cout << "[validate_tree] Checking: " << tree_path << "\n";

  // Register all node types with a null bridge (validation only, no ROS comms)
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;  // nullptr

  // Action nodes
  factory.registerNodeType<rtc_bt::MoveToPose>("MoveToPose", null_bridge);
  factory.registerNodeType<rtc_bt::MoveToJoints>("MoveToJoints", null_bridge);
  factory.registerNodeType<rtc_bt::GraspControl>("GraspControl", null_bridge);
  factory.registerNodeType<rtc_bt::TrackTrajectory>("TrackTrajectory", null_bridge);
  factory.registerNodeType<rtc_bt::SetGains>("SetGains", null_bridge);
  factory.registerNodeType<rtc_bt::SwitchController>("SwitchController", null_bridge);
  factory.registerNodeType<rtc_bt::ComputeOffsetPose>("ComputeOffsetPose");
  factory.registerNodeType<rtc_bt::ComputeSweepTrajectory>("ComputeSweepTrajectory");
  factory.registerNodeType<rtc_bt::ComputeTiltSequence>("ComputeTiltSequence");
  factory.registerNodeType<rtc_bt::GetCurrentPose>("GetCurrentPose", null_bridge);
  factory.registerNodeType<rtc_bt::WaitDuration>("WaitDuration");
  factory.registerNodeType<rtc_bt::MoveFinger>("MoveFinger", null_bridge);
  factory.registerNodeType<rtc_bt::FlexExtendFinger>("FlexExtendFinger", null_bridge);
  factory.registerNodeType<rtc_bt::SetHandPose>("SetHandPose", null_bridge);
  factory.registerNodeType<rtc_bt::UR5eHoldPose>("UR5eHoldPose", null_bridge);
  factory.registerNodeType<rtc_bt::MoveOpposition>("MoveOpposition", null_bridge);
  factory.registerNodeType<rtc_bt::TriggerShapeEstimation>("TriggerShapeEstimation", null_bridge);
  factory.registerNodeType<rtc_bt::WaitShapeResult>("WaitShapeResult", null_bridge);

  // Condition nodes
  factory.registerNodeType<rtc_bt::IsForceAbove>("IsForceAbove", null_bridge);
  factory.registerNodeType<rtc_bt::IsGrasped>("IsGrasped", null_bridge);
  factory.registerNodeType<rtc_bt::IsObjectDetected>("IsObjectDetected", null_bridge);
  factory.registerNodeType<rtc_bt::IsVisionTargetReady>("IsVisionTargetReady", null_bridge);
  factory.registerNodeType<rtc_bt::CheckShapeType>("CheckShapeType", null_bridge);

  try {
    auto tree = factory.createTreeFromFile(tree_path.string());

    // Count nodes
    std::size_t total_nodes = 0;
    for (const auto& subtree : tree.subtrees) {
      total_nodes += subtree->nodes.size();
    }

    std::cout << "[validate_tree] VALID ✓\n"
              << "  Subtrees: " << tree.subtrees.size() << "\n"
              << "  Total nodes: " << total_nodes << "\n";

    // List registered node types for reference
    std::cout << "\n  Registered node types:\n";
    auto manifests = factory.manifests();
    for (const auto& [name, manifest] : manifests) {
      std::cout << "    - " << name;
      if (!manifest.ports.empty()) {
        std::cout << " (";
        bool first = true;
        for (const auto& [port_name, port_info] : manifest.ports) {
          if (!first) std::cout << ", ";
          std::cout << port_name;
          first = false;
        }
        std::cout << ")";
      }
      std::cout << "\n";
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[validate_tree] INVALID ✗\n"
              << "  Error: " << e.what() << "\n";
    return 1;
  }
}
