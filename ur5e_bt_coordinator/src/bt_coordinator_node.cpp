#include "ur5e_bt_coordinator/bt_coordinator_node.hpp"

// Action nodes
#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"

// Condition nodes
#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <filesystem>

namespace rtc_bt {

BtCoordinatorNode::BtCoordinatorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("bt_coordinator", options)
{
  DeclareParameters();
}

void BtCoordinatorNode::Initialize()
{
  // Create ROS bridge — requires shared_from_this(), so must be called
  // after make_shared<BtCoordinatorNode>().
  bridge_ = std::make_shared<BtRosBridge>(shared_from_this());

  RegisterBtNodes();
  LoadTree();

  // Tick timer
  const auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
  tick_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BtCoordinatorNode::TickCallback, this));

  RCLCPP_INFO(get_logger(), "[BtCoordinator] Tree loaded: %s (tick %.1f Hz)",
              tree_file_.c_str(), tick_rate_hz_);
}

void BtCoordinatorNode::DeclareParameters()
{
  tree_file_ = declare_parameter("tree_file", "pick_and_place.xml");
  tick_rate_hz_ = declare_parameter("tick_rate_hz", 20.0);
}

void BtCoordinatorNode::RegisterBtNodes()
{
  auto bridge = bridge_;

  // ── Action nodes ──────────────────────────────────────────────────────
  factory_.registerNodeType<MoveToPose>("MoveToPose", bridge);
  factory_.registerNodeType<MoveToJoints>("MoveToJoints", bridge);
  factory_.registerNodeType<GraspControl>("GraspControl", bridge);
  factory_.registerNodeType<TrackTrajectory>("TrackTrajectory", bridge);
  factory_.registerNodeType<SetGains>("SetGains", bridge);
  factory_.registerNodeType<SwitchController>("SwitchController", bridge);
  factory_.registerNodeType<ComputeOffsetPose>("ComputeOffsetPose");
  factory_.registerNodeType<ComputeSweepTrajectory>("ComputeSweepTrajectory");
  factory_.registerNodeType<WaitDuration>("WaitDuration");

  // ── Condition nodes ───────────────────────────────────────────────────
  factory_.registerNodeType<IsForceAbove>("IsForceAbove", bridge);
  factory_.registerNodeType<IsGrasped>("IsGrasped", bridge);
  factory_.registerNodeType<IsObjectDetected>("IsObjectDetected", bridge);
}

void BtCoordinatorNode::LoadTree()
{
  std::string pkg_share =
      ament_index_cpp::get_package_share_directory("ur5e_bt_coordinator");
  auto tree_path = std::filesystem::path(pkg_share) / "trees" / tree_file_;

  if (!std::filesystem::exists(tree_path)) {
    RCLCPP_ERROR(get_logger(), "Tree file not found: %s", tree_path.c_str());
    throw std::runtime_error("BT tree file not found: " + tree_path.string());
  }

  tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(tree_path.string()));
}

void BtCoordinatorNode::TickCallback()
{
  if (!tree_) return;

  if (bridge_->IsEstopped()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "[BtCoordinator] E-STOP active, tree paused");
    return;
  }

  auto status = tree_->tickOnce();

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(get_logger(), "[BtCoordinator] Tree completed: SUCCESS");
    tick_timer_->cancel();
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(get_logger(), "[BtCoordinator] Tree completed: FAILURE");
    tick_timer_->cancel();
  }
}

}  // namespace rtc_bt
