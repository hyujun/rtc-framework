#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace rtc_bt {

/// Main ROS2 node that ticks a BehaviorTree at a configurable rate.
///
/// - Loads BT XML from the `trees/` directory
/// - Registers all custom BT nodes (action + condition)
/// - Shares a BtRosBridge instance with all BT nodes
/// - Ticks the tree at `tick_rate_hz` (default 20 Hz)
///
/// Usage:
///   auto node = std::make_shared<BtCoordinatorNode>();
///   node->Initialize();  // must call after construction (needs shared_from_this)
///   rclcpp::spin(node);
class BtCoordinatorNode : public rclcpp::Node {
public:
  explicit BtCoordinatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// Two-phase init: call after make_shared (shared_from_this requires it).
  void Initialize();

private:
  void DeclareParameters();
  void RegisterBtNodes();
  void LoadTree();
  void TickCallback();

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::shared_ptr<BtRosBridge> bridge_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Parameters
  std::string tree_file_;
  double tick_rate_hz_{20.0};
  bool repeat_{false};
  double repeat_delay_s_{1.0};

  // Repeat state
  rclcpp::TimerBase::SharedPtr repeat_timer_;
};

}  // namespace rtc_bt
