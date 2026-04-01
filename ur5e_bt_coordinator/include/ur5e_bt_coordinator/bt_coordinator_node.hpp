#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/bt_factory.h>

#if __has_include(<behaviortree_cpp/loggers/groot2_publisher.h>)
#define BT_GROOT2_AVAILABLE 1
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>

namespace rtc_bt {

/// Main ROS2 node that ticks a BehaviorTree at a configurable rate.
///
/// Features:
///   - Loads BT XML from the `trees/` directory (supports absolute paths)
///   - Registers all custom BT nodes (action + condition)
///   - Shares a BtRosBridge instance with all BT nodes
///   - Ticks the tree at `tick_rate_hz` (default 20 Hz)
///   - Runtime tree switching via `tree_file` parameter change
///   - Blackboard variable initialization from `bb.*` parameters
///   - Groot2 monitoring via `groot2_port` parameter
///   - Topic health watchdog with configurable timeout
///   - Step-by-step debugging mode
///   - Failure diagnosis logging
///
/// Usage:
///   auto node = std::make_shared<BtCoordinatorNode>(options);
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

  /// Inject bb.* parameters into the tree's root blackboard.
  void InitializeBlackboard();

  /// Log detailed failure diagnosis: which nodes failed and why.
  void LogFailureDiagnosis();

  /// Log topic health status.
  void WatchdogCheck();

  /// Parameter change callback for runtime tree switching and pause control.
  rcl_interfaces::msg::SetParametersResult OnParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  /// Service callback for single-step tick.
  void StepCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::shared_ptr<BtRosBridge> bridge_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Parameters
  std::string tree_file_;
  double tick_rate_hz_{20.0};
  bool repeat_{false};
  double repeat_delay_s_{1.0};
  bool paused_{false};
  bool step_mode_{false};
  int groot2_port_{0};
  double watchdog_timeout_s_{2.0};
  double watchdog_interval_s_{5.0};

  // Repeat state
  rclcpp::TimerBase::SharedPtr repeat_timer_;

  // Watchdog timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Step mode service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_service_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // Groot2 publisher
#ifdef BT_GROOT2_AVAILABLE
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
#endif

  // Step mode pending flag
  bool step_pending_{false};
};

}  // namespace rtc_bt
