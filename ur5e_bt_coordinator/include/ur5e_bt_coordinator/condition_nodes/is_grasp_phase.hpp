#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>

namespace rtc_bt {

/// Check if the Force-PI grasp controller is in a specific phase.
///
/// Returns SUCCESS when `grasp_phase` from GraspState matches the target
/// phase. When the optional `min_duration_s` input port is set to a positive
/// value, the node becomes stateful: it only returns SUCCESS once the target
/// phase has been continuously observed for at least that many seconds. Any
/// tick where the phase does not match resets the internal dwell timer.
///
/// Used to monitor the Force-PI grasp controller state machine from BT trees.
///
/// Input ports:
///   - phase (string): target phase name (default "holding")
///     Valid values: "idle", "approaching", "contact", "force_control",
///                   "holding", "releasing"
///   - min_duration_s (double, default 0.0): when > 0, the target phase must
///     be continuously observed for at least this long before SUCCESS is
///     returned. 0.0 = immediate success on first match (legacy behavior).
class IsGraspPhase : public BT::ConditionNode {
public:
  IsGraspPhase(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;

  // Stateful dwell tracking for the `min_duration_s` feature. `match_active_`
  // is true while the previous tick observed the target phase; `match_start_`
  // captures the wall-clock time of the first tick in the current match run.
  // Both are reset whenever the phase leaves the target or the node is first
  // entered.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time  match_start_{0, 0, RCL_STEADY_TIME};
  bool          match_active_{false};
};

}  // namespace rtc_bt
