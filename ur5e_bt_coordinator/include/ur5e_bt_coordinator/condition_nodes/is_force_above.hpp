#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <chrono>
#include <memory>

namespace rtc_bt {

/// Check if fingertip forces exceed a threshold.
///
/// Returns SUCCESS when at least `min_fingertips` have force magnitude
/// above `threshold_N` and have been sustained for `sustained_ms`.
///
/// Input ports:
///   - threshold_N (double): force threshold [N] (default 1.5)
///   - min_fingertips (int): minimum fingertips above threshold (default 2)
///   - sustained_ms (int): sustained duration [ms] (default 0, no sustain check)
class IsForceAbove : public BT::ConditionNode {
public:
  IsForceAbove(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::chrono::steady_clock::time_point sustained_start_;
  bool sustained_active_{false};
};

}  // namespace rtc_bt
