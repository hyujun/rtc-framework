#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Check if the Force-PI grasp controller is in a specific phase.
///
/// Returns SUCCESS when `grasp_phase` from GraspState matches the target phase.
/// Used to monitor the Force-PI grasp controller state machine from BT trees.
///
/// Input ports:
///   - phase (string): target phase name (default "holding")
///     Valid values: "idle", "approaching", "contact", "force_control",
///                   "holding", "releasing"
class IsGraspPhase : public BT::ConditionNode {
public:
  IsGraspPhase(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
