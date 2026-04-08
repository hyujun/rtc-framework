#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Trigger shape estimation via /shape/trigger topic and /shape/clear service.
///
/// When command is "start", clears accumulated data first (via /shape/clear
/// service + ClearShapeEstimate), then publishes "start" to /shape/trigger.
/// For other commands ("stop", "pause", "resume", "single"), publishes directly.
///
/// Input ports:
///   - command (string): trigger command ("start", "stop", "pause", "resume", "single")
class TriggerShapeEstimation : public BT::SyncActionNode {
public:
  TriggerShapeEstimation(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
