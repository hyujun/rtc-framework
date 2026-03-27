#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>

namespace rtc_bt {

/// Switch the active controller via /{ns}/select_controller topic.
///
/// Input ports:
///   - controller_name (string): target controller name
///   - timeout_s (double): time to wait for confirmation (default 3.0)
class SwitchController : public BT::StatefulActionNode {
public:
  SwitchController(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::string target_name_;
  double timeout_s_{3.0};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
