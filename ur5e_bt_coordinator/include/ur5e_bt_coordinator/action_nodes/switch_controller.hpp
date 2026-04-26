#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>

namespace rtc_bt {

/// Switch the active controller via the /rtc_cm/switch_controller srv (sync).
///
/// Input ports:
///   - controller_name (string): target controller name
///   - timeout_s (double): switch timeout [s] (default 3.0)
///   - load_gains (bool): request current gains after switch (default true).
///     When true, the node requests the active controller's current gains
///     and stores them on the Blackboard as "current_gains" so that a
///     subsequent SetGains node can use them as the base instead of
///     hard-coded defaults.
///
/// Output ports:
///   - current_gains (vector<double>): gains loaded from the controller
///     (only written when load_gains is true and gains are received)
class SwitchController : public BT::StatefulActionNode {
public:
  SwitchController(const std::string &name, const BT::NodeConfig &config,
                   std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::string target_name_;
  double timeout_s_{3.0};
  bool load_gains_{true};
  bool switch_confirmed_{false};
  bool gains_requested_{false};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace rtc_bt
