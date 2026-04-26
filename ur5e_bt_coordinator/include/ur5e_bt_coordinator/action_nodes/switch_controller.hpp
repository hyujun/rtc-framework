#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Switch the active controller via the /rtc_cm/switch_controller srv (sync).
///
/// Input ports:
///   - controller_name (string): target controller name
///   - timeout_s (double): switch timeout [s] (default 3.0)
///
/// Returns SUCCESS once the srv responds with ok=true (CM has committed
/// the swap and published the latched /rtc_cm/active_controller_name).
/// FAILURE on E-STOP, unknown name, or timeout.
class SwitchController : public BT::SyncActionNode {
public:
  SwitchController(const std::string &name, const BT::NodeConfig &config,
                   std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

} // namespace rtc_bt
