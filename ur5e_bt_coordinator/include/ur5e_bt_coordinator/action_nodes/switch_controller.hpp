#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>

namespace rtc_bt {

/// Switch the active controller via the /rtc_cm/switch_controller srv.
///
/// StatefulActionNode (async): onStart fires the switch request without
/// blocking and onRunning polls the response future with wait_for(0) across
/// ticks. A blocking wait here would deadlock the single-threaded executor
/// that must dispatch the very response the tick is waiting on.
///
/// Input ports:
///   - controller_name (string): target controller name
///   - timeout_s (double): switch timeout [s] (default 3.0)
///
/// Returns SUCCESS once the srv responds with ok=true (CM has committed the
/// swap and published the latched /rtc_cm/active_controller_name). Already-
/// active target short-circuits to SUCCESS on the first tick. FAILURE on
/// E-STOP / unknown name (srv ok=false) or timeout.
class SwitchController : public BT::StatefulActionNode {
 public:
  SwitchController(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::string target_;
  double timeout_s_{3.0};
  std::chrono::steady_clock::time_point start_time_;
  BtRosBridge::SwitchRequestHandle handle_;  // invalid future until sent
  bool sent_{false};
};

}  // namespace rtc_bt
