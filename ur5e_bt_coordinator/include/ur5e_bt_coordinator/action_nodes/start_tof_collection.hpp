#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Start buffering ToF snapshot messages from /tof/snapshot (500 Hz).
///
/// Clears any previously collected data and begins accumulation.
/// Safe to call even if a prior collection was interrupted (E-STOP, halt).
/// Pair with StopToFCollection after the motion phase completes.
///
/// This is a SyncActionNode — completes in a single tick.
class StartToFCollection : public BT::SyncActionNode {
public:
  StartToFCollection(const std::string& name, const BT::NodeConfig& config,
                     std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
