#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Stop buffering ToF snapshot messages and report the collected count.
///
/// The collected data remains accessible via bridge->GetCollectedToFData()
/// until the next StartToFCollection() call clears it.
///
/// Output ports:
///   - count (int): number of ToFSnapshot messages collected
class StopToFCollection : public BT::SyncActionNode {
public:
  StopToFCollection(const std::string& name, const BT::NodeConfig& config,
                    std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
