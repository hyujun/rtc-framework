#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <memory>

namespace rtc_bt {

/// Check if an object is currently grasped.
///
/// Verifies both force (fingertip contact) and position (hand not fully open).
///
/// Input ports:
///   - force_threshold_N (double): min force per fingertip [N] (default 1.0)
///   - min_fingertips (int): min fingertips in contact (default 2)
class IsGrasped : public BT::ConditionNode {
public:
  IsGrasped(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
