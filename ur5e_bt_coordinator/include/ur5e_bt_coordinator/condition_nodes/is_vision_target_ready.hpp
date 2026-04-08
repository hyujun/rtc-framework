#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <memory>

namespace rtc_bt {

/// Check if /world_target_info has valid (non-zero) data.
///
/// Returns SUCCESS if a valid Polygon message has been received on
/// /world_target_info with at least one non-zero point.
/// Returns FAILURE if the topic has not been published or all points are zero.
///
/// Output ports:
///   - pose (Pose6D): world target pose (points[0]=position, points[1]=orientation)
class IsVisionTargetReady : public BT::ConditionNode {
public:
  IsVisionTargetReady(const std::string& name, const BT::NodeConfig& config,
                      std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
