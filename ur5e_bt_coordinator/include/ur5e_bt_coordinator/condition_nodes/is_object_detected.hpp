#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <memory>

namespace rtc_bt {

/// Check if vision has detected an object and output its pose.
///
/// Returns SUCCESS if /world_target_info has valid (non-zero) data.
/// Position from /world_target_info, orientation from current TCP pose.
///
/// Output ports:
///   - pose (Pose6D): detected object pose (position from vision, orientation from TCP)
class IsObjectDetected : public BT::ConditionNode {
public:
  IsObjectDetected(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
