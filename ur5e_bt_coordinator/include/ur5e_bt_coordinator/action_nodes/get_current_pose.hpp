#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>

namespace rtc_bt {

/// Read the current TCP pose from the ROS bridge and write it to the Blackboard.
///
/// This is a SyncActionNode — completes in a single tick.
/// Useful for capturing the current EE pose as the starting point for
/// sweep trajectories or tilt sequences.
///
/// Output ports:
///   - pose (Pose6D): current TCP pose [x, y, z, roll, pitch, yaw]
class GetCurrentPose : public BT::SyncActionNode {
public:
  GetCurrentPose(const std::string& name, const BT::NodeConfig& config,
                 std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
