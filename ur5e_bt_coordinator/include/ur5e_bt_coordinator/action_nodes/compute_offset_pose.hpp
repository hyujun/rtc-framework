#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/action_node.h>

namespace rtc_bt {

/// Compute a new pose by applying an offset to an input pose.
///
/// Input ports:
///   - input_pose (Pose6D): base pose
///   - offset_x, offset_y, offset_z (double): translation offsets [m]
/// Output ports:
///   - output_pose (Pose6D): resulting pose (orientation preserved)
class ComputeOffsetPose : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rtc_bt
