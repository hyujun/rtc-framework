#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/action_node.h>

namespace rtc_bt {

/// Override the Z coordinate of an input pose with a constant absolute value.
///
/// When the `z` input port is NaN (or omitted), the input pose is passed
/// through unchanged — this acts as a no-op and preserves the caller-supplied
/// Z. Otherwise, the output pose copies the input pose and replaces its Z
/// with the provided constant. X, Y, and orientation are always preserved.
///
/// Intended use: allow the BT user to optionally fix the final Z coordinate
/// of the object grasp goal (e.g. to a known table height) instead of using
/// the noisy Z from vision detection.
///
/// Input ports:
///   - input_pose (Pose6D): base pose
///   - z         (double) : absolute Z [m]; NaN = disabled (pass-through)
/// Output ports:
///   - output_pose (Pose6D): input_pose with Z optionally overridden
class SetPoseZ : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rtc_bt
