#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/action_node.h>

namespace rtc_bt {

/// Compute a new pose by applying a 6-DOF offset to an input pose.
///
/// Translation is always additive (output.x = input.x + offset_x, etc.).
///
/// Rotation has three modes selected by `rotation_mode`:
///   - "add"        : output.rpy = input.rpy + offset_rpy  (component-wise sum;
///                    intuitive only for small offsets, ignores gimbal lock)
///   - "quat_body"  : output = input * offset   (compose in TCP/body frame)
///   - "quat_world" : output = offset * input   (compose in world frame)
///
/// Quat modes use Eigen ZYX-intrinsic conversion (= ROS standard RPY) and
/// emit a warning when the result is within 1e-3 rad of pitch = ±π/2.
///
/// Input ports:
///   - input_pose (Pose6D): base pose
///   - offset_x, offset_y, offset_z (double): translation offsets [m]
///   - offset_roll, offset_pitch, offset_yaw (double): rotation offsets [rad]
///   - rotation_mode (string): "add" | "quat_body" | "quat_world" (default "add")
/// Output ports:
///   - output_pose (Pose6D): resulting pose
class ComputeOffsetPose : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rtc_bt
