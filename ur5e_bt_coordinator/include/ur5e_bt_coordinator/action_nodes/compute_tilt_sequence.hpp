#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/action_node.h>

#include <vector>

namespace rtc_bt {

/// Generate a tilt-scan waypoint sequence around a base pose.
///
/// Creates waypoints that oscillate roll and pitch around the base pose,
/// mimicking the tilt phase of ExplorationMotionGenerator. Position is
/// kept constant; only orientation varies.
///
/// The sequence alternates between pitch and roll tilts:
///   step 0: pitch +amplitude
///   step 1: roll  +amplitude
///   step 2: pitch -amplitude
///   step 3: roll  -amplitude
///   step 4: return to base orientation
///   ... (pattern repeats for num_steps)
///
/// Input ports:
///   - base_pose (Pose6D): center pose for tilt oscillation
///   - amplitude_deg (double): tilt amplitude [deg] (default 15.0)
///   - num_steps (int): number of tilt waypoints (default 6)
/// Output ports:
///   - waypoints (vector<Pose6D>): generated tilt waypoint sequence
class ComputeTiltSequence : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rtc_bt
