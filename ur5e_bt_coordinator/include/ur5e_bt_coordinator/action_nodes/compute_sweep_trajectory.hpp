#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/action_node.h>

#include <vector>

namespace rtc_bt {

/// Compute a sweep trajectory (arc path) for towel unfolding.
///
/// Generates a series of waypoints from start_pose, sweeping in the given
/// direction with an arc profile (rises then descends).
///
/// Input ports:
///   - start_pose (Pose6D): starting pose
///   - direction_x, direction_y (double): sweep direction (normalized)
///   - distance (double): total sweep distance [m]
///   - arc_height (double): peak height offset during sweep [m]
///   - num_waypoints (int): number of waypoints (default 8)
/// Output ports:
///   - waypoints (vector<Pose6D>): generated waypoint sequence
class ComputeSweepTrajectory : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rtc_bt
