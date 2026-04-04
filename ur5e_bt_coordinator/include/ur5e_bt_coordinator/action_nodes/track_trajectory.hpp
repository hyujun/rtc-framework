#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <vector>

namespace rtc_bt {

/// Track a sequence of waypoints in task space.
///
/// Publishes each waypoint to /ur5e/joint_goal and waits until the TCP
/// is within tolerance before advancing to the next waypoint.
///
/// Input ports:
///   - waypoints (vector<Pose6D>): ordered waypoints
///   - position_tolerance (double): per-waypoint tolerance [m] (default 0.01)
///   - timeout_s (double): total timeout [s] (default 30.0)
class TrackTrajectory : public BT::StatefulActionNode {
public:
  TrackTrajectory(const std::string& name, const BT::NodeConfig& config,
                  std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::vector<Pose6D> waypoints_;
  std::size_t current_idx_{0};
  double pos_tol_{0.01};
  double timeout_s_{30.0};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
