#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>

namespace rtc_bt {

/// Move the arm TCP to a target 6D pose via task-space control.
///
/// Publishes target to /ur5e/joint_goal, then monitors /ur5e/gui_position
/// until the TCP reaches the target within tolerance or timeout.
///
/// Input ports:
///   - target (Pose6D): goal pose [x,y,z,roll,pitch,yaw]
///   - position_tolerance (double): position threshold [m] (default 0.005)
///   - orientation_tolerance (double): orientation threshold [rad] (default 0.05)
///   - timeout_s (double): timeout [s] (default 10.0)
class MoveToPose : public BT::StatefulActionNode {
public:
  MoveToPose(const std::string& name, const BT::NodeConfig& config,
             std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  Pose6D target_;
  double pos_tol_{0.005};
  double rot_tol_{0.05};
  double timeout_s_{10.0};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
