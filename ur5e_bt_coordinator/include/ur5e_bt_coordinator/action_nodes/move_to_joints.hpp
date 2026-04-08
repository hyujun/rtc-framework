#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <vector>

namespace rtc_bt {

/// Move the arm to target joint positions via joint-space control.
///
/// Input ports:
///   - target (std::vector<double>): joint target [q0..q5] in rad
///   - pose_name (string): named arm pose from poses.yaml (overrides target)
///   - tolerance (double): per-joint tolerance [rad] (default 0.01)
///   - timeout_s (double): timeout [s] (default 10.0)
class MoveToJoints : public BT::StatefulActionNode {
public:
  MoveToJoints(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::vector<double> target_;
  double tolerance_{0.01};
  double timeout_s_{10.0};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
