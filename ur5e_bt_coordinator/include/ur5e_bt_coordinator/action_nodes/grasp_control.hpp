#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rtc_bt {

/// Control the hand for grasping operations.
///
/// Modes:
///   - "open":   Move all motors to target_positions
///   - "close":  Incrementally close all motors at close_speed until max_position
///   - "pinch":  Incrementally close only the joints of pinch_fingers
///   - "preset": Move all motors to target_positions
///
/// In "close"/"pinch" mode, returns RUNNING each tick while incrementing
/// motor targets. Succeeds when all relevant motors reach max_position
/// (caller should use Parallel with IsForceAbove to stop early).
///
/// Input ports:
///   - mode (string): "open", "close", "pinch", "preset"
///   - target_positions (vector<double>): target for "open"/"preset" modes
///   - close_speed (double): closing speed [rad/s] (default 0.3)
///   - max_position (double): max motor position [rad] (default 1.4)
///   - pinch_fingers (string): comma-separated finger names for pinch mode,
///     resolved to hand-joint indices at runtime (default "thumb,index")
///   - timeout_s (double): timeout [s] (default 8.0)
class GraspControl : public BT::StatefulActionNode {
 public:
  GraspControl(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  std::shared_ptr<BtRosBridge> bridge_;
  std::string mode_;
  std::vector<double> hand_target_;
  // Hand-joint indices to close in "pinch" mode, resolved from the
  // pinch_fingers finger names via BtRosBridge::GetFingerJointIndices.
  std::vector<int> pinch_joint_indices_;
  double close_speed_{0.3};
  double max_position_{1.4};
  double timeout_s_{8.0};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_tick_time_;  // for measured dt in close/pinch
};

}  // namespace rtc_bt
