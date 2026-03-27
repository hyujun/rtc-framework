#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>

namespace rtc_bt {

/// Wait for a specified duration.
///
/// Input ports:
///   - duration_s (double): wait time [s]
class WaitDuration : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  double duration_s_{0.5};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
