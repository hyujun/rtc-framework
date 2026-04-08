#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>

#include <chrono>
#include <memory>

namespace rtc_bt {

/// Wait for a shape estimation result that meets the confidence threshold.
///
/// Polls the cached ShapeEstimate from BtRosBridge until confidence >= threshold
/// or timeout expires. On success, outputs the full ShapeEstimate message
/// to the Blackboard for downstream nodes.
///
/// Input ports:
///   - confidence_threshold (double): minimum confidence [0,1] (default 0.7)
///   - timeout_s (double): maximum wait time [s] (default 10.0)
///
/// Output ports:
///   - estimate (ShapeEstimate): the shape estimate message
class WaitShapeResult : public BT::StatefulActionNode {
public:
  WaitShapeResult(const std::string& name, const BT::NodeConfig& config,
                  std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  double confidence_threshold_{0.7};
  double timeout_s_{10.0};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
