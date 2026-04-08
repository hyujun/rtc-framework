#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/condition_node.h>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>

#include <memory>
#include <string>

namespace rtc_bt {

/// Extract and optionally match the shape type from a ShapeEstimate.
///
/// Always outputs shape_type (uint8) and shape_name (string) to the Blackboard.
/// If expected_type is specified, returns SUCCESS only if it matches;
/// otherwise always returns SUCCESS (pure extraction mode).
///
/// Input ports:
///   - estimate (ShapeEstimate): shape estimate from WaitShapeResult
///   - expected_type (string, optional): expected shape name to match
///     ("plane", "sphere", "cylinder", "box")
///
/// Output ports:
///   - shape_type (uint8): numeric shape type (0-4)
///   - shape_name (string): human-readable shape name
///   - confidence (double): estimation confidence [0,1]
class CheckShapeType : public BT::ConditionNode {
public:
  CheckShapeType(const std::string& name, const BT::NodeConfig& config,
                 std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
