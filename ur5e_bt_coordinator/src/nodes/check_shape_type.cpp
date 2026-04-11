#include "ur5e_bt_coordinator/condition_nodes/check_shape_type.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }

std::string ShapeTypeName(uint8_t type) {
  switch (type) {
    case 0: return "unknown";
    case 1: return "plane";
    case 2: return "sphere";
    case 3: return "cylinder";
    case 4: return "box";
    default: return "invalid";
  }
}

std::string ToLower(const std::string& s) {
  std::string result = s;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return result;
}
}  // namespace

CheckShapeType::CheckShapeType(const std::string& name, const BT::NodeConfig& config,
                               std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList CheckShapeType::providedPorts()
{
  return {
    BT::InputPort<shape_estimation_msgs::msg::ShapeEstimate>("estimate",
        "Shape estimate from WaitShapeResult"),
    BT::InputPort<std::string>("expected_type",
        "Expected shape name (plane|sphere|cylinder|box); omit for extraction only"),
    BT::OutputPort<unsigned>("shape_type", "Numeric shape type (0-4)"),
    BT::OutputPort<std::string>("shape_name", "Human-readable shape name"),
    BT::OutputPort<double>("confidence", "Estimation confidence [0,1]"),
  };
}

BT::NodeStatus CheckShapeType::tick()
{
  auto estimate = getInput<shape_estimation_msgs::msg::ShapeEstimate>("estimate");
  if (!estimate) {
    RCLCPP_ERROR(logger(), "[CheckShapeType] missing estimate port: %s",
                 estimate.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto& est = estimate.value();
  const auto name = ShapeTypeName(est.shape_type);

  // Always output to blackboard
  setOutput("shape_type", static_cast<unsigned>(est.shape_type));
  setOutput("shape_name", name);
  setOutput("confidence", est.confidence);

  RCLCPP_INFO(logger(), "[CheckShapeType] type=%s (%u), confidence=%.3f, points=%u",
              name.c_str(), est.shape_type, est.confidence, est.num_points_used);

  // If expected_type is specified, check for match
  auto expected = getInput<std::string>("expected_type");
  if (expected) {
    const auto expected_lower = ToLower(expected.value());
    if (expected_lower != name) {
      RCLCPP_WARN(logger(),
                  "[CheckShapeType] FAILURE: shape mismatch "
                  "(expected=%s, got=%s, confidence=%.3f)",
                  expected_lower.c_str(), name.c_str(), est.confidence);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
