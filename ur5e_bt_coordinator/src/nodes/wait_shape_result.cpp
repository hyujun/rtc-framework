#include "ur5e_bt_coordinator/action_nodes/wait_shape_result.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }

const char* ShapeTypeName(uint8_t type) {
  switch (type) {
    case 0: return "UNKNOWN";
    case 1: return "PLANE";
    case 2: return "SPHERE";
    case 3: return "CYLINDER";
    case 4: return "BOX";
    default: return "INVALID";
  }
}
}  // namespace

WaitShapeResult::WaitShapeResult(const std::string& name, const BT::NodeConfig& config,
                                 std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList WaitShapeResult::providedPorts()
{
  return {
    BT::InputPort<double>("confidence_threshold", 0.7, "Min confidence [0,1]"),
    BT::InputPort<double>("timeout_s", 10.0, "Timeout [s]"),
    BT::OutputPort<shape_estimation_msgs::msg::ShapeEstimate>("estimate",
        "Shape estimate result"),
  };
}

BT::NodeStatus WaitShapeResult::onStart()
{
  confidence_threshold_ = getInput<double>("confidence_threshold").value_or(0.7);
  timeout_s_ = getInput<double>("timeout_s").value_or(10.0);
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(logger(),
              "[WaitShapeResult] waiting (confidence>=%.2f, timeout=%.1fs)",
              confidence_threshold_, timeout_s_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitShapeResult::onRunning()
{
  shape_estimation_msgs::msg::ShapeEstimate estimate;
  if (bridge_->GetShapeEstimate(estimate)) {
    if (estimate.confidence >= confidence_threshold_ &&
        estimate.shape_type != 0 /* UNKNOWN */) {
      setOutput("estimate", estimate);
      RCLCPP_INFO(logger(),
                  "[WaitShapeResult] success: %s (confidence=%.3f, points=%u, elapsed=%.2fs)",
                  ShapeTypeName(estimate.shape_type),
                  estimate.confidence, estimate.num_points_used,
                  ElapsedSeconds(start_time_));
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(logger(),
                 "[WaitShapeResult] pending: %s (confidence=%.3f, points=%u)",
                 ShapeTypeName(estimate.shape_type),
                 estimate.confidence, estimate.num_points_used);
  }

  if (ElapsedSeconds(start_time_) > timeout_s_) {
    // Timeout — output whatever we have (may be useful for diagnostics)
    if (bridge_->GetShapeEstimate(estimate)) {
      setOutput("estimate", estimate);
      RCLCPP_WARN(logger(),
                  "[WaitShapeResult] timeout with partial: %s (confidence=%.3f)",
                  ShapeTypeName(estimate.shape_type), estimate.confidence);
    } else {
      RCLCPP_WARN(logger(), "[WaitShapeResult] timeout with no estimate");
    }
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitShapeResult::onHalted()
{
  RCLCPP_INFO(logger(), "[WaitShapeResult] halted (elapsed=%.2fs)",
              ElapsedSeconds(start_time_));
}

}  // namespace rtc_bt
