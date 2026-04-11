#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::CondLogger("is_grasped"); }
}  // namespace

IsGrasped::IsGrasped(const std::string& name, const BT::NodeConfig& config,
                     std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList IsGrasped::providedPorts()
{
  return {
    BT::InputPort<double>("force_threshold_N", 1.0, "Min force per fingertip [N]"),
    BT::InputPort<int>("min_fingertips", 2, "Min fingertips in contact"),
  };
}

BT::NodeStatus IsGrasped::tick()
{
  const double threshold = getInput<double>("force_threshold_N").value_or(1.0);
  const int min_ft = getInput<int>("min_fingertips").value_or(2);

  // Use 500Hz pre-computed grasp state from controller
  auto gs = bridge_->GetGraspState();

  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};

  // If BT threshold/min_fingertips match controller defaults, use aggregate directly
  if (std::abs(threshold - static_cast<double>(gs.force_threshold)) < 0.01 &&
      min_ft == gs.min_fingertips) {
    RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                          ::rtc_bt::logging::kThrottleFastMs,
                          "grasp_detected=%s contacts=%d max_force=%.2fN",
                          gs.grasp_detected ? "true" : "false",
                          gs.num_active_contacts, gs.max_force);
    if (gs.grasp_detected) {
      RCLCPP_INFO(logger(), "grasp confirmed (contacts=%d max_force=%.2fN)",
                  gs.num_active_contacts, gs.max_force);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN_THROTTLE(logger(), steady_clock,
                         ::rtc_bt::logging::kThrottleFastMs,
                         "not grasped (contacts=%d/%d max_force=%.2fN/%.2fN)",
                         gs.num_active_contacts, gs.min_fingertips,
                         gs.max_force, gs.force_threshold);
    return BT::NodeStatus::FAILURE;
  }

  // Custom threshold: recount from per-fingertip data
  int count = 0;
  float max_force = 0.0f;
  for (const auto& ft : gs.fingertips) {
    if (ft.inference_valid && ft.contact_flag > 0.5f &&
        ft.force_magnitude > static_cast<float>(threshold)) {
      ++count;
    }
    if (ft.force_magnitude > max_force) max_force = ft.force_magnitude;
  }

  bool grasped = (count >= min_ft);
  RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                        ::rtc_bt::logging::kThrottleFastMs,
                        "count=%d/%d threshold=%.2fN", count, min_ft, threshold);
  if (grasped) {
    RCLCPP_INFO(logger(), "grasp confirmed (custom: %d fingertips >= %.2fN)",
                count, threshold);
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN_THROTTLE(logger(), steady_clock,
                       ::rtc_bt::logging::kThrottleFastMs,
                       "custom check failed "
                       "(%d/%d fingertips >= %.2fN, max_force=%.2fN)",
                       count, min_ft, threshold, static_cast<double>(max_force));
  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
