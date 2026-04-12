#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::CondLogger("is_force_above"); }
}  // namespace

IsForceAbove::IsForceAbove(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList IsForceAbove::providedPorts()
{
  return {
    BT::InputPort<double>("threshold_N", 1.5, "Force threshold [N]"),
    BT::InputPort<int>("min_fingertips", 2, "Min fingertips above threshold"),
    BT::InputPort<int>("sustained_ms", 0, "Sustained duration [ms]"),
  };
}

BT::NodeStatus IsForceAbove::tick()
{
  const double threshold = getInput<double>("threshold_N").value_or(1.5);
  const int min_ft = getInput<int>("min_fingertips").value_or(2);
  const int sustained_ms = getInput<int>("sustained_ms").value_or(0);

  // Use 500Hz pre-computed grasp state from controller
  auto gs = bridge_->GetGraspState();

  int count;
  if (std::abs(threshold - static_cast<double>(gs.force_threshold)) < 0.01 &&
      min_ft == gs.min_fingertips) {
    // Controller defaults match — use pre-computed aggregate
    count = gs.num_active_contacts;
  } else {
    // Custom threshold — recount from per-fingertip data
    count = 0;
    for (const auto& ft : gs.fingertips) {
      if (ft.inference_valid && ft.contact_flag > 0.5f &&
          ft.force_magnitude > static_cast<float>(threshold)) {
        ++count;
      }
    }
  }

  bool condition_met = (count >= min_ft);

  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};

  RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                        ::rtc_bt::logging::kThrottleSlowMs,
                        "count=%d/%d max_force=%.2fN threshold=%.2fN",
                        count, min_ft, gs.max_force, threshold);

  if (sustained_ms <= 0) {
    if (condition_met) {
      RCLCPP_INFO(logger(), "triggered (%d fingertips >= %.2fN)", count, threshold);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock, ::rtc_bt::logging::kThrottleSlowMs,
      "%d/%d fingertips >= %.2fN (max_force=%.2fN)",
      count, min_ft, threshold, gs.max_force);
    return BT::NodeStatus::FAILURE;
  }

  // Sustained check
  if (condition_met) {
    if (!sustained_active_) {
      sustained_start_ = std::chrono::steady_clock::now();
      sustained_active_ = true;
      RCLCPP_DEBUG(logger(), "sustained check started (%dms required)", sustained_ms);
    }
    auto elapsed = std::chrono::steady_clock::now() - sustained_start_;
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    if (elapsed_ms >= sustained_ms) {
      RCLCPP_INFO(logger(), "sustained condition met (%dms, %d fingertips)",
                  sustained_ms, count);
      sustained_active_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock, ::rtc_bt::logging::kThrottleSlowMs,
      "condition met but not yet sustained "
      "(%ldms / %dms required, count=%d/%d)",
      static_cast<long>(elapsed_ms), sustained_ms, count, min_ft);
    return BT::NodeStatus::FAILURE;
  }

  sustained_active_ = false;
  RCLCPP_WARN_THROTTLE(
    logger(), steady_clock, ::rtc_bt::logging::kThrottleSlowMs,
    "%d/%d fingertips >= %.2fN "
    "(max_force=%.2fN, sustained_ms=%d not started)",
    count, min_ft, threshold, gs.max_force, sustained_ms);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
