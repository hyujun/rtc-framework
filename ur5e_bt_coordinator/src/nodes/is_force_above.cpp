#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"

#include <cmath>

namespace rtc_bt {

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

  if (sustained_ms <= 0) {
    return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Sustained check
  if (condition_met) {
    if (!sustained_active_) {
      sustained_start_ = std::chrono::steady_clock::now();
      sustained_active_ = true;
    }
    auto elapsed = std::chrono::steady_clock::now() - sustained_start_;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
        >= sustained_ms) {
      sustained_active_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  sustained_active_ = false;
  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
