#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

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
  double threshold = getInput<double>("threshold_N").value_or(1.5);
  int min_ft = getInput<int>("min_fingertips").value_or(2);
  int sustained_ms = getInput<int>("sustained_ms").value_or(0);

  auto forces = bridge_->GetFingertipForces();
  int count = CountActiveContacts(forces, static_cast<float>(threshold));
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
