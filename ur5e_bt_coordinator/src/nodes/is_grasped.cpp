#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"

namespace rtc_bt {

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

  // If BT threshold/min_fingertips match controller defaults, use aggregate directly
  if (std::abs(threshold - static_cast<double>(gs.force_threshold)) < 0.01 &&
      min_ft == gs.min_fingertips) {
    return gs.grasp_detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Custom threshold: recount from per-fingertip data
  int count = 0;
  for (const auto& ft : gs.fingertips) {
    if (ft.inference_valid && ft.contact_flag > 0.5f &&
        ft.force_magnitude > static_cast<float>(threshold)) {
      ++count;
    }
  }
  return (count >= min_ft) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
