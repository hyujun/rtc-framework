#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

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
  double threshold = getInput<double>("force_threshold_N").value_or(1.0);
  int min_ft = getInput<int>("min_fingertips").value_or(2);

  auto forces = bridge_->GetFingertipForces();
  int count = CountActiveContacts(forces, static_cast<float>(threshold));

  return (count >= min_ft) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
