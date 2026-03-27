#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"

namespace rtc_bt {

IsObjectDetected::IsObjectDetected(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList IsObjectDetected::providedPorts()
{
  return {
    BT::OutputPort<Pose6D>("pose"),
  };
}

BT::NodeStatus IsObjectDetected::tick()
{
  Pose6D pose;
  if (bridge_->GetObjectPose(pose)) {
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
