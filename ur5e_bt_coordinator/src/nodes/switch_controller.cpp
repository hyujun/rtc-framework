#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"

namespace rtc_bt {

SwitchController::SwitchController(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList SwitchController::providedPorts()
{
  return {
    BT::InputPort<std::string>("controller_name"),
    BT::InputPort<double>("timeout_s", 3.0, "Timeout [s]"),
  };
}

BT::NodeStatus SwitchController::onStart()
{
  auto name = getInput<std::string>("controller_name");
  if (!name) {
    throw BT::RuntimeError("SwitchController: missing controller_name");
  }
  target_name_ = name.value();
  timeout_s_ = getInput<double>("timeout_s").value_or(3.0);
  start_time_ = std::chrono::steady_clock::now();

  bridge_->PublishSelectController(target_name_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchController::onRunning()
{
  if (bridge_->GetActiveController() == target_name_) {
    return BT::NodeStatus::SUCCESS;
  }

  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (std::chrono::duration<double>(elapsed).count() > timeout_s_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
