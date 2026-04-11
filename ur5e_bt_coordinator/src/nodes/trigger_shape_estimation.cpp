#include "ur5e_bt_coordinator/action_nodes/trigger_shape_estimation.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("trigger_shape_estimation"); }
}  // namespace

TriggerShapeEstimation::TriggerShapeEstimation(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList TriggerShapeEstimation::providedPorts()
{
  return {
    BT::InputPort<std::string>("command", "start",
                                "Trigger command: start|stop|pause|resume|single"),
  };
}

BT::NodeStatus TriggerShapeEstimation::tick()
{
  auto command = getInput<std::string>("command");
  if (!command) {
    RCLCPP_ERROR(logger(), "missing command port");
    throw BT::RuntimeError("TriggerShapeEstimation: missing command");
  }

  const auto& cmd = command.value();

  if (cmd == "start") {
    // Clear accumulated point cloud and cached estimate before starting
    bridge_->CallShapeClear();
    bridge_->ClearShapeEstimate();
    RCLCPP_INFO(logger(), "cleared data, starting estimation");
  }

  bridge_->PublishShapeTrigger(cmd);
  RCLCPP_INFO(logger(), "command='%s'", cmd.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
