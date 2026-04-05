#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cctype>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }

// Normalize controller name for comparison: strip underscores and lowercase.
// e.g. "demo_task_controller" and "DemoTaskController" both become "demotaskcontroller".
std::string NormalizeName(const std::string& s) {
  std::string result;
  result.reserve(s.size());
  for (char c : s) {
    if (c != '_') result.push_back(
        static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return result;
}
}  // namespace

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
    RCLCPP_ERROR(logger(), "[SwitchController] missing controller_name port");
    throw BT::RuntimeError("SwitchController: missing controller_name");
  }
  target_name_ = name.value();
  timeout_s_ = getInput<double>("timeout_s").value_or(3.0);
  start_time_ = std::chrono::steady_clock::now();

  auto current = bridge_->GetActiveController();
  if (NormalizeName(current) == NormalizeName(target_name_)) {
    RCLCPP_DEBUG(logger(), "[SwitchController] already active: %s", target_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "[SwitchController] switching: %s -> %s (timeout=%.1fs)",
              current.c_str(), target_name_.c_str(), timeout_s_);
  bridge_->PublishSelectController(target_name_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchController::onRunning()
{
  if (NormalizeName(bridge_->GetActiveController()) == NormalizeName(target_name_)) {
    RCLCPP_INFO(logger(), "[SwitchController] active: %s (elapsed=%.2fs)",
                target_name_.c_str(), ElapsedSeconds(start_time_));
    return BT::NodeStatus::SUCCESS;
  }

  if (ElapsedSeconds(start_time_) > timeout_s_) {
    RCLCPP_ERROR(logger(), "[SwitchController] timeout switching to %s (%.1fs)",
                 target_name_.c_str(), timeout_s_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void SwitchController::onHalted()
{
  RCLCPP_INFO(logger(), "[SwitchController] halted (target=%s)", target_name_.c_str());
}

}  // namespace rtc_bt
