#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cctype>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("switch_controller"); }

// Normalize controller name: strip underscores, lowercase.
// e.g. "demo_task_controller" and "DemoTaskController" both become
// "demotaskcontroller".
std::string NormalizeName(const std::string &s) {
  std::string r;
  r.reserve(s.size());
  for (char c : s) {
    if (c != '_') {
      r.push_back(
          static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return r;
}
} // namespace

SwitchController::SwitchController(const std::string &name,
                                   const BT::NodeConfig &config,
                                   std::shared_ptr<BtRosBridge> bridge)
    : BT::SyncActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SwitchController::providedPorts() {
  return {
      BT::InputPort<std::string>("controller_name"),
      BT::InputPort<double>("timeout_s", 3.0, "Timeout [s]"),
  };
}

BT::NodeStatus SwitchController::tick() {
  auto name = getInput<std::string>("controller_name");
  if (!name) {
    RCLCPP_ERROR(logger(), "missing controller_name port");
    throw BT::RuntimeError("SwitchController: missing controller_name");
  }
  const auto target = name.value();
  const auto timeout_s = getInput<double>("timeout_s").value_or(3.0);

  const auto current = bridge_->GetActiveController();
  if (NormalizeName(current) == NormalizeName(target)) {
    RCLCPP_DEBUG(logger(), "already active: %s", target.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "switching: %s -> %s (timeout=%.1fs)", current.c_str(),
              target.c_str(), timeout_s);

  // Sync srv. The bridge's underlying client returns ok only after CM has
  // committed the swap (D-A4) and published the latched
  // /rtc_cm/active_controller_name update, so the active controller is
  // immediately ready for subsequent SetGains parameter calls.
  std::string err;
  if (!bridge_->RequestSwitchController(target, timeout_s, err)) {
    RCLCPP_ERROR(logger(), "switch_controller srv rejected '%s': %s",
                 target.c_str(), err.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace rtc_bt
