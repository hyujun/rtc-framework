#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cctype>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("switch_controller"); }

// Normalize controller name for comparison: strip underscores and lowercase.
// e.g. "demo_task_controller" and "DemoTaskController" both become
// "demotaskcontroller".
std::string NormalizeName(const std::string &s) {
  std::string result;
  result.reserve(s.size());
  for (char c : s) {
    if (c != '_')
      result.push_back(
          static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return result;
}
} // namespace

SwitchController::SwitchController(const std::string &name,
                                   const BT::NodeConfig &config,
                                   std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SwitchController::providedPorts() {
  return {
      BT::InputPort<std::string>("controller_name"),
      BT::InputPort<double>("timeout_s", 3.0, "Timeout [s]"),
      BT::InputPort<bool>("load_gains", true,
                          "Request current gains after switch"),
      BT::InputPort<bool>("use_service", true,
                          "Use /rtc_cm/switch_controller srv (D-A6 rollback "
                          "knob; removed in Phase 5)"),
      BT::OutputPort<std::vector<double>>("current_gains",
                                          "Gains loaded from controller"),
  };
}

BT::NodeStatus SwitchController::onStart() {
  auto name = getInput<std::string>("controller_name");
  if (!name) {
    RCLCPP_ERROR(logger(), "missing controller_name port");
    throw BT::RuntimeError("SwitchController: missing controller_name");
  }
  target_name_ = name.value();
  timeout_s_ = getInput<double>("timeout_s").value_or(3.0);
  load_gains_ = getInput<bool>("load_gains").value_or(true);
  use_service_ = getInput<bool>("use_service").value_or(true);
  switch_confirmed_ = false;
  gains_requested_ = false;
  start_time_ = std::chrono::steady_clock::now();

  auto current = bridge_->GetActiveController();
  if (NormalizeName(current) == NormalizeName(target_name_)) {
    RCLCPP_DEBUG(logger(), "already active: %s", target_name_.c_str());
    switch_confirmed_ = true;
    if (load_gains_) {
      bridge_->ClearCachedGains();
      bridge_->RequestCurrentGains();
      gains_requested_ = true;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(),
              "switching: %s -> %s (timeout=%.1fs, load_gains=%s, "
              "use_service=%s)",
              current.c_str(), target_name_.c_str(), timeout_s_,
              load_gains_ ? "true" : "false", use_service_ ? "true" : "false");

  if (use_service_) {
    // Phase 4 path — sync srv. The bridge's underlying client returns ok
    // only after CM has committed the swap (D-A4) and published the
    // latched /<robot_ns>/active_controller_name update, so polling here
    // would be redundant.
    std::string err;
    if (!bridge_->RequestSwitchController(target_name_, timeout_s_, err)) {
      RCLCPP_ERROR(logger(), "switch_controller srv rejected '%s': %s",
                   target_name_.c_str(), err.c_str());
      return BT::NodeStatus::FAILURE;
    }
    switch_confirmed_ = true;
    if (load_gains_) {
      bridge_->ClearCachedGains();
      bridge_->RequestCurrentGains();
      gains_requested_ = true;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Legacy path — fire-and-forget publish + onRunning() polls
  // /<robot_ns>/active_controller_name. Removed in Phase 5.
  bridge_->PublishSelectController(target_name_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchController::onRunning() {
  // Phase 1: wait for controller switch confirmation
  if (!switch_confirmed_) {
    if (NormalizeName(bridge_->GetActiveController()) ==
        NormalizeName(target_name_)) {
      switch_confirmed_ = true;
      RCLCPP_INFO(logger(), "active: %s (elapsed=%.2fs)", target_name_.c_str(),
                  ElapsedSeconds(start_time_));

      if (load_gains_) {
        bridge_->ClearCachedGains();
        bridge_->RequestCurrentGains();
        gains_requested_ = true;
        return BT::NodeStatus::RUNNING;
      }
      return BT::NodeStatus::SUCCESS;
    }

    if (ElapsedSeconds(start_time_) > timeout_s_) {
      RCLCPP_ERROR(logger(),
                   "timeout switching to '%s' after %.1fs (active='%s'). "
                   "Check that the RT controller manager is running and the "
                   "target controller is registered.",
                   target_name_.c_str(), timeout_s_,
                   bridge_->GetActiveController().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // Phase 2: wait for gains response (load_gains=true)
  if (gains_requested_ && bridge_->HasCachedGains()) {
    auto gains = bridge_->GetCachedGains();
    setOutput("current_gains", gains);
    RCLCPP_INFO(logger(), "loaded %zu gains from %s", gains.size(),
                target_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (ElapsedSeconds(start_time_) > timeout_s_) {
    RCLCPP_WARN(
        logger(),
        "gains response timeout for %s, continuing without cached gains",
        target_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SwitchController::onHalted() {
  RCLCPP_INFO(logger(), "halted (target=%s)", target_name_.c_str());
}

} // namespace rtc_bt
