#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"

#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cctype>
#include <chrono>

namespace rtc_bt {

namespace {
auto logger() {
  return ::rtc_bt::logging::ActionLogger("switch_controller");
}

// Normalize controller name: strip underscores, lowercase.
// e.g. "demo_task_controller" and "DemoTaskController" both become
// "demotaskcontroller".
std::string NormalizeName(const std::string& s) {
  std::string r;
  r.reserve(s.size());
  for (char c : s) {
    if (c != '_') {
      r.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return r;
}
}  // namespace

SwitchController::SwitchController(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SwitchController::providedPorts() {
  return {
      BT::InputPort<std::string>("controller_name"),
      BT::InputPort<double>("timeout_s", 3.0, "Timeout [s]"),
  };
}

BT::NodeStatus SwitchController::onStart() {
  auto name = getInput<std::string>("controller_name");
  if (!name) {
    RCLCPP_ERROR(logger(), "missing controller_name port");
    throw BT::RuntimeError("SwitchController: missing controller_name");
  }
  target_ = name.value();
  timeout_s_ = getInput<double>("timeout_s").value_or(3.0);

  const auto current = bridge_->GetActiveController();
  if (NormalizeName(current) == NormalizeName(target_)) {
    RCLCPP_DEBUG(logger(), "already active: %s", target_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "switching: %s -> %s (timeout=%.1fs)", current.c_str(), target_.c_str(),
              timeout_s_);

  start_time_ = std::chrono::steady_clock::now();
  sent_ = false;
  // Fire without blocking; if the service is not ready yet, onRunning retries
  // (poll-based grace) until timeout_s_. The srv returns ok only after CM has
  // committed the swap (D-A4) and latched /rtc_cm/active_controller_name, so
  // the active controller is immediately ready for a following SetGains.
  std::string err;
  handle_ = bridge_->RequestSwitchControllerAsync(target_, err);
  sent_ = handle_.future.valid();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchController::onRunning() {
  // Not yet sent (service was not ready): retry within the timeout budget.
  if (!sent_) {
    if (ElapsedSeconds(start_time_) > timeout_s_) {
      RCLCPP_ERROR(logger(), "switch_controller service unavailable within %.1fs (target=%s)",
                   timeout_s_, target_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    std::string err;
    handle_ = bridge_->RequestSwitchControllerAsync(target_, err);
    if (handle_.future.valid()) {
      sent_ = true;
      start_time_ = std::chrono::steady_clock::now();  // response clock starts at send
    }
    return BT::NodeStatus::RUNNING;
  }

  // Sent: poll the response future without blocking the executor.
  if (handle_.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto resp = handle_.future.get();
    if (!resp->ok) {
      RCLCPP_ERROR(logger(), "switch_controller srv rejected '%s': %s", target_.c_str(),
                   resp->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (ElapsedSeconds(start_time_) > timeout_s_) {
    RCLCPP_ERROR(logger(), "switch_controller timeout (%.1fs) target=%s", timeout_s_,
                 target_.c_str());
    bridge_->CancelSwitchControllerRequest(handle_.request_id);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void SwitchController::onHalted() {
  if (sent_) {
    bridge_->CancelSwitchControllerRequest(handle_.request_id);
  }
  RCLCPP_INFO(logger(), "halted (target=%s)", target_.c_str());
}

}  // namespace rtc_bt
