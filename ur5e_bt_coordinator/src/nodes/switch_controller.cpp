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

// True once the bridge has adopted `target` as the active controller. This is
// the observable edge of RewireControllerTopics: OnActiveController runs the
// rewire *first* and only then writes active_controller_, so a reader that
// sees the name match is guaranteed the param/grasp clients and the
// controller-owned target publishers are already bound to it.
bool BridgeRewiredTo(const BtRosBridge& bridge, const std::string& target) {
  return NormalizeName(bridge.GetActiveController()) == NormalizeName(target);
}
}  // namespace

SwitchController::SwitchController(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SwitchController::providedPorts() {
  return {
      BT::InputPort<std::string>("controller_name"),
      BT::InputPort<double>("timeout_s", 3.0, "Per-stage timeout [s]"),
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

  if (BridgeRewiredTo(*bridge_, target_)) {
    RCLCPP_DEBUG(logger(), "already active: %s", target_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "switching: %s -> %s (timeout=%.1fs/stage)",
              bridge_->GetActiveController().c_str(), target_.c_str(), timeout_s_);

  start_time_ = std::chrono::steady_clock::now();
  sent_ = false;
  awaiting_rewire_ = false;
  // Fire without blocking; if the service is not ready yet, onRunning retries
  // (poll-based grace) until timeout_s_.
  std::string err;
  handle_ = bridge_->RequestSwitchControllerAsync(target_, err);
  sent_ = handle_.future.valid();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchController::onRunning() {
  // ── Stage 2: srv said ok — wait for this node's bridge to catch up ───────
  //
  // The srv returning ok means CM committed the swap (D-A4) and latched
  // /rtc_cm/active_controller_name. It does NOT mean the tree can reach the
  // new controller: CM latching the name and this node's bridge acting on it
  // are different channels, and DDS orders neither against the srv response.
  // The bridge must still receive the latched name and run
  // RewireControllerTopics to rebind the target publishers and the
  // param/grasp clients. Issue #158 is what that gap looks like when the tree
  // does not wait for it — a following SetGains reads the stale name and
  // quietly configures the *previous* controller, reporting SUCCESS.
  //
  // So SUCCESS here means "the tree can talk to target_", and every node
  // sequenced after this one inherits that guarantee rather than re-deriving
  // it. The wait gets its own budget: stage 1 timed a service round-trip,
  // this times a latched-topic delivery + rewire, and blaming one on the
  // other's budget is what made #158 hard to read.
  if (awaiting_rewire_) {
    if (BridgeRewiredTo(*bridge_, target_)) {
      RCLCPP_INFO(logger(), "switched -> %s (bridge rewired)", target_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    if (ElapsedSeconds(start_time_) > timeout_s_) {
      RCLCPP_ERROR(logger(),
                   "switch_controller srv accepted '%s' but the bridge never rewired within "
                   "%.1fs (still on '%s') — /rtc_cm/active_controller_name not delivered?",
                   target_.c_str(), timeout_s_, bridge_->GetActiveController().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // ── Stage 1a: not yet sent (service was not ready) — retry within budget ─
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

  // ── Stage 1b: sent — poll the response future without blocking ───────────
  if (handle_.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto resp = handle_.future.get();
    if (!resp->ok) {
      RCLCPP_ERROR(logger(), "switch_controller srv rejected '%s': %s", target_.c_str(),
                   resp->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    // The latched name may already have overtaken the srv response, in which
    // case stage 2 has nothing to wait for.
    if (BridgeRewiredTo(*bridge_, target_)) {
      RCLCPP_INFO(logger(), "switched -> %s", target_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    awaiting_rewire_ = true;
    start_time_ = std::chrono::steady_clock::now();  // rewire clock starts at the ok
    return BT::NodeStatus::RUNNING;
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
  // Only stage 1 has an in-flight request to cancel; by stage 2 the response
  // future is already consumed and the switch itself has been committed by CM
  // (halting the tree does not roll it back).
  if (sent_ && !awaiting_rewire_) {
    bridge_->CancelSwitchControllerRequest(handle_.request_id);
  }
  RCLCPP_INFO(logger(), "halted (target=%s, stage=%s)", target_.c_str(),
              awaiting_rewire_ ? "awaiting_rewire" : "srv");
}

}  // namespace rtc_bt
