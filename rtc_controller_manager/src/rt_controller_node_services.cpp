// ── /rtc_cm/* services (Phase 3) ────────────────────────────────────────
//
// list_controllers — read-only snapshot of controllers_ + controller_states_.
//   Aux thread; no blocking on RT path. Response order matches controllers_.
//
// switch_controller — single-active D-A1 wrapper around
//   SwitchActiveController(name, message). STRICT validates inputs and
//   refuses violations; BEST_EFFORT trims to the first activate target and
//   warns on extras. Timeout field is accepted but currently unused — the
//   underlying switch helper is sync and bounded by sleep_for(1.5×dt) +
//   controller hooks (~ms). M-1 will measure actual latency in Phase 4.
//
// reset_fault — clear a LATCHED controller-local fault (issue #260). The
//   compliance family latches SAFE_STOP on a critical fault and never
//   auto-recovers, and until this service existed the only way out was a
//   process restart. Targets the ACTIVE controller only and requires the caller
//   to name it (operator confirmation); see ResetFault.srv for why an inactive
//   target is refused rather than queued. Confirms the outcome by re-reading
//   HasLatchedFault() after one control period, the same sleep_for(1.5×dt) the
//   switch path uses to let the RT loop observe an off-RT store.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <chrono>
#include <string>
#include <thread>

namespace urtc = rtc;

void RtControllerNode::CreateServices() {
  const rclcpp::QoS srv_qos = rclcpp::ServicesQoS();

  list_controllers_srv_ = create_service<rtc_msgs::srv::ListControllers>(
      "/rtc_cm/list_controllers",
      [this](const std::shared_ptr<rtc_msgs::srv::ListControllers::Request> /*req*/,
             std::shared_ptr<rtc_msgs::srv::ListControllers::Response> resp) {
        const std::size_t n = controllers_.size();
        resp->controllers.clear();
        resp->controllers.reserve(n);
        const int active_idx = active_controller_idx_.load(std::memory_order_acquire);
        for (std::size_t i = 0; i < n; ++i) {
          rtc_msgs::msg::ControllerState cs;
          cs.name = std::string(controllers_[i]->Name());
          const int s = controller_states_[i].load(std::memory_order_acquire);
          cs.state = (s == 1) ? "active" : "inactive";
          cs.is_active = (static_cast<int>(i) == active_idx) && (s == 1);
          cs.type = (i < controller_types_.size()) ? controller_types_[i] : "";
          if (i < controller_topic_configs_.size()) {
            cs.claimed_groups.reserve(controller_topic_configs_[i].groups.size());
            for (const auto& [group_name, _g] : controller_topic_configs_[i].groups) {
              cs.claimed_groups.push_back(group_name);
            }
          }
          resp->controllers.push_back(std::move(cs));
        }
      },
      srv_qos, cb_group_nrt_callback_);

  switch_controller_srv_ = create_service<rtc_msgs::srv::SwitchController>(
      "/rtc_cm/switch_controller",
      [this](const std::shared_ptr<rtc_msgs::srv::SwitchController::Request> req,
             std::shared_ptr<rtc_msgs::srv::SwitchController::Response> resp) {
        const auto strict = req->strictness;
        const bool best_effort = strict == rtc_msgs::srv::SwitchController::Request::BEST_EFFORT;

        // Single-active validation (D-A1). STRICT: any multi-target → reject.
        // BEST_EFFORT: trim to first entry, warn on extras.
        if (req->activate_controllers.size() > 1) {
          if (!best_effort) {
            resp->ok = false;
            resp->message =
                "Single-active: activate_controllers must have ≤ 1 "
                "entry under STRICT";
            return;
          }
          RCLCPP_WARN(get_logger(),
                      "switch_controller BEST_EFFORT: trimming "
                      "activate_controllers (%zu → 1)",
                      req->activate_controllers.size());
        }
        if (req->deactivate_controllers.size() > 1) {
          if (!best_effort) {
            resp->ok = false;
            resp->message =
                "Single-active: deactivate_controllers must have ≤ "
                "1 entry under STRICT";
            return;
          }
          RCLCPP_WARN(get_logger(),
                      "switch_controller BEST_EFFORT: trimming "
                      "deactivate_controllers (%zu → 1)",
                      req->deactivate_controllers.size());
        }

        // No activate target → either pure deactivate (unsupported in
        // single-active) or empty no-op. Empty is a no-op success; pure
        // deactivate is rejected because RT loop always needs an active idx.
        if (req->activate_controllers.empty()) {
          if (!req->deactivate_controllers.empty()) {
            resp->ok = false;
            resp->message =
                "Pure deactivate not supported under single-active (D-A1) — "
                "specify the replacement in activate_controllers";
            return;
          }
          resp->ok = true;
          resp->message = "no-op (empty request)";
          return;
        }

        const std::string& target = req->activate_controllers.front();
        std::string sub_msg;
        const bool ok = SwitchActiveController(target, sub_msg);
        resp->ok = ok;
        if (ok) {
          resp->message = "switched -> " + target + (sub_msg.empty() ? "" : " (" + sub_msg + ")");
        } else {
          resp->message = sub_msg;
        }
      },
      srv_qos, cb_group_nrt_callback_);

  reset_fault_srv_ = create_service<rtc_msgs::srv::ResetFault>(
      "/rtc_cm/reset_fault",
      [this](const std::shared_ptr<rtc_msgs::srv::ResetFault::Request> req,
             std::shared_ptr<rtc_msgs::srv::ResetFault::Response> resp) {
        const int active_idx = active_controller_idx_.load(std::memory_order_acquire);
        if (active_idx < 0 || static_cast<std::size_t>(active_idx) >= controllers_.size()) {
          resp->ok = false;
          resp->message = "no active controller";
          return;
        }
        const auto uidx = static_cast<std::size_t>(active_idx);
        auto& active = *controllers_[uidx];
        const std::string active_name(active.Name());

        // The name is the operator confirmation step, so an empty request is a
        // refusal and not a convenience default — the reply names the active
        // controller so the caller can re-issue it deliberately.
        if (req->controller_name.empty()) {
          resp->ok = false;
          resp->message = "controller_name is required (active controller: " + active_name + ")";
          return;
        }
        if (req->controller_name != active_name) {
          resp->ok = false;
          resp->message = "'" + req->controller_name + "' is not the active controller ('" +
                          active_name + "') — reset_fault targets the active controller only";
          return;
        }
        // Only a ticking controller can consume the request. Refusing here is
        // what keeps the flag from sitting on an inactive controller until its
        // next activation launders the latch (see ResetFault.srv).
        if (controller_states_[uidx].load(std::memory_order_acquire) != 1) {
          resp->ok = false;
          resp->message = "controller '" + active_name +
                          "' is not active — nothing is consuming reset requests";
          return;
        }

        // A global E-STOP does NOT block the reset: the RT loop keeps calling
        // Compute() while estopped and only substitutes the output, and the
        // flag is consumed ahead of the controller's own E-STOP early return.
        // The two latches are separate (E-8), so say plainly when the other one
        // is still up rather than implying the arm is free to move.
        const bool estopped = IsGlobalEstopped();
        const char* const estop_note =
            estopped ? " (global E-STOP still latched — clear it separately)" : "";

        if (!active.HasLatchedFault()) {
          resp->ok = true;
          resp->message = "no latched fault on '" + active_name + "' (no-op)" + estop_note;
          return;
        }

        active.ResetFault();

        // Let one RT tick consume the flag, then report what actually happened.
        // Same bound as the switch path: 1.5 × dt at the configured rate.
        const double rate_hz = (control_rate_ > 0.0) ? control_rate_ : rtc::kDefaultControlRateHz;
        std::this_thread::sleep_for(
            std::chrono::microseconds(static_cast<long>(1'500'000.0 / rate_hz)));

        // Still latched means the state machine re-latched on the same tick it
        // was reset — the fault cause has not gone away. Reporting ok=true here
        // would tell an operator the arm is recovered when it is not.
        if (active.HasLatchedFault()) {
          resp->ok = false;
          resp->message = "'" + active_name +
                          "' re-latched within one control period — the fault cause is still "
                          "present" +
                          std::string(estop_note);
          return;
        }
        resp->ok = true;
        resp->message = "fault latch cleared on '" + active_name + "'" + estop_note;
      },
      srv_qos, cb_group_nrt_callback_);

  RCLCPP_INFO(get_logger(),
              "Services ready: /rtc_cm/list_controllers, "
              "/rtc_cm/switch_controller, /rtc_cm/reset_fault");
}
