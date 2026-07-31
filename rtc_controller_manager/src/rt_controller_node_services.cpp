// ── /rtc_cm/* services (Phase 3) ────────────────────────────────────────
//
// list_controllers — read-only snapshot of controllers_ + controller_states_.
//   Aux thread; no blocking on RT path. Response order matches controllers_.
//   Also carries the four per-controller diagnostics (#287) read straight off
//   the base accessors: the latch flag plus the three mailbox counters. No
//   controller override is involved — RTControllerInterface already tracks all
//   four for every controller, and each accessor is a relaxed atomic load (the
//   latch is an acquire load of the controller's own flag), so reading them
//   from this callback adds no synchronisation with the RT thread. The values
//   are therefore a SNAPSHOT, not a transaction: two counters in one response
//   may come from different ticks. That is deliberate — anything stronger
//   would need the RT tick to publish through a SeqLock it does not have, to
//   answer a question ("has this been growing?") that only needs successive
//   polls to be ordered, which monotonicity already gives.
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
//   target is refused rather than queued. Confirms the outcome by waiting on
//   RtTickCount() — not on the clock — and re-reading HasLatchedFault(), so
//   "the fault cause is still present" and "nothing consumed the request" are
//   distinguishable answers rather than one ambiguous timeout.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
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
          // Per-controller diagnostics (#287) — see the file header.
          cs.has_latched_fault = controllers_[i]->HasLatchedFault();
          cs.target_drop_count = controllers_[i]->GetTargetDropCount();
          cs.target_reject_count = controllers_[i]->GetTargetRejectCount();
          cs.target_unhandled_count = controllers_[i]->GetTargetUnhandledCount();
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

        // Liveness FIRST. Only a ticking controller can consume the request,
        // and refusing here is what keeps the flag from sitting on an inactive
        // controller until its next activation launders the latch (see
        // ResetFault.srv). It has to precede the two name refusals below
        // because both of them quote active_name as "the active controller" —
        // a claim the index alone does not support: active_controller_idx_
        // starts at 1, not -1, so between on_configure (which creates this
        // service) and the first activation it names a controller that has
        // never run.
        if (controller_states_[uidx].load(std::memory_order_acquire) != 1) {
          resp->ok = false;
          resp->message = "no controller is active (the CM's active index names '" + active_name +
                          "', which is Inactive) — nothing is consuming reset requests";
          return;
        }

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

        // A global E-STOP does NOT block the reset: the RT loop keeps calling
        // Compute() while estopped and only substitutes the output, and the
        // flag is consumed ahead of the controller's own E-STOP early return.
        // The two latches are separate (E-8), so say plainly when the other one
        // is still up rather than implying the arm is free to move. Read at
        // REPLY time, never snapshotted before a wait: the watchdog and the
        // actuator-boundary escalation can latch it while we wait, and a reply
        // that omitted it would tell the operator the arm recovered at the
        // moment it stopped.
        const auto estop_note = [this]() -> const char* {
          return IsGlobalEstopped() ? " (global E-STOP still latched — clear it separately)" : "";
        };

        if (!active.HasLatchedFault()) {
          resp->ok = true;
          resp->message = "no latched fault on '" + active_name + "' (no-op)" + estop_note();
          return;
        }

        // Wait for the RT loop to CONSUME the request, then report what
        // actually happened. Not a fixed sleep: the request is consumed at the
        // HEAD of a tick but the snapshot HasLatchedFault() reads is stored at
        // its END, so the switch path's 1.5 × dt — sized to observe a store
        // made at the START of a tick — leaves only 0.5 × dt for Compute(),
        // and a pinocchio-heavy compliance tick can exceed that.
        //
        // TWO completed ticks are what proves the answer: the tick that
        // produces the first increment may have read the flag before our
        // store, but the next one cannot have started before it, and its
        // increment is released after its diagnostics are published.
        static constexpr std::uint64_t kTicksProvingObservation = 2;
        // Deadline, not a bound on the RT loop: 8 periods leaves room for the
        // two ticks plus overrun/jitter, and expiring is itself a diagnosis.
        static constexpr int kObservationDeadlinePeriods = 8;
        const auto period = ControlPeriod();
        const auto poll_interval = std::max(period / 4, std::chrono::microseconds(100));
        const std::uint64_t ticks_before = RtTickCount();

        active.ResetFault();

        const auto deadline =
            std::chrono::steady_clock::now() + period * kObservationDeadlinePeriods;
        bool rt_observed = false;
        while (std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(poll_interval);
          if (RtTickCount() - ticks_before >= kTicksProvingObservation) {
            rt_observed = true;
            break;
          }
        }

        // Nothing ticked. The latch is untouched and the fault cause is NOT
        // what this reply is about — an RT loop that is stalled, overrunning,
        // or still behind the startup gate produces exactly this, and blaming
        // the fault cause would send the operator after the wrong thing.
        if (!rt_observed) {
          resp->ok = false;
          resp->message = "'" + active_name +
                          "' — the RT loop did not complete a tick while the request was "
                          "pending, so nothing consumed it (loop stalled, overrunning, or not "
                          "running). The latch is unchanged; the request is not queued." +
                          estop_note();
          return;
        }

        // Still latched after a tick that observed the request means the state
        // machine re-latched on the same tick it was reset — the fault cause
        // has not gone away. Reporting ok=true here would tell an operator the
        // arm is recovered when it is not.
        if (active.HasLatchedFault()) {
          resp->ok = false;
          resp->message = "'" + active_name +
                          "' re-latched within one control period — the fault cause is still "
                          "present" +
                          estop_note();
          return;
        }
        resp->ok = true;
        resp->message = "fault latch cleared on '" + active_name + "'" + estop_note();
      },
      srv_qos, cb_group_nrt_callback_);

  RCLCPP_INFO(get_logger(),
              "Services ready: /rtc_cm/list_controllers, "
              "/rtc_cm/switch_controller, /rtc_cm/reset_fault");
}
