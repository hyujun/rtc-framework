// ── Active-controller switch helper (aux thread, D-A1) ──────────────────────
//
// Sync sequence (Phase 2 / D-A1):
//   1. precondition: name resolves, target != current, E-STOP idle
//   2. target.on_activate(prev_state)                  (base SUCCESS — the
//                                                       new controller will
//                                                       self-init its target
//                                                       slot on its first
//                                                       Compute() tick)
//   3. atomic store active_controller_idx_ = target    (release)
//   4. wait one RT tick (sleep_for(1.5 * dt))          (OQ-2 = sleep_for)
//   5. previous.on_deactivate(prev_state)              (sets state=Inactive)
//   6. publish /rtc_cm/active_controller_name          (latched)
//
// The race between step 3 and step 5 is benign by F-3: demo controllers'
// on_deactivate only toggles LifecyclePublishers (which drop internally
// when inactive) and the WBC MPC thread (Pause is idempotent). If a future
// controller introduces shared state cleanup that Compute() depends on,
// step 4 must be upgraded to a cv-based RT-tick acknowledgement.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace urtc = rtc;

bool RtControllerNode::SwitchActiveController(const std::string& name, std::string& message) {
  if (IsGlobalEstopped()) {
    message = "E-STOP active";
    return false;
  }
  const auto it = controller_name_to_idx_.find(name);
  if (it == controller_name_to_idx_.end()) {
    message = "Unknown controller name: " + name;
    return false;
  }
  const int target_idx = it->second;
  if (target_idx < 0 || target_idx >= static_cast<int>(controllers_.size())) {
    message = "Invalid controller index";
    return false;
  }
  const int prev_idx = active_controller_idx_.load(std::memory_order_acquire);
  if (prev_idx == target_idx) {
    message = "Already active: " + name;
    return true;  // no-op success
  }

  const auto target_uidx = static_cast<std::size_t>(target_idx);
  const rclcpp_lifecycle::State active_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                             "active");

  // Step 2: activate target. The base on_activate is a no-op SUCCESS — the
  // controller will seed its target slot from the current device state on
  // its first Compute() tick (controller-internal init policy).
  const auto rc = ActivateController(target_uidx, active_state);
  if (rc != CallbackReturn::SUCCESS) {
    message = "target on_activate failed";
    return false;
  }

  // Step 3: publish new active idx to RT loop.
  active_controller_idx_.store(target_idx, std::memory_order_release);

  // Step 4: let one RT tick observe the new idx before deactivating prev.
  // Sleep 1.5 × dt at the *configured* rate (≈ 3 ms at the default rate;
  // 0.75 ms at 2 kHz, 15 ms at 100 Hz). PREEMPT_RT missed deadlines surface
  // as overrun_count_ regression, not as a wrong-controller dispatch.
  const double rate_hz = (control_rate_ > 0.0) ? control_rate_ : rtc::kDefaultControlRateHz;
  const auto dt_us = static_cast<long>(1'500'000.0 / rate_hz);
  std::this_thread::sleep_for(std::chrono::microseconds(dt_us));

  // Step 5: deactivate prev (only if it was Active — defensive against
  // CM startup state where prev_idx may be the initial controller that
  // was already activated in CM on_activate).
  if (prev_idx >= 0 && prev_idx < static_cast<int>(controllers_.size())) {
    const auto prev_uidx = static_cast<std::size_t>(prev_idx);
    if (controller_states_[prev_uidx].load(std::memory_order_acquire) == 1) {
      (void)DeactivateController(prev_uidx, active_state);
    }
  }

  // Step 6: publish latched confirm. Payload is config_key (snake_case) —
  // downstream consumers compose `/<active>/...` namespaces, which must
  // match each controller's LifecycleNode namespace `/<config_key>`. See
  // rt_controller_node.cpp on_configure for the same rationale.
  std_msgs::msg::String ctrl_name_msg;
  ctrl_name_msg.data = controller_types_[target_uidx];
  active_ctrl_name_pub_->publish(ctrl_name_msg);

  RCLCPP_INFO(get_logger(), "Switched controller: %s → %s",
              prev_idx >= 0 && prev_idx < static_cast<int>(controllers_.size())
                  ? controllers_[static_cast<std::size_t>(prev_idx)]->Name().data()
                  : "(none)",
              controllers_[target_uidx]->Name().data());
  message = "ok";
  return true;
}
