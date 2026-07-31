// ── Global E-Stop trigger and clear ──────────────────────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <cstdio>  // std::snprintf
#include <string_view>

namespace urtc = rtc;

// ── Global E-Stop
// ──────────────────────────────────────────────────────────────
void RtControllerNode::TriggerGlobalEstop(std::string_view reason) noexcept {
  // Counted on EVERY entry, BEFORE the CAS — the calls the CAS turns away are
  // exactly the ones this has to see (issue #288). A trigger that arrives while
  // ClearGlobalEstop is mid-propagation finds the latch still up (that is the
  // #299 ordering) and returns here without propagating or recording a reason;
  // if the clear then lowered the latch, the safety event would be gone with no
  // trace. ClearGlobalEstop compares this counter across its propagation loop
  // and abandons the clear on any change.
  //
  // Only the UNGUARDED call sites can land here mid-clear —
  // ControlLoopThread::OnOverrun (consecutive_overrun) and OnLoopAborted
  // (sim_sync_timeout). CheckTimeouts and the output-validation escalation both
  // test IsGlobalEstopped() before calling, so they never reach the CAS while a
  // clear holds the latch up. Counting at entry rather than at those two sites
  // keeps the guarantee independent of which caller is guarded today.
  estop_trigger_requests_.fetch_add(1, std::memory_order_acq_rel);

  // Idempotent — only the first call logs and propagates.
  bool expected = false;
  if (!global_estop_.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                                             std::memory_order_relaxed)) {
    return;  // already estopped
  }

  // Fixed-size copy — no heap allocation on the RT path.
  std::snprintf(estop_reason_.data(), estop_reason_.size(), "%.*s", static_cast<int>(reason.size()),
                reason.data());

  // Propagate to all controllers — TriggerEstop is safe from any thread.
  for (auto& ctrl : controllers_) {
    ctrl->TriggerEstop();
    ctrl->SetHandEstop(true);
  }

  // Defer both the RCLCPP_ERROR and the /system/estop_status publish to the
  // non-RT log thread (DrainLog). This function is called from the RT loop
  // (watchdog timeout, consecutive overrun, invalid output), where a plain
  // rclcpp::Publisher::publish is a RT-10 violation — the latch above is what
  // actually stops the actuators, and the topic is a report of it.
  // `global_estop_` carries the value, so the drain never needs a payload.
  estop_status_pending_.store(true, std::memory_order_release);
  estop_log_pending_.store(true, std::memory_order_release);
}

RtControllerNode::EstopClearOutcome RtControllerNode::ClearGlobalEstop() noexcept {
  if (!global_estop_.load(std::memory_order_acquire)) {
    return EstopClearOutcome::kNotLatched;
  }
  // Sampled before the propagation loop and re-read after it (issue #288). This
  // is the only guard against a trigger the CAS swallows while we hold the
  // latch up — see TriggerGlobalEstop's counter comment for why the swallowed
  // call leaves no other trace.
  const std::uint64_t requests_before = estop_trigger_requests_.load(std::memory_order_acquire);

  // Mirror of TriggerGlobalEstop: the latch changes on the far side of the
  // propagation loop, not the near side. The RT loop decides substitution from
  // the latch alone (rt_controller_node_rt_loop.cpp Phase 2c), so lowering it
  // first would open a fail-open window — a tick landing mid-loop would forward
  // the raw output of a controller still holding its own E-STOP state. Ordered
  // this way the same tick substitutes hold_output_ for an already-cleared
  // controller instead, which is the safe direction.
  //
  // Propagating while the latch still reads true is only sound because no
  // in-tree ClearEstop() consults IsGlobalEstopped() — they store their own
  // atomic and return. Out-of-tree controllers arrive through the registry, so
  // that is an observation, not a contract: a ClearEstop() override must not
  // condition its behaviour on the global latch.
  for (auto& ctrl : controllers_) {
    ctrl->ClearEstop();
    ctrl->SetHandEstop(false);
  }

  if (estop_trigger_requests_.load(std::memory_order_acquire) != requests_before) {
    // Something asked for an E-STOP while we were clearing. Abandon: leave the
    // latch UP so the RT loop keeps substituting hold_output_, and put the
    // controllers back where the latch says they are. Re-asserting matters
    // because the alternative is a state that no longer heals — "latch up,
    // controllers cleared" is a safe TRANSIENT during a propagation loop, but
    // as a resting state it means the next successful clear finds nothing to
    // clear while the arm is still held.
    //
    // Neither pending flag is raised: the latch's value did not change, so the
    // /system/estop_status drain has nothing new to publish, and the deferred
    // log line would announce a clear that did not happen. The caller reports
    // this outcome instead (/rtc_cm/clear_estop).
    for (auto& ctrl : controllers_) {
      ctrl->TriggerEstop();
      ctrl->SetHandEstop(true);
    }
    return EstopClearOutcome::kRetriggered;
  }

  global_estop_.store(false, std::memory_order_release);
  estop_status_pending_.store(true, std::memory_order_release);
  // Defer the RCLCPP_INFO to the non-RT log thread (DrainLog).
  estop_log_pending_.store(true, std::memory_order_release);
  return EstopClearOutcome::kCleared;
}
