// ── Global E-Stop trigger and clear ──────────────────────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <array>
#include <cstdio>   // std::snprintf
#include <cstring>  // std::memcpy, ::strnlen
#include <optional>
#include <string>
#include <string_view>
#include <thread>  // std::this_thread::yield

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

  // Fixed-size seqlock write — no heap allocation, no lock on the RT path.
  WriteEstopReason(reason);

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

// ── E-STOP reason seqlock (#935) ─────────────────────────────────────────────
//
// The latch CAS makes the RT trigger the usual only writer, but not always:
// on_error triggers from the lifecycle thread while an RT loop that has not
// been joined yet can trigger too, and a clear landing between one trigger's
// CAS and its reason write lets a second trigger through the latch. So the
// writer takes the sequence from even to odd with a CAS, and a writer that
// finds it odd — or loses the CAS — drops its reason instead of waiting.
void RtControllerNode::WriteEstopReason(std::string_view reason) noexcept {
  std::array<char, kEstopReasonBytes> text{};
  std::snprintf(text.data(), text.size(), "%.*s", static_cast<int>(reason.size()), reason.data());

  std::uint64_t seq = estop_reason_seq_.load(std::memory_order_relaxed);
  // Acquire on success: a previous writer's words are ordered before ours.
  if ((seq & 1U) != 0U || !estop_reason_seq_.compare_exchange_strong(
                              seq, seq + 1, std::memory_order_acquire, std::memory_order_relaxed)) {
    return;
  }
  // Orders the odd sequence before the words: a reader that sees any new word
  // then sees the sequence move, and retries.
  std::atomic_thread_fence(std::memory_order_release);
  for (std::size_t i = 0; i < kEstopReasonWords; ++i) {
    std::uint64_t word = 0;
    std::memcpy(&word, text.data() + i * sizeof(word), sizeof(word));
    estop_reason_words_[i].store(word, std::memory_order_relaxed);
  }
  estop_reason_seq_.store(seq + 2, std::memory_order_release);
}

std::optional<std::uint64_t> RtControllerNode::ReadEstopReason(
    std::array<char, kEstopReasonBytes>& out) const noexcept {
  for (int attempt = 0; attempt < kEstopReasonReadAttempts; ++attempt) {
    const std::uint64_t before = estop_reason_seq_.load(std::memory_order_acquire);
    if ((before & 1U) == 0U) {
      for (std::size_t i = 0; i < kEstopReasonWords; ++i) {
        const std::uint64_t word = estop_reason_words_[i].load(std::memory_order_relaxed);
        std::memcpy(out.data() + i * sizeof(word), &word, sizeof(word));
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      if (estop_reason_seq_.load(std::memory_order_relaxed) == before) {
        if (before == estop_reason_cleared_seq_.load(std::memory_order_acquire)) {
          out[0] = '\0';
        }
        out.back() = '\0';  // terminated for %s callers whatever the words hold
        return before;
      }
    }
    std::this_thread::yield();
  }
  out[0] = '\0';
  return std::nullopt;
}

std::string RtControllerNode::GlobalEstopReason() const {
  std::array<char, kEstopReasonBytes> reason{};
  (void)ReadEstopReason(reason);
  return std::string(reason.data(), ::strnlen(reason.data(), reason.size()));
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
  // With a verification window open the reported value does not change here —
  // it is latch OR window (decision Q13) — and the RT loop raises the publish
  // when the window closes. Raising it anyway would only re-publish `true`.
  if (!IsEstopClearVerifying()) {
    estop_status_pending_.store(true, std::memory_order_release);
  }
  // Defer the RCLCPP_INFO to the non-RT log thread (DrainLog).
  estop_log_pending_.store(true, std::memory_order_release);
  return EstopClearOutcome::kCleared;
}

// ── Clear verification (issue #588) ─────────────────────────────────────────
std::uint32_t RtControllerNode::BeginEstopClearVerification() noexcept {
  // A fresh value per call, never 0 (0 = no window). The RT loop restarts its
  // count whenever the value changes, so a second clear never inherits the
  // ticks the first one had already survived.
  if (++estop_verify_last_token_ == 0U) {
    ++estop_verify_last_token_;
  }
  estop_verify_token_.store(estop_verify_last_token_, std::memory_order_release);
  return estop_verify_last_token_;
}

void RtControllerNode::ResetEstopHoldState() noexcept {
  // Dropping an open window changes the reported value (latch OR window) just
  // as the RT side closing it does, so it schedules the same publish. Without
  // it a deactivate that lands mid-window — RT joined before the closing tick,
  // then ClearGlobalEstop() finding the latch already down (kNotLatched) —
  // leaves the transient_local topic reading `true` for good.
  if (estop_verify_token_.exchange(0U, std::memory_order_acq_rel) != 0U) {
    estop_status_pending_.store(true, std::memory_order_release);
  }
  estop_verify_counted_token_ = 0U;
  estop_verify_ticks_ = 0;
  estop_hold_latched_slots_ = 0;
  estop_hold_num_channels_.fill(0);
  last_readable_mask_.fill(0);
}
