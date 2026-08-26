// ── §7.3 joint tail: the session accumulator behind the terminal report ──────
//
// `rtc::compliance::IntegrateAndBoundJointCommand` returns how many joints each
// of its two bounds moved on THIS tick, and until #484 both bindings threw that
// away. The consequence is a reading, not a missing number: an arm held against
// its joint band and an arm that has settled produce the SAME flat trajectory,
// and nothing downstream could tell them apart. The task-space axis already has
// its answer (`disp_limited` in `compliance_diag.csv` — the §7.5 barrier's own
// report); this is the joint axis getting one.
//
// WHY A COUNTER AND A LOG LINE RATHER THAN A CSV COLUMN. The question this
// answers is binary and operational — "is the arm pinned right now?" — and it is
// asked while watching a bring-up terminal, not while post-processing a session.
// A CSV column would additionally support quantifying the PIN FRACTION for gain
// tuning; if that is ever needed, this accumulator is what a column would be
// filled from, so nothing here has to be undone to add one.
//
// WHY IT IS SHARED. Both position-output bindings that self-integrate their
// command (`demo_compliance`, `demo_task`) run the same tail and have the same
// ambiguity, so this is one unit rather than two copies (P5) — the same call
// `joint_command_tail.hpp` itself made. Deliberately Eigen-free and
// controller-free: it takes the producer's report and gives back a decision.
//
// SINGLE PRODUCER. Every Accumulate() call comes from the one RT tick thread, so
// the atomics carry `relaxed` and no read-modify-write is needed on the maxima.
// They are atomic at all because the READERS are elsewhere — `on_deactivate`
// (after the CM has joined the RT loop) and the test seams — and a plain
// `std::uint64_t` torn across that boundary would be a data race on paper even
// where it is benign in practice.
#pragma once

#include "rtc_controllers/compliance/joint_command_tail.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <atomic>
#include <cstdint>

namespace integrated_bringup {

/// Per-activation totals for the §7.3 tail. Reset at `on_activate`: a resumed
/// controller reporting the previous session's clamp count would attribute the
/// last object's bring-up to this one.
struct JointTailStats {
  /// Ticks on which the `[q_min+d, q_max-d]` clamp moved at least one joint.
  std::atomic<std::uint64_t> clamp_ticks{0};
  /// Ticks on which the rate rebound then moved at least one joint. NOT a subset
  /// of the above in general, but in practice nearly always one: the solve
  /// output is velocity-clamped before the tail, so the only step the rebound
  /// can find too large is one the position clamp WIDENED (that is the whole
  /// reason §7.3 fixes the order). A rebound count that runs far ahead of the
  /// clamp count means something upstream stopped bounding `dq`.
  std::atomic<std::uint64_t> rebound_ticks{0};
  /// Joint-events, not ticks — 7 joints clamped on one tick counts 7 here and 1
  /// above. The ratio is "how much of the arm" versus "how often".
  std::atomic<std::uint64_t> clamp_joint_events{0};
  std::atomic<std::uint64_t> rebound_joint_events{0};
  /// Most joints the clamp moved on any single tick. One joint riding its limit
  /// is a reachability edge; the whole arm at once is a bad margin or a bad
  /// band.
  std::atomic<std::uint32_t> max_clamped_joints{0};
  /// CM tick index of the first and last tick either bound fired. 0 = never —
  /// the CM's `iteration` starts at 1 on the tick a controller first sees, so 0
  /// is not a legal "it happened here" value and needs no separate flag.
  std::atomic<std::uint64_t> first_tick{0};
  std::atomic<std::uint64_t> last_tick{0};

  void Reset() noexcept {
    clamp_ticks.store(0, std::memory_order_relaxed);
    rebound_ticks.store(0, std::memory_order_relaxed);
    clamp_joint_events.store(0, std::memory_order_relaxed);
    rebound_joint_events.store(0, std::memory_order_relaxed);
    max_clamped_joints.store(0, std::memory_order_relaxed);
    first_tick.store(0, std::memory_order_relaxed);
    last_tick.store(0, std::memory_order_relaxed);
  }

  /// True when either bound fired this tick. RT: noexcept, no allocation, plain
  /// integer stores.
  ///
  /// The return value is what gates the caller's throttled WARN, so the counting
  /// and the reporting cannot disagree about whether the band engaged — a caller
  /// that re-tested `report.position_clamped > 0` on its own would be a second
  /// copy of this predicate, and the pair would drift the first time the rebound
  /// half was added to one of them.
  bool Accumulate(const rtc::compliance::JointCommandTailReport& report,
                  std::uint64_t tick) noexcept {
    const bool clamped = report.position_clamped > 0;
    const bool rebounded = report.rate_rebounded > 0;
    if (!clamped && !rebounded) {
      return false;
    }
    if (clamped) {
      clamp_ticks.fetch_add(1, std::memory_order_relaxed);
      clamp_joint_events.fetch_add(static_cast<std::uint64_t>(report.position_clamped),
                                   std::memory_order_relaxed);
      const auto n = static_cast<std::uint32_t>(report.position_clamped);
      if (n > max_clamped_joints.load(std::memory_order_relaxed)) {
        max_clamped_joints.store(n, std::memory_order_relaxed);
      }
    }
    if (rebounded) {
      rebound_ticks.fetch_add(1, std::memory_order_relaxed);
      rebound_joint_events.fetch_add(static_cast<std::uint64_t>(report.rate_rebounded),
                                     std::memory_order_relaxed);
    }
    if (first_tick.load(std::memory_order_relaxed) == 0) {
      first_tick.store(tick, std::memory_order_relaxed);
    }
    last_tick.store(tick, std::memory_order_relaxed);
    return true;
  }

  /// Nothing fired for the whole activation. The summary caller uses this to
  /// stay silent rather than print a row of zeros — a clean run should not have
  /// to be read.
  [[nodiscard]] bool Quiet() const noexcept {
    return clamp_ticks.load(std::memory_order_relaxed) == 0 &&
           rebound_ticks.load(std::memory_order_relaxed) == 0;
  }
};

/// The end-of-activation line. NOT RT — call it from `on_deactivate`, after the
/// CM has stopped the RT loop.
///
/// Silent on a quiet run, by the same rule that keeps permanently-zero columns
/// out of the diagnostic PODs: a line that always prints stops being read, and
/// "the band never engaged" is the expected outcome. When it does print, the
/// tick window is the operationally useful half — it is what lets an operator
/// line the pin up against what the arm was doing at the time.
///
/// `tag` is the caller's usual log prefix ("[compliance]", "[task]") so the line
/// is greppable with the rest of that binding's output.
inline void LogJointTailSummary(const rclcpp::Logger& logger, const char* tag,
                                const JointTailStats& stats) {
  if (stats.Quiet()) {
    return;
  }
  RCLCPP_INFO(
      logger,
      "%s 7.3 joint band engaged this activation: clamp %llu tick(s)/%llu joint-event(s) "
      "(max %u joint(s) at once), rate-rebound %llu tick(s)/%llu joint-event(s), "
      "ticks %llu..%llu — a flat joint trajectory in that window is PINNED, not settled",
      tag, static_cast<unsigned long long>(stats.clamp_ticks.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(stats.clamp_joint_events.load(std::memory_order_relaxed)),
      stats.max_clamped_joints.load(std::memory_order_relaxed),
      static_cast<unsigned long long>(stats.rebound_ticks.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(stats.rebound_joint_events.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(stats.first_tick.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(stats.last_tick.load(std::memory_order_relaxed)));
}

}  // namespace integrated_bringup
