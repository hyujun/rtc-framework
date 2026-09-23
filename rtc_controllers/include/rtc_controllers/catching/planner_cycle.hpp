// ── One planner wake (dynamic_catching S6, L3 §5.3 "한 번 깨어났을 때의 순서") ──
//
// The whole body of a planner-thread iteration, as a class the thread merely
// calls: read the three inputs, run the single search entry point
// (`PlanOnce`, A-4 — the search strategy stays behind it), re-check the
// provenance, publish. The THREAD (wake source, scheduling, lifetime) lives in
// the integration binding; everything that decides what gets published lives
// here, ROS-free, so it is testable against plain SeqLocks.
//
// RT-1~10. The planner may run SCHED_FIFO (D-7a, shares the `mpc_main` role —
// E-7 decision J), so `Run` allocates nothing, takes no lock, logs nothing and
// throws nothing. Buffers are members sized at construction and filled with
// SeqLock::LoadInto: the covariance snapshot alone is 11.5 KB.
//
// WHAT S6-A IMPLEMENTS. The cycle, the provenance handling and a STUB search:
// `PlanOnce` never produces a candidate, so every search wake publishes a
// "no plan" snapshot. That is exactly what the controller did before a planner
// existed (TRACKING self-loops on NO_CATCHABLE_PLAN), so wiring the thread in
// changes no behaviour — the search itself lands in S6-B/S6-C behind the same
// entry point.
#pragma once

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <cstdint>

namespace rtc::catching {

/// What one wake did — the per-wake diagnostic record (L3 §8). S6-A carries
/// the provenance fields; the candidate counts and gate bitmask join in S6-B.
enum class CycleOutcome : std::uint8_t {
  kIdle = 0,    ///< no RT state yet, or a mode with nothing to plan for
  kNoInput,     ///< no trajectory of the current activation
  kPublished,   ///< a PlanSnapshot (valid or "no plan") was stored
  kSuperseded,  ///< the trajectory or the trial moved during compute — dropped
};

[[nodiscard]] constexpr const char* CycleOutcomeName(CycleOutcome o) noexcept {
  switch (o) {
    case CycleOutcome::kIdle:
      return "idle";
    case CycleOutcome::kNoInput:
      return "no_input";
    case CycleOutcome::kPublished:
      return "published";
    case CycleOutcome::kSuperseded:
      return "superseded";
  }
  return "unknown";
}

struct PlannerCycleRecord {
  CycleOutcome outcome{CycleOutcome::kIdle};
  /// This wake saw the RT's reset epoch move (a trial reset since last wake).
  bool reset_seen{false};
  /// The covariance box carried the same token as the trajectory (L3 §5.2).
  bool cov_matched{false};
  std::uint8_t mode{0};
  /// The published plan — meaningful only when `outcome == kPublished`.
  std::uint32_t plan_id{0};
  bool plan_valid{false};
  PlanReason reason{PlanReason::kNone};
  /// The trajectory this wake planned against.
  std::uint64_t snapshot_sequence{0};
  std::uint64_t track_generation{0};
  std::int64_t traj_recv_ns{0};
  /// Steady instants: when the wake started and when the plan was stored
  /// (0 when nothing was stored). `publish_ns − traj_recv_ns` is the
  /// receive → publish latency D-7a is judged on (plan §7.2).
  std::int64_t wake_ns{0};
  std::int64_t publish_ns{0};
};

static_assert(std::is_trivially_copyable_v<PlannerCycleRecord>);

/// The four boxes one cycle reads and writes. All owned by the controller.
struct PlannerCycleIo {
  const rtc::SeqLock<TrajectorySnapshot>* traj{nullptr};
  const rtc::SeqLock<CovarianceSnapshot>* cov{nullptr};
  const rtc::SeqLock<PlannerRtState>* rt{nullptr};
  rtc::SeqLock<PlanSnapshot>* plan{nullptr};
};

class PlannerCycle {
 public:
  /// Steady-clock source for `publish_ns`. Injected so tests can pin it; the
  /// default is `rtc::SteadyNowNs` (the same axis every age in the catching
  /// path is measured on).
  using ClockFn = std::int64_t (*)() noexcept;

  PlannerCycle() noexcept;

  /// Non-RT. Bind the boxes; false (and unbound) if any pointer is null.
  bool Bind(const PlannerCycleIo& io) noexcept;

  [[nodiscard]] bool Bound() const noexcept { return bound_; }

  /// Non-RT. Take the thread parameters (budget, wait pose).
  void Configure(const PlannerParams& params) noexcept { params_ = params; }

  void SetClock(ClockFn clock) noexcept { clock_ = clock; }

  /// One wake. RT-safe. `wake` is the instant the thread woke.
  [[nodiscard]] PlannerCycleRecord Run(NowReal wake) noexcept;

  /// The single search entry point (A-4, S6.6). S6-A: a stub that never
  /// yields a candidate — see the file header.
  [[nodiscard]] PlanSnapshot PlanOnce(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                      bool cov_matched, const PlannerRtState& rt,
                                      NowReal now) noexcept;

  /// The id the next publish will carry minus one — i.e. the last id used.
  [[nodiscard]] std::uint32_t LastPlanId() const noexcept { return last_plan_id_; }

  /// Test seam: called between the search and the provenance re-check, which
  /// is the window a newer trajectory or a reset has to land in for the
  /// re-check to matter. Production never sets it; without the seam the only
  /// way to hit that window is a timing race, and a race is not a test.
  using PostSearchHook = void (*)(void* context) noexcept;

  void SetPostSearchHookForTesting(PostSearchHook hook, void* context) noexcept {
    post_search_hook_ = hook;
    post_search_context_ = context;
  }

 private:
  PlannerCycleIo io_{};
  bool bound_{false};
  PlannerParams params_{};
  ClockFn clock_;
  std::uint32_t last_plan_id_{0};
  std::uint32_t seen_reset_epoch_{0};
  PostSearchHook post_search_hook_{nullptr};
  void* post_search_context_{nullptr};
  // Scratch copies, filled with SeqLock::LoadInto so a wake copies each
  // snapshot once, straight into these, rather than building a by-value
  // Load() on the stack first (covariance 11.5 KB, trajectory ~13 KB).
  TrajectorySnapshot traj_{};
  TrajectorySnapshot traj_recheck_{};
  CovarianceSnapshot cov_{};
};

/// Whether two provenance tokens name the same trajectory snapshot.
[[nodiscard]] constexpr bool SameSnapshot(const ProvenanceToken& a,
                                          const ProvenanceToken& b) noexcept {
  return a.activation_generation == b.activation_generation && a.generation == b.generation &&
         a.snapshot_sequence == b.snapshot_sequence;
}

}  // namespace rtc::catching
