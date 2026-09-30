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
// THE DECEL STEP (MPC E1-F03). When a decel planner is configured and the
// optional fifth box is bound, the monitor modes (and DECEL, see ActivityFor)
// also pre-compute the stop segment (decel_planner.hpp) and store it in its
// own box. It never changes the wake's CycleOutcome (MD-29) — the PlanSnapshot
// counters and the D-7a latency keep meaning what they meant; the decel
// account is PlannerCycleRecord::decel.
//
// WHAT S6-A IMPLEMENTS. The cycle, the provenance handling and a STUB search:
// `PlanOnce` never produces a candidate, so every search wake publishes a
// "no plan" snapshot. That is exactly what the controller did before a planner
// existed (TRACKING self-loops on NO_CATCHABLE_PLAN), so wiring the thread in
// changes no behaviour — the search itself lands in S6-B/S6-C behind the same
// entry point.
#pragma once

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/decel_planner.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/planner_search.hpp"
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
  kHeld,        ///< the switching rule kept the RT's current plan (§4.7, G)
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
    case CycleOutcome::kHeld:
      return "held";
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
  /// The search's own account (S6-B): candidate counts, judgement rejects,
  /// the chosen candidate's rank-gate bitmask, the switching decision, timing.
  SearchStats search{};
  /// The decel step's account (MPC E1-F03). `outcome == kOff` when no decel
  /// step ran this wake.
  DecelRecord decel{};
};

static_assert(std::is_trivially_copyable_v<PlannerCycleRecord>);

/// The boxes one cycle reads and writes. All owned by the controller. The
/// first four are required; `decel` is optional (no decel step without it).
struct PlannerCycleIo {
  const rtc::SeqLock<TrajectorySnapshot>* traj{nullptr};
  const rtc::SeqLock<CovarianceSnapshot>* cov{nullptr};
  const rtc::SeqLock<PlannerRtState>* rt{nullptr};
  rtc::SeqLock<PlanSnapshot>* plan{nullptr};
  rtc::SeqLock<DecelPlanSnapshot>* decel{nullptr};
};

class PlannerCycle {
 public:
  /// Steady-clock source for `publish_ns`. Injected so tests can pin it; the
  /// default is `rtc::SteadyNowNs` (the same axis every age in the catching
  /// path is measured on).
  using ClockFn = std::int64_t (*)() noexcept;

  PlannerCycle() noexcept;

  /// Non-RT. Bind the boxes; false (and unbound) if a required pointer is null.
  bool Bind(const PlannerCycleIo& io) noexcept;

  [[nodiscard]] bool Bound() const noexcept { return bound_; }

  /// Non-RT. Take the thread parameters (budget, wait pose).
  void Configure(const PlannerParams& params) { params_ = params; }

  /// Non-RT. Give the search its model (S6-B). Without it `PlanOnce` is the
  /// S6-A stub ("no plan"). False if the binding is unusable.
  bool ConfigureSearch(const PlannerModel& model, const PlannerConstants& constants,
                       const CatchPoseIkOptions& ik) {
    return search_.Configure(model, constants, params_, ik, clock_);
  }

  /// Drop the search's model (a configuration without one).
  void ClearSearch() noexcept { search_ = PlannerSearch{}; }

  [[nodiscard]] bool SearchConfigured() const noexcept { return search_.Configured(); }

  /// Non-RT. Build the decel planner from `Configure`'s `params.decel` and the
  /// cycle's clock. False (and unconfigured) with `error` naming the cause.
  bool ConfigureDecel(const DecelPlannerModel& model, const DecelPlannerConstants& consts,
                      std::string* error = nullptr) {
    return decel_.Configure(model, consts, params_.decel, clock_, error);
  }

  /// Drop the decel planner (a configuration without one).
  void ClearDecel() noexcept {
    // An invalid configuration IS the cleared state (Configure fails closed).
    static_cast<void>(decel_.Configure(DecelPlannerModel{}, DecelPlannerConstants{},
                                       DecelPlannerParams{}, nullptr));
  }

  [[nodiscard]] bool DecelConfigured() const noexcept { return decel_.Configured(); }

  /// The seq the last stored decel segment carries (0 = none yet). Monotone
  /// over the cycle's lifetime — not reset by Configure — so the RT's `>`
  /// admission never sees a re-configure's counter restart below its memory.
  [[nodiscard]] std::uint32_t LastDecelSeq() const noexcept { return last_decel_seq_; }

  void SetClock(ClockFn clock) noexcept {
    clock_ = clock;
    decel_.SetClock(clock);
  }

  /// One wake. RT-safe. `wake` is the instant the thread woke.
  [[nodiscard]] PlannerCycleRecord Run(NowReal wake) noexcept;

  /// The single search entry point (A-4, S6.6): `PlannerSearch::Plan` once a
  /// model is configured, the S6-A stub ("no plan") otherwise.
  [[nodiscard]] PlanSnapshot PlanOnce(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                      bool cov_matched, const PlannerRtState& rt, NowReal now,
                                      SearchStats& stats) noexcept;

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

  /// Test seam: called between the decel solve and its re-check (the window a
  /// reset or a plan change must land in for that re-check to matter).
  void SetPostDecelHookForTesting(PostSearchHook hook, void* context) noexcept {
    post_decel_hook_ = hook;
    post_decel_context_ = context;
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
  PostSearchHook post_decel_hook_{nullptr};
  void* post_decel_context_{nullptr};
  // The decel step (MPC E1-F03).
  void RunDecel(const PlannerRtState& rt, PlannerCycleRecord& rec) noexcept;
  DecelPlanner decel_;
  DecelPlanSnapshot decel_out_{};
  std::uint32_t last_decel_seq_{0};
  // Scratch copies, filled with SeqLock::LoadInto so a wake copies each
  // snapshot once, straight into these, rather than building a by-value
  // Load() on the stack first (covariance 11.5 KB, trajectory ~13 KB).
  PlannerSearch search_;
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
