// ── One planner wake (dynamic_catching S6, L3 §5.3 "한 번 깨어났을 때의 순서") ──
//
// The whole body of a planner-thread iteration, as a class the thread merely
// calls: read the three inputs, run the single search entry point
// (`PlanOnce`, A-4 — the search strategy stays behind it), re-check the
// provenance, publish. The THREAD (wake source, scheduling, lifetime) lives in
// the integration binding; everything that decides what gets published lives
// here, ROS-free, so it is testable against plain SeqLocks.
//
// THE TWO INTERFACES (E1-F12 #738, ARCH-3). A wake calls the search and the
// segment planner through CatchSearch (catch_search.hpp) and SegmentPlanner
// (segment_planner.hpp) only, and owns whichever implementation was installed
// behind each. "Installed" is "configured": the cycle never holds one that is
// not ready to run, so a wake asks a pointer, not the object. What knows the
// concrete types is the configure path alone — ConfigureSearch / ConfigureDecel
// build today's one implementation of each, and Decel() hands that
// implementation to tests.
//
// RT-1~10. The planner may run SCHED_FIFO (D-7a, shares the `mpc_main` role —
// E-7 decision J), so `Run` allocates nothing, takes no lock, logs nothing and
// throws nothing. Buffers are members sized at construction and filled with
// SeqLock::LoadInto: the covariance snapshot alone is 11.5 KB.
//
// THE DECEL PLANNER'S PART (MPC E1-F03 #629, E1-F08 #661). It runs when a
// segment planner is installed (DecelPlanner, decel_planner.hpp) and the
// optional fifth box is bound; without either, a wake is the search alone and
// the plan is published by itself. A replan never changes the wake's
// CycleOutcome (MD-29) — the PlanSnapshot counters and the D-7a latency keep
// meaning what they meant; the decel account is PlannerCycleRecord::decel.
//
// A search wake that produces a plan also solves its first segment
// (PlanFirst) and publishes the two as a PAIR (MD-56): segment first, then
// the plan, under one publish_ns — or neither, when the segment is withheld
// (kHeld). The pair's re-check accepts a newer
// snapshot of the same track (the first solve can outlast a trajectory
// period; demanding the same snapshot would drop every pair), refuses a pair
// whose t_c is no longer above T_freeze or whose segment would start before
// it can be read, and refuses one the RT started following another plan
// during. Right after a pair the search waits until the RT state postdates it
// by 3 ticks: a second plan published before the RT reports the first would
// overwrite the segment it may be adopting. Once the RT follows a plan the
// search is skipped (MD-57) and every wake through DECEL is a Replan, whose
// re-check is that the RT still reports the same source segment.
//
// WHAT S6-A IMPLEMENTS. The cycle, the provenance handling and a STUB search:
// `PlanOnce` never produces a candidate, so every search wake publishes a
// "no plan" snapshot. That is exactly what the controller did before a planner
// existed (TRACKING self-loops on NO_CATCHABLE_PLAN), so wiring the thread in
// changes no behaviour — the search itself lands in S6-B/S6-C behind the same
// entry point.
#pragma once

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/decel_planner.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/planner_search.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

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
  /// This wake's SEARCH produced a valid plan, published or not. A plan the
  /// switching rule kept back, or one withheld with its first segment (MD-62),
  /// leaves `plan_valid` false and this true; a wake that ran no search
  /// (idle, no input, monitor, replan) leaves it false.
  bool search_valid{false};
  PlanReason reason{PlanReason::kNone};
  /// The trajectory this wake planned against.
  std::uint64_t snapshot_sequence{0};
  std::uint64_t track_generation{0};
  std::int64_t traj_recv_ns{0};
  /// Steady instants: when the wake started and when the plan was stored
  /// (0 when nothing was stored). `publish_ns − traj_recv_ns` is the
  /// receive → publish latency D-7a is judged on (L3 §5.3).
  std::int64_t wake_ns{0};
  std::int64_t publish_ns{0};
  /// The search's own account (S6-B): candidate counts, judgement rejects,
  /// the chosen candidate's rank-gate bitmask, the switching decision, timing.
  SearchStats search{};
  /// The decel planner's account (MPC E1-F03). `outcome == kOff` when it
  /// solved nothing this wake.
  DecelRecord decel{};
};

static_assert(std::is_trivially_copyable_v<PlannerCycleRecord>);

/// The boxes one cycle reads and writes. All owned by the controller. The
/// first four are required; `decel` is optional (without it no segment is
/// solved and the plan is published alone).
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

  // Not copied, not moved: it owns the installed implementations and keeps a
  // typed pointer into one of them (Decel()), and the boxes it is bound to
  // are the controller's.
  PlannerCycle(const PlannerCycle&) = delete;
  PlannerCycle& operator=(const PlannerCycle&) = delete;

  /// Non-RT. Bind the boxes; false (and unbound) if a required pointer is null.
  bool Bind(const PlannerCycleIo& io) noexcept;

  [[nodiscard]] bool Bound() const noexcept { return bound_; }

  /// Non-RT. Take the thread parameters (budget, wait pose).
  void Configure(const PlannerParams& params) { params_ = params; }

  /// Non-RT. Build a PlannerSearch on this model (S6-B) with `Configure`'s
  /// params and the cycle's clock, and install it in place of whatever search
  /// was there — a NEW object every call, never the previous one configured
  /// again. Without a search `PlanOnce` is the S6-A stub ("no plan"). False,
  /// and no search installed, if the binding is unusable.
  bool ConfigureSearch(const PlannerModel& model, const PlannerConstants& constants,
                       const CatchPoseIkOptions& ik);

  /// Non-RT. Install a search built and configured by the caller, or none
  /// (nullptr). It keeps the clock it was configured with: SetClock does not
  /// reach a search. Not while the planner thread runs.
  void InstallSearch(std::unique_ptr<CatchSearch> search) noexcept { search_ = std::move(search); }

  /// Drop the search (a configuration without one).
  void ClearSearch() noexcept { search_.reset(); }

  /// A search is installed (ConfigureSearch succeeded, or InstallSearch).
  [[nodiscard]] bool SearchConfigured() const noexcept { return search_ != nullptr; }

  /// Non-RT. Build a DecelPlanner from `Configure`'s `params.decel` and the
  /// cycle's clock and install it as the segment planner, in place of whatever
  /// was there — a NEW object every call. False, and no segment planner
  /// installed, with `error` naming the cause.
  bool ConfigureDecel(const DecelPlannerModel& model, const DecelPlannerConstants& consts,
                      std::string* error = nullptr);

  /// Non-RT. Install a segment planner built and configured by the caller, or
  /// none (nullptr). It is handed the cycle's clock here, and again on every
  /// SetClock — whichever order the two calls come in, its solves are timed on
  /// the axis `publish_ns` is stamped on. Not while the planner thread runs.
  void InstallSegmentPlanner(std::unique_ptr<SegmentPlanner> planner) noexcept {
    decel_planner_ = nullptr;
    segment_planner_ = std::move(planner);
    if (segment_planner_ != nullptr) {
      segment_planner_->SetClock(clock_);
    }
  }

  /// Drop the segment planner (a configuration without one).
  void ClearDecel() noexcept {
    decel_planner_ = nullptr;
    segment_planner_.reset();
  }

  /// A segment planner is installed (ConfigureDecel succeeded, or
  /// InstallSegmentPlanner). It does not say WHICH: see Decel().
  [[nodiscard]] bool DecelConfigured() const noexcept { return segment_planner_ != nullptr; }

  /// The DecelPlanner ConfigureDecel installed (tests and diagnostics: its
  /// cores' parameters and the input each was last handed). When the planner
  /// in place is not one ConfigureDecel built — none installed, or one
  /// installed with InstallSegmentPlanner — this is an UNCONFIGURED
  /// DecelPlanner (`Configured()` false), as it was before any configure.
  ///
  /// The reference is good until the next ConfigureDecel, ClearDecel or
  /// InstallSegmentPlanner: each replaces the object it refers to.
  [[nodiscard]] const DecelPlanner& Decel() const noexcept;

  /// The seq the last stored decel segment carries (0 = none yet). Monotone
  /// over the cycle's lifetime — not reset by Configure — so the RT's `>`
  /// admission never sees a re-configure's counter restart below its memory.
  [[nodiscard]] std::uint32_t LastDecelSeq() const noexcept { return last_decel_seq_; }

  /// Non-RT. The clock `publish_ns` is read from, forwarded to the installed
  /// segment planner. NOT to an installed search: a search takes its clock
  /// when it is configured, so set this before ConfigureSearch.
  void SetClock(ClockFn clock) noexcept {
    clock_ = clock;
    if (segment_planner_ != nullptr) {
      segment_planner_->SetClock(clock);
    }
  }

  /// One wake. RT-safe. `wake` is the instant the thread woke.
  [[nodiscard]] PlannerCycleRecord Run(NowReal wake) noexcept;

  /// The single search entry point (A-4, S6.6): the installed search's `Plan`,
  /// the S6-A stub ("no plan") when there is none.
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

  /// Test seam: called between a pair's segment Store and its plan Store
  /// (MPC E1-F08) — the only place the store ORDER is observable without a
  /// race.
  void SetPairStoreHookForTesting(PostSearchHook hook, void* context) noexcept {
    pair_store_hook_ = hook;
    pair_store_context_ = context;
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
  PostSearchHook pair_store_hook_{nullptr};
  void* pair_store_context_{nullptr};

  // The decel planner's part of a wake (MPC E1-F08): a plan goes out with its
  // first segment, and a followed plan's segment is replanned. Off without a
  // decel box or an installed segment planner — the plan is then published
  // alone.
  [[nodiscard]] bool DecelActive() const noexcept {
    return io_.decel != nullptr && segment_planner_ != nullptr;
  }

  void PublishPair(const PlannerRtState& rt, PlanSnapshot& plan, PlannerCycleRecord& rec) noexcept;
  void RunReplan(const PlannerRtState& rt, const BallPrediction& ball,
                 PlannerCycleRecord& rec) noexcept;
  // The followed plan's ball as this wake read it: the scratch trajectory and
  // covariance. Empty when they are not that plan's track — the track of the
  // segments published for it, not the one the RT consumed last. Called with
  // a segment planner installed (DecelActive).
  [[nodiscard]] BallPrediction FollowedBall(const PlannerRtState& rt) const noexcept;
  std::int64_t pair_publish_ns_{0};  // the last pair's stamp; 0 after a reset
  // Null = none installed. Replaced on the configure path only (non-RT, the
  // planner thread stopped); a wake reads the pointers and never writes them.
  std::unique_ptr<SegmentPlanner> segment_planner_;
  // `segment_planner_` as the type ConfigureDecel built it, for Decel(). Null
  // when the installed planner came another way, or there is none.
  const DecelPlanner* decel_planner_{nullptr};
  DecelPlanSnapshot decel_out_{};
  std::uint32_t last_decel_seq_{0};
  // Scratch copies, filled with SeqLock::LoadInto so a wake copies each
  // snapshot once, straight into these, rather than building a by-value
  // Load() on the stack first (covariance 11.5 KB, trajectory ~13 KB).
  std::unique_ptr<CatchSearch> search_;
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
