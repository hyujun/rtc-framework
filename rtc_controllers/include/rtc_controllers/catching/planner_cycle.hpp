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
// behind each. "Installed" is "configured": the cycle is handed objects that
// are ready to run and builds none, so a wake asks a pointer, not the object.
// What knows the concrete types is the configure path alone — the integration
// package and the tests build each object with its own factory, configure it,
// and install it (InstallSearch / InstallSegmentPlanner).
//
// RT-1~10. The planner may run SCHED_FIFO (D-7a, shares the `mpc_main` role —
// E-7 decision J), so `Run` allocates nothing, takes no lock, logs nothing and
// throws nothing. Buffers are members sized at construction and filled with
// SeqLock::LoadInto: the covariance snapshot alone is 11.5 KB.
//
// THE MPC SEGMENT PLANNER'S PART (MPC E1-F03 #629, E1-F08 #661). It runs when a
// segment planner is installed and the optional fifth box is bound; without either, a wake is the
// search alone and the plan is published by itself. A replan never changes the wake's CycleOutcome
// (MD-29) — the PlanSnapshot counters and the D-7a latency keep meaning what they meant; the
// segment account is PlannerCycleRecord::segment.
//
// A search wake that produces a plan also solves its first segment
// (PlanFirst) and publishes the two as a PAIR (MD-56): segment first, then
// the plan, under one publish_ns — or neither, when the segment is withheld
// (kHeld). The pair's re-check accepts a newer
// snapshot of the same track (the first solve can outlast a trajectory
// period; demanding the same snapshot would drop every pair), refuses a pair
// whose t_c is no longer above T_freeze or whose segment would start before
// it can be read, and refuses one the RT started following another plan
// during. Right after a pair nothing is published until the RT state postdates
// it by 3 ticks or shows the pair's plan followed: the segment box has one
// slot, and whatever is stored before the RT has read the pair lands on top of
// the segment it may be adopting. Once the RT follows a plan every wake through
// DECEL replans its segment, and the re-check of a replan is that the RT still
// reports the same source segment.
//
// WHILE A PLAN IS FOLLOWED THE SEARCH GOES ON, AND MAY REPLACE IT (E1-F16,
// E1-F17). In APPROACH, until the first catch instant the RT followed since
// the last reset is within `planner.freeze.t_stop_plan`, a wake runs:
//   1. THE SEARCH, against the newest prediction and with the segments the RT
//      reports, so that a candidate's arm motion starts where the arm will be.
//      It is timed from the wake, and capped: one prediction period
//      (PlannerParams::dt_expected) has to hold it AND the replan behind it,
//      so it is handed that period minus the segment planner's replan budget
//      (CatchSearch::Plan's `budget_cap_ns`; no cap when either is unknown).
//   2. Then ONE of two things.
//      • The search chose ANOTHER plan (SwitchDecision::kReplaced): its first
//        segment is solved from the moving arm (SegmentPlanner::PlanFirst with
//        `rt.plan_active`) and the two go out as a REPLACEMENT PAIR — segment
//        first, one publish_ns, a new plan id, as a first pair does. The RT
//        holds it until that segment's node 0 and switches there; the arm
//        stays on the followed plan's segments until then, and the segment
//        planner keeps them. The pair is not solved for at all when a first
//        segment could not start before the followed plan freezes
//        (EarliestFirstStartNs against t_c − T_freeze). Its re-check drops it
//        (kSuperseded, nothing stored, no replan either) unless: the
//        prediction is still that track's; the trial and the activation are
//        the same; the RT still follows the same plan, in APPROACH, and holds
//        no replacement; the segment the solve started on is still the one
//        the RT reports for node 0; the new catch instant is above T_freeze
//        away; node 0 is before the followed plan's freeze and can still be
//        read.
//      • Anything else — the followed plan again, kept by the switching rule,
//        no candidate, no trajectory to search on, or a replacement whose
//        first segment was withheld (kHeld; that solve's account is
//        PlannerCycleRecord::replacement) — and the followed plan's segment is
//        REPLANNED, on an RT report read again after the search. The wake's
//        outcome is kHeld; the replan's account is `segment`.
// While the RT reports a replacement it holds (PlannerRtState::plan_pending) a
// wake runs neither: the RT takes nothing until it has switched or dropped it
// (kHeld). Should the RT drop the pair, its report is the old plan again with
// nothing held, and the next wake's search may replace it again under a new
// id. After `t_stop_plan`, and from COMMITTED on, a wake is the replan alone.
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
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <cstdint>
#include <memory>
#include <type_traits>
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
  /// The RT follows a plan on a segment planner's segments and the search chose
  /// ANOTHER one (SwitchDecision::kReplaced). Not published: the RT takes no
  /// replacement pair. The search's record says what it would have switched to.
  kHeldReplaceUnsupported,
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
    case CycleOutcome::kHeldReplaceUnsupported:
      return "held_replace_unsupported";
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
  /// The segment planner's account (MPC E1-F03). `outcome == kOff` when it
  /// solved nothing this wake.
  SegmentRecord segment{};
  /// The first segment of a REPLACEMENT that was withheld on this wake: the
  /// search chose another plan while the RT followed one, and PlanFirst
  /// returned false for it — the followed plan's segment was replanned
  /// instead, and that replan's account is `segment`. `outcome == kOff` on
  /// every other wake; a replacement pair that was published or superseded is
  /// accounted in `segment`, as a first pair is.
  SegmentRecord replacement{};
};

static_assert(std::is_trivially_copyable_v<PlannerCycleRecord>);

/// The boxes one cycle reads and writes. All owned by the controller. The
/// first four are required; `segment` is optional (without it no segment is
/// solved and the plan is published alone).
struct PlannerCycleIo {
  const rtc::SeqLock<TrajectorySnapshot>* traj{nullptr};
  const rtc::SeqLock<CovarianceSnapshot>* cov{nullptr};
  const rtc::SeqLock<PlannerRtState>* rt{nullptr};
  rtc::SeqLock<PlanSnapshot>* plan{nullptr};
  rtc::SeqLock<SegmentSnapshot>* segment{nullptr};
};

class PlannerCycle {
 public:
  /// Steady-clock source for `publish_ns`. Injected so tests can pin it; the
  /// default is `rtc::SteadyNowNs` (the same axis every age in the catching
  /// path is measured on).
  using ClockFn = std::int64_t (*)() noexcept;

  PlannerCycle() noexcept;

  // Not copied, not moved: it owns the installed implementations, and the boxes
  // it is bound to are the controller's.
  PlannerCycle(const PlannerCycle&) = delete;
  PlannerCycle& operator=(const PlannerCycle&) = delete;

  /// Non-RT. Bind the boxes; false (and unbound) if a required pointer is null.
  bool Bind(const PlannerCycleIo& io) noexcept;

  [[nodiscard]] bool Bound() const noexcept { return bound_; }

  /// Non-RT. Take the thread parameters (budget, wait pose).
  void Configure(const PlannerParams& params) { params_ = params; }

  /// Non-RT. Install a search built and configured by the caller, in place of
  /// whatever was there, or none (nullptr). It is handed the cycle's clock
  /// here, and again on every SetClock — whichever order the two calls come
  /// in, its budget is measured on the axis `publish_ns` is stamped on. Without
  /// a search `PlanOnce` is the S6-A stub ("no plan"). Not while the planner
  /// thread runs.
  void InstallSearch(std::unique_ptr<CatchSearch> search) noexcept {
    search_ = std::move(search);
    if (search_ != nullptr) {
      search_->SetClock(clock_);
    }
  }

  /// Drop the search (a configuration without one).
  void ClearSearch() noexcept { search_.reset(); }

  /// A search is installed (InstallSearch).
  [[nodiscard]] bool SearchConfigured() const noexcept { return search_ != nullptr; }

  /// Non-RT. Install a segment planner built and configured by the caller, in
  /// place of whatever was there, or none (nullptr). It is handed the cycle's clock here, and again
  /// on every SetClock — whichever order the two calls come in, its solves are timed on the axis
  /// `publish_ns` is stamped on. Not while the planner thread runs.
  void InstallSegmentPlanner(std::unique_ptr<SegmentPlanner> planner) noexcept {
    segment_planner_ = std::move(planner);
    if (segment_planner_ != nullptr) {
      segment_planner_->SetClock(clock_);
    }
  }

  /// Drop the segment planner (a configuration without one).
  void ClearSegmentPlanner() noexcept { segment_planner_.reset(); }

  /// A segment planner is installed (InstallSegmentPlanner). It does not say
  /// WHICH: the caller that built it keeps the typed object if it needs one.
  [[nodiscard]] bool SegmentPlannerConfigured() const noexcept {
    return segment_planner_ != nullptr;
  }

  /// The seq the last stored segment carries (0 = none yet). Monotone
  /// over the cycle's lifetime — not reset by Configure — so the RT's `>`
  /// admission never sees a re-configure's counter restart below its memory.
  [[nodiscard]] std::uint32_t LastSegmentSeq() const noexcept { return last_segment_seq_; }

  /// Non-RT. The clock `publish_ns` is read from, forwarded to the installed
  /// search and segment planner (a null clock is ignored by them, not
  /// installed). Not while the planner thread runs.
  void SetClock(ClockFn clock) noexcept {
    clock_ = clock;
    if (search_ != nullptr) {
      search_->SetClock(clock);
    }
    if (segment_planner_ != nullptr) {
      segment_planner_->SetClock(clock);
    }
  }

  /// One wake. RT-safe. `wake` is the instant the thread woke.
  [[nodiscard]] PlannerCycleRecord Run(NowReal wake) noexcept;

  /// The single search entry point (A-4, S6.6): the installed search's `Plan`,
  /// the S6-A stub ("no plan") when there is none. `arm` is what the segment
  /// planner reports the RT following, `budget_cap_ns` the bound on the
  /// search's budget, 0 = none (CatchSearch::Plan).
  [[nodiscard]] PlanSnapshot PlanOnce(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                      bool cov_matched, const PlannerRtState& rt,
                                      const ReportedSegments& arm, NowReal now,
                                      std::int64_t budget_cap_ns, SearchStats& stats) noexcept;

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

  /// Test seam: called between the segment solve and its re-check (the window a
  /// reset or a plan change must land in for that re-check to matter).
  void SetPostSegmentHookForTesting(PostSearchHook hook, void* context) noexcept {
    post_segment_hook_ = hook;
    post_segment_context_ = context;
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
  PostSearchHook post_segment_hook_{nullptr};
  void* post_segment_context_{nullptr};
  PostSearchHook pair_store_hook_{nullptr};
  void* pair_store_context_{nullptr};

  // The MPC segment planner's part of a wake (MPC E1-F08): a plan goes out with its
  // first segment, and a followed plan's segment is replanned. Off without a
  // segment box or an installed segment planner — the plan is then published
  // alone.
  [[nodiscard]] bool SegmentActive() const noexcept {
    return io_.segment != nullptr && segment_planner_ != nullptr;
  }

  void PublishPair(const PlannerRtState& rt, PlanSnapshot& plan, PlannerCycleRecord& rec) noexcept;
  // A pair's stores, after its re-check: the segment, then the plan, under
  // `publish_ns`; the search and the segment planner are told; the record
  // filled.
  void StorePair(PlanSnapshot& plan, std::int64_t publish_ns, PlannerCycleRecord& rec) noexcept;
  // The replacement pair of a wake whose search chose another plan than the
  // one `rt` follows (header: "WHILE A PLAN IS FOLLOWED"). True when the wake
  // is done with — the pair was stored, or dropped at its re-check
  // (kSuperseded). False when nothing was published and the followed plan's
  // segment is still to be replanned: no first segment could start before the
  // followed plan freezes, or the solve withheld it (rec.replacement).
  [[nodiscard]] bool PublishReplacement(const PlannerRtState& rt, PlanSnapshot& plan,
                                        PlannerCycleRecord& rec) noexcept;
  // The replan of a wake that searched first, on the RT's report read again.
  // `rt` is the wake's report: nothing is replanned when the newer one is of
  // another trial or plan, or holds a replacement.
  void ReplanBehindSearch(const PlannerRtState& rt, PlannerCycleRecord& rec) noexcept;
  // The cap of a following wake's search [ns], 0 = none: one prediction
  // period less the segment planner's replan budget. Called with a segment
  // planner installed.
  [[nodiscard]] std::int64_t FollowingSearchCapNs() const noexcept;
  void RunReplan(const PlannerRtState& rt, const BallPrediction& ball,
                 PlannerCycleRecord& rec) noexcept;
  // The followed plan's ball as this wake read it: the scratch trajectory and
  // covariance. Empty when they are not that plan's track — the track of the
  // segments published for it, not the one the RT consumed last. Called with
  // a segment planner installed (SegmentActive).
  [[nodiscard]] BallPrediction FollowedBall(const PlannerRtState& rt) const noexcept;
  // Whether a wake in which the RT follows a plan still runs the search
  // (header: "WHILE A PLAN IS FOLLOWED"). Called with a segment planner
  // installed and `rt.plan_active`.
  [[nodiscard]] bool SearchesWhileFollowing(const PlannerRtState& rt, NowReal wake) const noexcept;
  std::int64_t pair_publish_ns_{0};  // the last pair's stamp; 0 after a reset
  std::uint32_t pair_plan_id_{0};    // and its plan's id
  // The catch instant of the first plan the RT followed since the last reset
  // (0 = it follows none): what `t_stop_plan` is measured back from.
  std::int64_t first_followed_t_c_ns_{0};
  // Null = none installed. Replaced on the configure path only (non-RT, the
  // planner thread stopped); a wake reads the pointers and never writes them.
  std::unique_ptr<SegmentPlanner> segment_planner_;
  SegmentSnapshot segment_out_{};
  std::uint32_t last_segment_seq_{0};
  // Scratch copies, filled with SeqLock::LoadInto so a wake copies each
  // snapshot once, straight into these, rather than building a by-value
  // Load() on the stack first (covariance 11.5 KB, trajectory ~13 KB).
  std::unique_ptr<CatchSearch> search_;
  TrajectorySnapshot traj_{};
  TrajectorySnapshot traj_recheck_{};
  CovarianceSnapshot cov_{};
  // The segments the RT reports, as the segment planner copied them out for
  // this wake's search (CatchSearch::Plan's `arm`). A snapshot in it is
  // meaningful only under its flag.
  ReportedSegments reported_{};
};

/// Whether two provenance tokens name the same trajectory snapshot.
[[nodiscard]] constexpr bool SameSnapshot(const ProvenanceToken& a,
                                          const ProvenanceToken& b) noexcept {
  return a.activation_generation == b.activation_generation && a.generation == b.generation &&
         a.snapshot_sequence == b.snapshot_sequence;
}

}  // namespace rtc::catching
