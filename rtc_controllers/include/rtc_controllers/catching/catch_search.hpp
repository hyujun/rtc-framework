// ── The catch search, as one planner wake calls it (E1-F12 #738, ARCH-3) ─────
//
// What PlannerCycle knows of the search that turns a predicted ball into a
// catch plan: one cycle's candidates in, one PlanSnapshot out (L3 §4, the A-4
// single entry point), plus the three calls that keep the search's per-trial
// memory in step with what the cycle published and with the RT's trials. Which
// search runs behind it is the configuration's choice — a wake calls this
// interface and nothing else.
//
// The implementations are the grid search (a grid over the vision samples with
// a closed-form γ profile) and the NLP search.
//
// TWO THINGS CROSS BETWEEN THE SEARCH AND THE SEGMENT PLANNER, both through the
// cycle, neither interface knowing the other's implementation:
//   • segment planner → search: the segments the RT reports (ReportedSegments,
//     planner_io.hpp). A search that judges candidates by the arm motion they
//     need has to start that motion where the arm will be, and once the RT
//     follows a plan that is on a published segment.
//   • search → segment planner: the chosen candidate's arm trajectory
//     (CatchSolution, below), when the search solved one. A planner that would
//     solve the same problem again can publish it instead.
//
// ── Contract ──────────────────────────────────────────────────────────────────
//  • THREAD. Every member below except SetClock is called from
//    PlannerCycle::Run, on the planner thread — which may run SCHED_FIFO (D-7a). RT-1~10 apply to
//    all of them: no allocation, no lock, no log, no throw. An implementation sizes its buffers
//    when it is built and configured, on the non-RT configure path, before it is installed
//    (PlannerCycle::InstallSearch).
//  • ONE CALLER. The planner thread is the only caller once the object is
//    installed; nothing here is synchronised. SetClock is called while that
//    thread is not running.
//  • CLOCK. A search that measures its own budget is handed the cycle's clock
//    when the cycle installs it and again on every PlannerCycle::SetClock
//    (SetClock below), so the budget is measured on the axis the cycle stamps
//    `publish_ns` on.
//  • SearchStats is the record a wake leaves (search_stats.hpp). An
//    implementation fills what it has and leaves the rest at the default. The
//    cycle reads ONE field of it back: `publish` (see Plan).
#pragma once

#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"  // CovarianceSnapshot
#include "rtc_controllers/catching/trajectory.hpp"

#include <cstdint>
#include <type_traits>

namespace rtc::catching {

struct SearchStats;  // search_stats.hpp

/// @brief The arm trajectory a search solved for the candidate it chose, in
///        the form a segment planner publishes.
///
/// `seg` is a whole SegmentSnapshot — grid, nodes and the RT state it was
/// solved from — so that the one evaluator of a two-spacing segment
/// (NodeTrajectoryFollower::SampleJoints) reads a solution exactly as it reads
/// a published segment. `seg.publish_ns`, `seg.segment_seq` and `seg.plan_id`
/// are the cycle's to fill and are zero here.
///
/// A receiver uses it for a plan only when it is that plan's and of the same
/// RT report: `seg.t_c_ns == plan.t_c_ns && seg.rt_iteration == rt.rt_iteration`
/// — and, to publish it, when node 0 started on the segment the RT reports for
/// that instant: `source_seq` equal to SegmentPlanner::SourceSeq(rt, seg.t0_ns).
struct CatchSolution {
  SegmentSnapshot seg{};
  /// The `segment_seq` of the reported segment node 0's state was evaluated
  /// on; 0 when the arm started at rest on its command.
  std::uint32_t source_seq{0};
  double cost_reference{0.0};  ///< the solve's cost up to the catch node
  double cost_stop{0.0};       ///< and after it — not part of the choice
  bool feasible{false};        ///< every hard row within tolerance
  bool converged{false};       ///< the solve met its own stopping test
};

static_assert(std::is_trivially_copyable_v<CatchSolution>);

/// @brief The search one planner wake runs: a trajectory snapshot and its
///        covariance in, one catch plan (or "no plan") out.
///
/// Owned by PlannerCycle through this interface. See the header comment for
/// the thread and RT contract every override is held to.
class CatchSearch {
 public:
  using ClockFn = std::int64_t (*)() noexcept;

  virtual ~CatchSearch() = default;

  /// @brief Replace the steady clock the search measures its budget on
  ///        (non-RT; the planner thread is not running). A null `clock` is
  ///        ignored. Called by the cycle on install and on
  ///        PlannerCycle::SetClock.
  virtual void SetClock(ClockFn clock) noexcept = 0;

  /// @brief One search (RT-safe).
  /// @param traj the trajectory snapshot this wake plans against — already
  ///        checked to be valid and of the RT's activation
  /// @param cov the covariance box as read; `cov_matched` says whether it is
  ///        `traj`'s (same provenance token). An unmatched one is not that
  ///        prediction's uncertainty and must not be used as if it were.
  /// @param rt the RT's report: the current command and the plan it follows
  /// @param arm the segments `rt` reports pending and following, as copies
  ///        (the cycle's scratch — valid for this call). Both absent when the
  ///        RT follows no plan or no segment planner is installed
  /// @param now the planning 'now' on the steady axis (the wake instant)
  /// @param budget_cap_ns an upper bound on this call's compute budget [ns],
  ///        0 = none. The search runs on the smaller of its own budget and
  ///        this: the cycle caps a wake that has a replan to run behind the
  ///        search, so that the two fit one prediction period.
  /// @param[out] stats the wake's search record; `stats.publish` false tells
  ///             the cycle to publish nothing (the RT keeps the plan it has)
  /// @return the plan, with its provenance filled and `valid` false when no
  ///         candidate was chosen. `plan_id` and `publish_ns` are the cycle's.
  [[nodiscard]] virtual PlanSnapshot Plan(const TrajectorySnapshot& traj,
                                          const CovarianceSnapshot& cov, bool cov_matched,
                                          const PlannerRtState& rt, const ReportedSegments& arm,
                                          NowReal now, std::int64_t budget_cap_ns,
                                          SearchStats& stats) noexcept = 0;

  /// @brief The arm trajectory the last Plan() solved for the plan it
  ///        returned, or nullptr: no plan was chosen, or this search does not
  ///        solve one (RT-safe).
  ///
  /// Null IS "none" — there is no validity flag to forget. The pointer is good
  /// until the next Plan() or ResetTrial().
  [[nodiscard]] virtual const CatchSolution* Solution() const noexcept = 0;

  /// @brief A wake after the RT committed (monitorOnly, L3 §4.6): record what
  ///        the newest prediction says about the plan the RT follows. Nothing
  ///        is published from it (RT-safe).
  /// @param[out] stats reset, with `publish` false
  virtual void Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                       bool cov_matched, const PlannerRtState& rt,
                       SearchStats& stats) const noexcept = 0;

  /// @brief The cycle stored `plan` (after its provenance re-check). Called
  ///        for every stored snapshot, a "no plan" one included (RT-safe).
  ///
  /// Published is not followed: the RT may refuse it. What the arm is doing
  /// is what `rt` reports on a later call.
  virtual void NotePublished(const PlanSnapshot& plan) noexcept = 0;

  /// @brief The RT's reset epoch moved: drop every per-trial memory (RT-safe).
  virtual void ResetTrial() noexcept = 0;

 protected:
  // Copying is the concrete type's business; through the interface it would
  // slice.
  CatchSearch() = default;
  CatchSearch(const CatchSearch&) = default;
  CatchSearch& operator=(const CatchSearch&) = default;
  CatchSearch(CatchSearch&&) = default;
  CatchSearch& operator=(CatchSearch&&) = default;
};

}  // namespace rtc::catching
