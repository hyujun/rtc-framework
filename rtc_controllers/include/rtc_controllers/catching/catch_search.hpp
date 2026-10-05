// ── The catch search, as one planner wake calls it (E1-F12 #738, ARCH-3) ─────
//
// What PlannerCycle knows of the search that turns a predicted ball into a
// catch plan: one cycle's candidates in, one PlanSnapshot out (L3 §4, the A-4
// single entry point), plus the three calls that keep the search's per-trial
// memory in step with what the cycle published and with the RT's trials. Which
// search runs behind it is the configuration's choice — a wake calls this
// interface and nothing else.
//
// The one implementation today is GridCatchSearch (grid_catch_search.hpp): a grid
// over the vision samples with a closed-form γ profile.
//
// ── Contract ──────────────────────────────────────────────────────────────────
//  • THREAD. Every member below is called from PlannerCycle::Run, on the
//    planner thread — which may run SCHED_FIFO (D-7a). RT-1~10 apply to all of
//    them: no allocation, no lock, no log, no throw. An implementation sizes
//    its buffers when it is built and configured, on the non-RT configure
//    path, before it is installed (PlannerCycle::InstallSearch).
//  • ONE CALLER. The planner thread is the only caller once the object is
//    installed; nothing here is synchronised.
//  • CLOCK. A search that measures its own budget takes its clock when it is
//    configured. The cycle does not hand it one later (PlannerCycle::SetClock
//    reaches the segment planner only).
//  • SearchStats is the record a wake leaves (grid_catch_search.hpp). It is
//    declared there, with the search whose counters it holds; an
//    implementation fills what it has and leaves the rest at the default. The
//    cycle reads ONE field of it back: `publish` (see Plan).
#pragma once

#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"  // CovarianceSnapshot
#include "rtc_controllers/catching/trajectory.hpp"

namespace rtc::catching {

struct SearchStats;  // grid_catch_search.hpp

/// @brief The search one planner wake runs: a trajectory snapshot and its
///        covariance in, one catch plan (or "no plan") out.
///
/// Owned by PlannerCycle through this interface. See the header comment for
/// the thread and RT contract every override is held to.
class CatchSearch {
 public:
  virtual ~CatchSearch() = default;

  /// @brief One search (RT-safe).
  /// @param traj the trajectory snapshot this wake plans against — already
  ///        checked to be valid and of the RT's activation
  /// @param cov the covariance box as read; `cov_matched` says whether it is
  ///        `traj`'s (same provenance token). An unmatched one is not that
  ///        prediction's uncertainty and must not be used as if it were.
  /// @param rt the RT's report: the current command and the plan it follows
  /// @param now the planning 'now' on the steady axis (the wake instant)
  /// @param[out] stats the wake's search record; `stats.publish` false tells
  ///             the cycle to publish nothing (the RT keeps the plan it has)
  /// @return the plan, with its provenance filled and `valid` false when no
  ///         candidate was chosen. `plan_id` and `publish_ns` are the cycle's.
  [[nodiscard]] virtual PlanSnapshot Plan(const TrajectorySnapshot& traj,
                                          const CovarianceSnapshot& cov, bool cov_matched,
                                          const PlannerRtState& rt, NowReal now,
                                          SearchStats& stats) noexcept = 0;

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
