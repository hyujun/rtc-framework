// ── The segment planner, as one planner wake calls it (E1-F12 #738, ARCH-3) ──
//
// What PlannerCycle knows of the planner that solves the joint-node segments
// the RT follows from APPROACH to the end of the stop (planner_cycle.hpp "THE
// DECEL PLANNER'S PART"): the first segment of a plan the search just
// produced, later segments of the plan the RT follows, and the questions the
// cycle's re-checks ask about them. The cycle owns the SeqLock, the re-check
// and the segment counter; which planner solves is the configuration's choice.
//
// The one implementation today is DecelPlanner (decel_planner.hpp): linearised
// joint-space QPs on a grid anchored at the catch instant.
//
// ── Contract ──────────────────────────────────────────────────────────────────
//  • THREAD. Every member except SetClock is called from PlannerCycle::Run, on
//    the planner thread — which may run SCHED_FIFO (D-7a). RT-1~10 apply to
//    them: no lock, no log, no throw, and no allocation of the
//    implementation's own (a solver's internal ones are that implementation's
//    recorded exception, not a licence). Buffers are sized when the object is
//    built and configured, on the non-RT configure path, before it is
//    installed (PlannerCycle::InstallSegmentPlanner).
//  • ONE CALLER. The planner thread is the only caller once the object is
//    installed; nothing here is synchronised. SetClock is called while that
//    thread is not running.
//  • THE BALL comes as a VIEW of the wake's own trajectory and covariance
//    (BallPrediction), not as a value at one instant: what a planner needs of
//    the prediction — a point at the catch instant, a sample per node — is its
//    own business. The cycle decides only WHETHER the wake has the followed
//    plan's ball, and hands over an empty view when it does not.
//  • CLOCK. The cycle hands the planner its clock when it installs it and
//    again on every PlannerCycle::SetClock, so the solves are timed on the
//    axis the cycle stamps `publish_ns` on.
//  • DecelRecord is the record a solve leaves (decel_planner.hpp). It is
//    declared there, with the planner whose counters it holds; an
//    implementation fills what it has and leaves the rest at the default —
//    with ONE exception, because the cycle reads it back: `source_seq` after a
//    Replan that returned true (see Replan). The cycle itself writes
//    `outcome` (kPublished / kSuperseded), `decel_seq` and `publish_ns`.
#pragma once

#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"  // CovarianceSnapshot
#include "rtc_controllers/catching/trajectory.hpp"

#include <cstdint>

namespace rtc::catching {

struct DecelRecord;  // decel_planner.hpp

/// @brief The ball's prediction as one wake read it: the trajectory snapshot
///        and the covariance box, not owned.
///
/// Empty (both pointers null) when the wake has no prediction of the followed
/// plan's ball — a wake after the catch reads none, and a trajectory of
/// another track is not this catch's. The pointers are the cycle's scratch
/// copies and are valid for the call they are passed to, no longer.
struct BallPrediction {
  const TrajectorySnapshot* traj{nullptr};
  const CovarianceSnapshot* cov{nullptr};
  /// `cov` carries the same provenance token as `traj` (L3 §5.2). False: it
  /// is another snapshot's covariance and says nothing about this one.
  bool cov_matched{false};

  [[nodiscard]] constexpr bool Empty() const noexcept { return traj == nullptr || cov == nullptr; }
};

/// @brief The planner of the segments the RT follows through one catch.
///
/// Owned by PlannerCycle through this interface. See the header comment for
/// the thread and RT contract every override is held to.
class SegmentPlanner {
 public:
  using ClockFn = std::int64_t (*)() noexcept;

  virtual ~SegmentPlanner() = default;

  /// @brief Replace the steady clock the solves are timed on (non-RT; the
  ///        planner thread is not running). A null `clock` is ignored. Called
  ///        by the cycle on install and on PlannerCycle::SetClock.
  virtual void SetClock(ClockFn clock) noexcept = 0;

  /// @brief The RT's reset epoch moved: drop every per-trial memory — the
  ///        published segments with it (RT-safe).
  virtual void ResetTrial() noexcept = 0;

  /// @brief The first segment of a plan the search just produced (RT-safe;
  ///        MD-56). A plan is published only together with it.
  /// @param rt the RT's report this wake started from
  /// @param plan the search's plan, its `plan_id` already the one the cycle
  ///        will publish it under
  /// @param ball this wake's prediction — the one `plan` was searched on.
  ///        Never empty here.
  /// @param[out] out the segment; its `publish_ns` and `decel_seq` are the
  ///             cycle's to fill
  /// @param[out] rec the solve's record
  /// @return true when `out` is publishable. False withholds the plan too.
  [[nodiscard]] virtual bool PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                                       const BallPrediction& ball, DecelPlanSnapshot& out,
                                       DecelRecord& rec) noexcept = 0;

  /// @brief A later segment of the plan `rt` follows (RT-safe; MD-58).
  /// @param ball the followed plan's ball as this wake read it, or an empty
  ///        view: after the catch, and when the trajectory in the box is not
  ///        that plan's track
  /// @param[out] rec the solve's record. On a true return `rec.source_seq`
  ///             MUST be the `decel_seq` of the segment the solve started
  ///             from — what SourceSeq(rt, out.t0_ns) answers. It is a publish
  ///             gate, not a diagnostic: the cycle asks SourceSeq again with
  ///             the RT's newest report and drops the segment (kSuperseded)
  ///             when the two differ. A planner that left it at the default
  ///             would have every replan dropped.
  /// @return true when `out` is publishable; the cycle's re-check decides.
  [[nodiscard]] virtual bool Replan(const PlannerRtState& rt, const BallPrediction& ball,
                                    DecelPlanSnapshot& out, DecelRecord& rec) noexcept = 0;

  /// @brief The cycle stored `p` (its `decel_seq` and `publish_ns` filled): a
  ///        later Replan may start from it (RT-safe). Called right after the
  ///        PlanFirst or Replan that returned true for it, and only then.
  virtual void NotePublished(const DecelPlanSnapshot& p) noexcept = 0;

  /// @brief The track generation of the plan `rt` follows, from the segments
  ///        published for it (RT-safe).
  ///
  /// After the freeze the RT keeps the committed track while
  /// `rt.track_generation` moves on to whatever it consumed last, so that
  /// field does not say whose ball the plan catches.
  /// @return false when nothing was published for that plan.
  [[nodiscard]] virtual bool FollowedTrack(const PlannerRtState& rt,
                                           std::uint64_t& generation) const noexcept = 0;

  /// @brief Whether the RT can still read a segment whose node 0 is at
  ///        `t0_ns` when it is published at `publish_ns` (RT-safe). The
  ///        cycle's re-checks ask it at the publish stamp.
  [[nodiscard]] virtual bool StartsInTime(std::int64_t publish_ns,
                                          std::int64_t t0_ns) const noexcept = 0;

  /// @brief The RT's control period [ns] (RT-safe).
  [[nodiscard]] virtual std::int64_t ControlDtNs() const noexcept = 0;

  /// @brief The `decel_seq` of the segment a replan at `t_eff_ns` starts from
  ///        — one the RT reports pending or following, never inferred — or 0
  ///        when there is none (RT-safe).
  [[nodiscard]] virtual std::uint32_t SourceSeq(const PlannerRtState& rt,
                                                std::int64_t t_eff_ns) const noexcept = 0;

 protected:
  // Copying is the concrete type's business; through the interface it would
  // slice.
  SegmentPlanner() = default;
  SegmentPlanner(const SegmentPlanner&) = default;
  SegmentPlanner& operator=(const SegmentPlanner&) = default;
  SegmentPlanner(SegmentPlanner&&) = default;
  SegmentPlanner& operator=(SegmentPlanner&&) = default;
};

}  // namespace rtc::catching
