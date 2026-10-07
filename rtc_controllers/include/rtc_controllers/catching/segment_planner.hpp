// ── The segment planner, as one planner wake calls it (E1-F12 #738, ARCH-3) ──
//
// What PlannerCycle knows of the planner that solves the joint-node segments
// the RT follows from APPROACH to the end of the stop (planner_cycle.hpp "THE
// MPC SEGMENT PLANNER'S PART"): the first segment of a plan the search just
// produced — the first plan of a catch, or the REPLACEMENT of the plan the RT
// follows — later segments of the plan the RT follows, and the questions the
// cycle's re-checks ask about them. The cycle owns the SeqLock, the re-check
// and the segment counter; which planner solves is the configuration's choice.
//
// The one implementation today is the mpc segment planner: linearised joint-space
// QPs on a grid anchored at the catch instant.
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
//  • SegmentRecord is the record a solve leaves (below). An implementation
//    fills what it has and leaves the rest at the default — with ONE
//    exception, because the cycle reads it back: `source_seq` after a Replan
//    that returned true, and after a PlanFirst that returned true for an RT
//    that follows a plan (see both). The cycle itself writes `outcome`
//    (kPublished / kSuperseded), `segment_seq` and `publish_ns`.
#pragma once

#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"  // CovarianceSnapshot
#include "rtc_controllers/catching/trajectory.hpp"

#include <cstdint>
#include <limits>

namespace rtc::catching {

struct CatchSolution;  // catch_search.hpp

/// What one segment step did (the planner events CSV's segment columns). The CSV
/// writes the NAME (SegmentOutcomeName), never the value: the values carry no
/// meaning outside a build and move when an enumerator is added or removed.
enum class SegmentOutcome : std::uint8_t {
  kOff = 0,  ///< not attempted: not configured, or not a mode that plans a segment
  kNoState,  ///< no followed plan / t_c, unseeded command, or a size mismatch
  /// The RT's report is older than the planner's age limit (or from the
  /// future): what it reports may no longer be what the arm does (an RT stall).
  kStaleState,
  kUpToDate,          ///< the published segment already starts at this t_eff or later
  kPastReplanWindow,  ///< t_eff beyond t_c + k_max·Δ_s (MD-31)
  kInputNonFinite,    ///< the predicted x₀ (w_⊥ > 0: or the ball's speed) is not finite
  kSolveFailed,       ///< the core refused or the QP failed (core_reason)
  kBudget,            ///< solved after budget_s
  kLate,              ///< t_eff passed while solving
  kSlack,             ///< slack non-finite or over its threshold (MD-33)
  kReady,             ///< publishable; the cycle's re-check decides
  kPublished,         ///< stored (set by the cycle)
  kSuperseded,        ///< the trial or the followed plan moved during the solve (cycle)
  // E1-F08 (#661). A solved segment whose packed form fails the terminal-rest
  // or node check is kSolveFailed with core_reason kNone.
  kNotAtRest,    ///< first solve, no plan followed: max |q̇_cmd| above approach.rest_tol
  kTooLate,      ///< first solve: not even one pre-catch interval fits before t_c
  kNotFollowed,  ///< replan, or a replacement's first solve: no segment of ours the RT
                 ///< reports pending or following
  kNoBall,       ///< no usable ball at t_c (pre-catch point); w_⊥ > 0: no stop-path line
  kCatchError,   ///< catch-node position error not finite or over catch_pos_err_max
  kSpeed,        ///< a between-node velocity extremum over q̇_max
  // kNoBall is NOT pre-catch only. With `cost.w_perp` > 0 it is also what a
  // solve reports when it has no stop-path line to run on (header note): the
  // first solve or a pre-catch replan whose ball is slower than v_eps at t_c,
  // and a STOP grid point — at or after the catch, where no ball is read at
  // all — whose source segment carries no line. kInputNonFinite likewise
  // covers a ball speed that is not finite.
};

[[nodiscard]] constexpr const char* SegmentOutcomeName(SegmentOutcome o) noexcept {
  switch (o) {
    case SegmentOutcome::kOff:
      return "off";
    case SegmentOutcome::kNoState:
      return "no_state";
    case SegmentOutcome::kStaleState:
      return "stale_state";
    case SegmentOutcome::kUpToDate:
      return "up_to_date";
    case SegmentOutcome::kPastReplanWindow:
      return "past_replan_window";
    case SegmentOutcome::kInputNonFinite:
      return "input_non_finite";
    case SegmentOutcome::kSolveFailed:
      return "solve_failed";
    case SegmentOutcome::kBudget:
      return "budget";
    case SegmentOutcome::kLate:
      return "late";
    case SegmentOutcome::kSlack:
      return "slack";
    case SegmentOutcome::kReady:
      return "ready";
    case SegmentOutcome::kPublished:
      return "published";
    case SegmentOutcome::kSuperseded:
      return "superseded";
    case SegmentOutcome::kNotAtRest:
      return "not_at_rest";
    case SegmentOutcome::kTooLate:
      return "too_late";
    case SegmentOutcome::kNotFollowed:
      return "not_followed";
    case SegmentOutcome::kNoBall:
      return "no_ball";
    case SegmentOutcome::kCatchError:
      return "catch_error";
    case SegmentOutcome::kSpeed:
      return "speed";
  }
  return "unknown";
}

/// Which solve a record describes (E1-F08).
enum class SegmentKind : std::uint8_t {
  kNone = 0,  ///< no solve was chosen (the record's default)
  kFirst,     ///< the first segment of a plan, solved with the search (MD-56)
  kSame,      ///< a pre-catch grid point the source already starts at (MD-58)
  kAdvance,   ///< a later pre-catch grid point
  kStop,      ///< a stop core: the catch node or a post-catch grid point
};

[[nodiscard]] constexpr const char* SegmentKindName(SegmentKind k) noexcept {
  switch (k) {
    case SegmentKind::kNone:
      return "none";
    case SegmentKind::kFirst:
      return "first";
    case SegmentKind::kSame:
      return "same";
    case SegmentKind::kAdvance:
      return "advance";
    case SegmentKind::kStop:
      return "stop";
  }
  return "unknown";
}

struct SegmentRecord {
  SegmentOutcome outcome{SegmentOutcome::kOff};
  /// The solving core's own reason as its code: an enumerator of the installed
  /// planner's core, 0 = none. Comparable and digestable, meaningless without
  /// knowing the planner.
  std::uint8_t core_reason{0};
  /// That code's name as the planner's core spells it (static storage, never
  /// null) — what a log or CSV writes.
  const char* core_reason_name{"none"};
  /// Grid index of node 0: t_eff = t_c + k·Δ_s for a stop grid point, −n_pre
  /// for a pre-catch one (the CSV's segment_k).
  std::int32_t k{-1};
  std::int32_t n_nodes{0};
  std::uint32_t segment_seq{0};  ///< the published segment's seq (cycle)
  bool x0_clamped{false};        ///< the start state (q or q̇) was projected into the box
  /// x₀ came from a segment the RT reports: a replan, and the first solve of a
  /// replacement (the RT follows a plan).
  bool x0_from_segment{false};
  bool presolved{false};  ///< no reference: kinematic pre-solve + solve
  /// First solve: the segment is the search's own solution (CatchSolution),
  /// re-evaluated by the planner and published as it is — nothing was solved.
  bool from_search{false};
  bool cold_retry{false};  ///< a stop core's reference was refused, re-solved without it
  std::int32_t iterations{0};
  std::int32_t qp_status{-1};
  std::int64_t solve_ns{0};    ///< solve start → solve end (the budget's measure)
  std::int64_t publish_ns{0};  ///< the stored segment's stamp (cycle); 0 when not stored
  double slack_max{std::numeric_limits<double>::quiet_NaN()};
  double slack_terminal_max{std::numeric_limits<double>::quiet_NaN()};
  double tau_ratio_max{std::numeric_limits<double>::quiet_NaN()};

  SegmentKind kind{SegmentKind::kNone};
  bool cold_start{false};      ///< the core's main QP started from zero
  bool solver_retried{false};  ///< the core's own warm → cold re-run (cold_retried)
  /// First solve: the reference's target was outside the core's position
  /// box (clamped), or — from an arm at rest — its minimum-jerk speed over
  /// 0.9·η_v·q̇_max (scaled). A reference that starts on a moving arm is never
  /// scaled.
  bool ref_clamped{false};
  bool ref_scaled{false};
  double ref_scale{std::numeric_limits<double>::quiet_NaN()};      ///< smallest per-joint factor
  double ref_shortfall{std::numeric_limits<double>::quiet_NaN()};  ///< largest |d| cut [rad]
  /// First solve: the largest joint speed of the start — max |q̇_cmd| with no
  /// plan followed, max |q̇₀| of the reported segment at node 0's instant (as
  /// read, before the projection) under a replacement.
  double x0_speed{std::numeric_limits<double>::quiet_NaN()};
  /// Catch node at the solution (FK), when the solve ran the catch terms.
  double catch_pos_err{std::numeric_limits<double>::quiet_NaN()};   ///< [m]
  double catch_axis_err{std::numeric_limits<double>::quiet_NaN()};  ///< [rad]
  double catch_gamma{std::numeric_limits<double>::quiet_NaN()};
  /// ‖v̂_b − J_v q̇‖ at the catch node [m/s], and the core's velocity slack s_v
  /// (fraction of v_rel_allow, linear model; 0 when the slack row is off).
  double catch_v_rel{std::numeric_limits<double>::quiet_NaN()};
  double slack_v{std::numeric_limits<double>::quiet_NaN()};
  /// max over nodes and between-node extrema of |q̇|/q̇_max.
  double speed_ratio_max{std::numeric_limits<double>::quiet_NaN()};
  bool w_p_fallback{false};  ///< W_p was the constant w_const·I (no usable Σ_p)
  double w_delta_scale{std::numeric_limits<double>::quiet_NaN()};
  /// The segment x₀ came from (a replan: the reference too): a replan, and the
  /// first solve of a replacement. 0 when x₀ is the reported command.
  std::uint32_t source_seq{0};
};

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
  ///
  /// What `rt` reports decides where the segment starts:
  ///  - NO PLAN FOLLOWED (`rt.plan_active` false): the arm rests on its
  ///    command — x₀ = (q_cmd, 0, 0), and an arm that does not rest is
  ///    kNotAtRest.
  ///  - A PLAN FOLLOWED: `plan` REPLACES it. The arm is on that plan's
  ///    segments until the RT switches at the new segment's node 0, so
  ///    x₀ = (q, q̇, q̈) is the segment SourceSeq(rt, node 0's instant) names,
  ///    evaluated at that instant by NodeTrajectoryFollower::SampleJoints —
  ///    as a replan's is. No such segment is kNotFollowed. The command has to
  ///    be seeded. The followed plan's segments are kept: Reported, SourceSeq
  ///    and FollowedTrack go on answering for the followed plan, and a later
  ///    Replan of it still finds its source.
  ///
  /// Either way node 0 is on the grid anchored at `plan.t_c_ns`, and the
  /// segment carries `plan`'s id, catch instant and track.
  /// @param rt the RT's report this wake started from
  /// @param plan the search's plan, its `plan_id` already the one the cycle
  ///        will publish it under
  /// @param ball this wake's prediction — the one `plan` was searched on.
  ///        Never empty here.
  /// @param solution the arm trajectory the search solved for `plan`, or
  ///        nullptr when it solved none (CatchSearch::Solution). A planner
  ///        that solves its own problem ignores it; one that would solve the
  ///        search's problem again may publish it instead, after the match
  ///        check CatchSolution names. The cycle's scratch — valid for this
  ///        call.
  /// @param[out] out the segment; its `publish_ns` and `segment_seq` are the
  ///             cycle's to fill
  /// @param[out] rec the solve's record. On a true return for an RT that
  ///             follows a plan `rec.source_seq` MUST be the `segment_seq` of
  ///             the segment the solve started from — what
  ///             SourceSeq(rt, out.t0_ns) answers — so that the cycle can ask
  ///             again with the RT's newest report, as it does for a replan.
  /// @return true when `out` is publishable. False withholds the plan too.
  [[nodiscard]] virtual bool PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                                       const BallPrediction& ball, const CatchSolution* solution,
                                       SegmentSnapshot& out, SegmentRecord& rec) noexcept = 0;

  /// @brief A later segment of the plan `rt` follows (RT-safe; MD-58).
  /// @param ball the followed plan's ball as this wake read it, or an empty
  ///        view: after the catch, and when the trajectory in the box is not
  ///        that plan's track
  /// @param[out] rec the solve's record. On a true return `rec.source_seq`
  ///             MUST be the `segment_seq` of the segment the solve started
  ///             from — what SourceSeq(rt, out.t0_ns) answers. It is a publish
  ///             gate, not a diagnostic: the cycle asks SourceSeq again with
  ///             the RT's newest report and drops the segment (kSuperseded)
  ///             when the two differ. A planner that left it at the default
  ///             would have every replan dropped.
  /// @return true when `out` is publishable; the cycle's re-check decides.
  [[nodiscard]] virtual bool Replan(const PlannerRtState& rt, const BallPrediction& ball,
                                    SegmentSnapshot& out, SegmentRecord& rec) noexcept = 0;

  /// @brief The cycle stored `p` (its `segment_seq` and `publish_ns` filled): a
  ///        later Replan may start from it (RT-safe). Called right after the
  ///        PlanFirst or Replan that returned true for it, and only then. The
  ///        first segment of a replacement is remembered BESIDE the followed
  ///        plan's segments, not instead of them.
  virtual void NotePublished(const SegmentSnapshot& p) noexcept = 0;

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

  /// @brief The budget of one Replan [ns], > 0 (RT-safe): what a wake that
  ///        searches while a plan is followed has to leave for the replan
  ///        behind the search.
  [[nodiscard]] virtual std::int64_t ReplanBudgetNs() const noexcept = 0;

  /// @brief The earliest instant node 0 of a first segment solved from
  ///        `now_ns` can be at [ns, lead axis] (RT-safe): `now_ns` plus the
  ///        arm's command latency, the first-solve budget and two control
  ///        periods — the bound PlanFirst places node 0 behind. The cycle asks
  ///        it before a replacement's first solve: a segment that could not
  ///        start before the followed plan freezes is not worth solving.
  [[nodiscard]] virtual std::int64_t EarliestFirstStartNs(std::int64_t now_ns) const noexcept = 0;

  /// @brief The segments `rt` reports pending and following, copied from
  ///        what this planner published (RT-safe) — what the cycle hands the
  ///        search, so that a candidate's arm motion starts where the arm
  ///        will be. The followed one is a segment of the plan `rt` follows;
  ///        the pending one is that plan's, or the first segment of the
  ///        replacement `rt` holds (`rt.plan_pending`).
  /// @param[out] out both flags are always written; a snapshot is written
  ///             only when its flag is true (see ReportedSegments)
  virtual void Reported(const PlannerRtState& rt, ReportedSegments& out) const noexcept = 0;

  /// @brief The `segment_seq` of the segment a solve at `t_eff_ns` starts from
  ///        — a replan, or the first segment of a replacement: one the RT
  ///        reports pending or following, never inferred — or 0 when there is
  ///        none (RT-safe). SourceSegmentAt (planner_io.hpp) on what Reported()
  ///        answers.
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
