// ── The docking segment planner (E1-F16 #742) ────────────────────────────────
//
// The second SegmentPlanner (segment_planner.hpp, ARCH-3): the joint-node
// segments the RT follows, solved by MpcDockingSegmentCore — the nonlinear
// docking problem (entrance plane, corridor, capture set, timing window,
// impact) on a grid anchored at the catch instant, with the stop behind it.
// What the cycle calls is the interface; this header is for the configure
// path that builds one and for tests.
//
// ── What a solve is ───────────────────────────────────────────────────────────
//  • GRID. A segment has n_pre pre-catch intervals of Δ_pre before the catch
//    node and the stop part's n_stop intervals of Δ_s after it; node 0 is at
//    t_c − n_pre·Δ_pre. One core is built per n_pre in 1..n_pre_max
//    (`approach.n_pre_max`), each with one move block per pre-catch interval
//    and the stop part's blocks (`stop.blocks`). Every segment runs to the end
//    of the stop.
//  • THE CATCH NODE is where the ball crosses the hand's entrance plane (the
//    core's ℓ_kc = 0): under this planner a plan's t_c is that crossing,
//    whichever search produced it.
//  • PlanFirst — the first segment of a plan. Where it starts depends on what
//    the RT does:
//      – NO PLAN FOLLOWED: from an arm AT REST on its command (max |q̇_cmd| ≤
//        `approach.rest_tol`; otherwise kNotAtRest), x₀ = (q_cmd, 0, 0).
//      – A PLAN FOLLOWED (rt.plan_active): the segment is the first of that
//        plan's REPLACEMENT and the arm is moving. x₀ = (q, q̇, q̈) is the
//        segment the RT reports pending or following (SourceSeq at node 0's
//        instant) evaluated there, projected into the core's box as a
//        replan's is; none reported is kNotFollowed. The followed plan's
//        segments stay in the ring beside the new one: the arm is on them
//        until the RT switches at the new segment's node 0.
//    Either way:
//      – A search that solved the same problem hands its solution in
//        (CatchSolution). When it is this plan's, of this RT report, on this
//        planner's grid, and started on the segment the RT reports for its
//        node 0 (CatchSolution::source_seq is SourceSeq there — none, for an
//        arm that follows no plan), it is NOT solved again: the nodes are
//        re-evaluated on this planner's own core against the wake's
//        prediction (the hard rows' violation, the slacks, the torque) and
//        published as they are, bit for bit. Whether the solve that produced
//        them converged is the search's statement (CatchSolution::converged):
//        an evaluation does not converge.
//      – Otherwise the core solves from x₀, started by its initialisation QP
//        toward the plan's catch pose (plan.q_star).
//  • Replan — a later segment of the followed plan, at the first grid point
//    the replan budget still reaches (the one that leaves the most pre-catch
//    intervals). x₀ is the source segment (the one the RT reports pending or
//    following, SourceSeq) evaluated at node 0's instant;
//    the start trajectory is that segment sampled at the new nodes' instants.
//    The catch instant does not move.
//  • NOTHING IS PUBLISHED AFTER THE CATCH. A replan needs at least one
//    pre-catch interval (the core has no problem without one); once the budget
//    no longer reaches a grid point before t_c the last published segment
//    carries the arm through the catch and to rest (kPastReplanWindow).
//
// ── What is published ─────────────────────────────────────────────────────────
// A segment comes out publishable (kReady) only when: the solve ended inside
// its budget (`budget.first_s` / `budget.replan_s`, also the core's deadline);
// node 0 can still be read by the RT (StartsInTime); every hard row holds on
// the nonlinear model and the solve converged; the corridor and speed-envelope
// slacks are within `publish.slack_c_max` / `publish.slack_v_max` (they are
// soft in the problem, so a "feasible" solution may lean on them); the joint
// speed stays within the rating BETWEEN the nodes as well; and the packed
// nodes pass ValidateSegmentNodes. The cycle's own re-checks follow.
//
// ── Contract ──────────────────────────────────────────────────────────────────
//  • Configure() is non-RT: it Inits every core and runs one warm-up solve on
//    each (a core's first solve is the slow one).
//  • PlanFirst / Replan and the queries are RT-safe apart from the QP solver:
//    this planner allocates nothing in them; ProxQP allocates inside its own
//    update()/solve() — a recorded RT-1 violation, counted and not asserted
//    (agent_docs/invariants.md §RT Path, #654).
//  • Joint order: the RT speaks DEVICE order (PlannerRtState, the payload);
//    the cores speak the model's pinocchio velocity order. The mapping is
//    `device_of_model`, as for the search.
#pragma once

#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/catching/segment_ring.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rtc::catching {

/// Largest age of the RT's report (steady now − rt_state_ns) a solve starts
/// from: the same bound the other segment planner uses, for the same reason —
/// the RT stores every tick, so anything older means the tick stalled.
inline constexpr std::int64_t kMpcDockingMaxRtStateAgeNs = 50'000'000;

/// The arm the planner plans for (configure time, from the binding).
struct MpcDockingSegmentPlannerModel {
  std::shared_ptr<const pinocchio::Model> arm;  ///< hand-locked catch sub-model
  pinocchio::FrameIndex catch_frame{0};         ///< its +z is the approach axis
  int nv{0};
  /// `device_of_model[j]` = the arm device's index of model joint j.
  std::array<int, kMaxPlanNv> device_of_model{};
  /// The cores' box, model order (the caller's margins already applied).
  MpcDockingSegmentCoreLimits limits;
  /// The RATING q̇_max, model order [rad/s] — what the between-node speed is
  /// held to (the cores' own velocity box, in `limits`, may sit inside it).
  std::array<double, kMaxPlanNv> qd_rating{};
  /// A pose the warm-up solves start from, DEVICE order (the wait pose).
  std::array<double, kMaxPlanNv> warm_pose{};
};

struct MpcDockingSegmentPlannerConstants {
  double t_arm_s{0.0};       ///< `joint_cmd.lag.T_arm` — real → lead axis
  double control_dt{0.002};  ///< the RT period [s]
};

/// @brief The docking segment planner (header note).
class MpcDockingSegmentPlanner final : public SegmentPlanner {
 public:
  MpcDockingSegmentPlanner() = default;

  /// @brief Build the n_pre_max cores and warm each up (non-RT).
  /// @param params the planner's values; `params.core` already carries the
  ///        hand's identified values (ApplyHandDocking) — the grid fields in
  ///        it are overwritten per core from `params`' own
  /// @return false, the planner unconfigured and `error` (if given) naming the
  ///         cause — a core's refusal by its MpcDockingReason name.
  [[nodiscard]] bool Configure(const MpcDockingSegmentPlannerModel& model,
                               const MpcDockingSegmentPlannerConstants& consts,
                               const MpcDockingSegmentPlannerParams& params, ClockFn clock,
                               std::string* error = nullptr);

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  void SetClock(ClockFn clock) noexcept override;

  void ResetTrial() noexcept override;

  [[nodiscard]] bool PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                               const BallPrediction& ball, const CatchSolution* solution,
                               SegmentSnapshot& out, SegmentRecord& rec) noexcept override;

  [[nodiscard]] bool Replan(const PlannerRtState& rt, const BallPrediction& ball,
                            SegmentSnapshot& out, SegmentRecord& rec) noexcept override;

  void NotePublished(const SegmentSnapshot& p) noexcept override;

  [[nodiscard]] bool FollowedTrack(const PlannerRtState& rt,
                                   std::uint64_t& generation) const noexcept override;

  /// publish + T_arm + 2 ticks < t0: the one statement of that lead, used by
  /// the solve's own late check and by the cycle's re-checks.
  [[nodiscard]] bool StartsInTime(std::int64_t publish_ns,
                                  std::int64_t t0_ns) const noexcept override {
    return publish_ns + t_arm_ns_ + 2 * h_ns_ < t0_ns;
  }

  [[nodiscard]] std::int64_t ControlDtNs() const noexcept override { return h_ns_; }

  /// `budget.replan_s` [ns].
  [[nodiscard]] std::int64_t ReplanBudgetNs() const noexcept override { return replan_ns_; }

  /// now + T_arm + `budget.first_s` + 2 ticks: the bound PlanFirst's grid
  /// point is the first one behind.
  [[nodiscard]] std::int64_t EarliestFirstStartNs(std::int64_t now_ns) const noexcept override {
    return now_ns + t_arm_ns_ + first_ns_ + 2 * h_ns_;
  }

  void Reported(const PlannerRtState& rt, ReportedSegments& out) const noexcept override;

  [[nodiscard]] std::uint32_t SourceSeq(const PlannerRtState& rt,
                                        std::int64_t t_eff_ns) const noexcept override;

  // ── Tests and diagnostics ──
  [[nodiscard]] const MpcDockingSegmentPlannerParams& Params() const noexcept { return params_; }

  [[nodiscard]] int Nv() const noexcept { return nv_; }

  /// The core for `n_pre` pre-catch intervals (1..n_pre_max), or nullptr.
  [[nodiscard]] const MpcDockingSegmentCore* Core(int n_pre) const noexcept;

  /// The result the core for `n_pre` last wrote (after Configure: the
  /// warm-up's), or nullptr.
  [[nodiscard]] const MpcDockingSegmentCoreResult* LastResult(int n_pre) const noexcept;

  /// The input the core for `n_pre` was last handed (after Configure: the
  /// warm-up's), or nullptr — the start state and the catch target a solve
  /// ran from (tests).
  [[nodiscard]] const MpcDockingSegmentCoreInput* LastInputForTesting(int n_pre) const noexcept;

  /// The slowest and the summed configure-time warm-up solve [ns].
  [[nodiscard]] std::int64_t WarmUpMaxNs() const noexcept { return warmup_max_ns_; }

  [[nodiscard]] std::int64_t WarmUpTotalNs() const noexcept { return warmup_total_ns_; }

  /// Test seam: bracket every solver call (a core's Solve or Evaluate) with
  /// `hook(begin, user)` — what a C-level allocation gate has to step around,
  /// since the QP solver allocates (#654). nullptr removes it.
  using SolverHook = void (*)(bool begin, void* user) noexcept;

  void SetSolverHookForTesting(SolverHook hook, void* user) noexcept {
    solver_hook_ = hook;
    solver_user_ = user;
  }

  /// Test seam: the stage hook of every core (MpcDockingSegmentCore::SetStageHook).
  void SetCoreStageHookForTesting(MpcDockingSegmentCore::StageHook hook, void* user) noexcept;

 private:
  [[nodiscard]] bool WarmUp(const MpcDockingSegmentPlannerModel& model, std::string& why);
  // `need_command`: the RT's command must be seeded (a replan starts on it; a
  // first segment starts where the RT will seed it).
  [[nodiscard]] bool CheckState(const PlannerRtState& rt, std::int64_t start, bool need_command,
                                SegmentRecord& rec) const noexcept;
  // The instant of node k of a segment that starts at `t0_ns` with `n_pre`
  // pre-catch intervals.
  [[nodiscard]] std::int64_t NodeNs(std::int64_t t0_ns, int n_pre, int k) const noexcept;
  // The ball at nodes 0..n_pre of a segment starting at `t0_ns` (the catch
  // node with its covariance), and the stop line through the catch point.
  void SetBall(const BallPrediction& ball, std::int64_t t0_ns, int n_pre,
               MpcDockingSegmentCoreInput& in) const noexcept;
  // Whether `seg` is on this planner's grid for its own n_pre.
  [[nodiscard]] bool OnGrid(const SegmentSnapshot& seg) const noexcept;
  // Whether a search's solution started where a first segment of this report
  // starts: on the segment the RT reports for the solution's node 0 — and on
  // none when the RT follows no plan.
  [[nodiscard]] bool StartsOnTheReport(const PlannerRtState& rt,
                                       const CatchSolution& solution) const noexcept;
  // What the ring keeps when a first segment comes out publishable: the
  // followed plan's segments under a replacement, nothing otherwise.
  void NoteFirst(const PlannerRtState& rt) noexcept;
  // What the core returned, judged: kReady or the reason it is withheld.
  [[nodiscard]] SegmentOutcome Judge(const MpcDockingSegmentCoreResult& r, bool ok, bool converged,
                                     int n_pre, std::int64_t start, std::int64_t end,
                                     std::int64_t budget_ns, std::int64_t t_eff,
                                     SegmentRecord& rec) const noexcept;
  void Pack(const PlannerRtState& rt, std::uint64_t track_generation, std::uint32_t plan_id,
            std::int64_t t_c, std::int64_t t_eff, int n_pre, bool x0_clamped,
            const MpcDockingSegmentCoreResult& r, SegmentSnapshot& out) const noexcept;
  [[nodiscard]] bool RunCore(bool evaluate, MpcDockingSegmentCore& core,
                             const MpcDockingSegmentCoreInput& in,
                             MpcDockingSegmentCoreResult& res) noexcept;

  bool configured_{false};
  MpcDockingSegmentPlannerParams params_{};
  ClockFn clock_{nullptr};
  int nv_{0};
  std::array<int, kMaxPlanNv> device_of_model_{};
  std::array<double, kMaxPlanNv> qd_rating_{};
  std::int64_t dt_pre_ns_{0};
  std::int64_t dt_stop_ns_{0};
  std::int64_t first_ns_{0};
  std::int64_t replan_ns_{0};
  std::int64_t t_arm_ns_{0};
  std::int64_t h_ns_{0};
  // n_pre = j + 1 for index j. Behind pointers: a core owns a model copy and a
  // solver and is not meant to move.
  std::vector<std::unique_ptr<MpcDockingSegmentCore>> cores_;
  std::vector<MpcDockingSegmentCoreInput> inputs_;
  std::vector<MpcDockingSegmentCoreResult> results_;
  // The published segments of the followed plan and of a replacement
  // published for it. Nothing is remembered beside a segment: no solve here
  // starts after the catch, where a line would be inherited.
  SegmentRing<NoSegmentPayload> ring_{};
  std::int64_t warmup_max_ns_{0};
  std::int64_t warmup_total_ns_{0};
  SolverHook solver_hook_{nullptr};
  void* solver_user_{nullptr};
};

/// @brief A new, configured MpcDockingSegmentPlanner (non-RT), or nullptr with
///        `error` naming the cause when Configure refuses.
[[nodiscard]] std::unique_ptr<MpcDockingSegmentPlanner> MakeMpcDockingSegmentPlanner(
    const MpcDockingSegmentPlannerModel& model, const MpcDockingSegmentPlannerConstants& consts,
    const MpcDockingSegmentPlannerParams& params, MpcDockingSegmentPlanner::ClockFn clock,
    std::string* error = nullptr);

}  // namespace rtc::catching
