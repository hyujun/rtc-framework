// ── The planner's search: one cycle's candidates → one plan ──────────────────
//
// The CatchSearch (catch_search.hpp) `PlannerCycle::PlanOnce` delegates to once
// the binding has given it a model — the one implementation of that interface
// today. A 1-D search over the vision sample times with one IK per candidate,
// not a joint (q*, t_c) optimisation.
//
// CANDIDATES. The vision samples themselves (no interpolation, so no covariance
// needs interpolating), every n_s-th, whose lead lies in the window:
//   n_s = max(1, round(Δ_slice / Δ_v))         (1 if Δ_v ≤ 0 or non-finite)
//   T_lead,min ≤ ℓ_k = t_k − now ≤ T_lead,max
// Δ_v [s] sample spacing, Δ_slice = `planner.search.grid.slice.dt`, T_lead,min =
// `slice.t_lead_min` (unset: `freeze.T_freeze`; neither: no candidate),
// T_lead,max = `slice.t_max`. ℓ_k [s] is read against the real now. The first
// `n_settle` snapshots of a new track epoch plan nothing (kUncertainty).
//
// TWO KINDS OF GATE. JUDGEMENT gates remove a candidate —
// they are about whether the arm can be put there at all: input finiteness
// (and ‖v‖ ≥ `ik.v_eps`), the arm's reach bound (reach_bound.hpp — a necessary
// condition read off the model, tested before the IK so that the IK is not
// spent on a point no pose reaches), IK convergence and manipulability (the
// IK's own catchability gate). No gate judges WHERE the catch point is: a
// point the IK reaches inside the joint limits is a candidate wherever it
// lies, and the reach bound removes none of those.
// RANK gates do not remove: uncertainty, reach time, γ window, commit lead,
// the error budget and the rollout each add w_pen = `score.penalty` to the
// candidate's score when they fail (RankGateBit). The system attempts what it
// can reach and reports how often the attempt was doomed; the offline map
// stays strict.
//
// ORDER AND BUDGET. The cheap terms — input, reach bound, uncertainty,
// lateness — are computed for every candidate and give a PRE-score
//   J_pre,k = w_σ σ_k/r_cap + w_late (T_lead,max − ℓ_k) + w_pen·[¬(σ_k ≤ κ_σ r_cap)]
// σ_k [m] = SigmaMax at sample k (NaN unknown — its term is dropped and the
// bracket counts it as a failure), r_cap [m] capture radius, κ_σ =
// `kappa_sigma`. IK (≈2 ms each on the development PC) runs on the best
// `planner.search.grid.max_ik` in pre-score order — the followed plan's
// candidate (|t_k − t_c,cur| ≤ n_s Δ_v / 2) first, so the switching rule
// always gets its verdict. Before every candidate but the first the search
// stops when elapsed + ĉ_IK + ĉ_roll > T_bud (T_bud = min(`budget_s`, the
// caller's cap)), ĉ ← max(c_measured, ĉ − ĉ/8) the decaying-maximum cost of
// one IK / one rollout. Rank gates and the full score follow each successful IK:
//   J_k = w_σ σ_k/r_cap + w_t max_i t_min,i / (ℓ_k − T_arm) + w_q ‖q* − q_n‖²
//         + w_late (T_lead,max − ℓ_k) − w_γ γ_f + w_pen · popcount(M_k)
// the σ term only for a known σ_k, the t term only for a usable reach result
// (TMinChecked) and ℓ_k > T_arm; q* the IK solution, q_n the IK seed (the wait
// pose), M_k the failed RankGateBit set; weights are `score.w_*`. The lowest
// J_k wins (a tie keeps the one evaluated first).
//
// WHERE THE REACH STARTS. The reach-time gate asks whether the arm gets
// from its state at now_lead to the candidate's q* in the lead. Before a plan
// that state is the command the RT reports. While the RT follows a segment
// planner's segments it is the segment the RT will be on at now_lead — the
// pending one from its own node 0 on, else the followed one (SourceSegmentAt)
// — evaluated at now_lead by the evaluator the RT samples with. Only that
// gate's start moves: the reference the rollout runs starts where it did, and
// the switching rule is unchanged.
//
// SWITCHING AND FREEZE. When the RT is following a plan this search
// published, nothing is published (the RT keeps the plan it has) unless:
//   t_c,cur − now > T_freeze,  a candidate passed,  and either
//   better ⇔ (current infeasible ∨ J_cur − J_best > Δ_J)  and the jump bound
//            holds → replace, or
//   ¬better, the best IS the current candidate and ‖Δp_c‖ > ε_term and the
//            jump bound holds → refresh (the same candidate, moved prediction).
// J_cur is the passed candidate within n_s Δ_v / 2 of t_c,cur (none: infeasible),
// Δ_J = `switch.delta_J`, Δp_c = p_c,best − p_c,cur. The jump bound is
// SwitchAccelStepBound ≤ η_jump·a_max at its worst over `switch.samples`
// instants of [now_lead, now_lead + `budget_s` + 2h] (h the control period);
// it is not judged while the arm follows segments.
//
// γ_f AND T_w come from the γ rollout (ChooseGamma) over [min(γ_min, γ_max),
// γ_max] — an empty window is searched at the arm's limit γ_max. A window
// that cannot be judged skips the rollout: γ_f = 0, T_w = the longest ramp.
//
// RT-1~10: every buffer is sized in Configure; `Plan` allocates nothing,
// throws nothing, logs nothing.
#pragma once

#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/gamma_rollout.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/rank_gates.hpp"
#include "rtc_controllers/catching/reach_bound.hpp"
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/catching/unit_speed.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>

namespace rtc::catching {

/// The arm model the search plans in (configure time, owned by the binding).
struct GridCatchSearchModel {
  /// The planner thread's own handle on the catch sub-model. No joint
  /// reorder may be installed on it (CatchPoseIk refuses one).
  rtc_urdf_bridge::RtModelHandle* handle{nullptr};
  pinocchio::FrameIndex catch_frame{0};
  int nv{0};
  /// Joint count of the arm DEVICE the RT reports (PlannerRtState::nv, the
  /// width of a reported segment and of `planner.wait_pose`). 0 = `nv`: the
  /// model is the whole device. Larger when the model is a chain inside a
  /// wider device — the search then reads and writes its own joints' entries
  /// only, and nothing it decides depends on the others.
  int n_dev{0};
  /// `device_of_model[j]` = the arm device's index of model joint j: an
  /// injection into [0, n_dev). The RT speaks device order (PlannerRtState,
  /// PlanSnapshot::q_star); the model speaks its own.
  std::array<int, kMaxPlanNv> device_of_model{};
  std::array<double, kMaxPlanNv> qdot_max{};   ///< model order [rad/s], device ratings
  std::array<double, kMaxPlanNv> qddot_max{};  ///< model order [rad/s²], the acceleration box
  bool accel_box{false};                       ///< false: reach time is unjudgeable
};

/// Constants that live outside `planner.*` in the profile, resolved by the
/// binding. NaN marks a value the profile leaves TBD — every gate that needs
/// it then fails (a rank gate) rather than using a guess.
struct GridCatchSearchConstants {
  double eta_v{0.9};  ///< `planner.search.grid.gamma.eta_v`
  double v_max{
      std::numeric_limits<double>::quiet_NaN()};  ///< `planner.search.grid.reference.v_max`
  double a_dec{std::numeric_limits<double>::quiet_NaN()};  ///< `planner.search.grid.stop.a_dec`
  double t_arm_s{0.0};                                     ///< `joint_cmd.lag.T_arm`
  /// `robot.hand.T_close_lead` [s] — what the hand sequencer subtracts from the
  /// catch instant. It fills `plan.t_cmd_ns` and, with h/2, T_arm and the
  /// margin, the commit lead (kRankCommitLead). NaN: the close instant is the
  /// catch instant and the commit gate fails.
  double t_close_lead{std::numeric_limits<double>::quiet_NaN()};
  /// The MEASURED closure time, T_close,tot = `robot.hand.T_close_e2e` + h/2
  /// — what the γ window and MaxCatchableSpeed use.
  double t_close_total{std::numeric_limits<double>::quiet_NaN()};
  double ball_mass{0.0};  ///< `core.ball.mass` [kg] — the impulse estimate
  /// The soft-catch reference the rollout runs (SoftCatchTranslation): ω, ζ,
  /// and the limits it is judged against —
  /// `planner.search.grid.reference.{omega, zeta, a_max}`, the search's own
  /// keys. NaN a_max → the rollout is unjudgeable (a rank failure).
  double ref_omega{10.0};
  double ref_zeta{1.0};
  double ref_a_max{std::numeric_limits<double>::quiet_NaN()};
  double control_dt{0.002};  ///< the rollout's confirmation step [s]
  /// The arm follows a segment planner's segments (`planner.segment.mode` other
  /// than closed_form). The η_jump bound of the switching rule is the step a
  /// switch puts into the soft-catch reference's u_des — and there is no such
  /// reference under those modes (the RT reports none), so the bound is not
  /// judged: a switch is decided by ΔJ alone.
  bool follows_segments{false};
};

/// Rank-gate failure bits (the gate bitmask the CSV carries).
enum RankGateBit : std::uint16_t {
  kRankUncertainty = 1U << 0,  ///< σ_max > κ_σ r_cap, or σ unknown
  kRankReach = 1U << 1,        ///< ReachTimeFeasible fails
  kRankGamma = 1U << 2,        ///< γ window empty or unjudgeable
  kRankCommitLead = 1U << 3,   ///< lead < T_close,lead + h/2 + T_arm + margin
  kRankErrorBudget = 1U << 4,  ///< ¬(n_σ σ_gap ≤ r_cap), CatchErrorSigma
  kRankRollout = 1U << 5,      ///< no (γ_f, T_w) passes the whole-interval rollout
};

/// The largest step a plan switch can put into the soft-catch reference's
/// acceleration u_des [m/s²]: the RT adopts a replacement with γ continuous but
/// its quintic ramp restarted (γ̇ = γ̈ = 0, controller.cpp's adoption), so
///
///   Δu = ω²(1−γ) Δp_c − (2ζωγ̇ + γ̈)(o − p_c) − 2γ̇ v_o
///
/// with γ, γ̇, γ̈ the reference's before the switch, p_c the OLD catch point and
/// (o, v_o) the ball target at the switch. Returned is the triangle bound
///
///   ω²|1−γ| ‖Δp_c‖ + (2ζω|γ̇| + |γ̈|) ‖o − p_c‖ + 2|γ̇| ‖v_o‖,
///
/// which the rule holds to η_jump·a_max. The ramp terms carry ‖o − p_c‖, not
/// ‖Δp_c‖: mid-ramp they are nonzero even for a switch that moves nothing.
/// A NaN argument gives NaN, which fails every `<=` budget test.
[[nodiscard]] inline double SwitchAccelStepBound(double omega, double zeta, double gamma,
                                                 double gamma_d, double gamma_dd, double dp_norm,
                                                 double xo_norm, double v_o_norm) noexcept {
  const double ramp = 2.0 * zeta * omega * std::fabs(gamma_d) + std::fabs(gamma_dd);
  return omega * omega * std::fabs(1.0 - gamma) * dp_norm + ramp * xo_norm +
         2.0 * std::fabs(gamma_d) * v_o_norm;
}

class GridCatchSearch final : public CatchSearch {
 public:
  /// Non-RT. Size every buffer. False (and unconfigured) if the model binding
  /// is unusable — no handle, nv outside (0, kMaxPlanNv] or wider than the
  /// catch-pose IK can be sized for (rtc::kMaxRobotDOF), a device narrower than
  /// the model or wider than kMaxPlanNv, a `device_of_model` that is not an
  /// injection into the device, a wait pose that is not one entry per DEVICE
  /// joint.
  bool Configure(const GridCatchSearchModel& model, const GridCatchSearchConstants& constants,
                 const PlannerParams& params, const CatchPoseIkOptions& ik, ClockFn clock);

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  /// Replace the steady clock the search measures its budget on (non-RT; the
  /// planner thread is not running). A null `clock` is ignored.
  void SetClock(ClockFn clock) noexcept override {
    if (clock != nullptr) {
      clock_ = clock;
    }
  }

  /// One search. RT-safe. `rt` gives the current command and the plan the RT
  /// follows; `now` is the planning 'now' on the steady axis. `arm` gives the
  /// start of the reach-time gate while the RT follows a segment (header
  /// note); with no segment in it, or one that cannot be read at now_lead (its
  /// joint count is not the model's, the instant is before its node 0, a value
  /// is not finite), the start is the reported command. `budget_cap_ns` > 0
  /// bounds the IK budget from above: the candidates are evaluated until
  /// min(`planner.search.grid.budget_s`, the cap) is spent — the first one
  /// always is.
  [[nodiscard]] PlanSnapshot Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt,
                                  const ReportedSegments& arm, NowReal now,
                                  std::int64_t budget_cap_ns, SearchStats& stats) noexcept override;

  /// Always nullptr: this search solves no arm trajectory.
  [[nodiscard]] const CatchSolution* Solution() const noexcept override { return nullptr; }

  /// Record only: σ_ℓ at the t_c of the plan the RT follows
  /// (`rt.plan_id`). RT-safe.
  void Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov, bool cov_matched,
               const PlannerRtState& rt, SearchStats& stats) const noexcept override;

  /// The cycle published `plan` (after its provenance re-check): remember it.
  /// It becomes "the current plan" only once the RT reports following it
  /// (`rt.plan_id`) — a publish the RT refused (freeze, age, a newer one)
  /// must not stand in for what the arm is actually doing.
  void NotePublished(const PlanSnapshot& plan) noexcept override;

  /// A trial reset (the RT's reset epoch moved): forget the current plan and
  /// the settle count.
  void ResetTrial() noexcept override;

  /// σ_max = √λ_max(Σ_pp) [m] of sample k (Σ_pp its symmetrised position
  /// block), or NaN when unknown.
  [[nodiscard]] static double SigmaMax(const CovarianceSnapshot& cov, int k) noexcept;

  /// The catch frame's reach bound, computed in Configure from the model. A
  /// candidate whose catch point is farther than its radius plus the IK's
  /// position tolerance from its centre is refused before the IK.
  [[nodiscard]] const ReachBound& Reach() const noexcept { return reach_; }

  /// The joints this search plans, as device indices: every joint of the
  /// device, or — on a chain of it — the chain's (`PlanSnapshot::joint_set` of
  /// the plans it publishes). 0 before Configure.
  [[nodiscard]] JointSet Joints() const noexcept { return joint_set_; }

  /// The IK seed in force after the last `Plan` call, model order (tests).
  [[nodiscard]] const Eigen::VectorXd& IkSeedForTesting() const noexcept { return seed_; }

  /// The joint position and velocity the last `Plan` started every candidate's
  /// reach from, model order, `nv` entries each (tests). A `Plan` that ended
  /// before it judged a candidate leaves an earlier call's.
  [[nodiscard]] std::span<const double> ReachStartPositionForTesting() const noexcept {
    return {q0_.data(), static_cast<std::size_t>(model_.nv)};
  }

  [[nodiscard]] std::span<const double> ReachStartVelocityForTesting() const noexcept {
    return {w0_.data(), static_cast<std::size_t>(model_.nv)};
  }

 private:
  struct Candidate {
    int k{0};
    double lead_s{0.0};
    double sigma{0.0};
    bool sigma_known{false};
    double pre_score{0.0};
    JudgeReject reject{JudgeReject::kNotEvaluated};
    bool passed{false};
    double score{0.0};
  };

  bool configured_{false};
  GridCatchSearchModel model_{};
  GridCatchSearchConstants constants_{};
  PlannerParams params_{};
  CatchPoseIkOptions ik_options_{};
  ClockFn clock_{nullptr};

  CatchPoseIk ik_;
  ReachBound reach_{};
  UnitSpeedSolver unit_speed_;
  /// The unsaturated reference the rollout runs (constructed in Configure).
  std::optional<SoftCatchTranslation> rollout_ds_;
  RolloutSettings rollout_{};
  /// Recent per-candidate cost estimates [ns] — the budget check asks whether
  /// the NEXT candidate still fits, so the cycle overruns by at most the
  /// estimate's error rather than by one whole candidate.
  std::int64_t ik_cost_ns_{0};
  std::int64_t rollout_cost_ns_{0};
  Eigen::VectorXd seed_;  // model order, sized in Configure; the seed in force this cycle
  /// The configure-time (YAML) seed, model order. `Plan` starts every cycle
  /// from it and lays the RT's adopted wait pose over it, so an activation
  /// that does not adopt (refused, or source `yaml`) never inherits the
  /// pose an earlier activation adopted.
  Eigen::VectorXd seed_yaml_;
  int n_dev_{0};           // the device's joint count (model_.n_dev resolved)
  JointSet joint_set_{0};  // the model's joints, as device indices
  std::array<double, kMaxPlanNv> q_star_{};
  std::array<double, kMaxPlanNv> qdot_u_{};
  std::array<double, kMaxPlanNv> q0_{};
  std::array<double, kMaxPlanNv> w0_{};
  std::array<double, kMaxPlanNv> qdot_plan_{};
  std::array<Candidate, kCap> cands_{};
  std::array<int, kCap> order_{};

  // Settle state (the first `n_settle` snapshots of a track epoch).
  bool track_known_{false};
  std::uint64_t track_generation_{0};
  std::uint64_t last_sequence_{0};
  bool sequence_seen_{false};
  int settle_seen_{0};

  // The plan the RT is following, as this search published it: resolved per
  // call from the RT's plan id against the recent publishes.
  struct Current {
    bool valid{false};
    std::uint32_t plan_id{0};
    std::int64_t t_c_ns{0};
    std::array<double, 3> p_c{};
  };

  [[nodiscard]] Current Followed(const PlannerRtState& rt) const noexcept;
  /// The switch step bound (SwitchAccelStepBound) at its worst over the
  /// instants the RT may adopt this cycle's publish, on the ramp it runs.
  [[nodiscard]] double SwitchStep(const TrajectorySnapshot& traj, const PlannerRtState& rt,
                                  NowLead now_lead, double dp) const noexcept;
  static constexpr std::size_t kPublishedRing = 8;
  std::array<Current, kPublishedRing> published_{};
  std::size_t published_next_{0};
  Current current_{};
};

/// @brief A new, configured GridCatchSearch (non-RT).
///
/// The one place the configure path builds this search, so the integration
/// package and the tests build it the same way; the result is handed to
/// PlannerCycle::InstallSearch.
/// @return nullptr when Configure refuses the binding.
[[nodiscard]] std::unique_ptr<GridCatchSearch> MakeGridCatchSearch(
    const GridCatchSearchModel& model, const GridCatchSearchConstants& constants,
    const PlannerParams& params, const CatchPoseIkOptions& ik, GridCatchSearch::ClockFn clock);

}  // namespace rtc::catching
