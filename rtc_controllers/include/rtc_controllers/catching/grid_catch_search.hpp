// ── The planner's search: one cycle's candidates → one plan (S6-B, L3 §4) ────
//
// The CatchSearch (catch_search.hpp) `PlannerCycle::PlanOnce` delegates to once
// the binding has given it a model — the one implementation of that interface
// today. Candidates are the vision samples themselves (L3 §5.3, thinned to
// `planner.search.grid.slice.dt`) whose lead lies in [slice.t_lead_min, slice.t_max].
//
// TWO KINDS OF GATE (decision C, D-27). JUDGEMENT gates remove a candidate —
// they are about whether the arm can be put there at all and where it would
// stop: input finiteness, IK convergence, manipulability (the IK's own
// catchability gate, D-18), and the workspace box around p_c and p_stop.
// RANK gates do not remove: uncertainty, reach time, γ window, commit lead and
// the error budget each add `score.penalty` to the candidate's score when they
// fail (decision D). Under D-27 the system attempts what it can reach and
// reports how often the attempt was doomed (S8 separates attempts from
// successes); the offline map stays strict.
//
// ORDER (L3 §4.1, with the IK budget of R-2). The cheap terms — input,
// workspace at p_c, uncertainty, lateness — are computed for every candidate
// and give a PRE-score; IK (≈2 ms each on the development PC) runs on the best
// `planner.search.grid.max_ik` of those, in pre-score order, until `planner.search.grid.budget_s`
// is spent. Rank gates, the stopping point and the full score follow each successful IK. The best
// full score wins (§4.10).
//
// SWITCHING (§4.7) AND FREEZE (decision G). When the RT is following a plan
// this search published, a better candidate replaces it only if it improves
// the score by more than `switch.delta_J` (or the current one is no longer
// feasible) and the jump limits hold. Within `freeze.T_freeze` of the current
// catch instant nothing replaces it. Holding is "publish nothing": the RT keeps
// the plan it has.
//
// γ_f (S6-B). Without the rollout (S6-C, §4.8) γ_f is the window's lower end —
// the least the arm must retreat to let the hand close in time — clipped to
// what the arm can do when the window is empty. The rollout will choose within
// the window.
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

namespace rtc::catching {

/// The arm model the search plans in (configure time, owned by the binding).
struct GridCatchSearchModel {
  /// The planner thread's own handle on the catch sub-model (R-3). No joint
  /// reorder may be installed on it (CatchPoseIk refuses one).
  rtc_urdf_bridge::RtModelHandle* handle{nullptr};
  pinocchio::FrameIndex catch_frame{0};
  int nv{0};
  /// `device_of_model[j]` = the arm device's index of model joint j. The RT
  /// speaks device order (PlannerRtState, PlanSnapshot::q_star); the model
  /// speaks its own.
  std::array<int, kMaxPlanNv> device_of_model{};
  std::array<double, kMaxPlanNv> qdot_max{};   ///< model order [rad/s], device ratings
  std::array<double, kMaxPlanNv> qddot_max{};  ///< model order [rad/s²], the D-16 box
  bool accel_box{false};                       ///< false: reach time is unjudgeable
};

/// Constants that live outside `planner.*` in the profile, resolved by the
/// binding. NaN marks a value the profile leaves TBD — every gate that needs
/// it then fails (a rank gate) rather than using a guess.
struct GridCatchSearchConstants {
  double eta_v{0.9};  ///< `planner.search.grid.gamma.eta_v` (D-9)
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
  /// (§4.5) — what the γ window and MaxCatchableSpeed use.
  double t_close_total{std::numeric_limits<double>::quiet_NaN()};
  double ball_mass{0.0};  ///< `core.ball.mass` [kg] — the impulse estimate
  /// The L4 reference (§4.8 rollout): ω, ζ, and the limits it is judged
  /// against — `planner.search.grid.reference.{omega, zeta, a_max}`, the
  /// search's own keys. NaN a_max → the rollout is unjudgeable (a rank failure).
  double ref_omega{10.0};
  double ref_zeta{1.0};
  double ref_a_max{std::numeric_limits<double>::quiet_NaN()};
  double control_dt{0.002};  ///< the rollout's confirmation step [s]
  /// The arm follows a segment planner's segments (`planner.segment.mode` other
  /// than closed_form). The η_jump bound of the switching rule (§4.7) is the
  /// step a switch puts into the L4 reference's u_des — and there is no such
  /// reference under those modes (the RT reports none), so the bound is not
  /// judged: a switch is decided by ΔJ alone.
  bool follows_segments{false};
};

/// Rank-gate failure bits (decision E: the gate bitmask the CSV carries).
enum RankGateBit : std::uint16_t {
  kRankUncertainty = 1U << 0,  ///< σ_max > κ_σ r_cap, or σ unknown (§4.4)
  kRankReach = 1U << 1,        ///< t_min exceeds the lead (§4.3)
  kRankGamma = 1U << 2,        ///< γ window empty or unjudgeable (§4.5)
  kRankCommitLead = 1U << 3,   ///< lead < T_close,lead + h/2 + T_arm + margin (§4.11)
  kRankErrorBudget = 1U << 4,  ///< n_σ σ_gap > r_cap (§4.6)
  kRankRollout = 1U << 5,      ///< no (γ_f, T_w) passes the whole-interval rollout (§4.8)
};

/// The largest step a plan switch can put into the L4 reference's u_des
/// (L3 §4.7, decision ⑥): the RT adopts a replacement with γ continuous but
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
  /// is unusable — no handle, nv outside (0, kMaxPlanNv], a wait pose that is
  /// not one entry per arm joint.
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
  /// follows; `now` is the planning 'now' on the steady axis. `arm` is NOT
  /// READ: this search scores a candidate by closed-form gates on the catch
  /// point, not by an arm motion that would have to start somewhere.
  [[nodiscard]] PlanSnapshot Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt,
                                  const ReportedSegments& arm, NowReal now,
                                  SearchStats& stats) noexcept override;

  /// Always nullptr: this search solves no arm trajectory.
  [[nodiscard]] const CatchSolution* Solution() const noexcept override { return nullptr; }

  /// monitorOnly (§4.6): σ_ℓ at the t_c of the plan the RT follows
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

  /// σ_max = √λ_max(Σ_pp) of sample k, or NaN when unknown (L3 §4.4).
  [[nodiscard]] static double SigmaMax(const CovarianceSnapshot& cov, int k) noexcept;

  /// The IK seed in force after the last `Plan` call, model order (tests).
  [[nodiscard]] const Eigen::VectorXd& IkSeedForTesting() const noexcept { return seed_; }

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
  UnitSpeedSolver unit_speed_;
  /// The unsaturated reference the rollout runs (constructed in Configure).
  std::optional<SoftCatchTranslation> rollout_ds_;
  RolloutSettings rollout_{};
  /// Recent per-candidate cost estimates [ns] — the budget check asks whether
  /// the NEXT candidate still fits, so the cycle overruns by at most the
  /// estimate's error rather than by one whole candidate (G3-C).
  std::int64_t ik_cost_ns_{0};
  std::int64_t rollout_cost_ns_{0};
  Eigen::VectorXd seed_;  // model order, sized in Configure; the seed in force this cycle
  /// The configure-time (YAML) seed, model order. `Plan` starts every cycle
  /// from it and lays the RT's adopted wait pose over it, so an activation
  /// that does not adopt (refused, or source `yaml`) never inherits the
  /// pose an earlier activation adopted.
  Eigen::VectorXd seed_yaml_;
  std::array<double, kMaxPlanNv> q_star_{};
  std::array<double, kMaxPlanNv> qdot_u_{};
  std::array<double, kMaxPlanNv> q0_{};
  std::array<double, kMaxPlanNv> w0_{};
  std::array<double, kMaxPlanNv> qdot_plan_{};
  std::array<Candidate, kCap> cands_{};
  std::array<int, kCap> order_{};

  // Settle state (§4.4).
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
  /// The §4.7 switch step bound (SwitchAccelStepBound) at its worst over the
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
