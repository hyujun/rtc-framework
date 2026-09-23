// ── The planner's search: one cycle's candidates → one plan (S6-B, L3 §4) ────
//
// What `PlannerCycle::PlanOnce` delegates to once the binding has given it a
// model. Candidates are the vision samples themselves (L3 §5.3, thinned to
// `planner.slice.dt`) whose lead lies in [slice.t_lead_min, slice.t_max].
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
// `planner.max_ik` of those, in pre-score order, until `planner.budget_s` is
// spent. Rank gates, the stopping point and the full score follow each
// successful IK. The best full score wins (§4.10).
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
#include "rtc_controllers/catching/gamma_rollout.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/rank_gates.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/catching/unit_speed.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>

namespace rtc::catching {

/// The arm model the search plans in (configure time, owned by the binding).
struct PlannerModel {
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
struct PlannerConstants {
  double eta_v{0.9};                                       ///< `planner.gamma.eta_v` (D-9)
  double v_max{std::numeric_limits<double>::quiet_NaN()};  ///< `reference.v_max`
  double a_dec{std::numeric_limits<double>::quiet_NaN()};  ///< `supervisor.decel.a_dec`
  double t_arm_s{0.0};                                     ///< `joint_cmd.lag.T_arm`
  /// `robot.hand.T_close_e2e` and T_close,tot = that + h/2 (§4.5).
  double t_close_e2e{std::numeric_limits<double>::quiet_NaN()};
  double t_close_total{std::numeric_limits<double>::quiet_NaN()};
  double ball_mass{0.0};  ///< `core.ball.mass` [kg] — the impulse estimate
  /// The L4 reference (§4.8 rollout): ω, ζ, and the limits it is judged
  /// against. NaN a_max → the rollout is unjudgeable (a rank failure).
  double ref_omega{10.0};
  double ref_zeta{1.0};
  double ref_a_max{std::numeric_limits<double>::quiet_NaN()};
  double control_dt{0.002};  ///< the rollout's confirmation step [s]
};

/// Rank-gate failure bits (decision E: the gate bitmask the CSV carries).
enum RankGateBit : std::uint16_t {
  kRankUncertainty = 1U << 0,  ///< σ_max > κ_σ r_cap, or σ unknown (§4.4)
  kRankReach = 1U << 1,        ///< t_min exceeds the lead (§4.3)
  kRankGamma = 1U << 2,        ///< γ window empty or unjudgeable (§4.5)
  kRankCommitLead = 1U << 3,   ///< lead < T_close,tot + T_arm + margin (§4.11)
  kRankErrorBudget = 1U << 4,  ///< n_σ σ_gap > r_cap (§4.6)
  kRankRollout = 1U << 5,      ///< no (γ_f, T_w) passes the whole-interval rollout (§4.8)
};

/// Why a JUDGEMENT gate removed a candidate.
enum class JudgeReject : std::uint8_t {
  kNone = 0,
  kInput,           ///< non-finite p/v, or ‖v̂‖ below v_eps (NUM-7)
  kIk,              ///< IK did not converge / was refused
  kManipulability,  ///< the IK's catchability gate (D-18)
  kWorkspace,       ///< p_c or p_stop outside `workspace.catch_box`
  kNotEvaluated,    ///< outside the IK budget (pre-filter or budget_s)
};
inline constexpr std::size_t kJudgeRejectCount = 6;

/// What the switching rule decided about the plan the RT is following.
enum class SwitchDecision : std::uint8_t {
  kNoCurrent = 0,    ///< the RT follows no plan of ours — publish the best
  kReplaced,         ///< a better candidate replaced it (§4.7)
  kHeldHysteresis,   ///< not better by delta_J
  kHeldJump,         ///< better, but the jump limits refuse the switch
  kHeldFreeze,       ///< within T_freeze of the current t_c (decision G)
  kHeldNoCandidate,  ///< no candidate passed; the RT keeps what it has
};

[[nodiscard]] constexpr const char* SwitchDecisionName(SwitchDecision d) noexcept {
  switch (d) {
    case SwitchDecision::kNoCurrent:
      return "no_current";
    case SwitchDecision::kReplaced:
      return "replaced";
    case SwitchDecision::kHeldHysteresis:
      return "held_hysteresis";
    case SwitchDecision::kHeldJump:
      return "held_jump";
    case SwitchDecision::kHeldFreeze:
      return "held_freeze";
    case SwitchDecision::kHeldNoCandidate:
      return "held_no_candidate";
  }
  return "unknown";
}

/// One cycle's search diagnostics (L3 §8) — the planner CSV's body.
struct SearchStats {
  bool settling{false};          ///< inside n_settle after a track change
  std::uint16_t n_in_window{0};  ///< candidates whose lead is in the slice window
  std::uint16_t n_ik{0};         ///< IK solves run
  std::uint16_t n_pass{0};       ///< candidates past every judgement gate
  std::array<std::uint16_t, kJudgeRejectCount> judge_rejects{};
  bool budget_hit{false};  ///< budget_s ran out with IK candidates left
  std::int64_t search_ns{0};
  std::int64_t ik_ns_max{0};       ///< the slowest single IK this cycle
  std::int64_t rollout_ns_max{0};  ///< the slowest single candidate's rollout choice
  std::uint16_t n_rollouts{0};     ///< rollouts run this cycle (coarse + fine)
  /// The chosen candidate (valid only if a plan was produced).
  std::uint16_t chosen_rank_mask{0};
  double chosen_score{0.0};
  double chosen_lead_s{0.0};
  double chosen_gamma_f{0.0};
  double chosen_t_w{0.0};                  ///< the γ window the rollout chose [s]
  bool chosen_rollout_window_only{false};  ///< γ_f chosen on window peaks (approach saturates)
  SwitchDecision decision{SwitchDecision::kNoCurrent};
  /// Whether the caller should publish the returned snapshot. False when the
  /// switching rule holds the RT's current plan.
  bool publish{true};
  /// monitorOnly (§4.6): σ_ℓ at the committed t_c from the newest covariance.
  double sigma_l{std::numeric_limits<double>::quiet_NaN()};
};

class PlannerSearch {
 public:
  using ClockFn = std::int64_t (*)() noexcept;

  /// Non-RT. Size every buffer. False (and unconfigured) if the model binding
  /// is unusable — no handle, nv outside (0, kMaxPlanNv], a wait pose that is
  /// not one entry per arm joint.
  bool Configure(const PlannerModel& model, const PlannerConstants& constants,
                 const PlannerParams& params, const CatchPoseIkOptions& ik, ClockFn clock);

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  /// One search. RT-safe. `rt` gives the current command and the plan the RT
  /// follows; `now` is the planning 'now' on the steady axis.
  [[nodiscard]] PlanSnapshot Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt, NowReal now,
                                  SearchStats& stats) noexcept;

  /// monitorOnly (§4.6): σ_ℓ at the current plan's t_c. RT-safe.
  void Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov, bool cov_matched,
               SearchStats& stats) const noexcept;

  /// The cycle published `plan` (after its provenance re-check): remember it
  /// as the plan the switching rule compares against.
  void NotePublished(const PlanSnapshot& plan) noexcept;

  /// A trial reset (the RT's reset epoch moved): forget the current plan and
  /// the settle count.
  void ResetTrial() noexcept;

  /// σ_max = √λ_max(Σ_pp) of sample k, or NaN when unknown (L3 §4.4).
  [[nodiscard]] static double SigmaMax(const CovarianceSnapshot& cov, int k) noexcept;

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
  PlannerModel model_{};
  PlannerConstants constants_{};
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
  Eigen::VectorXd seed_;  // model order, sized in Configure
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

  // The plan the RT is following, as this search published it.
  struct Current {
    bool valid{false};
    std::uint32_t plan_id{0};
    std::int64_t t_c_ns{0};
    std::array<double, 3> p_c{};
  } current_{};
};

}  // namespace rtc::catching
