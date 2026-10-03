// ── Decel planner: the APPROACH–stop segments, on the planner thread ─────────
// (dynamic_catching MPC plan E1-F03 #629 · E1-F08 #661; decisions MD-24 –
// MD-33, MD-55 – MD-64, MD-70)
//
// What PlannerCycle runs once a decel MPC is configured: solve the joint-node
// segment the RT follows from APPROACH to the end of the stop (decel_mpc.hpp)
// and hand back a DecelPlanSnapshot for the cycle to publish. ROS-free, so
// the whole decision is testable against plain values; the cycle owns the
// SeqLock, the re-check and the counter.
//
// ── The grid and the stop end (MD-10, MD-31, MD-54) ───────────────────────────
// Nodes sit on t_c − j·Δ_pre (pre-catch, j ≤ n_pre_max) and t_c + k·Δ_s
// (stop), t_c the catch instant of the plan the segment belongs to. The stop
// ends at t_c + N_s·Δ_s wherever a segment starts: a segment that starts at
// the stop grid point k solves N_s − k nodes with its own DecelMpc, built at
// configure time (the core's N is fixed at Init, and re-solving N_s nodes
// from a later start would push the end out on every replan). Post-catch
// replans happen only for k ≤ k_max.
//
// ── The two solves (MD-55 – MD-64) ────────────────────────────────────────────
//  • PlanFirst — the search's wake, for the plan it is about to publish. The
//    arm rests at its wait pose (max |q̇_cmd| ≤ rest_tol, else kNotAtRest) —
//    before a plan the RT reports the measured pose with no command behind it
//    (cmd_seeded false), which is where it seeds the command on adoption;
//    n_pre = min(n_pre_max, ⌊(t_c − now_lead − first − 2h)/Δ_pre⌋) ≥ 1
//    (else kTooLate). x₀ = (q_cmd, 0, 0); the reference is a per-joint
//    minimum-jerk reach to q_star (clamped to the core's box, slowed to
//    0.9·η_v·q̇_max), held from the catch node on; p̂_b, v̂_b, a_d are the
//    plan's, Σ_p the snapshot's. Cold start, w_Δ scale 0, no retry: a catch
//    core cannot be solved without a reference.
//  • Replan — the grid point earliest = now_lead + replan + 2h reaches: the
//    largest pre-catch count that fits, else the catch node, else the first
//    stop grid point ≥ earliest (k ≤ k_max, else kPastReplanWindow). Its
//    source is a segment the RT REPORTS (SourceSeq) — x₀ is that segment at
//    t_eff projected into the core's box, the reference that segment at the
//    new grid's node instants (a column subset: the grid is anchored at t_c
//    and the end is shared). The same pre-catch grid point is re-solved with
//    the newer prediction (`replan.same_point`); a stop grid point at most
//    once (kUpToDate). A stop core whose reference is refused (trust region,
//    rest) is re-solved once without it.
//  • Publishable only when every check holds, each written as a positive
//    comparison so a NaN fails: solved, within its budget (measured from the
//    start of THIS solve, not from the wake), before t_eff on the lead axis,
//    slack (MD-33), the catch-node position error (catch cores), the velocity
//    extrema between nodes (≤ q̇_max), node N at rest to the core's reference
//    tolerance (a published segment is the next solve's reference), and the
//    packed payload passing ValidateDecelNodes. The relative-velocity slack
//    s_v (`catch.rho_v` > 0) is NOT among them: it is recorded
//    (DecelRecord::slack_v) and never judged — no threshold is defined for
//    it, and with γ_ref < 1 it is above 0 by construction.
//
// There is no stop-only planner (MD-70): a plan is published only with a
// first segment that starts before t_c, so Configure refuses n_pre_max < 1.
//
// ── The stop-path line (`cost.w_perp` > 0; #698) ──────────────────────────────
// With w_⊥ > 0 the cores penalise, on the nodes from the catch on, the catch
// frame's distance from a line (DecelMpcInput::p_c, d_hat). The planner fills
// that line on every solve; none runs on the input's default (origin, x).
//  • The line is the BALL's: through its predicted position at t_c along its
//    direction of travel there, d̂ = v̂_b/‖v̂_b‖ — the very vectors the solve
//    hands the core as p_b and v_b (PlanFirst: the plan's p_c and v_c; a
//    pre-catch Replan: the ball target's). It is the line the search reserves
//    its stopping point on.
//  • A stop core has no ball: it takes the line REMEMBERED for the plan — the
//    one the plan's last PUBLISHED catch-core segment was solved with
//    (NotePublished) — also when the caller passes a valid ball target: the
//    stop line does not move after the catch. The retry without a reference
//    solves on the same line.
//  • The configure warm-ups solve on a synthetic line: through the catch
//    frame at the mid pose along the synthetic ball's travel (that frame's
//    −z), the same one for the stop cores and the catch cores.
//  • Fail closed: a ball slower than `planner.ik.v_eps` has no direction
//    (kNoBall), a speed that is not finite is kInputNonFinite, and a stop
//    grid point of a plan with no remembered line is kNoBall — each withheld
//    BEFORE the solve.
// With w_⊥ = 0 none of this runs: no line is built, remembered or required,
// and no solve is withheld for one.
//
// ── Contracts ─────────────────────────────────────────────────────────────────
//  • Configure() is non-RT: it Inits the k_max + 1 stop cores and the
//    n_pre_max catch cores (armature ZERO — MD-25: armature is not used in
//    control), sizes every buffer, and solves each core once (a warm-up:
//    ProxQP's first solve is the slowest); a warm-up that fails fails the
//    configure.
//  • PlanFirst() / Replan() are noexcept, log nothing, throw nothing, and
//    allocate nothing outside ProxQP (MD-22 / MD-23 — ProxQP's own mallocs
//    are #654).
//  • Joint order: the RT speaks DEVICE order (PlannerRtState, the payload);
//    the cores speak the model's pinocchio velocity order. The mapping is
//    `device_of_model`, as for the search.
#pragma once

#include "rtc_controllers/catching/decel_mpc.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"  // CovarianceSnapshot
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace rtc::catching {

/// Largest age of the RT's report (steady now − rt_state_ns) a solve starts
/// from. The RT stores every tick, so anything older means the
/// tick stalled; 50 ms is one default planner wake timeout, 25 ticks at 2 ms.
inline constexpr std::int64_t kDecelMaxRtStateAgeNs = 50'000'000;

/// What one decel step did (the planner events CSV's decel columns). The CSV
/// writes the NAME (DecelOutcomeName), never the value: the values carry no
/// meaning outside a build and move when an enumerator is added or removed.
enum class DecelOutcome : std::uint8_t {
  kOff = 0,  ///< not attempted: not configured, or not a decel mode
  kNoState,  ///< no followed plan / t_c, unseeded command, or a size mismatch
  /// The RT's report is older than kDecelMaxRtStateAgeNs (or from the
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
  kNotAtRest,    ///< first solve: max |q̇_cmd| above approach.rest_tol
  kTooLate,      ///< first solve: not even one pre-catch interval fits before t_c
  kNotFollowed,  ///< replan: no segment of ours the RT reports pending or following
  kNoBall,       ///< a pre-catch grid point without a usable ball prediction at t_c
  kCatchError,   ///< catch-node position error not finite or over catch_pos_err_max
  kSpeed,        ///< a between-node velocity extremum over q̇_max
  // With `cost.w_perp` > 0 two of the above also say that the stop-path line
  // could not be built (header note): kNoBall — the ball is slower than v_eps
  // at t_c (first solve or pre-catch replan), or a stop grid point of a plan
  // no line is remembered for; kInputNonFinite — the ball's speed overflows.
};

[[nodiscard]] const char* DecelOutcomeName(DecelOutcome o) noexcept;

/// Which solve a record describes (E1-F08).
enum class DecelKind : std::uint8_t {
  kNone = 0,  ///< no solve was chosen (the record's default)
  kFirst,     ///< the first segment of a plan, solved with the search (MD-56)
  kSame,      ///< a pre-catch grid point the source already starts at (MD-58)
  kAdvance,   ///< a later pre-catch grid point
  kStop,      ///< a stop core: the catch node or a post-catch grid point
};

[[nodiscard]] const char* DecelKindName(DecelKind k) noexcept;

struct DecelRecord {
  DecelOutcome outcome{DecelOutcome::kOff};
  DecelMpcReason core_reason{DecelMpcReason::kNone};
  /// Grid index of node 0: t_eff = t_c + k·Δ_s for a stop grid point, −n_pre
  /// for a pre-catch one (the CSV's decel_k).
  std::int32_t k{-1};
  std::int32_t n_nodes{0};
  std::uint32_t decel_seq{0};  ///< the published segment's seq (cycle)
  bool x0_clamped{false};      ///< the start state (q or q̇) was projected into the box
  bool from_segment{false};    ///< replan: x₀ came from a segment the RT reports
  bool presolved{false};       ///< no reference: kinematic pre-solve + solve
  bool cold_retry{false};      ///< a stop core's reference was refused, re-solved without it
  std::int32_t iterations{0};
  std::int32_t qp_status{-1};
  std::int64_t solve_ns{0};    ///< solve start → solve end (the budget's measure)
  std::int64_t publish_ns{0};  ///< the stored segment's stamp (cycle); 0 when not stored
  double slack_max{std::numeric_limits<double>::quiet_NaN()};
  double slack_terminal_max{std::numeric_limits<double>::quiet_NaN()};
  double tau_ratio_max{std::numeric_limits<double>::quiet_NaN()};

  DecelKind kind{DecelKind::kNone};
  bool cold_start{false};      ///< the core's main QP started from zero
  bool solver_retried{false};  ///< the core's own warm → cold re-run (cold_retried)
  /// First solve: the reference's target was outside the core's position
  /// box (clamped), or its minimum-jerk speed over 0.9·η_v·q̇_max (scaled).
  bool ref_clamped{false};
  bool ref_scaled{false};
  double ref_scale{std::numeric_limits<double>::quiet_NaN()};      ///< smallest per-joint factor
  double ref_shortfall{std::numeric_limits<double>::quiet_NaN()};  ///< largest |d| cut [rad]
  double x0_speed{std::numeric_limits<double>::quiet_NaN()};       ///< first solve: max |q̇_cmd|
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
  std::uint32_t source_seq{0};  ///< replan: the segment x₀ and the reference came from
};

/// The arm the decel planner plans for (configure time, from the binding).
/// Every array is MODEL order, `nv` entries.
struct DecelPlannerModel {
  std::shared_ptr<const pinocchio::Model> arm;  ///< hand-locked catch sub-model
  pinocchio::FrameIndex catch_frame{0};
  int nv{0};
  std::array<int, kMaxPlanNv> device_of_model{};
  std::array<double, kMaxPlanNv> q_min{};     ///< [rad]
  std::array<double, kMaxPlanNv> q_max{};     ///< [rad]
  std::array<double, kMaxPlanNv> qdot_max{};  ///< [rad/s], device ratings
  std::array<double, kMaxPlanNv> tau_max{};   ///< [N·m], device ratings
};

struct DecelPlannerConstants {
  double eta_v{0.9};         ///< `planner.gamma.eta_v` (the core's velocity row)
  double t_arm_s{0.0};       ///< `joint_cmd.lag.T_arm` — real → lead axis
  double control_dt{0.002};  ///< the RT period [s]
  /// `planner.ik.v_eps` [m/s] — the ball speed below which its direction of
  /// travel (and so a_d) is undefined (MakeDecelBallTarget).
  double v_eps{1e-6};
};

/// The ball at a plan's catch instant as the solves take it
/// (MD-63), MODEL world. `valid` covers p_b / v_b / a_d; `sigma_valid` the
/// position covariance, which only weights the position term.
struct DecelBallTarget {
  bool valid{false};
  Eigen::Vector3d p_b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a_d{Eigen::Vector3d::UnitZ()};  ///< −v_b/‖v_b‖
  Eigen::Matrix3d sigma_p{Eigen::Matrix3d::Zero()};
  bool sigma_valid{false};
};

/// @brief Whether a node trajectory's joint velocity stays within q̇_max
///        BETWEEN its nodes too (RT-safe, pure; MD-62).
///
/// The QP bounds q̇ at the nodes only (by η_v·q̇_max). Between nodes k and
/// k+1 q̇ is quadratic with an interior extremum only where q̈ changes sign,
/// at τ* = Δ_k·q̈_k/(q̈_k − q̈_{k+1}), of value q̇_k + ½·q̈_k·τ* — no division
/// by the jerk. Intervals before node `n_pre` are `dt_pre` long, the rest
/// `dt`. Every node velocity and acceleration must be finite, and every node
/// and extremum velocity within the RATING q̇_max (not η_v·q̇_max).
/// @param qd,qdd n × (N+1) nodes, model order
/// @param qd_max n ratings [rad/s], > 0
/// @param[out] ratio_max max |q̇|/q̇_max over nodes and extrema; +inf when a
///             value is not finite or the shapes do not match
/// @return false on any value outside, non-finite, or a shape mismatch.
[[nodiscard]] bool DecelBetweenNodeSpeedOk(const Eigen::Ref<const Eigen::MatrixXd>& qd,
                                           const Eigen::Ref<const Eigen::MatrixXd>& qdd, int n_pre,
                                           double dt_pre, double dt, std::span<const double> qd_max,
                                           double& ratio_max) noexcept;

/// @brief The ball at `t_c_ns` from one trajectory snapshot and its
///        covariance (RT-safe, pure).
///
/// Position and velocity are SampleAt's: valid only inside the horizon (not
/// extrapolated) and with ‖v‖ > v_eps. Σ_p is the position block of the
/// covariance bracketing t_c by SampleAt's own integer rule
/// (t_i ≤ t_c < t_{i+1}; the last sample is itself): at t_c == t_i exactly
/// sample i's block alone — a zero weight times a NaN neighbour would still be
/// NaN — otherwise the blocks of i and i + 1 interpolated linearly on integer
/// ns. Both must be finite and the covariance must belong to the same snapshot
/// (`cov_matched`); otherwise `sigma_valid` is false.
[[nodiscard]] DecelBallTarget MakeDecelBallTarget(const TrajectorySnapshot& traj,
                                                  const CovarianceSnapshot& cov, bool cov_matched,
                                                  std::int64_t t_c_ns, double v_eps) noexcept;

class DecelPlanner {
 public:
  using ClockFn = std::int64_t (*)() noexcept;

  DecelPlanner() = default;

  /// @brief Build the k_max + 1 stop cores and the n_pre_max catch cores and
  ///        warm each up (non-RT). On failure — `params.n_pre_max` < 1
  ///        included (MD-70) — the planner is unconfigured and `error` (if
  ///        given) names the cause.
  [[nodiscard]] bool Configure(const DecelPlannerModel& model, const DecelPlannerConstants& consts,
                               const DecelPlannerParams& params, ClockFn clock,
                               std::string* error = nullptr);

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  /// Replace the clock (tests pin it; the cycle forwards its own).
  void SetClock(ClockFn clock) noexcept {
    if (clock != nullptr) {
      clock_ = clock;
    }
  }

  /// Drop the per-trial state (a trial reset, a new followed plan) — the
  /// published segments and the remembered stop-path line with them.
  void ResetTrial() noexcept;

  [[nodiscard]] const DecelPlannerParams& Params() const noexcept { return params_; }

  [[nodiscard]] int Nv() const noexcept { return nv_; }

  /// The stop core that starts at the stop grid point k, 0..k_max (tests and
  /// diagnostics).
  [[nodiscard]] const DecelMpc& Core(int k) const noexcept {
    return *cores_[static_cast<std::size_t>(k)];
  }

  /// @brief The first segment of a plan the search just produced (RT-safe;
  ///        MD-56). The arm rests at the reported command; the reference is a
  ///        minimum-jerk reach to the plan's q_star over the pre-catch part.
  ///        A reported pose outside the core's position box (inside the m_q
  ///        margin of a limit) is projected into it, as Replan does, and
  ///        marked `x0_clamped` — the core refuses a start outside its box,
  ///        and a plan is published only with its segment.
  /// @param plan the search's plan, its `plan_id` already the one the cycle
  ///        will publish it under
  /// @param ball the ball at plan.t_c_ns (Σ_p only — p, v, a_d are the plan's)
  /// @return true when `out` holds a publishable segment (kReady); its
  ///         publish_ns and decel_seq are the caller's.
  [[nodiscard]] bool PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                               const DecelBallTarget& ball, DecelPlanSnapshot& out,
                               DecelRecord& rec) noexcept;

  /// @brief A later segment of the plan the RT follows (RT-safe; MD-58): the
  ///        grid point the replan budget reaches, solved from the segment the
  ///        RT reports pending or following (SourceSeq).
  /// @param ball the ball at rt.plan_t_c_ns; read at pre-catch grid points
  ///        only (a stop core's stop-path line is the plan's remembered one)
  [[nodiscard]] bool Replan(const PlannerRtState& rt, const DecelBallTarget& ball,
                            DecelPlanSnapshot& out, DecelRecord& rec) noexcept;

  /// The cycle published `p` (its decel_seq filled): a later replan may be
  /// solved from it. With `cost.w_perp` > 0, a catch-core segment (n_pre > 0)
  /// that is the one this planner last solved also makes its stop-path line
  /// the plan's remembered one.
  void NotePublished(const DecelPlanSnapshot& p) noexcept;

  /// The track generation of the plan `rt` follows, from the segments
  /// published for it (the first one carries the plan's own token). False when
  /// nothing was published for that plan. After the freeze the RT keeps the
  /// committed track while `rt.track_generation` moves on to whatever it last
  /// consumed, so that field does not say whose ball the plan catches.
  [[nodiscard]] bool FollowedTrack(const PlannerRtState& rt,
                                   std::uint64_t& generation) const noexcept;

  /// Whether a segment whose node 0 is at `t0_ns` can still be read by the RT
  /// when it is published at `publish_ns`: publish + T_arm + 2 ticks < t0.
  /// The one statement of that lead — the solve's own late check and the
  /// cycle's re-checks at the publish stamp all use it.
  [[nodiscard]] bool StartsInTime(std::int64_t publish_ns, std::int64_t t0_ns) const noexcept {
    return publish_ns + t_arm_ns_ + 2 * h_ns_ < t0_ns;
  }

  /// The control period [ns].
  [[nodiscard]] std::int64_t ControlDtNs() const noexcept { return h_ns_; }

  /// The decel_seq of the segment a replan at `t_eff_ns` starts from, 0 when
  /// there is none (kNotFollowed): the one the RT reports pending (when it
  /// starts no later than t_eff), else the one it reports following — never
  /// inferred.
  [[nodiscard]] std::uint32_t SourceSeq(const PlannerRtState& rt,
                                        std::int64_t t_eff_ns) const noexcept;

  /// The catch core for `n_pre` pre-catch intervals (1..n_pre_max), and the
  /// parameters each core was built with (tests and diagnostics).
  [[nodiscard]] const DecelMpc& ApproachCore(int n_pre) const noexcept {
    return *catch_cores_[static_cast<std::size_t>(n_pre - 1)];
  }

  [[nodiscard]] const DecelMpcParams& ApproachCoreParams(int n_pre) const noexcept {
    return catch_params_[static_cast<std::size_t>(n_pre - 1)];
  }

  [[nodiscard]] const DecelMpcParams& StopCoreParams(int k) const noexcept {
    return stop_params_[static_cast<std::size_t>(k)];
  }

  /// The input the catch core for `n_pre` (1..n_pre_max) / the stop core k
  /// (0..k_max) was last handed — after Configure, the warm-up's (tests and
  /// diagnostics: what line a solve ran on).
  [[nodiscard]] const DecelMpcInput& ApproachCoreInput(int n_pre) const noexcept {
    return catch_inputs_[static_cast<std::size_t>(n_pre - 1)];
  }

  [[nodiscard]] const DecelMpcInput& StopCoreInput(int k) const noexcept {
    return inputs_[static_cast<std::size_t>(k)];
  }

  [[nodiscard]] const DecelPlannerConstants& Constants() const noexcept { return consts_; }

  /// The rest tolerance Judge holds a published segment's node N to — the
  /// profile's `linearization.reference_rest_tol` (tests and diagnostics).
  [[nodiscard]] double ReferenceRestTol() const noexcept { return rest_tol_ref_; }

  /// The slowest and the summed configure-time warm-up solve [ns] (MD-64):
  /// each is a core's FIRST solve, the one a trial would otherwise pay for.
  [[nodiscard]] std::int64_t WarmUpMaxNs() const noexcept { return warmup_max_ns_; }

  [[nodiscard]] std::int64_t WarmUpTotalNs() const noexcept { return warmup_total_ns_; }

 private:
  bool configured_{false};
  DecelPlannerParams params_{};
  ClockFn clock_{nullptr};
  int nv_{0};
  std::array<int, kMaxPlanNv> device_of_model_{};
  std::array<double, kMaxPlanNv> v_box_{};  // η_v · q̇_max, model order
  std::int64_t dt_ns_{0};
  std::int64_t t_arm_ns_{0};

  // The stop cores, k = 0..k_max. Behind pointers: a core owns a model copy
  // and a solver and is not meant to move.
  std::vector<std::unique_ptr<DecelMpc>> cores_;
  std::vector<DecelMpcInput> inputs_;  // sized per instance
  std::vector<DecelMpcResult> results_;

  // Where the solve's state comes from and how it is judged and packed.
  [[nodiscard]] bool ConfigureApproach(const DecelPlannerModel& model, std::string& why);
  [[nodiscard]] bool WarmUp(const DecelPlannerModel& model, std::string& why);
  [[nodiscard]] bool CheckState(const PlannerRtState& rt, std::int64_t start, bool need_command,
                                DecelRecord& rec) const noexcept;
  [[nodiscard]] bool SetCatchInputs(const Eigen::Vector3d& p_b, const Eigen::Vector3d& v_b,
                                    const Eigen::Vector3d& a_d, const DecelBallTarget& ball,
                                    bool first, DecelMpcInput& in, DecelRecord& rec) const noexcept;
  // The stop-path line of a catch-core solve (w_⊥ > 0 only), from the ball as
  // `in` already carries it: p_c = in.p_b, d̂ = in.v_b/‖in.v_b‖. False — the
  // outcome recorded, nothing written — when no direction can be built.
  [[nodiscard]] bool SetStopLine(DecelMpcInput& in, DecelRecord& rec) const noexcept;
  [[nodiscard]] DecelOutcome Judge(const DecelMpcResult& r, bool ok, int n_pre, int n_total,
                                   bool catch_core, std::int64_t start, std::int64_t end,
                                   std::int64_t budget_ns, std::int64_t t_eff,
                                   DecelRecord& rec) const noexcept;
  void PackSegment(const PlannerRtState& rt, std::uint64_t track_generation, std::uint32_t plan_id,
                   std::int64_t t_c, std::int64_t t_eff, int n_pre, int k0, int n_total,
                   const DecelMpcResult& r, DecelPlanSnapshot& out) const noexcept;
  [[nodiscard]] const DecelPlanSnapshot* FindInRing(std::uint32_t seq) const noexcept;
  [[nodiscard]] bool ColdStartFor(bool catch_core, int index, std::int64_t t_eff,
                                  std::uint32_t plan_id, std::int64_t t_c) const noexcept;
  // `ok` false forgets the last solve: a call the core refused before its QP
  // left that core's solver holding ANOTHER problem's iterates, and a failed
  // QP reset it — either way the next solve of that point is a cold one.
  void NoteSolve(bool ok, bool catch_core, int index, std::int64_t t_eff, std::uint32_t plan_id,
                 std::int64_t t_c) noexcept;

  DecelPlannerConstants consts_{};
  // n_pre = j + 1 for index j. Same box as the stop cores (η_v, η_τ, m_q):
  // at t_c the stop core takes over a state the catch core left inside it.
  std::vector<std::unique_ptr<DecelMpc>> catch_cores_;
  std::vector<DecelMpcInput> catch_inputs_;
  std::vector<DecelMpcResult> catch_results_;
  std::vector<DecelMpcParams> catch_params_;
  std::vector<DecelMpcParams> stop_params_;
  // Model order: the rating q̇_max, and the core's position box (read from
  // the core: the margin is min(m_q, half the range)).
  std::array<double, kMaxPlanNv> qd_max_{};
  std::array<double, kMaxPlanNv> q_lo_{};
  std::array<double, kMaxPlanNv> q_hi_{};
  std::int64_t dt_pre_ns_{0};
  std::int64_t first_ns_{0};
  std::int64_t replan_ns_{0};
  std::int64_t h_ns_{0};               // control_dt
  double rest_tol_ref_{1e-4};          // the profile's linearization.reference_rest_tol
  Eigen::VectorXd jerk_weight_model_;  // cost.jerk_weight in model order; empty = all 1
  std::int64_t warmup_max_ns_{0};
  std::int64_t warmup_total_ns_{0};

  // Published segments of the plan in `ring_plan_id_` / `ring_t_c_ns_`, oldest
  // first. A segment the RT last reported pending or following is never the
  // one evicted (MD-58): a burst of same-point re-solves cannot push it out.
  static constexpr int kRingSize = 8;
  std::array<DecelPlanSnapshot, kRingSize> ring_{};
  int ring_n_{0};
  std::uint32_t ring_plan_id_{0};
  std::int64_t ring_t_c_ns_{0};
  std::uint32_t reported_pending_seq_{0};
  std::uint32_t reported_active_seq_{0};

  // The last solve, for the cold-start rule: a new plan, another core or
  // another grid point is a new problem.
  bool last_solve_valid_{false};
  bool last_solve_catch_{false};
  int last_solve_index_{0};
  std::int64_t last_solve_t_eff_{0};
  std::uint32_t last_solve_plan_id_{0};
  std::int64_t last_solve_t_c_{0};

  // The stop-path line (`cost.w_perp` > 0; nothing below is read otherwise).
  // Fixed-size, so remembering one allocates nothing. A plan is the pair
  // (plan_id, t_c), as for the ring.
  struct StopLine {
    bool valid{false};
    std::uint32_t plan_id{0};
    std::int64_t t_c_ns{0};
    std::int64_t t0_ns{0};  // the segment's node 0 (solved_line_ only)
    Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};
    Eigen::Vector3d d_hat{Eigen::Vector3d::UnitX()};
  };

  bool perp_on_{false};
  // The line of the catch-core solve that last came out publishable, with
  // the segment it belongs to: NotePublished recognises that segment by it.
  StopLine solved_line_{};
  // The line of the plan's last PUBLISHED catch-core segment — what its stop
  // cores solve on.
  StopLine plan_line_{};
};

}  // namespace rtc::catching
