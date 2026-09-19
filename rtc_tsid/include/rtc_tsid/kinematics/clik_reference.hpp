#pragma once

#include "rtc_math/se3/axis_align.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage C-1 / WBC Kinematic QP: ClikReferenceGenerator — velocity-level
// closed-loop IK (CLIK) producing a one-step (q_ref, v_ref) low-level command
// anchored at the measured state. The Kinematic WBC position backbone of the
// 2-QP (Kinematic CLIK-QP + Dynamic TSID-ID) reorganisation.
//
// Single weighted box-constrained QP (decision variable z = v ∈ Rᴺ, N == nv;
// the reduced "wbc" model contract nq == nv == n_actuated). The original
// 3-level damped-pseudo-inverse + nullspace projection is promoted to one
// proxsuite QP whose weighted cost soft-prioritises L1 ≫ L2 ≫ L3:
//   min_v  w_task·‖J_task·v − r_task‖²        (L1: TCP tracking, arm only)
//        + w_arm ·‖S_arm·v  − v_post_arm‖²    (L2: arm posture)
//        + w_hand·‖S_hand·v − v_post_hand‖²   (L3: hand posture)
//        + μ²·‖v‖²                            (damping, absorbs J♯ damping)
//   s.t.  lᵢ ≤ vᵢ ≤ uᵢ                        (per-joint velocity ∩ position box)
// with
//   J_task ∈ R⁶ˣᴺ : rf.J's arm columns (hand columns 0; an arm-tip frame's
//                   hand columns are ~0 by kinematics, zeroed for safety),
//   r_task = Kx ⊙ e_x (6-vec SE(3) error, base-frame aligned — see below),
//   S_arm / S_hand   : index-selection of the arm / hand velocity components,
//   v_post = K·(q_des − q) on the respective index set.
// Assembled as a fixed-dim QP (n_vars = N, n_eq = 0, n_ineq = N, C = Iₙ):
//   H = w_task·J_taskᵀJ_task + diag(w_arm@arm, w_hand@hand) + μ²·I,
//   g = −(w_task·J_taskᵀr_task + scatter(w_arm·v_post_arm, w_hand·v_post_hand)).
// w_task ≫ w_arm,w_hand ≫ μ² makes L2/L3 live in L1's residual (nullspace)
// space — a soft-priority approximation of the original hard nullspace
// projection (μ²/w_task reproduces the original damped-inverse damping).
//
// Box bounds (per velocity index i, dt = control period):
//   lᵢ = max(−v_limit, (q_minᵢ − qᵢ)/dt),  uᵢ = min(+v_limit, (q_maxᵢ − qᵢ)/dt).
// When already limit-violating (qᵢ out of [q_min,q_max]) the box can invert
// (lᵢ > uᵢ); it is forced feasible by lᵢ ← min(lᵢ, uᵢ) so only motion back
// toward the feasible set is allowed, and the collapsed value is re-clamped to
// [−v_limit, +v_limit] — recovery runs at the full limit but never past it, in
// either direction (the raw collapse is bounded below q_min and unbounded
// above q_max). v_limit ≤ 0 disables the velocity bound, which also leaves the
// collapsed recovery unbounded; an empty q_min/q_max disables the position
// bound (±∞).
//
// Error convention: the SHARED helper kinematics/se3_error.hpp
// (ComputeTaskPoseError) — identical to SE3Task / ObjectSE3Task (dynamics WBC),
// the U1 unification. It is the LWA-frame BodyLog6 error
// twistLocalToWorld(log6(T⁻¹T_d)), matching the registered-frame
// LOCAL_WORLD_ALIGNED Jacobian rf.J. rtc_math's quaternion+atan2 log3 is robust
// at θ=π (no explicit clamp). The velocity law stays first-order (U1):
// r_task = Kx ⊙ e_x — no Jlog6⁻¹ on this kinematic path. No separate FK —
// the caller registers tip/base frames on the shared PinocchioCache and
// calls cache.Update() once per tick.
//
// One-step target with selectable anchoring: q_ref = q_anchor + v_ref·dt.
//   reseed_anchor = true  → q_anchor = q_meas (measured anchoring; the velocity
//     correction rides on the freshly measured state — the classic closed-loop
//     CLIK form). This is the default and the post-failure / first-call value.
//   reseed_anchor = false → q_anchor = q_ref_prev (carry-forward: the desired
//     position integrates from the PREVIOUS desired, decoupled from per-tick
//     measured noise — mirrors DemoTaskController's desired_q_ integrator).
// The caller drives reseed_anchor from "a new goal arrived / phase re-entry"
// events so the command re-anchors to measured only at those edges. Only the
// integration anchor switches; the velocity solve (J_task, e_x, posture, box)
// stays measured-based, so v_ref remains a closed-loop correction. To bound the
// open-loop drift that carry-forward can accumulate when low-level tracking
// lags, anchor_drift_max clamps |q_ref_i − q_meas_i| per joint (≤ 0 → off).
//
// Requires nq == nv (reduced revolute/prismatic tree). Compute() returns
// false on precondition violation, QP non-convergence, or non-finite result;
// the caller holds last for that tick and must not consume the outputs (the
// failure branch leaves q_ref = q_meas, v_ref = 0, and forces the next call to
// re-anchor to measured).
//
// All allocations happen in Init(); Compute() is RT-safe and noexcept.
// ────────────────────────────────────────────────
class ClikReferenceGenerator {
 public:
  struct Config {
    std::vector<int> arm_v_idx;   // arm velocity indices, Pinocchio order (≥1)
    std::vector<int> hand_v_idx;  // hand velocity indices (may be empty)
    double damping_sq{1e-4};      // μ² damping (H regularization, finite and > 0)
    double v_limit{1.5};          // per-joint |v_ref| clamp [rad/s], finite; ≤ 0 → off
    // Soft-priority weights (w_task ≫ w_arm,w_hand ≫ damping_sq). μ²/w_task is
    // the effective L1 damped-inverse damping — keep w_task=1 to reproduce the
    // legacy damped right-inverse at damping_sq.
    double w_task{1.0};   // L1 TCP tracking weight (finite and > 0)
    double w_arm{1e-2};   // L2 arm posture weight (finite and >= 0)
    double w_hand{1e-2};  // L3 hand posture weight (finite and >= 0)
    // Per-joint position limits [nv] for the position-aware velocity box
    // (lᵢ/uᵢ above). Empty → position bound disabled (velocity box only); both
    // sides must be empty or both full-nv. Every component must be FINITE and
    // ordered q_minᵢ ≤ q_maxᵢ (equality = a locked joint, allowed) — Init()
    // throws otherwise. ±inf is not an accepted "unbounded" encoding here:
    // leave the box empty instead. A caller deriving the box from a model that
    // reports unbounded joints (Pinocchio uses ±inf for continuous joints) must
    // therefore drop the box rather than forward the infinities. See Init().
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    // Anti-windup clamp on the carry-forward anchor: max |q_ref_i − q_meas_i|
    // [rad] the integrated desired may lead/lag the measured state. ≤ 0 → off.
    // Only bites in carry-forward mode (reseed_anchor=false) under tracking lag;
    // on a reseed tick the gap is just v_ref·dt.
    double anchor_drift_max{0.0};  // rad; finite; ≤ 0 → off
    // ProxQP outer-iteration cap (≥ 1). The default is the value CLIK always
    // used (QPSolverConfig), so an unset field keeps the legacy solve. Hitting
    // the cap is a failed Compute() with LastSolve().status =
    // PROXQP_MAX_ITER_REACHED (G5-C3).
    int max_iter{20};
    // Per-joint velocity limits [nv] (rad/s or m/s), each finite and > 0.
    // Empty → the scalar v_limit applies to every joint (legacy). When set it
    // replaces v_limit everywhere v_limit is used, including the re-clamp of a
    // collapsed position box.
    Eigen::VectorXd v_limit_per_joint;
    // Acceleration box [nv] (rad/s² or m/s²), each finite and > 0; empty → off.
    // Intersects the velocity ∩ position box with v_prev ± a_max·dt, v_prev =
    // the v_ref of the last successful Compute() (0 after Init / ResetAnchor).
    // When the intersection is empty the acceleration window wins: the joint
    // gets l = u = clamp(v*, v_prev − a·dt, v_prev + a·dt), v* the point of the
    // velocity ∩ position interval nearest v_prev, and LastSolve() raises
    // bound_conflict with the joint's bit in conflict_mask (L5 §4.3). Requires
    // nv ≤ 64 (the mask width). The same a_max must feed the planner's
    // reach-time check so the plan matches what CLIK executes (D-16).
    Eigen::VectorXd a_max;
    // Smoothing weight w_s ≥ 0 (finite): adds (w_s/2)·‖v − v_prev‖² to the
    // cost, pulling the command toward the previous one (L5 §4.3). 0 → off.
    // Keep w_s ≪ w_task like the posture weights, or it competes with tracking.
    double w_smooth{0.0};
    // Command-value evaluation (D-6, L5 §4.2). The caller passes a cache
    // updated at the COMMAND state q_c (its previous QRef() on the arm), not
    // at the measured state, so error, Jacobian and box are evaluated along
    // the commanded path and servo lag stays out of the loop. Then:
    //   - the anchor is cache.q every tick (reseed_anchor is ignored);
    //   - after the first successful call, cache.q on the arm indices must
    //     equal the previous QRef() (|Δ| ≤ 1e-12) — otherwise the call fails
    //     with LastSolve().command_mismatch (a wiring error: a measured q
    //     would silently turn this back into measured-state CLIK). Hand
    //     indices are not checked: the hand is commanded elsewhere (L6);
    //   - anchor_drift_max must be ≤ 0 (Init throws): there is no measured q
    //     to bound against; tracking is supervised outside (L7 TRACK_ERR).
    bool evaluate_at_command{false};
    // Weight of the two approach-axis rows of the position + axis Compute()
    // overload (L5 §4.3 w_a; finite and > 0). Unused by the SE3 overload.
    double w_axis{0.5};
  };

  // Pre-allocates all workspaces and validates the config (indices in
  // [0, nv), no duplicates, arm/hand disjoint, damping_sq > 0, and a finite
  // ordered position box — NUM-7: the size checks alone are not a finiteness
  // gate, because Compute()'s std::max/std::min box assembly launders a
  // non-finite bound into the velocity limit and leaves the outputs finite).
  // Non-RT — call from on_configure; throws std::runtime_error on invalid
  // config.
  void Init(int nv, const Config& config);

  // Gains (RT-safe setters; the controller forwards its SeqLock'd gains).
  void SetTaskGain(const Eigen::Matrix<double, 6, 1>& kx) noexcept { kx_ = kx; }

  void SetPostureGains(double ka, double kh) noexcept {
    ka_ = ka;
    kh_ = kh;
  }

  /// Approach-axis gain K_a [1/s] of the position + axis overload (the
  /// position gain is SetTaskGain()'s linear part).
  void SetAxisGain(double ka) noexcept { k_axis_ = ka; }

  /// Target of the position + approach-axis task (dynamic_catching L5 §4.2),
  /// all in the base frame of the call. Roll about the axis is left free —
  /// the arm posture term resolves it.
  struct PositionAxisTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};             ///< frame origin [m]
    Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};                ///< desired frame +z, unit
    Eigen::Vector3d linear_velocity_ff{Eigen::Vector3d::Zero()};   ///< [m/s]
    Eigen::Vector3d angular_velocity_ff{Eigen::Vector3d::Zero()};  ///< [rad/s]
  };

  // One CLIK step anchored at the measured (cache.q, cache.v) of this tick.
  //   tcp_frame_idx / base_frame_idx : indices into cache.registered_frames
  //     (base_frame_idx < 0 → universe fast-path, like SE3Task).
  //   placement_des : desired TCP pose **in the base frame**
  //     (SetSe3Reference contract of SE3Task).
  //   q_posture_des : full [nq] desired posture; arm/hand entries are
  //     selected through the Config index sets.
  // Returns false on dimension mismatch / invalid frame index / dt ≤ 0 /
  // non-finite result — outputs must not be consumed in that case (the
  // finite-guard branch leaves q_ref = q_meas, v_ref = 0 as a safe value;
  // precondition branches leave stale values).
  // reseed_anchor: true = anchor q_ref at the measured state this tick (default,
  // closed-loop); false = carry forward from the previous q_ref. The first call
  // after Init (and the call after any failure) always re-anchors to measured
  // regardless, so the integrator never starts from an undefined anchor.
  // twist_ff (optional, D-5): desired TCP twist [linear; angular] in the same
  // axes as the pose error (base-frame aligned), added as
  // r_task = Kx ⊙ e_x + twist_ff. nullptr → the legacy law. A non-finite
  // twist_ff fails the call before the solve.
  [[nodiscard]] bool Compute(const PinocchioCache& cache, int tcp_frame_idx, int base_frame_idx,
                             const pinocchio::SE3& placement_des,
                             const Eigen::VectorXd& q_posture_des, double dt,
                             bool reseed_anchor = true,
                             const Eigen::Matrix<double, 6, 1>* twist_ff = nullptr) noexcept;

  // Position (3 rows) + approach axis (2 rows) step for a frame whose LOCAL
  // +z must point along target.axis, e.g. a catch frame (L5 §4.2):
  //   J_p = rf.J rows 0–2 (LOCAL_WORLD_ALIGNED),  r_p = Kx[0:3] ⊙ e_p + v_ff
  //   J_a = S·R_WCᵀ·rf.J rows 3–5,                r_a = S·R_WCᵀ·(K_a·e_a + ω_ff)
  // with S = [I₂ 0] (LOCAL x, y), e_a = rtc::math::se3::AxisAlignError(z_C, a)
  // and everything expressed in world-aligned axes: base_frame_idx only moves
  // the target into the world (unlike the SE3 overload, whose error stays in
  // base-aligned axes). Cost w_task‖J_p v − r_p‖² + w_axis‖J_a v − r_a‖² plus
  // the same posture, damping, smoothing and boxes as the SE3 overload.
  // Fails before the solve on a non-finite target or an axis that is not
  // unit (AxisAlignError invalid); LastAxisRegion() reports the branch.
  [[nodiscard]] bool Compute(const PinocchioCache& cache, int frame_idx, int base_frame_idx,
                             const PositionAxisTarget& target, const Eigen::VectorXd& q_posture_des,
                             double dt, bool reseed_anchor = true) noexcept;

  /// Axis-alignment branch of the last position + axis Compute().
  [[nodiscard]] rtc::math::se3::AxisAlignRegion LastAxisRegion() const noexcept {
    return axis_region_;
  }

  /// ‖e_p‖ [m] and θ = ‖e_a‖ [rad] of the last position + axis Compute().
  [[nodiscard]] double PositionErrorNorm() const noexcept { return position_error_norm_; }

  [[nodiscard]] double AxisErrorAngle() const noexcept { return axis_error_angle_; }

  [[nodiscard]] const Eigen::VectorXd& QRef() const noexcept { return q_ref_; }

  [[nodiscard]] const Eigen::VectorXd& VRef() const noexcept { return v_ref_; }

  // √det(J_a·J_aᵀ + μ²·I) from the LDLT diagonal pivots of the last
  // Compute() — damped manipulability measure (≥ μ⁶ > 0 by construction)
  // for logging / A-B shadow diagnostics (Stage C-2 DeviceWbcLog).
  [[nodiscard]] double Manipulability() const noexcept { return manipulability_; }

  // ‖e_x‖ of the last Compute() (6D mixed: position [m] + rotation [rad]).
  [[nodiscard]] double TcpErrorNorm() const noexcept { return tcp_error_norm_; }

  /// Solver outcome of the last Compute() (dynamic_catching S2.2b, L5 §5.1).
  /// Diagnostic only — it never changes the outputs. Filled on every call;
  /// a call rejected by its preconditions leaves reached_solve = false.
  struct SolveDiagnostics {
    bool reached_solve{false};     ///< preconditions held and the QP was solved
    bool converged{false};         ///< QP converged with finite iterates
    bool non_finite{false};        ///< QP iterates or q_ref / v_ref were non-finite
    int status{-1};                ///< proxsuite::proxqp::QPSolverOutput (0 = SOLVED), −1 = none
    int iterations{0};             ///< ProxQP outer iterations
    double solve_time_us{0.0};     ///< wall time of the solve [µs]
    bool command_mismatch{false};  ///< evaluate_at_command: cache.q (arm) ≠ previous QRef()
    bool bound_conflict{false};    ///< acceleration box overrode velocity ∩ position
    std::uint64_t conflict_mask{0};  ///< bit i = velocity index i conflicted
  };

  [[nodiscard]] const SolveDiagnostics& LastSolve() const noexcept { return last_solve_; }

  /// Forget the integration anchor and the previous velocity: the next
  /// Compute() re-anchors q_ref to cache.q and the acceleration box starts
  /// from v_prev = 0. Call at activation, re-arm and E-STOP release (L5 §4.2)
  /// — leaving v_prev from the previous run would make the first tick's
  /// acceleration box relative to a stale velocity. RT-safe.
  void ResetAnchor() noexcept {
    anchor_initialized_ = false;
    v_prev_.setZero();
  }

 private:
  // Shared stages of Compute(). Each keeps the floating-point accumulation
  // order of the original single-function Compute(), which the golden-vector
  // regression (test/test_clik_golden.cpp) pins bit-for-bit.
  [[nodiscard]] bool PreconditionsHold(const PinocchioCache& cache, int tcp_frame_idx,
                                       int base_frame_idx, const Eigen::VectorXd& q_posture_des,
                                       double dt) const noexcept;
  // evaluate_at_command: cache.q on the arm equals the previous QRef().
  [[nodiscard]] bool CommandStateMatches(const PinocchioCache& cache) const noexcept;
  void ComputePostureReferences(const PinocchioCache& cache,
                                const Eigen::VectorXd& q_posture_des) noexcept;
  // H/g must already hold the task terms.
  void AddPostureAndDamping() noexcept;
  void AssembleBox(const Eigen::VectorXd& q, double dt) noexcept;
  [[nodiscard]] bool SolveAndIntegrate(const PinocchioCache& cache, double dt,
                                       bool reseed_anchor) noexcept;

  int nv_{0};
  int n_arm_{0};
  int n_hand_{0};
  std::vector<int> arm_v_idx_;
  std::vector<int> hand_v_idx_;
  double damping_sq_{1e-4};
  double v_limit_{1.5};
  Eigen::VectorXd v_limit_per_joint_;  // [nv] or empty (scalar v_limit_)
  Eigen::VectorXd a_max_;              // [nv] or empty (acceleration box off)
  Eigen::VectorXd v_prev_;             // [nv] v_ref of the last successful Compute()
  double w_smooth_{0.0};               // smoothing weight, 0 → off
  bool evaluate_at_command_{false};    // cache.q is the command state q_c
  double w_axis_{0.5};                 // approach-axis rows weight
  double k_axis_{0.0};                 // approach-axis gain K_a
  rtc::math::se3::AxisAlignRegion axis_region_{rtc::math::se3::AxisAlignRegion::kInvalidInput};
  double position_error_norm_{0.0};
  double axis_error_angle_{0.0};
  Eigen::MatrixXd j_pos_;   // [3 × nv] position rows, arm columns
  Eigen::MatrixXd j_axis_;  // [2 × nv] approach-axis rows, arm columns
  double w_task_{1.0};
  double w_arm_{1e-2};
  double w_hand_{1e-2};
  Eigen::VectorXd q_min_;         // [nv] or empty (position box disabled)
  Eigen::VectorXd q_max_;         // [nv] or empty
  double anchor_drift_max_{0.0};  // carry-forward anti-windup clamp [rad], ≤0 → off
  // False until the first successful Compute(); forces a measured re-anchor on
  // the first call (and after any failure) so q_ref never integrates from a
  // stale/zero anchor.
  bool anchor_initialized_{false};

  // Gains (L1 task / L2 arm posture / L3 hand posture)
  Eigen::Matrix<double, 6, 1> kx_{Eigen::Matrix<double, 6, 1>::Zero()};
  double ka_{0.0};
  double kh_{0.0};

  // Last-Compute diagnostics
  SolveDiagnostics last_solve_;
  double manipulability_{0.0};
  double tcp_error_norm_{0.0};

  // Outputs
  Eigen::VectorXd q_ref_;  // [nv] (nq == nv contract)
  Eigen::VectorXd v_ref_;  // [nv]

  // QP (decision variable v ∈ Rᴺ, N == nv; n_eq = 0, n_ineq = N box).
  QPSolverWrapper qp_solver_;
  QPData qp_data_;

  // Pre-allocated workspace
  Eigen::MatrixXd j_task_;              // [6 × nv] arm columns of rf.J (hand cols 0)
  Eigen::VectorXd v_post_arm_;          // [n_arm] L2 arm posture velocity
  Eigen::VectorXd v_post_hand_;         // [n_hand] L3 hand posture velocity
  Eigen::Matrix<double, 6, 1> e_x_;     // SE(3) error
  Eigen::Matrix<double, 6, 1> r_task_;  // Kx ⊙ e_x (L1 task-velocity reference)
  // Manipulability-only 6×6 (diag continuity; J♯ no longer used for the solve).
  Eigen::MatrixXd j_arm_;           // [6 × n_arm] gathered arm columns
  Eigen::Matrix<double, 6, 6> m6_;  // J_a·J_aᵀ + μ²·I
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt6_;
};

}  // namespace rtc::tsid
