#pragma once

#include "rtc_tsid/solver/qp_solver_wrapper.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

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
// toward the feasible set is allowed (v_limit may be exceeded in that
// recovery direction — intentional). v_limit ≤ 0 disables the velocity bound;
// an empty q_min/q_max disables the position bound (±∞).
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
    double damping_sq{1e-4};      // μ² damping (H regularization, > 0)
    double v_limit{1.5};          // per-joint |v_ref| clamp [rad/s], ≤ 0 → off
    // Soft-priority weights (w_task ≫ w_arm,w_hand ≫ damping_sq). μ²/w_task is
    // the effective L1 damped-inverse damping — keep w_task=1 to reproduce the
    // legacy damped right-inverse at damping_sq.
    double w_task{1.0};   // L1 TCP tracking weight
    double w_arm{1e-2};   // L2 arm posture weight
    double w_hand{1e-2};  // L3 hand posture weight
    // Per-joint position limits [nv] for the position-aware velocity box
    // (lᵢ/uᵢ above). Empty → position bound disabled (velocity box only).
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    // Anti-windup clamp on the carry-forward anchor: max |q_ref_i − q_meas_i|
    // [rad] the integrated desired may lead/lag the measured state. ≤ 0 → off.
    // Only bites in carry-forward mode (reseed_anchor=false) under tracking lag;
    // on a reseed tick the gap is just v_ref·dt.
    double anchor_drift_max{0.0};
  };

  // Pre-allocates all workspaces and validates the config (indices in
  // [0, nv), no duplicates, arm/hand disjoint, damping_sq > 0). Non-RT —
  // call from on_configure; throws std::runtime_error on invalid config.
  void Init(int nv, const Config& config);

  // Gains (RT-safe setters; the controller forwards its SeqLock'd gains).
  void SetTaskGain(const Eigen::Matrix<double, 6, 1>& kx) noexcept { kx_ = kx; }

  void SetPostureGains(double ka, double kh) noexcept {
    ka_ = ka;
    kh_ = kh;
  }

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
  [[nodiscard]] bool Compute(const PinocchioCache& cache, int tcp_frame_idx, int base_frame_idx,
                             const pinocchio::SE3& placement_des,
                             const Eigen::VectorXd& q_posture_des, double dt,
                             bool reseed_anchor = true) noexcept;

  [[nodiscard]] const Eigen::VectorXd& QRef() const noexcept { return q_ref_; }

  [[nodiscard]] const Eigen::VectorXd& VRef() const noexcept { return v_ref_; }

  // √det(J_a·J_aᵀ + μ²·I) from the LDLT diagonal pivots of the last
  // Compute() — damped manipulability measure (≥ μ⁶ > 0 by construction)
  // for logging / A-B shadow diagnostics (Stage C-2 DeviceWbcLog).
  [[nodiscard]] double Manipulability() const noexcept { return manipulability_; }

  // ‖e_x‖ of the last Compute() (6D mixed: position [m] + rotation [rad]).
  [[nodiscard]] double TcpErrorNorm() const noexcept { return tcp_error_norm_; }

 private:
  int nv_{0};
  int n_arm_{0};
  int n_hand_{0};
  std::vector<int> arm_v_idx_;
  std::vector<int> hand_v_idx_;
  double damping_sq_{1e-4};
  double v_limit_{1.5};
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
