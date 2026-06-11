#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage C-1: ClikReferenceGenerator — velocity-level closed-loop IK (CLIK)
// producing a one-step (q_ref, v_ref) low-level command anchored at the
// measured state, as an alternative reference source to acceleration-level
// integration of the TSID solution (command_source switch, Stage C-2).
//
// 3-level structure (robot-agnostic — joint index sets injected via Config):
//   L1 — TCP tracking, arm joints only:
//          v_arm = J_a♯ · (Kx ⊙ e_x)
//        with the damped right-inverse J_a♯ = J_aᵀ·(J_a·J_aᵀ + μ²·I₆)⁻¹
//        evaluated by in-place LDLT on the 6×6 normal matrix (GraspCache
//        pattern — no SVD on the RT path).
//   L2 — arm posture in the L1 nullspace, evaluated without forming the
//        projector:  v_p = Ka·(q_a_des − q_a),
//          v_arm = v_p + J_a♯·(Kx ⊙ e_x − J_a·v_p)
//        (algebraically identical to J_a♯·v_task + (I − J_a♯·J_a)·v_p).
//   L3 — hand posture, no projection: v_hand = Kh·(q_h_des − q_h).
//
// Error convention is identical to SE3Task (kinematics/se3_error.hpp):
// PinocchioCache registered-frame LOCAL_WORLD_ALIGNED Jacobian, base-frame
// position difference + separated log3 rotation error with θ→π clamp.
// One velocity-law-specific alignment on top: the log3 rotation error is a
// tip(body)-frame vector, while the LWA Jacobian's angular rows are
// world(≈base)-aligned — so e_rot is rotated into the base frame before
// feedback (e_base = R_tip_in_base·e_body = log3(R_des·R_curᵀ); norm
// unchanged → same scale as SE3Task for A/B comparison). No separate FK —
// the caller registers tip/base frames on the shared PinocchioCache and
// calls cache.Update() once per tick.
//
// One-step target with measured anchoring:
//   v_ref[idx] = clamp(v, ±v_limit) on arm/hand index sets (others 0),
//   q_ref = q_meas + v_ref·dt.
// Requires nq == nv (reduced revolute/prismatic tree — the DemoWbc reduced
// "wbc" model contract). Compute() returns false on precondition violation
// or non-finite intermediate result; the caller falls back to the
// integrator path for that tick and must not consume the outputs.
//
// All allocations happen in Init(); Compute() is RT-safe and noexcept.
// ────────────────────────────────────────────────
class ClikReferenceGenerator {
 public:
  struct Config {
    std::vector<int> arm_v_idx;   // arm velocity indices, Pinocchio order (≥1)
    std::vector<int> hand_v_idx;  // hand velocity indices (may be empty)
    double damping_sq{1e-4};      // μ² of the damped right-inverse (> 0)
    double v_limit{1.5};          // per-joint |v_ref| clamp [rad/s], ≤ 0 → off
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
  [[nodiscard]] bool Compute(const PinocchioCache& cache, int tcp_frame_idx, int base_frame_idx,
                             const pinocchio::SE3& placement_des,
                             const Eigen::VectorXd& q_posture_des, double dt) noexcept;

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

  // Pre-allocated workspace
  Eigen::MatrixXd j_arm_;       // [6 × n_arm] gathered arm columns of rf.J
  Eigen::VectorXd v_post_arm_;  // [n_arm] L2 posture velocity v_p
  Eigen::VectorXd v_arm_;       // [n_arm]
  Eigen::Matrix<double, 6, 6> m6_;    // J_a·J_aᵀ + μ²·I
  Eigen::Matrix<double, 6, 1> e_x_;   // SE(3) error
  Eigen::Matrix<double, 6, 1> rhs6_;  // Kx⊙e − J_a·v_p, then M⁻¹·(·) in-place
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt6_;
};

}  // namespace rtc::tsid
