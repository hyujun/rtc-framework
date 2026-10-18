#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Acceleration-level joint limit constraint — ONE-TICK reachability of the
// position and velocity limits. (Not the multi-step viability bound of
// Del Prete 2018: this box only asks that the next sample respect q / v
// limits, which is why a joint can still be parked inside the margin band.)
//
// Position limit → acceleration bound  (q⁺ = q + v·dt + ½·a·dt²):
//   a_i ≥ 2·(q_min_i - q_i - v_i·dt) / dt²
//   a_i ≤ 2·(q_max_i - q_i - v_i·dt) / dt²
// Velocity limit → acceleration bound  (v⁺ = v + a·dt):
//   a_i ≥ (v_min_i - v_i) / dt
//   a_i ≤ (v_max_i - v_i) / dt
// Row i: l_i = max(pos_lb, vel_lb), u_i = min(pos_ub, vel_ub); C = [I_nv | 0].
//
// Inside the position-margin band the two cross (l_i > u_i). The row is then
// collapsed onto the velocity bound it crossed (l = u = vel_lb at the upper
// limit, vel_ub at the lower): the box stays non-empty and the QP is asked to
// reverse the joint toward the range at its velocity limit within one tick
// (#551). That acceleration, |v_lim − v| / dt, is a violent command: the QP
// stays solvable with hard torque limits (the other joints redistribute the
// coupled torque — test_tsid_wqp pins this), but the collapse is a safety net,
// not a substitute for keeping joints out of the band. InvertedCount() says
// how many rows needed it this tick. Config: dt, position_margin, velocity_margin.
// Keep position_margin SMALLER than the margin of whatever generates the
// position reference — a joint parked at that margin otherwise sits in this
// band on every tick.
//
// Floating-base: the first 6 DoF are unconstrained (l = -∞, u = +∞).
// ────────────────────────────────────────────────
class JointLimitConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "joint_limit"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int EqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int IneqDim(const ContactState& contacts) const noexcept override;

  void ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                       const RobotModelInfo& robot_info, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> A_block,
                       Eigen::Ref<Eigen::VectorXd> b_block) noexcept override;

  void ComputeInequality(const PinocchioCache& cache, const ContactState& contacts,
                         const RobotModelInfo& robot_info, int n_vars,
                         Eigen::Ref<Eigen::MatrixXd> C_block, Eigen::Ref<Eigen::VectorXd> l_block,
                         Eigen::Ref<Eigen::VectorXd> u_block) noexcept override;

  // Rows of the last ComputeInequality whose assembled box was empty
  // (l_i > u_i) before any repair. Rows with non-finite q_i / v_i are not
  // counted. Diagnostic only — read by the controller's wbc_diag log (#551).
  [[nodiscard]] int InvertedCount() const noexcept { return inverted_count_; }

 private:
  int nv_{0};
  int inverted_count_{0};
  bool floating_base_{false};

  double dt_{0.002};         // 제어 주기 (s)
  double margin_pos_{0.05};  // position limit margin (rad)
  double margin_vel_{0.1};   // velocity limit margin (rad/s)

  Eigen::VectorXd q_min_;        // [nv] (margin 적용 후)
  Eigen::VectorXd q_max_;        // [nv] (margin 적용 후)
  Eigen::VectorXd v_min_;        // [nv] (margin 적용 후, = -v_max + margin)
  Eigen::VectorXd v_max_limit_;  // [nv] (margin 적용 후, = v_max - margin)

  // Pre-allocated workspace
  Eigen::VectorXd a_pos_lb_;  // [nv]
  Eigen::VectorXd a_pos_ub_;  // [nv]
  Eigen::VectorXd a_vel_lb_;  // [nv]
  Eigen::VectorXd a_vel_ub_;  // [nv]
};

}  // namespace rtc::tsid
