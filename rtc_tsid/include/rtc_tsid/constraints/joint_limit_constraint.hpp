#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Acceleration-level joint limit viability constraint
// (Del Prete 2018)
//
// Position limit → acceleration bound:
//   a_i ≥ 2·(q_min_i - q_i - v_i·dt) / dt²
//   a_i ≤ 2·(q_max_i - q_i - v_i·dt) / dt²
//
// Velocity limit → acceleration bound:
//   a_i ≥ (v_min_i - v_i) / dt
//   a_i ≤ (v_max_i - v_i) / dt
//
// 최종: l_i = max(pos_lb, vel_lb), u_i = min(pos_ub, vel_ub)
// C = [I_{nv} | 0]  (nv × n_vars)
//
// Floating-base: 처음 6 DoF는 무제약 (l=-∞, u=+∞)
// ────────────────────────────────────────────────
class JointLimitConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "joint_limit";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int eq_dim(
      const ContactState& contacts) const noexcept override;
  [[nodiscard]] int ineq_dim(
      const ContactState& contacts) const noexcept override;

  void compute_equality(
      const PinocchioCache& cache,
      const ContactState& contacts,
      const RobotModelInfo& robot_info,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> A_block,
      Eigen::Ref<Eigen::VectorXd> b_block) noexcept override;

  void compute_inequality(
      const PinocchioCache& cache,
      const ContactState& contacts,
      const RobotModelInfo& robot_info,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> C_block,
      Eigen::Ref<Eigen::VectorXd> l_block,
      Eigen::Ref<Eigen::VectorXd> u_block) noexcept override;

 private:
  int nv_{0};
  bool floating_base_{false};

  double dt_{0.002};              // 제어 주기 (s)
  double margin_pos_{0.05};       // position limit margin (rad)
  double margin_vel_{0.1};        // velocity limit margin (rad/s)

  Eigen::VectorXd q_min_;         // [nv] (margin 적용 후)
  Eigen::VectorXd q_max_;         // [nv] (margin 적용 후)
  Eigen::VectorXd v_min_;         // [nv] (margin 적용 후, = -v_max + margin)
  Eigen::VectorXd v_max_limit_;   // [nv] (margin 적용 후, = v_max - margin)

  // Pre-allocated workspace
  Eigen::VectorXd a_pos_lb_;      // [nv]
  Eigen::VectorXd a_pos_ub_;      // [nv]
  Eigen::VectorXd a_vel_lb_;      // [nv]
  Eigen::VectorXd a_vel_ub_;      // [nv]
};

}  // namespace rtc::tsid
