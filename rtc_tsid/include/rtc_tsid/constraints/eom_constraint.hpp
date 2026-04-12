#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Equations of Motion constraint
//
// Fixed-base (S = I): eq_dim = 0 (EoM implicit, τ post-solve 역산)
// Floating-base:      eq_dim = nv - n_actuated (보통 6)
//   [I - SᵀS]·(M·a + h - Jcᵀ·λ) = 0
//   → unactuated DoF의 force balance
// ────────────────────────────────────────────────
class EomConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "eom";
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
  int n_unactuated_{0};   // nv - n_actuated (0 for fixed-base)
  bool floating_base_{false};

  // Pre-allocated workspace
  Eigen::MatrixXd P_;      // [nv × nv] projection matrix I - SᵀS
  Eigen::MatrixXd PM_;     // [n_unactuated × nv] P*M
  Eigen::VectorXd Ph_;     // [n_unactuated] P*h
};

}  // namespace rtc::tsid
