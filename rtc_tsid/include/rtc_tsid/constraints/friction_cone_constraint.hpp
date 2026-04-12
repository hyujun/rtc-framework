#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Linearized friction cone constraint
//
// Point contact (3D force [fx, fy, fz]):
//   n_faces면 피라미드: cos(θk)·fx + sin(θk)·fy - μ·fz ≤ 0
//   unilateral:         -fz ≤ 0
//   → (n_faces + 1) ineq rows per contact
//
// Surface contact (6D wrench): point cone + moment limits (TODO Phase 3)
//
// C_block: a열 = 0, λ_i열에 cone matrix
// ────────────────────────────────────────────────
class FrictionConeConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "friction_cone";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int eq_dim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int ineq_dim(const ContactState& contacts) const noexcept override;

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

  void set_contact_manager(const ContactManagerConfig* manager) noexcept {
    manager_ = manager;
  }

 private:
  int nv_{0};
  const ContactManagerConfig* manager_{nullptr};

  // Pre-computed cone matrices per contact (init 시 생성)
  // cone_matrices_[i]: [(n_faces+1) × contact_dim] linearized cone
  std::vector<Eigen::MatrixXd> cone_matrices_;
};

}  // namespace rtc::tsid
