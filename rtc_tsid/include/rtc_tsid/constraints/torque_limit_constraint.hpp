#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Torque limit constraint
//
// τ = S·(M·a + h - Jcᵀ·λ)
// τ_min ≤ τ ≤ τ_max
//
// C = [S·M | -S·Jcᵀ]    (n_actuated × n_vars)
// l = τ_min - S·h
// u = τ_max - S·h
// ────────────────────────────────────────────────
class TorqueLimitConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "torque_limit"; }

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

  void SetContactManager(const ContactManagerConfig* manager) noexcept { manager_ = manager; }

 private:
  int nv_{0};
  int na_{0};
  // URDF effort_limit 에 곱하는 margin (0 < tau_scale ≤ 1). YAML 키 누락 시 1.0
  // (raw URDF 사용). init 에서 (0, 1] 범위 위반 시 throw — silent margin 변경 방지.
  double tau_scale_{1.0};
  const ContactManagerConfig* manager_{nullptr};

  // Pre-allocated workspace
  Eigen::MatrixXd SM_;  // [na × nv] S·M
  Eigen::VectorXd Sh_;  // [na] S·h
};

}  // namespace rtc::tsid
