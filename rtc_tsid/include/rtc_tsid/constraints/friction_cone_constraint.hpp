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
// Surface contact (6D wrench):
//   force cone + unilateral (위와 동일)
//   CoP rectangle: |m_x| ≤ l_y·f_z, |m_y| ≤ l_x·f_z  (4 rows)
//   yaw moment:   |m_z| ≤ μ_τ·f_z                    (2 rows)
//   → 총 (n_faces + 7) rows.
//
// patch=0 / μ_τ=0 (default) 일 때: f_z 열은 0 이지만 moment 열은 ±1 이 유지되어
// `+m ≤ 0 ∧ -m ≤ 0` → `m = 0` 의 stiff equality 로 작동 — 즉 patch=0 surface 는
// "moment 가 강제로 0 인 point-like restraint". 'trivial 행' 아님, 의도된 동작.
// (surface API 로 hemispherical fingertip 같은 effectively-point contact 를 표현.)
//
// C_block: a열 = 0, λ_i열에 cone matrix
// ────────────────────────────────────────────────
class FrictionConeConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "friction_cone"; }

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
  const ContactManagerConfig* manager_{nullptr};

  // Pre-computed cone matrices per contact (init 시 생성)
  // cone_matrices_[i]: [(n_faces+1) × contact_dim] linearized cone
  std::vector<Eigen::MatrixXd> cone_matrices_;
};

}  // namespace rtc::tsid
