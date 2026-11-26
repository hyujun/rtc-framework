#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

#include <cstdint>
#include <vector>

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
// zero_torsion_equality (YAML, default false): a surface contact whose
// torsional_friction_coeff is exactly 0 gets the yaw pair `±n·m ≤ 0` as ONE
// equality row `n·m = 0` instead (EqDim + 1, IneqDim − 2; n the contact's unit
// normal, as the pair's). The feasible set is the same; the solver no longer
// carries two always-active rows whose multipliers are not unique, which cost
// the locomotion QP several outer steps a tick (#1201). The yaw margin is then
// −|n·m| of the equality row, the value the pair's two rows gave. A contact
// with μ_τ > 0, or a point contact, keeps its rows.
//
// The same feasible set is not the same infeasibility verdict: on the
// locomotion QP made infeasible through the dynamics (proxsuite 0.8.2, the
// default eps_primal_inf 1e-4), the pair is certified PRIMAL_INFEASIBLE in 8
// outer steps while the equality settles at a dual-feasible point with a
// large primal residual and runs to max_iter — its certificate passes only
// from eps_primal_inf 1e-2. Either way a QP that cannot be solved is bounded
// by the iteration caps, not by the certificate (#1201).
//
// C_block: a열 = 0, λ_i열에 cone matrix
//
// Surface tangent basis (#1018): t1 is the contact frame's x projected on the
// plane ⊥ n, or — when x is too close to n — t1 = y_p × n from the frame's y.
// The choice has hysteresis per contact so a frame whose x hovers near the
// normal does not flip t1 (by up to 180° about n, swapping the CoP
// rectangle's half-lengths) tick to tick: the x rule is kept while its
// projection is ≥ kSurfaceBasisXToY (0.08) and the y rule while it is
// ≤ kSurfaceBasisYToX (0.12). A contact without a branch yet (after
// SetContactManager) splits at kSurfaceBasisFresh (0.1). The branch survives
// ticks on which the contact is inactive or not evaluated (a formulation
// skips a constraint with no rows): inside the band both rules are valid
// bases, and the rule then changes only when the projection leaves the band,
// however often the constraint is evaluated. No blend: the two rules' t1 can
// point up to 180° apart, where an average vanishes. A non-finite frame or
// normal keeps the branch and makes the contact's rows non-finite, so the
// solve fails on them. The branch is the constraint's own state, sized in
// SetContactManager (init) and written in place on the tick (RT-safe).
// ────────────────────────────────────────────────
class FrictionConeConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "friction_cone"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int EqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int IneqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int MaxEqDim(const ContactManagerConfig& contacts) const noexcept override;
  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig& contacts) const noexcept override;

  void ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                       const RobotModelInfo& robot_info, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> A_block,
                       Eigen::Ref<Eigen::VectorXd> b_block) noexcept override;

  void ComputeInequality(const PinocchioCache& cache, const ContactState& contacts,
                         const RobotModelInfo& robot_info, int n_vars,
                         Eigen::Ref<Eigen::MatrixXd> C_block, Eigen::Ref<Eigen::VectorXd> l_block,
                         Eigen::Ref<Eigen::VectorXd> u_block) noexcept override;

  // Per active contact: the face rows and the unilateral row lower
  // friction_n, the CoP rows cop_nm, the yaw rows yaw_nm.
  void MinSlack(const ContactState& contacts, const Eigen::Ref<const Eigen::MatrixXd>& C_block,
                const Eigen::Ref<const Eigen::VectorXd>& l_block,
                const Eigen::Ref<const Eigen::VectorXd>& u_block, const Eigen::VectorXd& x,
                WbcMargins& margins) const noexcept override;

  // Init-time: also sizes (and resets) the per-contact surface-basis branch.
  void SetContactManager(const ContactManagerConfig* manager);

  // Whether the zero-torsion surface contacts carry their yaw as an equality
  // (zero_torsion_equality, read in Init).
  [[nodiscard]] bool ZeroTorsionEquality() const noexcept { return zero_torsion_equality_; }

  // Hysteresis thresholds on ‖frame x projected ⊥ n‖ (see the class comment).
  static constexpr double kSurfaceBasisXToY = 0.08;
  static constexpr double kSurfaceBasisYToX = 0.12;
  static constexpr double kSurfaceBasisFresh = 0.1;

 private:
  int nv_{0};
  const ContactManagerConfig* manager_{nullptr};
  bool zero_torsion_equality_{false};

  // Per contact: the unit normal of its last yaw equality row (MinSlack reads
  // the row's value at the solution). [manager->contacts.size()], sized in
  // SetContactManager, written in place by ComputeEquality.
  std::vector<Eigen::Vector3d> yaw_normal_;

  // The contact carries its yaw as one equality row (see the class comment).
  [[nodiscard]] bool YawAsEquality(const ContactConfig& cfg) const noexcept {
    return zero_torsion_equality_ && cfg.contact_dim == 6 && cfg.torsional_friction_coeff == 0.0;
  }

  // Per contact: which rule built the last surface basis (0 none, 1 frame x,
  // 2 frame y). [manager->contacts.size()], sized in SetContactManager.
  std::vector<std::int8_t> surface_branch_;

  // Pre-computed cone matrices per contact (init 시 생성)
  // cone_matrices_[i]: [(n_faces+1) × contact_dim] linearized cone
  std::vector<Eigen::MatrixXd> cone_matrices_;
};

}  // namespace rtc::tsid
