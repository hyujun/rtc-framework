#include "rtc_tsid/constraints/friction_cone_constraint.hpp"

#include <cmath>

namespace rtc::tsid {

void FrictionConeConstraint::init(const pinocchio::Model& /*model*/,
                                  const RobotModelInfo& robot_info, PinocchioCache& /*cache*/,
                                  const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
}

int FrictionConeConstraint::eq_dim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int FrictionConeConstraint::ineq_dim(const ContactState& contacts) const noexcept {
  if (!manager_)
    return 0;

  int total = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active)
      continue;
    const auto& cfg = manager_->contacts[i];
    // Point contact (cdim=3): n_faces + 1 (cone faces + unilateral -fz ≤ 0)
    // Surface contact (cdim=6): + 4 (CoP rectangle) + 2 (yaw moment) = n_faces + 7.
    // patch=0 / μ_τ=0 인 surface 라도 trivial 행 (0 ≤ 0) 으로 유지 — ineq_dim 일관.
    int per_contact = cfg.friction_faces + 1;
    if (cfg.contact_dim == 6) {
      per_contact += 6;
    }
    total += per_contact;
  }
  return total;
}

void FrictionConeConstraint::compute_equality(const PinocchioCache& /*cache*/,
                                              const ContactState& /*contacts*/,
                                              const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                              Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
                                              Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {}

void FrictionConeConstraint::compute_inequality(const PinocchioCache& /*cache*/,
                                                const ContactState& contacts,
                                                const RobotModelInfo& /*robot_info*/,
                                                int /*n_vars*/, Eigen::Ref<Eigen::MatrixXd> C_block,
                                                Eigen::Ref<Eigen::VectorXd> l_block,
                                                Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  if (!manager_)
    return;

  int row = 0;
  int lambda_offset = 0;

  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active) {
      continue;
    }

    const auto& cfg = manager_->contacts[i];
    const int cdim = cfg.contact_dim;
    const int n_faces = cfg.friction_faces;
    const double mu = cfg.friction_coeff;

    // Linearized friction cone (force components, common to point & surface):
    // cos(θk)·fx + sin(θk)·fy - μ·fz ≤ 0,  k = 0..n_faces-1
    // -fz ≤ 0  (unilateral: fz ≥ 0)
    // Surface(6D) 는 아래 분기에서 moment (CoP + yaw) 행을 추가.

    const int lambda_col = nv_ + lambda_offset;

    for (int k = 0; k < n_faces; ++k) {
      const double theta = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(n_faces);
      // C[row, lambda_col + 0] = cos(θ)   (fx)
      // C[row, lambda_col + 1] = sin(θ)   (fy)
      // C[row, lambda_col + 2] = -μ       (fz)
      C_block(row, lambda_col + 0) = std::cos(theta);
      C_block(row, lambda_col + 1) = std::sin(theta);
      C_block(row, lambda_col + 2) = -mu;
      // l ≤ C·z ≤ u  →  C·z ≤ 0  →  l = -inf, u = 0
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
    }

    // Unilateral: -fz ≤ 0  →  fz ≥ 0
    C_block(row, lambda_col + 2) = -1.0;
    l_block(row) = -1e10;
    u_block(row) = 0.0;
    ++row;

    // Surface contact (cdim=6): wrench cone — CoP rectangle + yaw moment.
    // λ = [fx, fy, fz, mx, my, mz] (contact frame, +z normal, xy patch plane).
    //
    // CoP_x ∈ [-l_x, l_x]:  m_y - l_x·f_z ≤ 0,  -m_y - l_x·f_z ≤ 0
    // CoP_y ∈ [-l_y, l_y]:  m_x - l_y·f_z ≤ 0,  -m_x - l_y·f_z ≤ 0
    // Yaw:  |m_z| ≤ μ_τ·f_z:  m_z - μ_τ·f_z ≤ 0,  -m_z - μ_τ·f_z ≤ 0
    //
    // patch=0 / μ_τ=0 default 인 surface 도 행 자체는 추가. *결과*: f_z 열은
    // 0 이 되지만 moment 열 ±1 이 살아 `m = 0` 의 stiff equality 로 작동 — 즉
    // point-like restraint (vacuous 아님). ineq_dim 일관성 우선.
    if (cdim == 6) {
      const double lx = cfg.patch_half_length_x;
      const double ly = cfg.patch_half_length_y;
      const double mu_tau = cfg.torsional_friction_coeff;

      // m_y - l_x·f_z ≤ 0
      C_block(row, lambda_col + 2) = -lx;
      C_block(row, lambda_col + 4) = 1.0;
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
      // -m_y - l_x·f_z ≤ 0
      C_block(row, lambda_col + 2) = -lx;
      C_block(row, lambda_col + 4) = -1.0;
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
      // m_x - l_y·f_z ≤ 0
      C_block(row, lambda_col + 2) = -ly;
      C_block(row, lambda_col + 3) = 1.0;
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
      // -m_x - l_y·f_z ≤ 0
      C_block(row, lambda_col + 2) = -ly;
      C_block(row, lambda_col + 3) = -1.0;
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
      // m_z - μ_τ·f_z ≤ 0
      C_block(row, lambda_col + 2) = -mu_tau;
      C_block(row, lambda_col + 5) = 1.0;
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
      // -m_z - μ_τ·f_z ≤ 0
      C_block(row, lambda_col + 2) = -mu_tau;
      C_block(row, lambda_col + 5) = -1.0;
      l_block(row) = -1e10;
      u_block(row) = 0.0;
      ++row;
    }

    lambda_offset += cdim;
  }
}

}  // namespace rtc::tsid
