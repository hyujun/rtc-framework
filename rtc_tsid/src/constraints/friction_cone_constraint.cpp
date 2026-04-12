#include "rtc_tsid/constraints/friction_cone_constraint.hpp"

#include <cmath>

namespace rtc::tsid {

void FrictionConeConstraint::init(
    const pinocchio::Model& /*model*/,
    const RobotModelInfo& robot_info,
    PinocchioCache& /*cache*/,
    const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
}

int FrictionConeConstraint::eq_dim(
    const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int FrictionConeConstraint::ineq_dim(
    const ContactState& contacts) const noexcept {
  if (!manager_) return 0;

  int total = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active) continue;
    const auto& cfg = manager_->contacts[i];
    // Point contact: n_faces + 1 (cone edges + unilateral fz >= 0)
    // Surface contact: same for now (moment constraints in Phase 3)
    total += cfg.friction_faces + 1;
  }
  return total;
}

void FrictionConeConstraint::compute_equality(
    const PinocchioCache& /*cache*/,
    const ContactState& /*contacts*/,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
    Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {
}

void FrictionConeConstraint::compute_inequality(
    const PinocchioCache& /*cache*/,
    const ContactState& contacts,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> C_block,
    Eigen::Ref<Eigen::VectorXd> l_block,
    Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  if (!manager_) return;

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

    // Linearized friction cone for point contact (fx, fy, fz):
    // cos(θk)·fx + sin(θk)·fy - μ·fz ≤ 0,  k = 0..n_faces-1
    // -fz ≤ 0  (unilateral: fz ≥ 0)
    //
    // For surface (6D), same on force components [0:3], ignore moments for now

    const int lambda_col = nv_ + lambda_offset;

    for (int k = 0; k < n_faces; ++k) {
      const double theta =
          2.0 * M_PI * static_cast<double>(k) / static_cast<double>(n_faces);
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

    lambda_offset += cdim;
  }
}

}  // namespace rtc::tsid
