#include "rtc_tsid/constraints/torque_limit_constraint.hpp"

namespace rtc::tsid {

void TorqueLimitConstraint::init(
    const pinocchio::Model& /*model*/,
    const RobotModelInfo& robot_info,
    PinocchioCache& /*cache*/,
    const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
  na_ = robot_info.n_actuated;

  SM_.setZero(na_, nv_);
  Sh_.setZero(na_);
}

int TorqueLimitConstraint::eq_dim(
    const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int TorqueLimitConstraint::ineq_dim(
    const ContactState& /*contacts*/) const noexcept {
  return na_;
}

void TorqueLimitConstraint::compute_equality(
    const PinocchioCache& /*cache*/,
    const ContactState& /*contacts*/,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
    Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {
}

void TorqueLimitConstraint::compute_inequality(
    const PinocchioCache& cache,
    const ContactState& contacts,
    const RobotModelInfo& robot_info,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> C_block,
    Eigen::Ref<Eigen::VectorXd> l_block,
    Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  // τ = S·(M·a + h - Jcᵀ·λ)
  // τ_min ≤ S·(M·a + h - Jcᵀ·λ) ≤ τ_max
  //
  // C·z = S·M·a - S·Jcᵀ·λ
  // l = τ_min - S·h
  // u = τ_max - S·h

  // C[:, 0:nv] = S·M
  SM_.noalias() = robot_info.S * cache.M;
  C_block.leftCols(nv_) = SM_;

  // C[:, nv+offset : nv+offset+cdim] = -S·Jc_iᵀ  for each active contact
  if (manager_) {
    int lambda_offset = 0;
    for (size_t i = 0; i < contacts.contacts.size(); ++i) {
      if (!contacts.contacts[i].active) continue;
      if (i >= cache.contact_frames.size()) continue;

      const int cdim = manager_->contacts[i].contact_dim;
      const auto& Jc = cache.contact_frames[i].J;

      // -S · Jc[:cdim, :]ᵀ = -(S · Jc.topRows(cdim).transpose())
      // Result: [na × cdim]
      C_block.block(0, nv_ + lambda_offset, na_, cdim).noalias() =
          -(robot_info.S * Jc.topRows(cdim).transpose());

      lambda_offset += cdim;
    }
  }

  // l = τ_min - S·h,  u = τ_max - S·h
  Sh_.noalias() = robot_info.S * cache.h;
  l_block.head(na_) = robot_info.tau_min - Sh_;
  u_block.head(na_) = robot_info.tau_max - Sh_;
}

}  // namespace rtc::tsid
