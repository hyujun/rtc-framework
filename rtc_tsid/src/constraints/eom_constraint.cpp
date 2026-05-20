#include "rtc_tsid/constraints/eom_constraint.hpp"

namespace rtc::tsid {

void EomConstraint::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                         PinocchioCache& /*cache*/, const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
  floating_base_ = robot_info.floating_base;
  n_unactuated_ = floating_base_ ? (nv_ - robot_info.n_actuated) : 0;

  if (floating_base_) {
    // Projection matrix P = I - Sᵀ·S
    // S = [0_{na×6} | I_{na}], Sᵀ·S = [0 0; 0 I_{na}]
    // P = I - SᵀS = [I_6 0; 0 0]  → 상위 6행만 non-zero
    P_.setIdentity(nv_, nv_);
    P_.noalias() -= robot_info.S.transpose() * robot_info.S;

    PM_.setZero(n_unactuated_, nv_);
    Ph_.setZero(n_unactuated_);
  }
}

int EomConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  return n_unactuated_;
}

int EomConstraint::IneqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

void EomConstraint::ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                                     const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                     Eigen::Ref<Eigen::MatrixXd> A_block,
                                     Eigen::Ref<Eigen::VectorXd> b_block) noexcept {
  if (!floating_base_ || n_unactuated_ == 0)
    return;

  // [I-SᵀS]·(M·a + h - Jcᵀ·λ) = 0
  // → P·M·a - P·Jcᵀ·λ = -P·h
  //
  // A_block = [P·M | -P·Jcᵀ]    (n_unactuated × n_vars)
  // b_block = -P·h

  // P·M — P의 상위 n_unactuated행만 non-zero이므로 M의 상위 n_unactuated행 추출
  // P = [I_{nu} 0; 0 0] → P·M = M.topRows(nu)
  A_block.leftCols(nv_) = cache.M.topRows(n_unactuated_);

  // -P·Jcᵀ·λ: active contact 순회.
  // P = diag(I_{nu}, 0) 이므로 P·Jcᵀ = Jcᵀ.topRows(nu) = Jc[:cdim, :].T.topRows(nu)
  // = Jc.topRows(cdim).leftCols(nu).transpose().
  // contact_dim 은 manager 에서 조회; manager 미주입(legacy) 시 point(cdim=3) 가정.
  int lambda_offset = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active)
      continue;
    if (i >= cache.contact_frames.size())
      continue;

    const int cdim = (manager_ != nullptr && i < manager_->contacts.size())
                         ? manager_->contacts[i].contact_dim
                         : 3;

    const auto& Jc = cache.contact_frames[i].J;
    A_block.block(0, nv_ + lambda_offset, n_unactuated_, cdim) =
        -Jc.topRows(cdim).leftCols(n_unactuated_).transpose();

    lambda_offset += cdim;
  }

  // b = -P·h = -h.head(n_unactuated)
  b_block.head(n_unactuated_) = -cache.h.head(n_unactuated_);
}

void EomConstraint::ComputeInequality(const PinocchioCache& /*cache*/,
                                       const ContactState& /*contacts*/,
                                       const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                       Eigen::Ref<Eigen::MatrixXd> /*C_block*/,
                                       Eigen::Ref<Eigen::VectorXd> /*l_block*/,
                                       Eigen::Ref<Eigen::VectorXd> /*u_block*/) noexcept {
  // No inequality constraints from EoM
}

}  // namespace rtc::tsid
