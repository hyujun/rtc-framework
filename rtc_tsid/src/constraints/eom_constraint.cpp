#include "rtc_tsid/constraints/eom_constraint.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#pragma GCC diagnostic pop

#include <stdexcept>

namespace rtc::tsid {

void EomConstraint::Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                         PinocchioCache& /*cache*/, const YAML::Node& /*constraint_config*/) {
  // The base rows below are the top six rows of M a + h − Jcᵀλ: that is the
  // unactuated part only when the root is a free-flyer (velocity rows 0..5)
  // and every other DoF is actuated — S = [0 | I], no passive joints.
  if (robot_info.nv != model.nv) {
    throw std::invalid_argument("eom: RobotModelInfo.nv does not match the model");
  }
  if (robot_info.floating_base) {
    const bool free_flyer_root =
        model.joints.size() > 1 &&
        model.joints[1].shortname() == pinocchio::JointModelFreeFlyer::classname();
    const int na = robot_info.n_actuated;
    const bool s_is_0_i =
        free_flyer_root && robot_info.S.rows() == na && robot_info.S.cols() == model.nv &&
        na == model.nv - 6 && robot_info.S.leftCols(6).isZero(0.0) &&
        robot_info.S.rightCols(na).isApprox(Eigen::MatrixXd::Identity(na, na), 0.0);
    if (!s_is_0_i) {
      throw std::invalid_argument(
          "eom: a floating base needs a free-flyer root and S = [0 | I] (nv − 6 actuated)");
    }
  }

  nv_ = robot_info.nv;
  floating_base_ = robot_info.floating_base;
  n_unactuated_ = floating_base_ ? (nv_ - robot_info.n_actuated) : 0;
}

int EomConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  return n_unactuated_;
}

int EomConstraint::IneqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int EomConstraint::MaxEqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return n_unactuated_;
}

int EomConstraint::MaxIneqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
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

  // Stage A-5a: fixed-dim QP — λ column slot is fixed per contact index in
  // the manager. Inactive contacts leave their column block as zero (A_block
  // was zero-init by caller) so the QP sees them as having zero contribution
  // to EoM; the solver picks λ_i = 0 via Hessian regularization.
  int lambda_offset = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = (manager_ != nullptr && i < manager_->contacts.size())
                         ? manager_->contacts[i].contact_dim
                         : 3;

    if (!contacts.contacts[i].active || i >= cache.contact_frames.size()) {
      lambda_offset += cdim;
      continue;
    }

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
