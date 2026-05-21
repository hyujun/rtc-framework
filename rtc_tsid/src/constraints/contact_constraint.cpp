#include "rtc_tsid/constraints/contact_constraint.hpp"

namespace rtc::tsid {

void ContactConstraint::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                             PinocchioCache& /*cache*/, const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
}

int ContactConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  // Stage A-5a: fixed-dim QP — reserve one row block per contact slot
  // regardless of activation. Inactive contacts produce all-zero rows
  // (trivial 0=0 equality, no constraint on the QP).
  if (!manager_)
    return 0;
  return manager_->max_contact_vars;
}

int ContactConstraint::IneqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

void ContactConstraint::ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                                        const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                        Eigen::Ref<Eigen::MatrixXd> A_block,
                                        Eigen::Ref<Eigen::VectorXd> b_block) noexcept {
  if (!manager_)
    return;

  // Stage A-5a: fixed-dim equality block. Active contacts get the
  // Jc_i·a = -dotJc_i·v rows; inactive contacts get zero rows (left as
  // the caller's zero-init, so 0·a = 0 is trivially satisfied).
  int row = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager_->contacts[i].contact_dim;

    if (!contacts.contacts[i].active || i >= cache.contact_frames.size()) {
      row += cdim;
      continue;
    }

    const auto& fc = cache.contact_frames[i];

    // Jc_i · a = 0  →  A[row:row+cdim, 0:nv] = Jc_i[:cdim, :]
    A_block.block(row, 0, cdim, nv_) = fc.J.topRows(cdim);

    // b = -dJc_i · v (contact acceleration bias)
    b_block.segment(row, cdim) = -fc.dJv.head(cdim);

    row += cdim;
  }
}

void ContactConstraint::ComputeInequality(const PinocchioCache& /*cache*/,
                                          const ContactState& /*contacts*/,
                                          const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                          Eigen::Ref<Eigen::MatrixXd> /*C_block*/,
                                          Eigen::Ref<Eigen::VectorXd> /*l_block*/,
                                          Eigen::Ref<Eigen::VectorXd> /*u_block*/) noexcept {}

}  // namespace rtc::tsid
