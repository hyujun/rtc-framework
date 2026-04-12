#include "rtc_tsid/constraints/contact_constraint.hpp"

namespace rtc::tsid {

void ContactConstraint::init(const pinocchio::Model& /*model*/,
                             const RobotModelInfo& robot_info,
                             PinocchioCache& /*cache*/,
                             const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
}

int ContactConstraint::eq_dim(const ContactState& contacts) const noexcept {
  return contacts.active_contact_vars;
}

int ContactConstraint::ineq_dim(
    const ContactState& /*contacts*/) const noexcept {
  return 0;
}

void ContactConstraint::compute_equality(
    const PinocchioCache& cache,
    const ContactState& contacts,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> A_block,
    Eigen::Ref<Eigen::VectorXd> b_block) noexcept {
  if (!manager_) return;

  int row = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active) continue;
    if (i >= cache.contact_frames.size()) continue;

    const int cdim = manager_->contacts[i].contact_dim;
    const auto& fc = cache.contact_frames[i];

    // Jc_i · a = 0  →  A[row:row+cdim, 0:nv] = Jc_i[:cdim, :]
    A_block.block(row, 0, cdim, nv_) = fc.J.topRows(cdim);

    // b = -dJc_i · v (contact acceleration bias)
    b_block.segment(row, cdim) = -fc.dJv.head(cdim);

    row += cdim;
  }
}

void ContactConstraint::compute_inequality(
    const PinocchioCache& /*cache*/,
    const ContactState& /*contacts*/,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> /*C_block*/,
    Eigen::Ref<Eigen::VectorXd> /*l_block*/,
    Eigen::Ref<Eigen::VectorXd> /*u_block*/) noexcept {
}

}  // namespace rtc::tsid
