#include "rtc_tsid/contact/contact_manager.hpp"

namespace rtc::tsid {

void ContactManager::Init(const ContactManagerConfig& manager_cfg, int nv) noexcept {
  manager_cfg_ = &manager_cfg;
  nv_ = nv;
}

int ContactManager::ActiveLambdaDim(const ContactState& contacts) const noexcept {
  if (!manager_cfg_)
    return 0;
  int dim = 0;
  for (size_t i = 0; i < contacts.contacts.size() && i < manager_cfg_->contacts.size(); ++i) {
    if (contacts.contacts[i].active) {
      dim += manager_cfg_->contacts[i].contact_dim;
    }
  }
  return dim;
}

void ContactManager::ComputeGraspMatrix(const PinocchioCache& cache, const ContactState& contacts,
                                        const ObjectFrame& object_frame,
                                        Eigen::Ref<Eigen::MatrixXd> G_out) const noexcept {
  if (!manager_cfg_)
    return;

  const Eigen::Vector3d& p_o = object_frame.p_w;

  int col = 0;
  for (size_t i = 0; i < contacts.contacts.size() && i < manager_cfg_->contacts.size() &&
                     i < cache.contact_frames.size();
       ++i) {
    if (!contacts.contacts[i].active)
      continue;

    const int cdim = manager_cfg_->contacts[i].contact_dim;
    const Eigen::Vector3d p_ci = cache.contact_frames[i].oMf.translation();

    if (cdim == 3) {
      // Point contact: 6×3 force-only block.
      const Eigen::Matrix<double, 6, 3> B = PointGraspBlock(p_ci, p_o);
      G_out.block(0, col, 6, 3) = B;
    } else if (cdim == 6) {
      // Surface contact: 6×6 Adjoint dual.
      const Eigen::Matrix<double, 6, 6> Ad = AdjointDualWorldAligned(p_ci, p_o);
      G_out.block(0, col, 6, 6) = Ad;
    }
    col += cdim;
  }
}

void ContactManager::StackContactJacobians(const PinocchioCache& cache,
                                           const ContactState& contacts,
                                           Eigen::Ref<Eigen::MatrixXd> J_out) const noexcept {
  if (!manager_cfg_)
    return;

  int row = 0;
  for (size_t i = 0; i < contacts.contacts.size() && i < manager_cfg_->contacts.size() &&
                     i < cache.contact_frames.size();
       ++i) {
    if (!contacts.contacts[i].active)
      continue;

    const int cdim = manager_cfg_->contacts[i].contact_dim;
    // Pinocchio J is [6 × nv] LOCAL_WORLD_ALIGNED; top-3 rows = linear,
    // top-6 = full. Matches ContactConstraint convention.
    J_out.block(row, 0, cdim, nv_) = cache.contact_frames[i].J.topRows(cdim);
    row += cdim;
  }
}

void ContactManager::StackContactJDotV(const PinocchioCache& cache, const ContactState& contacts,
                                       Eigen::Ref<Eigen::VectorXd> v_out) const noexcept {
  if (!manager_cfg_)
    return;

  int row = 0;
  for (size_t i = 0; i < contacts.contacts.size() && i < manager_cfg_->contacts.size() &&
                     i < cache.contact_frames.size();
       ++i) {
    if (!contacts.contacts[i].active)
      continue;

    const int cdim = manager_cfg_->contacts[i].contact_dim;
    v_out.segment(row, cdim) = cache.contact_frames[i].dJv.head(cdim);
    row += cdim;
  }
}

}  // namespace rtc::tsid
