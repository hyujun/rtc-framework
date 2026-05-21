#include "rtc_tsid/tasks/internal_force_task.hpp"

namespace rtc::tsid {

void InternalForceTask::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                             PinocchioCache& /*cache*/, const YAML::Node& task_config) {
  nv_ = robot_info.nv;
  current_residual_dim_ = 0;

  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  // Workspace sized once SetContactManager() wires manager_; init at 0.
  lambda_squeeze_des_.setZero(0);
  lambda_des_active_.setZero(0);
  r_projected_.setZero(0);
  has_reference_ = false;
}

void InternalForceTask::SetContactManager(const ContactManagerConfig* manager) noexcept {
  manager_ = manager;
  if (!manager_) {
    return;
  }
  const int mcv = manager_->max_contact_vars;
  lambda_squeeze_des_.setZero(mcv);
  lambda_des_active_.setZero(mcv);
  r_projected_.setZero(mcv);
  has_reference_ = false;
}

void InternalForceTask::SetSqueezeReference(const Eigen::VectorXd& lambda_squeeze_des) noexcept {
  if (!manager_) {
    return;
  }
  const int mcv = manager_->max_contact_vars;
  if (lambda_squeeze_des.size() < mcv) {
    return;  // malformed input — keep prior reference
  }
  lambda_squeeze_des_.head(mcv) = lambda_squeeze_des.head(mcv);
  has_reference_ = true;
}

void InternalForceTask::UpdateResidualDim(const ContactState& contacts) noexcept {
  if (!manager_ || !mgr_rt_ || !grasp_cache_) {
    current_residual_dim_ = 0;
    return;
  }
  const int n_active = mgr_rt_->ActiveLambdaDim(contacts);
  const int rank_g = grasp_cache_->Rank();
  const int nullity = n_active - rank_g;
  current_residual_dim_ = (nullity > 0) ? n_active : 0;
}

void InternalForceTask::ComputeResidual(const PinocchioCache& /*cache*/,
                                        const ControlReference& /*ref*/,
                                        const ContactState& contacts, int /*n_vars*/,
                                        Eigen::Ref<Eigen::MatrixXd> J_block,
                                        Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  // Recompute residual_dim from the current GraspCache rank (one-tick lag
  // pattern matching ForceTask / ContactConsistencyTask / ObjectWrenchTask).
  UpdateResidualDim(contacts);
  if (current_residual_dim_ == 0) {
    return;
  }
  if (!manager_ || !mgr_rt_ || !grasp_cache_ || !has_reference_) {
    return;
  }

  const int n_active = mgr_rt_->ActiveLambdaDim(contacts);

  // Pack the active-slot reference into lambda_des_active_[0..n_active-1].
  auto lda = lambda_des_active_.head(n_active);
  int active_off = 0;
  int slot_off = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager_->contacts[i].contact_dim;
    if (contacts.contacts[i].active) {
      lda.segment(active_off, cdim) = lambda_squeeze_des_.segment(slot_off, cdim);
      active_off += cdim;
    }
    slot_off += cdim;
  }

  // r_projected_ = P_N · λ_des_active. P_N from GraspCache is n_λ × n_λ
  // sized to the current active dim.
  auto P_N = grasp_cache_->ProjN();
  auto r_proj = r_projected_.head(n_active);
  r_proj.noalias() = P_N * lda;

  // J_block right block has a 1 at each (active row, active slot col) pair.
  // r_block holds the projected reference in active-row order.
  int row = 0;
  int lambda_offset_in_z = nv_;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager_->contacts[i].contact_dim;
    if (contacts.contacts[i].active) {
      // J(row..row+cdim-1, lambda_offset_in_z..+cdim-1) = I_cdim.
      for (int d = 0; d < cdim; ++d) {
        J_block(row + d, lambda_offset_in_z + d) = 1.0;
      }
      r_block.segment(row, cdim) = r_proj.segment(row, cdim);
      row += cdim;
    }
    lambda_offset_in_z += cdim;
  }
}

}  // namespace rtc::tsid
