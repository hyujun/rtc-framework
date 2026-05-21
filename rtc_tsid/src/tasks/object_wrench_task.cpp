#include "rtc_tsid/tasks/object_wrench_task.hpp"

namespace rtc::tsid {

void ObjectWrenchTask::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                            PinocchioCache& /*cache*/, const YAML::Node& task_config) {
  nv_ = robot_info.nv;
  current_residual_dim_ = 0;

  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  // Scratch buffer is sized in SetContactManager() once manager_ wiring
  // arrives (matches ForceTask convention). Leave at 0 here.
  g_active_.setZero(6, 0);
}

void ObjectWrenchTask::SetContactManager(const ContactManagerConfig* manager) noexcept {
  manager_ = manager;
  if (manager_) {
    g_active_.setZero(6, manager_->max_contact_vars);
  }
}

void ObjectWrenchTask::UpdateResidualDim(const ContactState& contacts) noexcept {
  current_residual_dim_ = (contacts.active_count > 0) ? 6 : 0;
}

void ObjectWrenchTask::ComputeResidual(const PinocchioCache& cache, const ControlReference& /*ref*/,
                                       const ContactState& contacts, int /*n_vars*/,
                                       Eigen::Ref<Eigen::MatrixXd> J_block,
                                       Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  current_residual_dim_ = (contacts.active_count > 0) ? 6 : 0;
  if (current_residual_dim_ == 0) {
    return;
  }
  if (!manager_ || !mgr_rt_ || !object_) {
    return;
  }

  // Build the active-packed G into the leading active_lambda_dim columns
  // of g_active_, then unpack into J_block at fixed contact slot offsets.
  const int n_active = mgr_rt_->ActiveLambdaDim(contacts);
  if (n_active == 0) {
    return;
  }

  auto g_packed = g_active_.topLeftCorner(6, n_active);
  g_packed.setZero();
  mgr_rt_->ComputeGraspMatrix(cache, contacts, *object_, g_packed);

  // Unpack into J_block. Lambda slot offsets advance for every contact
  // (active or not, matches Stage A-5a fixed-dim QP convention); active
  // contacts copy their cdim columns from g_packed, inactive slots stay 0.
  int lambda_offset_in_z = nv_;
  int active_col_in_g = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager_->contacts[i].contact_dim;
    if (contacts.contacts[i].active) {
      J_block.block(0, lambda_offset_in_z, 6, cdim) = g_packed.block(0, active_col_in_g, 6, cdim);
      active_col_in_g += cdim;
    }
    lambda_offset_in_z += cdim;
  }

  // r = w_obj_des
  r_block.head<6>() = w_obj_des_;
}

}  // namespace rtc::tsid
