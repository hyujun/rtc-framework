#include "rtc_tsid/tasks/object_se3_task.hpp"

#include "rtc_tsid/kinematics/se3_error.hpp"

#include <cmath>

namespace rtc::tsid {

void ObjectSE3Task::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                         PinocchioCache& /*cache*/, const YAML::Node& task_config) {
  nv_ = robot_info.nv;
  current_residual_dim_ = 0;

  // Default mask: all 6 axes active.
  mask_.fill(true);
  active_mask_count_ = 6;
  if (task_config && task_config["mask"]) {
    const auto& mask_node = task_config["mask"];
    active_mask_count_ = 0;
    for (int i = 0; i < 6 && i < static_cast<int>(mask_node.size()); ++i) {
      mask_[static_cast<size_t>(i)] = (mask_node[static_cast<size_t>(i)].as<int>() != 0);
      if (mask_[static_cast<size_t>(i)])
        ++active_mask_count_;
    }
  }

  if (task_config && task_config["kp"]) {
    const auto& kp_node = task_config["kp"];
    if (kp_node.IsScalar()) {
      kp_.setConstant(kp_node.as<double>());
    } else {
      for (int i = 0; i < 6 && i < static_cast<int>(kp_node.size()); ++i) {
        kp_(i) = kp_node[static_cast<size_t>(i)].as<double>();
      }
    }
  }
  if (task_config && task_config["kd"]) {
    const auto& kd_node = task_config["kd"];
    if (kd_node.IsScalar()) {
      kd_.setConstant(kd_node.as<double>());
    } else {
      for (int i = 0; i < 6 && i < static_cast<int>(kd_node.size()); ++i) {
        kd_(i) = kd_node[static_cast<size_t>(i)].as<double>();
      }
    }
  }

  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  // Workspaces are sized in SetContactManager() when manager_ wiring
  // arrives (matches ObjectWrenchTask / InternalForceTask). Leave at 0.
  jc_stack_.setZero(0, 0);
  djv_stack_.setZero(0);
  j_obj_full_.setZero(6, 0);
}

void ObjectSE3Task::SetContactManager(const ContactManagerConfig* manager) noexcept {
  manager_ = manager;
  if (!manager_) {
    return;
  }
  const int mcv = manager_->max_contact_vars;
  jc_stack_.setZero(mcv, nv_);
  djv_stack_.setZero(mcv);
  j_obj_full_.setZero(6, nv_);
}

void ObjectSE3Task::SetSe3Reference(const pinocchio::SE3& placement_des,
                                    const Eigen::Matrix<double, 6, 1>& v_des,
                                    const Eigen::Matrix<double, 6, 1>& a_ff) noexcept {
  placement_des_ = placement_des;
  v_des_ = v_des;
  a_ff_ = a_ff;
}

void ObjectSE3Task::SetGains(const Eigen::Matrix<double, 6, 1>& kp,
                             const Eigen::Matrix<double, 6, 1>& kd) noexcept {
  kp_ = kp;
  kd_ = kd;
}

void ObjectSE3Task::UpdateResidualDim(const ContactState& contacts) noexcept {
  if (!manager_ || !mgr_rt_ || !grasp_cache_ || !state_provider_) {
    current_residual_dim_ = 0;
    return;
  }
  if (!state_provider_->IsValid()) {
    current_residual_dim_ = 0;
    return;
  }
  const int n_active = mgr_rt_->ActiveLambdaDim(contacts);
  current_residual_dim_ = (n_active > 0) ? active_mask_count_ : 0;
}

void ObjectSE3Task::ComputeResidual(const PinocchioCache& cache, const ControlReference& /*ref*/,
                                    const ContactState& contacts, int /*n_vars*/,
                                    Eigen::Ref<Eigen::MatrixXd> J_block,
                                    Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  // One-tick lag pattern (matches Object/Internal force tasks).
  UpdateResidualDim(contacts);
  if (current_residual_dim_ == 0) {
    return;
  }
  if (!manager_ || !mgr_rt_ || !grasp_cache_ || !state_provider_) {
    return;
  }

  const int n_active = mgr_rt_->ActiveLambdaDim(contacts);
  if (n_active == 0) {
    return;
  }

  // ── Build stacked contact Jacobians and J̇_c·v bias (active-packed) ──
  auto jc_packed = jc_stack_.topLeftCorner(n_active, nv_);
  auto djv_packed = djv_stack_.head(n_active);
  jc_packed.setZero();
  djv_packed.setZero();
  mgr_rt_->StackContactJacobians(cache, contacts, jc_packed);
  mgr_rt_->StackContactJDotV(cache, contacts, djv_packed);

  // ── (Gᵀ)⁺ from GraspCache: 6 × n_active. Kinematic projection: ──
  //   J_obj   = (Gᵀ)⁺ · J_c_stack         ∈ R^{6 × nv}
  //   bias    = (Gᵀ)⁺ · (J̇_c · v)_stack   ∈ R^6
  auto GTpinv = grasp_cache_->GTPinv();
  j_obj_full_.noalias() = GTpinv * jc_packed;
  const Eigen::Matrix<double, 6, 1> bias = GTpinv * djv_packed;

  // ── Object SE(3) PD reference (world-aligned at p_o) ──
  const ObjectFrame& obj = state_provider_->GetPose();
  const Eigen::Matrix<double, 6, 1>& v_obj = state_provider_->GetTwist();

  // Pose / velocity error via the shared WBC helper (U1, LWA frame). The object
  // frame is world-aligned at p_o, and v_obj is its world-aligned spatial twist,
  // so the LWA convention applies directly (same as SE3Task / ClikReference).
  const pinocchio::SE3 obj_pose(obj.R_w, obj.p_w);
  error_full_ = ComputeTaskPoseError(obj_pose, placement_des_);

  // Velocity error = exact ė (Jlog6 fwd); reduces to v_des − v_obj at small error.
  v_error_full_ = ComputeTaskVelocityError(obj_pose, placement_des_, v_obj, v_des_);

  // Desired object acceleration.
  a_des_full_ = a_ff_ + kp_.cwiseProduct(error_full_) + kd_.cwiseProduct(v_error_full_);

  // r_full = a_obj_des - (Gᵀ)⁺ · (J̇_c · v)_stack.
  r_full_ = a_des_full_ - bias;

  // ── Emit into J_block / r_block by mask ──
  // J_block is [active_mask_count × n_vars]; left nv columns = j_obj_full
  // rows, right max_contact_vars columns = 0 (caller zero-init guarantees).
  int row = 0;
  for (int i = 0; i < 6; ++i) {
    if (!mask_[static_cast<size_t>(i)])
      continue;
    J_block.row(row).head(nv_) = j_obj_full_.row(i);
    r_block(row) = r_full_(i);
    ++row;
  }
}

}  // namespace rtc::tsid
