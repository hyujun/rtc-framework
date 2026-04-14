#include "rtc_tsid/tasks/force_task.hpp"

namespace rtc::tsid {

void ForceTask::init(const pinocchio::Model& /*model*/,
                     const RobotModelInfo& robot_info,
                     PinocchioCache& /*cache*/,
                     const YAML::Node& task_config) {
  nv_ = robot_info.nv;

  // Weight / priority
  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  // max_contact_vars는 set_contact_manager 후에 결정
  // 여기서는 0으로 초기화, set_contact_manager 호출 시 재할당
  max_contact_vars_ = 0;
  current_residual_dim_ = 0;
  lambda_des_.resize(0);
  has_local_ref_ = false;
}

void ForceTask::update_residual_dim(
    const ContactState& contacts) noexcept {
  current_residual_dim_ = contacts.active_contact_vars;
}

void ForceTask::compute_residual(
    const PinocchioCache& /*cache*/,
    const ControlReference& ref,
    const ContactState& contacts,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> J_block,
    Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  if (!manager_) return;

  // active contact에 따라 residual dim 갱신
  current_residual_dim_ = contacts.active_contact_vars;
  if (current_residual_dim_ == 0) return;

  // reference 선택: local이 설정됐으면 local, 아니면 ControlReference
  const auto& lambda_ref = has_local_ref_ ? lambda_des_ : ref.lambda_des;

  int row = 0;
  int lambda_offset_in_z = nv_;   // QP 변수에서 λ 시작 위치
  int lambda_offset_in_ref = 0;   // reference 벡터에서의 offset

  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const int cdim = manager_->contacts[i].contact_dim;

    if (!contacts.contacts[i].active) {
      // 비활성 contact: QP 변수 offset은 진행하지 않음
      // (WQP/HQP에서 active contact만 λ에 포함)
      lambda_offset_in_ref += cdim;
      continue;
    }

    // J_block: λ_i 열에 Identity
    for (int d = 0; d < cdim; ++d) {
      J_block(row + d, lambda_offset_in_z + d) = 1.0;
    }

    // r_block: λ_des_i
    if (lambda_offset_in_ref + cdim <= lambda_ref.size()) {
      r_block.segment(row, cdim) =
          lambda_ref.segment(lambda_offset_in_ref, cdim);
    }

    row += cdim;
    lambda_offset_in_z += cdim;
    lambda_offset_in_ref += cdim;
  }
}

void ForceTask::set_force_reference(
    int contact_index,
    const Eigen::VectorXd& lambda_des) noexcept {
  if (!manager_) return;

  // contact_index에 해당하는 offset 계산
  int offset = 0;
  for (int i = 0; i < contact_index &&
       i < static_cast<int>(manager_->contacts.size()); ++i) {
    offset += manager_->contacts[static_cast<size_t>(i)].contact_dim;
  }

  if (!has_local_ref_) {
    lambda_des_.setZero(max_contact_vars_);
    has_local_ref_ = true;
  }

  const int cdim = manager_->contacts[static_cast<size_t>(contact_index)].contact_dim;
  if (offset + cdim <= lambda_des_.size()) {
    lambda_des_.segment(offset, cdim) = lambda_des.head(cdim);
  }
}

void ForceTask::set_force_references(
    const Eigen::VectorXd& lambda_des_all) noexcept {
  if (lambda_des_.size() < lambda_des_all.size()) {
    return;  // 크기 불일치 → 무시 (RT-safe: 할당 금지)
  }
  lambda_des_.head(lambda_des_all.size()) = lambda_des_all;
  has_local_ref_ = true;
}

void ForceTask::set_contact_manager(
    const ContactManagerConfig* manager) noexcept {
  manager_ = manager;
  if (manager_) {
    max_contact_vars_ = manager_->max_contact_vars;
    lambda_des_.setZero(max_contact_vars_);
  }
}

}  // namespace rtc::tsid
