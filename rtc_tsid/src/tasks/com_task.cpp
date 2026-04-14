#include "rtc_tsid/tasks/com_task.hpp"

namespace rtc::tsid {

void CoMTask::init(const pinocchio::Model& /*model*/,
                   const RobotModelInfo& robot_info,
                   PinocchioCache& cache,
                   const YAML::Node& task_config) {
  nv_ = robot_info.nv;

  // CoM 계산 활성화
  cache.compute_com = true;

  // Default PD gains
  kp_ = Eigen::Vector3d::Constant(100.0);
  kd_ = Eigen::Vector3d::Constant(20.0);

  if (task_config && task_config["kp"]) {
    const auto& kp_node = task_config["kp"];
    if (kp_node.IsScalar()) {
      kp_.setConstant(kp_node.as<double>());
    } else {
      for (int i = 0; i < 3 && i < static_cast<int>(kp_node.size()); ++i) {
        kp_(i) = kp_node[static_cast<size_t>(i)].as<double>();
      }
    }
  }
  if (task_config && task_config["kd"]) {
    const auto& kd_node = task_config["kd"];
    if (kd_node.IsScalar()) {
      kd_.setConstant(kd_node.as<double>());
    } else {
      for (int i = 0; i < 3 && i < static_cast<int>(kd_node.size()); ++i) {
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

  // Reference / workspace 초기화
  has_local_ref_ = false;
  com_des_.setZero();
  v_des_.setZero();
  a_ff_.setZero();
  e_pos_.setZero();
  e_vel_.setZero();
  a_des_.setZero();
}

void CoMTask::compute_residual(
    const PinocchioCache& cache,
    const ControlReference& /*ref*/,
    const ContactState& /*contacts*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> J_block,
    Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  // J_block = [J_com | 0]  (3 × n_vars)
  J_block.leftCols(nv_) = cache.Jcom;

  // 위치/속도 오차
  e_pos_ = com_des_ - cache.com_position;
  e_vel_.noalias() = v_des_ - cache.Jcom * cache.v;

  // desired acceleration
  a_des_ = a_ff_ + kp_.cwiseProduct(e_pos_) + kd_.cwiseProduct(e_vel_);

  // r_block = a_des - com_drift
  // QP: min ‖J_com · a - r_block‖²
  // J_com · a = a_des - com_drift → a가 이 drift를 보상
  r_block.head(3) = a_des_ - cache.com_drift;
}

void CoMTask::set_com_reference(
    const Eigen::Vector3d& com_des,
    const Eigen::Vector3d& v_des,
    const Eigen::Vector3d& a_ff) noexcept {
  com_des_ = com_des;
  v_des_ = v_des;
  a_ff_ = a_ff;
  has_local_ref_ = true;
}

void CoMTask::set_gains(const Eigen::Vector3d& kp,
                        const Eigen::Vector3d& kd) noexcept {
  kp_ = kp;
  kd_ = kd;
}

}  // namespace rtc::tsid
