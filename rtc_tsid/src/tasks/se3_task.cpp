#include "rtc_tsid/tasks/se3_task.hpp"

#include <cmath>

namespace rtc::tsid {

void SE3Task::init(const pinocchio::Model& model,
                   const RobotModelInfo& robot_info,
                   PinocchioCache& cache,
                   const YAML::Node& task_config) {
  nv_ = robot_info.nv;

  // Task name (YAML override 가능)
  if (task_config && task_config["name"]) {
    name_ = task_config["name"].as<std::string>();
  }

  // Frame 등록
  const auto frame_name = task_config["frame"].as<std::string>();
  const auto frame_id = model.getFrameId(frame_name);
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
    // init 단계에서만 호출되므로 exception 허용
    throw std::runtime_error(
        "SE3Task: frame '" + frame_name + "' not found in model");
  }
  registered_frame_idx_ = cache.register_frame(frame_name, frame_id);

  // Mask (default: 6D 전부 활성)
  mask_.fill(true);
  active_dim_ = 6;
  if (task_config && task_config["mask"]) {
    const auto& mask_node = task_config["mask"];
    active_dim_ = 0;
    for (int i = 0; i < 6 && i < static_cast<int>(mask_node.size()); ++i) {
      mask_[static_cast<size_t>(i)] = (mask_node[static_cast<size_t>(i)].as<int>() != 0);
      if (mask_[static_cast<size_t>(i)]) ++active_dim_;
    }
  }

  // PD gains (default)
  kp_ = Eigen::Matrix<double, 6, 1>::Constant(100.0);
  kd_ = Eigen::Matrix<double, 6, 1>::Constant(20.0);

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

  // Weight / priority
  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  // Reference 초기화
  v_des_.setZero();
  a_ff_.setZero();

  // Workspace 초기화
  error_full_.setZero();
  v_error_full_.setZero();
  a_des_full_.setZero();
}

void SE3Task::compute_residual(
    const PinocchioCache& cache,
    const ControlReference& /*ref*/,
    const ContactState& /*contacts*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> J_block,
    Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  const auto& rf = cache.registered_frames[static_cast<size_t>(registered_frame_idx_)];

  // ── 위치 오차 ──
  error_full_.head<3>() = placement_des_.translation() - rf.oMf.translation();

  // ── 자세 오차: log3(R_currᵀ · R_des) ──
  // θ → π singularity 보호
  const Eigen::Matrix3d R_err =
      rf.oMf.rotation().transpose() * placement_des_.rotation();
  const Eigen::AngleAxisd aa(R_err);
  const double angle = aa.angle();

  if (angle < M_PI - 1e-4) {
    error_full_.tail<3>() = pinocchio::log3(R_err);
  } else {
    // π 근처: 방향 유지, 크기 clamp
    error_full_.tail<3>() = aa.axis() * (M_PI - 1e-4);
  }

  // ── 속도 오차: v_des - J · v ──
  v_error_full_.noalias() = v_des_ - rf.J * cache.v;

  // ── desired acceleration: a_ff + Kp·e_pos + Kd·e_vel ──
  a_des_full_ = a_ff_ + kp_.cwiseProduct(error_full_) +
                kd_.cwiseProduct(v_error_full_);

  // ── QP: J·a = a_des - dJ·v  →  J_block = J, r_block = a_des - dJv ──
  // Mask 적용: 활성 축만 추출
  int row = 0;
  for (int i = 0; i < 6; ++i) {
    if (!mask_[static_cast<size_t>(i)]) continue;
    J_block.row(row) = rf.J.row(i).head(nv_);
    r_block(row) = a_des_full_(i) - rf.dJv(i);
    ++row;
  }
}

void SE3Task::set_se3_reference(
    const pinocchio::SE3& placement_des,
    const Eigen::Matrix<double, 6, 1>& v_des,
    const Eigen::Matrix<double, 6, 1>& a_ff) noexcept {
  placement_des_ = placement_des;
  v_des_ = v_des;
  a_ff_ = a_ff;
}

void SE3Task::set_gains(
    const Eigen::Matrix<double, 6, 1>& kp,
    const Eigen::Matrix<double, 6, 1>& kd) noexcept {
  kp_ = kp;
  kd_ = kd;
}

}  // namespace rtc::tsid
