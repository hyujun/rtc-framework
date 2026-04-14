#include "rtc_tsid/tasks/momentum_task.hpp"

namespace rtc::tsid {

void MomentumTask::init(const pinocchio::Model& /*model*/,
                        const RobotModelInfo& robot_info,
                        PinocchioCache& cache,
                        const YAML::Node& task_config) {
  nv_ = robot_info.nv;

  // Centroidal momentum 계산 활성화
  cache.compute_centroidal = true;

  // Mode 파싱
  mode_ = Mode::kAngularRegularize;
  if (task_config && task_config["mode"]) {
    const auto mode_str = task_config["mode"].as<std::string>();
    if (mode_str == "full_track") {
      mode_ = Mode::kFullTrack;
    }
    // "angular_regularize" 또는 기타 → default
  }

  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  hg_dot_des_.setZero();
}

void MomentumTask::compute_residual(
    const PinocchioCache& cache,
    const ControlReference& /*ref*/,
    const ContactState& /*contacts*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> J_block,
    Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  if (mode_ == Mode::kAngularRegularize) {
    // Angular momentum → 0: Ag_angular · a + hg_drift_angular = 0
    // J_block = Ag[3:6, :nv], r_block = -hg_drift[3:6]
    J_block.leftCols(nv_) = cache.Ag.bottomRows(3);
    r_block.head(3) = -cache.hg_drift.tail(3);
  } else {
    // Full track: Ag · a = hg_dot_des - hg_drift
    // J_block = Ag[:, :nv], r_block = hg_dot_des - hg_drift
    J_block.leftCols(nv_) = cache.Ag;
    r_block.head(6) = hg_dot_des_ - cache.hg_drift;
  }
}

void MomentumTask::set_momentum_reference(
    const Eigen::Matrix<double, 6, 1>& hg_dot_des) noexcept {
  hg_dot_des_ = hg_dot_des;
}

}  // namespace rtc::tsid
