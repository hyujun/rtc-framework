#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {

void PostureTask::init(const pinocchio::Model& /*model*/,
                       const RobotModelInfo& robot_info,
                       PinocchioCache& /*cache*/,
                       const YAML::Node& task_config) {
  nv_ = robot_info.nv;

  // Default PD gains
  kp_ = Eigen::VectorXd::Constant(nv_, 10.0);
  kd_ = Eigen::VectorXd::Constant(nv_, 1.0);

  // YAML override
  if (task_config && task_config["kp"]) {
    const auto& kp_node = task_config["kp"];
    if (kp_node.IsScalar()) {
      kp_.setConstant(kp_node.as<double>());
    } else {
      for (int i = 0; i < nv_ && i < static_cast<int>(kp_node.size()); ++i) {
        kp_(i) = kp_node[static_cast<size_t>(i)].as<double>();
      }
    }
  }
  if (task_config && task_config["kd"]) {
    const auto& kd_node = task_config["kd"];
    if (kd_node.IsScalar()) {
      kd_.setConstant(kd_node.as<double>());
    } else {
      for (int i = 0; i < nv_ && i < static_cast<int>(kd_node.size()); ++i) {
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

  // Local reference buffers
  local_q_des_.setZero(robot_info.nq);
  local_v_des_.setZero(nv_);
  local_a_ff_.setZero(nv_);
  has_local_ref_ = false;
}

void PostureTask::compute_residual(
    const PinocchioCache& cache,
    const ControlReference& ref,
    const ContactState& /*contacts*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> J_block,
    Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  // J = [I_{nv} | 0_{nv × n_contact_vars}]
  // J_block은 caller가 zero-init 해줌 → 좌상단 I만 설정
  for (int i = 0; i < nv_; ++i) {
    J_block(i, i) = 1.0;
  }

  // Reference 선택: local이 설정됐으면 local, 아니면 ControlReference
  const auto& q_des = has_local_ref_ ? local_q_des_ : ref.q_des;
  const auto& v_des = has_local_ref_ ? local_v_des_ : ref.v_des;
  const auto& a_ff = has_local_ref_ ? local_a_ff_ : ref.a_des;

  // a_des = Kp·(q_des - q) + Kd·(v_des - v) + a_ff
  // Note: q 공간에서의 차이는 nv 차원
  // Floating-base의 경우 q (nq) ≠ v (nv), pinocchio::difference 필요
  // 여기서는 간단히 head(nv) 사용 (fixed-base에서는 nq==nv)
  const int nq = cache.q.size();
  const int nv = nv_;

  if (nq == nv) {
    // Fixed-base: q_err = q_des - q
    r_block.head(nv) =
        kp_.cwiseProduct(q_des.head(nv) - cache.q.head(nv)) +
        kd_.cwiseProduct(v_des.head(nv) - cache.v.head(nv)) +
        a_ff.head(nv);
  } else {
    // Floating-base: pinocchio::difference(model, q, q_des) for proper SE3 diff
    // 간소화: v 공간에서 직접 차이 계산
    // TODO: pinocchio::difference 사용 (Phase 3에서 floating-base 본격 지원 시)
    r_block.head(nv) =
        kd_.cwiseProduct(v_des.head(nv) - cache.v.head(nv)) +
        a_ff.head(nv);
  }
}

void PostureTask::set_reference(const Eigen::VectorXd& q_des,
                                const Eigen::VectorXd& v_des,
                                const Eigen::VectorXd& a_ff) noexcept {
  local_q_des_ = q_des;
  local_v_des_ = v_des;
  local_a_ff_ = a_ff;
  has_local_ref_ = true;
}

void PostureTask::set_gains(const Eigen::VectorXd& kp,
                            const Eigen::VectorXd& kd) noexcept {
  kp_ = kp;
  kd_ = kd;
}

}  // namespace rtc::tsid
