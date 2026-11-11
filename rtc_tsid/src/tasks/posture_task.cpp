#include "rtc_tsid/tasks/posture_task.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

namespace rtc::tsid {

void PostureTask::Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                       PinocchioCache& /*cache*/, const YAML::Node& task_config) {
  nv_ = robot_info.nv;
  n_base_ = robot_info.floating_base ? nv_ - robot_info.n_actuated : 0;
  const int na = nv_ - n_base_;
  // The actuated q entries follow the root's: 7 on a free-flyer, none fixed.
  const int nq_base = robot_info.floating_base ? 7 : 0;
  position_error_ = (robot_info.nq - nq_base == na);

  // Default PD gains — the base rows (if any) get none.
  kp_ = Eigen::VectorXd::Constant(nv_, 10.0);
  kd_ = Eigen::VectorXd::Constant(nv_, 1.0);
  kp_.head(n_base_).setZero();
  kd_.head(n_base_).setZero();

  // YAML override: a scalar sets the actuated entries, a list sets them all.
  const auto apply = [&](const YAML::Node& node, Eigen::VectorXd& gain) {
    if (node.IsScalar()) {
      gain.tail(na).setConstant(node.as<double>());
    } else {
      for (int i = 0; i < nv_ && i < static_cast<int>(node.size()); ++i) {
        gain(i) = node[static_cast<size_t>(i)].as<double>();
      }
    }
  };
  if (task_config && task_config["kp"]) {
    apply(task_config["kp"], kp_);
  }
  if (task_config && task_config["kd"]) {
    apply(task_config["kd"], kd_);
  }

  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  // Local reference buffers — a valid configuration, not a zero quaternion.
  local_q_des_ = pinocchio::neutral(model);
  local_v_des_.setZero(nv_);
  local_a_ff_.setZero(nv_);
  has_local_ref_ = false;
}

void PostureTask::ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                                  const ContactState& /*contacts*/, int /*n_vars*/,
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

  // a_des = Kp·(q_des − q) + Kd·(v_des − v) + a_ff. The position error is on
  // the actuated joints only (q.tail(na) ↔ v.tail(na)); the base rows, if
  // any, have no Kp term.
  // Same expression order as the fixed-base formula, so a fixed base (no base
  // rows) is unchanged bit for bit.
  const int nv = nv_;
  if (!position_error_) {
    r_block.head(nv) = kd_.cwiseProduct(v_des.head(nv) - cache.v.head(nv)) + a_ff.head(nv);
    return;
  }
  const int nb = n_base_;
  const int na = nv - nb;
  r_block.head(nb) = kd_.head(nb).cwiseProduct(v_des.head(nb) - cache.v.head(nb)) + a_ff.head(nb);
  r_block.tail(na) = kp_.tail(na).cwiseProduct(q_des.tail(na) - cache.q.tail(na)) +
                     kd_.tail(na).cwiseProduct(v_des.tail(na) - cache.v.tail(na)) + a_ff.tail(na);
}

bool PostureTask::SetReference(const Eigen::VectorXd& q_des, const Eigen::VectorXd& v_des,
                               const Eigen::VectorXd& a_ff) noexcept {
  if (q_des.size() != local_q_des_.size() || v_des.size() != nv_ || a_ff.size() != nv_ ||
      !q_des.allFinite() || !v_des.allFinite() || !a_ff.allFinite()) {
    return false;
  }
  local_q_des_ = q_des;
  local_v_des_ = v_des;
  local_a_ff_ = a_ff;
  has_local_ref_ = true;
  return true;
}

void PostureTask::SetGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd) noexcept {
  kp_ = kp;
  kd_ = kd;
}

}  // namespace rtc::tsid
