#pragma once

#include "rtc_tsid/core/task_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Joint-space acceleration tracking task
//
// cost = w * ‖a - a_des‖²
// a_des = Kp*(q_des - q) + Kd*(v_des - v) + a_ff
//
// J_block = [I_{nv} | 0_{nv × n_contact_vars}]
// r_block = a_des
// residual_dim = nv
//
// Floating base (free-flyer root, every other joint actuated and 1-DoF): the
// position error is taken on the actuated joints only, q_des.tail(na) −
// q.tail(na); the six base rows carry the Kd and a_ff terms but no Kp term
// (base pose tracking is not this task's). Base Kp / Kd default to 0, a
// scalar YAML `kp` / `kd` sets the actuated entries, an nv-long list sets all
// (its base Kp entries are unused). The local q_des starts at
// pinocchio::neutral (a unit quaternion).
// ────────────────────────────────────────────────
class PostureTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "posture"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int ResidualDim() const noexcept override { return nv_; }

  void ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                       const ContactState& contacts, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> J_block,
                       Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  // Per-task reference 설정 (external에서 호출). Sizes nq / nv / nv and finite
  // values, or it is refused (false) and the previous reference stays. Copies
  // into the buffers Init sized — no allocation.
  [[nodiscard]] bool SetReference(const Eigen::VectorXd& q_des, const Eigen::VectorXd& v_des,
                                  const Eigen::VectorXd& a_ff) noexcept;

  // PD gains 설정
  void SetGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd) noexcept;

 private:
  int nv_{0};
  int n_base_{0};  // rows without a position error: 6 on a floating base, else 0
  // q.tail(nv − n_base) and v.tail(nv − n_base) pair up joint by joint (every
  // non-base joint is 1-DoF). When not, only the Kd and a_ff terms are used.
  bool position_error_{true};

  // PD gains
  Eigen::VectorXd kp_;  // [nv]
  Eigen::VectorXd kd_;  // [nv]

  // Per-task reference (ControlReference의 q_des/v_des/a_des를 사용하거나
  // SetReference()로 별도 주입 가능)
  bool has_local_ref_{false};
  Eigen::VectorXd local_q_des_;
  Eigen::VectorXd local_v_des_;
  Eigen::VectorXd local_a_ff_;
};

}  // namespace rtc::tsid
