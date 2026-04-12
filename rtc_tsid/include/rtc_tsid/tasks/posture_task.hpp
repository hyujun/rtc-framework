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
// ────────────────────────────────────────────────
class PostureTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "posture";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int residual_dim() const noexcept override { return nv_; }

  void compute_residual(
      const PinocchioCache& cache,
      const ControlReference& ref,
      const ContactState& contacts,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> J_block,
      Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  // Per-task reference 설정 (external에서 호출)
  void set_reference(const Eigen::VectorXd& q_des,
                     const Eigen::VectorXd& v_des,
                     const Eigen::VectorXd& a_ff) noexcept;

  // PD gains 설정
  void set_gains(const Eigen::VectorXd& kp,
                 const Eigen::VectorXd& kd) noexcept;

 private:
  int nv_{0};

  // PD gains
  Eigen::VectorXd kp_;  // [nv]
  Eigen::VectorXd kd_;  // [nv]

  // Per-task reference (ControlReference의 q_des/v_des/a_des를 사용하거나
  // set_reference()로 별도 주입 가능)
  bool has_local_ref_{false};
  Eigen::VectorXd local_q_des_;
  Eigen::VectorXd local_v_des_;
  Eigen::VectorXd local_a_ff_;
};

}  // namespace rtc::tsid
