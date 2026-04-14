#pragma once

#include "rtc_tsid/core/task_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Center of Mass position tracking task
//
// cost = w * ‖J_com · a - r_des‖²
//
// e_pos = com_des - com_curr                      [3]
// e_vel = v_com_des - J_com · v                   [3]
// a_des = a_ff + Kp·e_pos + Kd·e_vel              [3]
//
// J_block = J_com   [3 × nv], λ 열은 0
// r_block = a_des - com_drift
// residual_dim = 3
// ────────────────────────────────────────────────
class CoMTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "com";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int residual_dim() const noexcept override { return 3; }

  void compute_residual(
      const PinocchioCache& cache,
      const ControlReference& ref,
      const ContactState& contacts,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> J_block,
      Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  /// @brief CoM reference 설정 (RT-safe)
  void set_com_reference(
      const Eigen::Vector3d& com_des,
      const Eigen::Vector3d& v_des = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& a_ff = Eigen::Vector3d::Zero()) noexcept;

  /// @brief PD gains 설정 (RT-safe)
  void set_gains(const Eigen::Vector3d& kp,
                 const Eigen::Vector3d& kd) noexcept;

 private:
  int nv_{0};

  // PD gains
  Eigen::Vector3d kp_;
  Eigen::Vector3d kd_;

  // Reference
  bool has_local_ref_{false};
  Eigen::Vector3d com_des_;
  Eigen::Vector3d v_des_;
  Eigen::Vector3d a_ff_;

  // Pre-allocated workspace
  Eigen::Vector3d e_pos_;
  Eigen::Vector3d e_vel_;
  Eigen::Vector3d a_des_;
};

}  // namespace rtc::tsid
