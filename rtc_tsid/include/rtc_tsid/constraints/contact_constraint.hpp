#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Contact acceleration zero constraint
//
// 각 active contact i: Jc_i · a + dJc_i · v = 0
//   point: Jc_i = J[:3, :] → 3행
//   surface: Jc_i = J[:6, :] → 6행
//
// eq_dim = Σ active_contact_dim_i
// ineq_dim = 0
// ────────────────────────────────────────────────
class ContactConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "contact_accel";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int eq_dim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int ineq_dim(const ContactState& contacts) const noexcept override;

  void compute_equality(
      const PinocchioCache& cache,
      const ContactState& contacts,
      const RobotModelInfo& robot_info,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> A_block,
      Eigen::Ref<Eigen::VectorXd> b_block) noexcept override;

  void compute_inequality(
      const PinocchioCache& cache,
      const ContactState& contacts,
      const RobotModelInfo& robot_info,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> C_block,
      Eigen::Ref<Eigen::VectorXd> l_block,
      Eigen::Ref<Eigen::VectorXd> u_block) noexcept override;

  // ContactManagerConfig 참조 설정 (init 후 호출)
  void set_contact_manager(const ContactManagerConfig* manager) noexcept {
    manager_ = manager;
  }

 private:
  int nv_{0};
  const ContactManagerConfig* manager_{nullptr};
};

}  // namespace rtc::tsid
