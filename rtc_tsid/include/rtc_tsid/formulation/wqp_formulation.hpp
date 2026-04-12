#pragma once

#include <memory>
#include <vector>

#include "rtc_tsid/core/formulation_base.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// WQP (Weighted QP) Formulation
//
// 모든 task를 weighted sum으로 단일 QP에 조립
// priority() 무시, weight()만 사용
// ────────────────────────────────────────────────
class WQPFormulation final : public FormulationBase {
 public:
  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            const ContactManagerConfig& contact_cfg,
            const YAML::Node& config) override;

  void add_task(std::unique_ptr<TaskBase> task) override;
  void add_constraint(std::unique_ptr<ConstraintBase> constraint) override;

  [[nodiscard]] TaskBase* get_task(std::string_view name) override;
  [[nodiscard]] ConstraintBase* get_constraint(std::string_view name) override;

  void apply_preset(const PhasePreset& preset) noexcept override;

  [[nodiscard]] const SolveResult& solve(
      const PinocchioCache& cache,
      const ControlReference& ref,
      const ContactState& contacts,
      const RobotModelInfo& robot_info) noexcept override;

  [[nodiscard]] std::string_view type() const noexcept override {
    return "wqp";
  }

 private:
  int nv_{0};
  int max_n_vars_{0};
  int max_n_eq_{0};
  int max_n_ineq_{0};

  std::vector<std::unique_ptr<TaskBase>> tasks_;
  std::vector<std::unique_ptr<ConstraintBase>> constraints_;

  QPData qp_data_;
  QPSolverWrapper qp_solver_;
  SolveResult result_;

  // Task residual workspace
  Eigen::MatrixXd J_workspace_;
  Eigen::VectorXd r_workspace_;
  int max_residual_dim_{0};
};

}  // namespace rtc::tsid
