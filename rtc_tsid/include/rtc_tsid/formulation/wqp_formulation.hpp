#pragma once

#include "rtc_tsid/core/formulation_base.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

#include <memory>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// WQP (Weighted QP) Formulation
//
// 모든 task를 weighted sum으로 단일 QP에 조립
// Priority() 무시, Weight()만 사용
// ────────────────────────────────────────────────
class WQPFormulation final : public FormulationBase {
 public:
  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
            const ContactManagerConfig& contact_cfg, const YAML::Node& config) override;

  void AddTask(std::unique_ptr<TaskBase> task) override;
  void AddConstraint(std::unique_ptr<ConstraintBase> constraint) override;

  [[nodiscard]] TaskBase* GetTask(std::string_view name) override;
  [[nodiscard]] ConstraintBase* GetConstraint(std::string_view name) override;

  void ApplyPreset(const PhasePreset& preset) noexcept override;

  [[nodiscard]] const SolveResult& Solve(const PinocchioCache& cache, const ControlReference& ref,
                                         const ContactState& contacts,
                                         const RobotModelInfo& robot_info) noexcept override;

  [[nodiscard]] std::string_view Type() const noexcept override { return "wqp"; }

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
