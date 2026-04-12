#pragma once

#include <memory>
#include <vector>

#include "rtc_tsid/core/formulation_base.hpp"
#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// HQP (Hierarchical QP) Formulation
//
// Task priority로 level 분류 → level별 순차 QP solve
// 이전 level의 최적해를 equality constraint로 추가 (cascaded QP)
//
// Level k:
//   min Σ_{i∈level_k} wᵢ·‖Jᵢ·z - rᵢ‖²
//   s.t. global constraints
//      + J_prev · z = J_prev · z*_prev  (이전 level 최적 보존)
// ────────────────────────────────────────────────
class HQPFormulation final : public FormulationBase {
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
    return "hqp";
  }

 private:
  // Level별 task 인덱스 재구성 (active task만)
  void rebuild_level_indices() noexcept;

  int nv_{0};
  int max_n_vars_{0};
  int max_levels_{4};

  std::vector<std::unique_ptr<TaskBase>> tasks_;
  std::vector<std::unique_ptr<ConstraintBase>> constraints_;

  // Level별 task 인덱스 (rebuild_level_indices()로 갱신)
  // level_task_indices_[k] = {task index들}
  std::vector<std::vector<int>> level_task_indices_;
  int active_max_level_{0};

  // Level별 QP data + solver (pre-allocated)
  std::vector<QPData> qp_data_per_level_;
  std::vector<QPSolverWrapper> qp_solvers_;

  // 이전 level Jacobian stack (cascaded equality)
  Eigen::MatrixXd J_prev_stack_;      // [max_prev_rows × max_n_vars]
  Eigen::VectorXd Jz_prev_stack_;     // [max_prev_rows]
  int max_prev_rows_{0};

  // Task residual workspace
  Eigen::MatrixXd J_workspace_;
  Eigen::VectorXd r_workspace_;
  int max_residual_dim_{0};

  // Max constraint dimensions
  int max_n_eq_base_{0};
  int max_n_ineq_{0};

  SolveResult result_;
};

}  // namespace rtc::tsid
