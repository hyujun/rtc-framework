#include "rtc_tsid/formulation/wqp_formulation.hpp"

#include <algorithm>
#include <limits>

namespace rtc::tsid {

void WQPFormulation::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                          const ContactManagerConfig& contact_cfg, const YAML::Node& config) {
  nv_ = robot_info.nv;
  max_n_vars_ = nv_ + contact_cfg.max_contact_vars;
  contact_cfg_ = contact_cfg;

  // QP rows = Σ MaxEqDim / MaxIneqDim of the constraints added after Init;
  // AddConstraint grows them and re-sizes the QP (init only).
  max_n_eq_ = 0;
  max_n_ineq_ = 0;

  // Solver config — kept for the re-size in AddConstraint.
  solver_cfg_ = QPSolverConfig{};  // max_iter 20 · max_iter_in 100, as before
  if (config && config["wqp"] && config["wqp"]["solver"]) {
    solver_cfg_ = ParseQPSolverConfig(config["wqp"]["solver"], solver_cfg_);
  }
  ResizeQp();

  // Result buffer
  result_.Init(max_n_vars_);

  // Workspace — will be sized on first add_task, or use max_n_vars as default
  max_residual_dim_ = max_n_vars_;
  J_workspace_.setZero(max_residual_dim_, max_n_vars_);
  r_workspace_.setZero(max_residual_dim_);
}

void WQPFormulation::AddTask(std::unique_ptr<TaskBase> task) {
  const int rdim = task->ResidualDim();
  if (rdim > max_residual_dim_) {
    max_residual_dim_ = rdim;
    J_workspace_.setZero(max_residual_dim_, max_n_vars_);
    r_workspace_.setZero(max_residual_dim_);
  }
  tasks_.push_back(std::move(task));
}

void WQPFormulation::AddConstraint(std::unique_ptr<ConstraintBase> constraint) {
  max_n_eq_ += std::max(0, constraint->MaxEqDim(contact_cfg_));
  max_n_ineq_ += std::max(0, constraint->MaxIneqDim(contact_cfg_));
  constraints_.push_back(std::move(constraint));
  ResizeQp();
}

void WQPFormulation::ResizeQp() {
  // A new solver object: also drops the warm start, which is init-time only.
  qp_data_.Init(max_n_vars_, max_n_eq_, max_n_ineq_);
  qp_solver_.Init(max_n_vars_, max_n_eq_, max_n_ineq_, solver_cfg_);
}

TaskBase* WQPFormulation::GetTask(std::string_view name) {
  for (auto& t : tasks_) {
    if (t->Name() == name)
      return t.get();
  }
  return nullptr;
}

ConstraintBase* WQPFormulation::GetConstraint(std::string_view name) {
  for (auto& c : constraints_) {
    if (c->Name() == name)
      return c.get();
  }
  return nullptr;
}

void WQPFormulation::ApplyPreset(const PhasePreset& preset) noexcept {
  for (const auto& tp : preset.task_presets) {
    if (auto* t = GetTask(tp.task_name)) {
      t->SetActive(tp.active);
      t->SetWeight(tp.weight);
      t->SetPriority(tp.priority);
    }
  }
  for (const auto& cp : preset.constraint_presets) {
    if (auto* c = GetConstraint(cp.constraint_name)) {
      c->SetActive(cp.active);
    }
  }
}

const SolveResult& WQPFormulation::Solve(const PinocchioCache& cache, const ControlReference& ref,
                                         const ContactState& contacts,
                                         const RobotModelInfo& robot_info) noexcept {
  // Stage A-5a + RT-safety: fully fixed-dim QP. n_vars = max_n_vars_,
  // n_eq = max_n_eq_, n_ineq = max_n_ineq_ — solver dim 은 변하지 않는다.
  // Inactive task/constraint 는 zero row + (-inf, +inf) bound 로 패딩되어
  // trivially feasible 하게 처리된다. 이로써 QPSolverWrapper 는 매 tick
  // update() 만 호출하면 되고 RT path 에서 heap (re)alloc 이 발생하지 않는다.
  const int n_vars = max_n_vars_;

  // ── Cost: H, g ──
  auto H = qp_data_.H.topLeftCorner(n_vars, n_vars);
  auto g_vec = qp_data_.g.head(n_vars);
  H.setZero();
  g_vec.setZero();

  for (const auto& task : tasks_) {
    if (!task->IsActive())
      continue;

    const int rdim = task->ResidualDim();
    auto J_view = J_workspace_.topLeftCorner(rdim, n_vars);
    auto r_view = r_workspace_.head(rdim);
    J_view.setZero();
    r_view.setZero();

    task->ComputeResidual(cache, ref, contacts, n_vars, J_view, r_view);

    const double w = task->Weight();
    H.noalias() += w * J_view.transpose() * J_view;
    g_vec.noalias() -= w * J_view.transpose() * r_view;
  }

  // Small regularization for numerical stability
  for (int i = 0; i < n_vars; ++i) {
    H(i, i) += 1e-8;
  }

  // ── Equality constraints: A, b (zero-padded to max_n_eq_) ──
  // Inactive 또는 active 영역 밖은 0 = 0 자명식.
  qp_data_.A.topLeftCorner(max_n_eq_, n_vars).setZero();
  qp_data_.b.head(max_n_eq_).setZero();

  // A constraint whose rows do not fit (its EqDim / IneqDim above the
  // MaxEqDim / MaxIneqDim it reported) is skipped and counted; the QP is then
  // not solved — one without those rows is not the problem the caller posed.
  int rows_dropped = 0;
  int n_eq_used = 0;
  for (const auto& con : constraints_) {
    if (!con->IsActive())
      continue;
    const int ed = con->EqDim(contacts);
    if (ed <= 0)
      continue;
    if (n_eq_used + ed > max_n_eq_) {
      rows_dropped += ed;
      continue;
    }

    auto A_view = qp_data_.A.block(n_eq_used, 0, ed, n_vars);
    auto b_view = qp_data_.b.segment(n_eq_used, ed);
    con->ComputeEquality(cache, contacts, robot_info, n_vars, A_view, b_view);
    n_eq_used += ed;
  }

  // ── Inequality constraints: C, l, u (padded to max_n_ineq_) ──
  // Active 영역 밖은 C=0, l=-inf, u=+inf (trivially feasible).
  qp_data_.C.topLeftCorner(max_n_ineq_, n_vars).setZero();
  qp_data_.l.head(max_n_ineq_).setConstant(-std::numeric_limits<double>::infinity());
  qp_data_.u.head(max_n_ineq_).setConstant(std::numeric_limits<double>::infinity());

  int n_ineq_used = 0;
  for (const auto& con : constraints_) {
    if (!con->IsActive())
      continue;
    const int id = con->IneqDim(contacts);
    if (id <= 0)
      continue;
    if (n_ineq_used + id > max_n_ineq_) {
      rows_dropped += id;
      continue;
    }

    auto C_view = qp_data_.C.block(n_ineq_used, 0, id, n_vars);
    auto l_view = qp_data_.l.segment(n_ineq_used, id);
    auto u_view = qp_data_.u.segment(n_ineq_used, id);
    // ComputeInequality 가 l/u 를 덮어쓰므로 padding 의미는 active 영역 밖에서만 유효.
    con->ComputeInequality(cache, contacts, robot_info, n_vars, C_view, l_view, u_view);
    n_ineq_used += id;
  }

  // n_vars / n_eq / n_ineq 는 caller (테스트·로깅) 가 active dim 을 알 수 있도록 기록만.
  // QPSolverWrapper 는 max_n_* 로 고정해 호출한다.
  qp_data_.n_vars = n_vars;
  qp_data_.n_eq = n_eq_used;
  qp_data_.n_ineq = n_ineq_used;
  result_.n_eq_used = n_eq_used;
  result_.n_ineq_used = n_ineq_used;
  result_.rows_dropped = rows_dropped;

  if (rows_dropped > 0) {
    result_.MarkNotSolved(kRowBudgetExceeded);
    return result_;
  }

  // ── Solve ──
  const auto& solver_result = qp_solver_.Solve(qp_data_);

  result_.x_opt.head(n_vars) = solver_result.x_opt.head(n_vars);
  result_.converged = solver_result.converged;
  result_.status = solver_result.status;
  result_.non_finite = solver_result.non_finite;
  result_.solve_time_us = solver_result.solve_time_us;
  result_.iterations = solver_result.iterations;
  result_.outer_iterations = solver_result.outer_iterations;
  result_.levels_solved = 1;

  return result_;
}

void WQPFormulation::Reset() noexcept {
  for (const auto& con : constraints_) {
    con->Reset();
  }
  qp_solver_.ResetWarmStart();
}

void WQPFormulation::OnTorqueApplied(const Eigen::VectorXd& tau) noexcept {
  for (const auto& con : constraints_) {
    con->OnTorqueApplied(tau);
  }
}

}  // namespace rtc::tsid
