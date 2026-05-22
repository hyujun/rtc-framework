#include "rtc_tsid/formulation/hqp_formulation.hpp"

#include <limits>

namespace rtc::tsid {

void HQPFormulation::Init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                          const ContactManagerConfig& contact_cfg, const YAML::Node& config) {
  nv_ = robot_info.nv;
  max_n_vars_ = nv_ + contact_cfg.max_contact_vars;

  if (config && config["hqp"] && config["hqp"]["max_levels"]) {
    max_levels_ = config["hqp"]["max_levels"].as<int>();
  }

  // Base equality: EoM + contacts
  const int eom_eq = robot_info.floating_base ? (robot_info.nv - robot_info.n_actuated) : 0;
  max_n_eq_base_ = eom_eq + contact_cfg.max_contact_vars + 16;

  // Max inequality — point: friction_faces + 1; surface: + 6 (CoP + yaw).
  int max_friction = 0;
  for (const auto& c : contact_cfg.contacts) {
    int per_contact = c.friction_faces + 1;
    if (c.contact_dim == 6) {
      per_contact += 6;
    }
    max_friction += per_contact;
  }
  max_n_ineq_ = robot_info.n_actuated + max_friction + 16;

  // Max previous-level equality rows = sum of all task residual dims
  // Conservative: max_n_vars * max_levels
  max_prev_rows_ = max_n_vars_ * max_levels_;
  // Total max equality per level = base + prev stack
  max_n_eq_per_level_ = max_n_eq_base_ + max_prev_rows_;

  // Solver config
  QPSolverConfig solver_cfg;
  if (config && config["hqp"] && config["hqp"]["solver_per_level"]) {
    const auto& sc = config["hqp"]["solver_per_level"];
    solver_cfg.max_iter = sc["max_iter"].as<int>(10);
    solver_cfg.eps_abs = sc["eps_abs"].as<double>(1e-6);
    solver_cfg.eps_rel = sc["eps_rel"].as<double>(0.0);
    solver_cfg.verbose = sc["verbose"].as<bool>(false);
  }

  // Pre-allocate per-level resources
  qp_data_per_level_.resize(static_cast<size_t>(max_levels_));
  qp_solvers_.resize(static_cast<size_t>(max_levels_));
  for (int k = 0; k < max_levels_; ++k) {
    qp_data_per_level_[static_cast<size_t>(k)].Init(max_n_vars_, max_n_eq_per_level_, max_n_ineq_);
    qp_solvers_[static_cast<size_t>(k)].Init(max_n_vars_, max_n_eq_per_level_, max_n_ineq_,
                                             solver_cfg);
  }

  // Level index storage
  level_task_indices_.resize(static_cast<size_t>(max_levels_));

  // Cascaded equality buffer
  J_prev_stack_.setZero(max_prev_rows_, max_n_vars_);
  Jz_prev_stack_.setZero(max_prev_rows_);

  // Task workspace
  max_residual_dim_ = max_n_vars_;
  J_workspace_.setZero(max_residual_dim_, max_n_vars_);
  r_workspace_.setZero(max_residual_dim_);

  result_.Init(max_n_vars_);
}

void HQPFormulation::AddTask(std::unique_ptr<TaskBase> task) {
  const int rdim = task->ResidualDim();
  if (rdim > max_residual_dim_) {
    max_residual_dim_ = rdim;
    J_workspace_.setZero(max_residual_dim_, max_n_vars_);
    r_workspace_.setZero(max_residual_dim_);
  }
  tasks_.push_back(std::move(task));
}

void HQPFormulation::AddConstraint(std::unique_ptr<ConstraintBase> constraint) {
  constraints_.push_back(std::move(constraint));
}

TaskBase* HQPFormulation::GetTask(std::string_view name) {
  for (auto& t : tasks_) {
    if (t->Name() == name)
      return t.get();
  }
  return nullptr;
}

ConstraintBase* HQPFormulation::GetConstraint(std::string_view name) {
  for (auto& c : constraints_) {
    if (c->Name() == name)
      return c.get();
  }
  return nullptr;
}

void HQPFormulation::ApplyPreset(const PhasePreset& preset) noexcept {
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

void HQPFormulation::RebuildLevelIndices() noexcept {
  for (auto& v : level_task_indices_) {
    v.clear();
  }
  active_max_level_ = -1;

  for (int i = 0; i < static_cast<int>(tasks_.size()); ++i) {
    if (!tasks_[static_cast<size_t>(i)]->IsActive())
      continue;
    const int prio = tasks_[static_cast<size_t>(i)]->Priority();
    if (prio >= 0 && prio < max_levels_) {
      level_task_indices_[static_cast<size_t>(prio)].push_back(i);
      if (prio > active_max_level_) {
        active_max_level_ = prio;
      }
    }
  }
}

const SolveResult& HQPFormulation::Solve(const PinocchioCache& cache, const ControlReference& ref,
                                         const ContactState& contacts,
                                         const RobotModelInfo& robot_info) noexcept {
  // Stage A-5a: fixed-dim QP (see WQPFormulation::Solve for rationale).
  const int n_vars = max_n_vars_;

  // Rebuild level indices from current task active/priority state
  RebuildLevelIndices();

  if (active_max_level_ < 0) {
    // No active tasks
    result_.converged = false;
    result_.levels_solved = 0;
    return result_;
  }

  // ── Compute base constraints (shared across all levels) ──
  // Use level 0's QPData as scratch for base constraints.
  // Fixed-dim padding: A/b zero, C=0/l=-inf/u=+inf 로 max 영역 전체 초기화.
  // QPSolverWrapper 가 항상 max-dim 으로 update() 하므로 active 영역 밖도 trivially feasible 해야
  // 함.
  auto& base_qp = qp_data_per_level_[0];
  base_qp.A.topLeftCorner(max_n_eq_per_level_, n_vars).setZero();
  base_qp.b.head(max_n_eq_per_level_).setZero();
  base_qp.C.topLeftCorner(max_n_ineq_, n_vars).setZero();
  base_qp.l.head(max_n_ineq_).setConstant(-std::numeric_limits<double>::infinity());
  base_qp.u.head(max_n_ineq_).setConstant(std::numeric_limits<double>::infinity());

  int base_n_eq = 0;
  for (const auto& con : constraints_) {
    if (!con->IsActive())
      continue;
    const int ed = con->EqDim(contacts);
    if (ed <= 0)
      continue;
    if (base_n_eq + ed > max_n_eq_per_level_)
      break;

    auto A_view = base_qp.A.block(base_n_eq, 0, ed, n_vars);
    auto b_view = base_qp.b.segment(base_n_eq, ed);
    con->ComputeEquality(cache, contacts, robot_info, n_vars, A_view, b_view);
    base_n_eq += ed;
  }

  int base_n_ineq = 0;
  for (const auto& con : constraints_) {
    if (!con->IsActive())
      continue;
    const int id = con->IneqDim(contacts);
    if (id <= 0)
      continue;
    if (base_n_ineq + id > max_n_ineq_)
      break;

    auto C_view = base_qp.C.block(base_n_ineq, 0, id, n_vars);
    auto l_view = base_qp.l.segment(base_n_ineq, id);
    auto u_view = base_qp.u.segment(base_n_ineq, id);
    con->ComputeInequality(cache, contacts, robot_info, n_vars, C_view, l_view, u_view);
    base_n_ineq += id;
  }

  // ── Cascaded QP solve ──
  int prev_stack_rows = 0;
  int levels_solved = 0;
  result_.converged = false;

  for (int k = 0; k <= active_max_level_ && k < max_levels_; ++k) {
    const auto& task_indices = level_task_indices_[static_cast<size_t>(k)];
    if (task_indices.empty())
      continue;

    auto& qp = qp_data_per_level_[static_cast<size_t>(k)];

    // Fixed-dim padding (QPSolverWrapper 는 항상 max-dim 으로 update 함):
    // 매 level QP buffer 의 max 영역 전체를 trivial 값으로 초기화.
    // Active region (base + cascaded) 은 아래에서 덮어쓴다.
    if (k > 0) {
      qp.A.topLeftCorner(max_n_eq_per_level_, n_vars).setZero();
      qp.b.head(max_n_eq_per_level_).setZero();
      qp.C.topLeftCorner(max_n_ineq_, n_vars).setZero();
      qp.l.head(max_n_ineq_).setConstant(-std::numeric_limits<double>::infinity());
      qp.u.head(max_n_ineq_).setConstant(std::numeric_limits<double>::infinity());
    } else {
      // Level 0: base_qp 가 이미 padding 됐지만, prev_stack_rows 가 0 인 region 위쪽 (cascaded)
      // 은 base_qp 자체 padding 으로 covered. 추가 작업 불필요.
    }

    // ── Cost: H_k, g_k from level k tasks ──
    auto H = qp.H.topLeftCorner(n_vars, n_vars);
    auto g_vec = qp.g.head(n_vars);
    H.setZero();
    g_vec.setZero();

    // Collect level-k J rows for cascaded equality (store for next level)
    int level_J_rows = 0;

    for (const int ti : task_indices) {
      const auto& task = tasks_[static_cast<size_t>(ti)];
      const int rdim = task->ResidualDim();

      auto J_view = J_workspace_.topLeftCorner(rdim, n_vars);
      auto r_view = r_workspace_.head(rdim);
      J_view.setZero();
      r_view.setZero();

      task->ComputeResidual(cache, ref, contacts, n_vars, J_view, r_view);

      const double w = task->Weight();
      H.noalias() += w * J_view.transpose() * J_view;
      g_vec.noalias() -= w * J_view.transpose() * r_view;

      // Store J rows for cascaded equality at next level
      if (prev_stack_rows + level_J_rows + rdim <= max_prev_rows_) {
        J_prev_stack_.block(prev_stack_rows + level_J_rows, 0, rdim, n_vars) = J_view;
        level_J_rows += rdim;
      }
    }

    // Regularization
    for (int i = 0; i < n_vars; ++i) {
      H(i, i) += 1e-8;
    }

    // ── Equality: base constraints + cascaded from previous levels ──
    int n_eq = base_n_eq + prev_stack_rows;

    // Copy base constraints into this level's QPData
    if (k > 0) {
      qp.A.topLeftCorner(base_n_eq, n_vars) = base_qp.A.topLeftCorner(base_n_eq, n_vars);
      qp.b.head(base_n_eq) = base_qp.b.head(base_n_eq);
    }

    // Add cascaded equality from previous levels: J_prev · z = J_prev · z*_prev
    if (prev_stack_rows > 0) {
      qp.A.block(base_n_eq, 0, prev_stack_rows, n_vars) =
          J_prev_stack_.topLeftCorner(prev_stack_rows, n_vars);
      qp.b.segment(base_n_eq, prev_stack_rows) = Jz_prev_stack_.head(prev_stack_rows);
    }

    // ── Inequality (same for all levels) ──
    if (k > 0) {
      qp.C.topLeftCorner(base_n_ineq, n_vars) = base_qp.C.topLeftCorner(base_n_ineq, n_vars);
      qp.l.head(base_n_ineq) = base_qp.l.head(base_n_ineq);
      qp.u.head(base_n_ineq) = base_qp.u.head(base_n_ineq);
    }

    qp.n_vars = n_vars;
    qp.n_eq = n_eq;
    qp.n_ineq = base_n_ineq;

    // ── Solve level k ──
    const auto& solver_result = qp_solvers_[static_cast<size_t>(k)].Solve(qp);

    if (!solver_result.converged) {
      // Infeasible → 직전 level z* 사용
      break;
    }

    // Store solution
    result_.x_opt.head(n_vars) = solver_result.x_opt.head(n_vars);
    result_.converged = true;
    result_.solve_time_us += solver_result.solve_time_us;
    result_.iterations += solver_result.iterations;
    ++levels_solved;

    // Update J_prev · z* for next level cascaded equality
    // Jz_prev_stack_[prev_stack_rows : prev_stack_rows + level_J_rows]
    //   = J_level_k · z*_k
    if (level_J_rows > 0 && prev_stack_rows + level_J_rows <= max_prev_rows_) {
      Jz_prev_stack_.segment(prev_stack_rows, level_J_rows) =
          J_prev_stack_.block(prev_stack_rows, 0, level_J_rows, n_vars) *
          solver_result.x_opt.head(n_vars);
      prev_stack_rows += level_J_rows;
    }
  }

  result_.levels_solved = levels_solved;
  return result_;
}

}  // namespace rtc::tsid
