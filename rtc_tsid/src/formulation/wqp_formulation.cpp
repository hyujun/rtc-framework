#include "rtc_tsid/formulation/wqp_formulation.hpp"

namespace rtc::tsid {

void WQPFormulation::init(const pinocchio::Model & /*model*/,
                          const RobotModelInfo &robot_info,
                          const ContactManagerConfig &contact_cfg,
                          const YAML::Node &config) {
  nv_ = robot_info.nv;
  max_n_vars_ = nv_ + contact_cfg.max_contact_vars;

  // Max equality: EoM (floating-base: nv-na) + contacts (max_contact_vars)
  // + 여유분
  const int eom_eq =
      robot_info.floating_base ? (robot_info.nv - robot_info.n_actuated) : 0;
  max_n_eq_ = eom_eq + contact_cfg.max_contact_vars + 16;

  // Max inequality: torque limits (n_actuated) + friction cones
  // (rough upper bound: max_contacts * (friction_faces + 1) + n_actuated)
  int max_friction = 0;
  for (const auto &c : contact_cfg.contacts) {
    max_friction += c.friction_faces + 1;
  }
  max_n_ineq_ = robot_info.n_actuated + max_friction + 16;

  // QP data pre-allocate
  qp_data_.init(max_n_vars_, max_n_eq_, max_n_ineq_);

  // Solver init
  QPSolverConfig solver_cfg;
  if (config && config["wqp"] && config["wqp"]["solver"]) {
    const auto &sc = config["wqp"]["solver"];
    solver_cfg.max_iter = sc["max_iter"].as<int>(20);
    solver_cfg.eps_abs = sc["eps_abs"].as<double>(1e-6);
    solver_cfg.eps_rel = sc["eps_rel"].as<double>(0.0);
    solver_cfg.verbose = sc["verbose"].as<bool>(false);
  }
  qp_solver_.init(max_n_vars_, max_n_eq_, max_n_ineq_, solver_cfg);

  // Result buffer
  result_.init(max_n_vars_);

  // Workspace — will be sized on first add_task, or use max_n_vars as default
  max_residual_dim_ = max_n_vars_;
  J_workspace_.setZero(max_residual_dim_, max_n_vars_);
  r_workspace_.setZero(max_residual_dim_);
}

void WQPFormulation::add_task(std::unique_ptr<TaskBase> task) {
  const int rdim = task->residual_dim();
  if (rdim > max_residual_dim_) {
    max_residual_dim_ = rdim;
    J_workspace_.setZero(max_residual_dim_, max_n_vars_);
    r_workspace_.setZero(max_residual_dim_);
  }
  tasks_.push_back(std::move(task));
}

void WQPFormulation::add_constraint(
    std::unique_ptr<ConstraintBase> constraint) {
  constraints_.push_back(std::move(constraint));
}

TaskBase *WQPFormulation::get_task(std::string_view name) {
  for (auto &t : tasks_) {
    if (t->name() == name)
      return t.get();
  }
  return nullptr;
}

ConstraintBase *WQPFormulation::get_constraint(std::string_view name) {
  for (auto &c : constraints_) {
    if (c->name() == name)
      return c.get();
  }
  return nullptr;
}

void WQPFormulation::apply_preset(const PhasePreset &preset) noexcept {
  for (const auto &tp : preset.task_presets) {
    if (auto *t = get_task(tp.task_name)) {
      t->set_active(tp.active);
      t->set_weight(tp.weight);
      t->set_priority(tp.priority);
    }
  }
  for (const auto &cp : preset.constraint_presets) {
    if (auto *c = get_constraint(cp.constraint_name)) {
      c->set_active(cp.active);
    }
  }
}

const SolveResult &
WQPFormulation::solve(const PinocchioCache &cache, const ControlReference &ref,
                      const ContactState &contacts,
                      const RobotModelInfo &robot_info) noexcept {
  const int n_vars = nv_ + contacts.active_contact_vars;

  // ── Cost: H, g ──
  auto H = qp_data_.H.topLeftCorner(n_vars, n_vars);
  auto g_vec = qp_data_.g.head(n_vars);
  H.setZero();
  g_vec.setZero();

  for (const auto &task : tasks_) {
    if (!task->is_active())
      continue;

    const int rdim = task->residual_dim();
    auto J_view = J_workspace_.topLeftCorner(rdim, n_vars);
    auto r_view = r_workspace_.head(rdim);
    J_view.setZero();
    r_view.setZero();

    task->compute_residual(cache, ref, contacts, n_vars, J_view, r_view);

    const double w = task->weight();
    // H += w * Jᵀ·J
    H.noalias() += w * J_view.transpose() * J_view;
    // g += -w * Jᵀ·r
    g_vec.noalias() -= w * J_view.transpose() * r_view;
  }

  // Small regularization for numerical stability
  for (int i = 0; i < n_vars; ++i) {
    H(i, i) += 1e-8;
  }

  // ── Equality constraints: A, b ──
  int n_eq = 0;
  for (const auto &con : constraints_) {
    if (!con->is_active())
      continue;
    const int ed = con->eq_dim(contacts);
    if (ed <= 0)
      continue;

    auto A_view = qp_data_.A.block(n_eq, 0, ed, n_vars);
    auto b_view = qp_data_.b.segment(n_eq, ed);
    A_view.setZero();
    b_view.setZero();

    con->compute_equality(cache, contacts, robot_info, n_vars, A_view, b_view);
    n_eq += ed;
  }

  // ── Inequality constraints: C, l, u ──
  int n_ineq = 0;
  for (const auto &con : constraints_) {
    if (!con->is_active())
      continue;
    const int id = con->ineq_dim(contacts);
    if (id <= 0)
      continue;

    auto C_view = qp_data_.C.block(n_ineq, 0, id, n_vars);
    auto l_view = qp_data_.l.segment(n_ineq, id);
    auto u_view = qp_data_.u.segment(n_ineq, id);
    C_view.setZero();
    l_view.setZero();
    u_view.setZero();

    con->compute_inequality(cache, contacts, robot_info, n_vars, C_view, l_view,
                            u_view);
    n_ineq += id;
  }

  qp_data_.n_vars = n_vars;
  qp_data_.n_eq = n_eq;
  qp_data_.n_ineq = n_ineq;

  // ── Solve ──
  const auto &solver_result = qp_solver_.solve(qp_data_);

  result_.x_opt.head(n_vars) = solver_result.x_opt.head(n_vars);
  result_.converged = solver_result.converged;
  result_.solve_time_us = solver_result.solve_time_us;
  result_.iterations = solver_result.iterations;
  result_.levels_solved = 1;

  return result_;
}

} // namespace rtc::tsid
