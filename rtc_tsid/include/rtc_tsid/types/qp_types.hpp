#pragma once

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// QP 데이터 (Formulation 내부에서 사용, pre-allocated)
// ────────────────────────────────────────────────
struct QPData {
  Eigen::MatrixXd H;  // [max_n_vars × max_n_vars] cost Hessian
  Eigen::VectorXd g;  // [max_n_vars]               cost gradient
  Eigen::MatrixXd A;  // [max_n_eq × max_n_vars]    equality constraint
  Eigen::VectorXd b;  // [max_n_eq]
  Eigen::MatrixXd C;  // [max_n_ineq × max_n_vars]  inequality constraint
  Eigen::VectorXd l;  // [max_n_ineq]               lower bound
  Eigen::VectorXd u;  // [max_n_ineq]               upper bound

  int n_vars{0};
  int n_eq{0};
  int n_ineq{0};

  // Pre-allocate buffers to maximum size (init 시 1회 호출)
  void Init(int max_n_vars, int max_n_eq, int max_n_ineq) {
    H.setZero(max_n_vars, max_n_vars);
    g.setZero(max_n_vars);
    A.setZero(max_n_eq, max_n_vars);
    b.setZero(max_n_eq);
    C.setZero(max_n_ineq, max_n_vars);
    l.setZero(max_n_ineq);
    u.setZero(max_n_ineq);
  }
};

// SolveResult::status when the active constraints needed more QP rows than the
// formulation sized from their MaxEqDim / MaxIneqDim: the rows that did not fit
// were dropped and the QP was NOT solved (converged = false). proxsuite's own
// QPSolverOutput values are non-negative; −1 means no attempt was made.
inline constexpr int kRowBudgetExceeded = -2;

// ────────────────────────────────────────────────
// QP Solve 결과
// ────────────────────────────────────────────────
struct SolveResult {
  Eigen::VectorXd x_opt;  // [max_n_vars] pre-allocated
  bool converged{false};
  // proxsuite::proxqp::QPSolverOutput of the last solve as int (0 = SOLVED);
  // −1 = the wrapper returned before solving (not initialised);
  // kRowBudgetExceeded = rows were dropped, not solved.
  int status{-1};
  // Solver iterates were non-finite (converged is then false even if status
  // reports SOLVED — see QPSolverWrapper::Solve).
  bool non_finite{false};
  double solve_time_us{0.0};
  // ProxQP's INNER (Newton) iterations summed over its outer proximal steps
  // (info.iter) — what max_iter × max_iter_in bounds.
  int iterations{0};
  // ProxQP's OUTER (proximal / BCL) steps (info.iter_ext) — what
  // QPSolverConfig::max_iter bounds. 0 when no solve ran.
  int outer_iterations{0};
  // The solve started from the previous solve's iterates (either
  // WarmStartMode), not from zero or the equality-constrained solution — false
  // on the first solve after Init.
  bool warm_started{false};
  // ProxQP's primal residual max(‖A x − b‖∞, ‖[C x − u]₊ + [C x − l]₋‖∞) and dual
  // residual ‖H x + g + Aᵀ y + Cᵀ z‖∞ at the iterate it stopped at, in the
  // caller's units — reported whether or not it converged (a failed solve
  // tells how far it got; its dual residual may be of multipliers ProxQP has
  // since reset). NaN when no solve ran, or when ProxQP returned an
  // infeasibility certificate instead of an iterate (status 2 or 4).
  double pri_res{std::numeric_limits<double>::quiet_NaN()};
  double dua_res{std::numeric_limits<double>::quiet_NaN()};
  // max |A x − b| over the rows of the constraint named "eom" (the floating
  // base's equations of motion) at the same iterate — what a dynamics check
  // reads, apart from the contact and task rows pri_res mixes in. 0 when the
  // QP has no such rows (none added, or switched off); NaN when pri_res is NaN
  // or the iterate is not finite.
  double eom_residual_inf{std::numeric_limits<double>::quiet_NaN()};
  int levels_solved{0};  // WQP: 항상 1, HQP: 실제 solve한 level 수
  // Constraint rows assembled this tick (HQP: the shared constraints, without
  // the cascaded rows of earlier levels), and the rows of active constraints
  // that did not fit the formulation's row budget. rows_dropped > 0 comes with
  // status = kRowBudgetExceeded and converged = false.
  int n_eq_used{0};
  int n_ineq_used{0};
  int rows_dropped{0};

  void Init(int max_n_vars) { x_opt.setZero(max_n_vars); }

  // A tick on which no QP was solved: `status` says why (−1 or
  // kRowBudgetExceeded), the solve diagnostics are zeroed, x_opt is left as is.
  void MarkNotSolved(int why) noexcept {
    converged = false;
    status = why;
    non_finite = false;
    solve_time_us = 0.0;
    iterations = 0;
    outer_iterations = 0;
    warm_started = false;
    pri_res = std::numeric_limits<double>::quiet_NaN();
    dua_res = std::numeric_limits<double>::quiet_NaN();
    eom_residual_inf = std::numeric_limits<double>::quiet_NaN();
    levels_solved = 0;
  }
};

// ────────────────────────────────────────────────
// Phase Preset 시스템 — WQP/HQP 통합
// ────────────────────────────────────────────────
struct TaskPreset {
  std::string task_name;
  bool active{true};
  double weight{1.0};
  int priority{0};  // HQP level 분류용. WQP에서는 무시
};

struct ConstraintPreset {
  std::string constraint_name;
  bool active{true};
};

struct PhasePreset {
  std::string phase_name;
  std::vector<TaskPreset> task_presets;
  std::vector<ConstraintPreset> constraint_presets;
};

// YAML로부터 phase preset 로드 (init 시 1회)
std::unordered_map<std::string, PhasePreset> LoadPhasePresets(const YAML::Node& config);

}  // namespace rtc::tsid
