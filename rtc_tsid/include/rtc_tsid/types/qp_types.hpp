#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

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
  void init(int max_n_vars, int max_n_eq, int max_n_ineq) {
    H.setZero(max_n_vars, max_n_vars);
    g.setZero(max_n_vars);
    A.setZero(max_n_eq, max_n_vars);
    b.setZero(max_n_eq);
    C.setZero(max_n_ineq, max_n_vars);
    l.setZero(max_n_ineq);
    u.setZero(max_n_ineq);
  }
};

// ────────────────────────────────────────────────
// QP Solve 결과
// ────────────────────────────────────────────────
struct SolveResult {
  Eigen::VectorXd x_opt;     // [max_n_vars] pre-allocated
  bool converged{false};
  double solve_time_us{0.0};
  int iterations{0};
  int levels_solved{0};       // WQP: 항상 1, HQP: 실제 solve한 level 수

  void init(int max_n_vars) {
    x_opt.setZero(max_n_vars);
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
std::unordered_map<std::string, PhasePreset> load_phase_presets(
    const YAML::Node& config);

}  // namespace rtc::tsid
