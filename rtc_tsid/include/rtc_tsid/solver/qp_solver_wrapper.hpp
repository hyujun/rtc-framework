#pragma once

#include <memory>

#include "rtc_tsid/types/qp_types.hpp"

// ProxSuite 헤더 — 컴파일러 경고 억제
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <proxsuite/proxqp/dense/dense.hpp>
#pragma GCC diagnostic pop

namespace rtc::tsid {

// ────────────────────────────────────────────────
// QP solver 설정 (QPSolverWrapper 바깥에 정의하여 default arg 문제 회피)
// ────────────────────────────────────────────────
struct QPSolverConfig {
  double eps_abs{1e-6};
  double eps_rel{0.0};
  int max_iter{20};
  bool verbose{false};
};

// ────────────────────────────────────────────────
// ProxSuite dense QP solver 래핑
//
// - init() 시 max dimension으로 QP 객체 생성 (1회 할당)
// - solve() 시 update() + solve()로 warm-start 유지
// - compute 경로에서 동적 할당 없음 (ProxSuite 내부 workspace 재사용)
// ────────────────────────────────────────────────
class QPSolverWrapper {
 public:
  QPSolverWrapper() = default;

  // max dimension으로 ProxSuite QP 객체 생성
  void init(int max_n_vars, int max_n_eq, int max_n_ineq,
            const QPSolverConfig& config = QPSolverConfig{});

  // QP solve (RT-safe: 사전 할당된 workspace만 사용)
  // qp.n_vars, n_eq, n_ineq가 이전 호출과 다르면 내부 re-init
  // 결과는 내부 result_에 저장, reference 반환
  [[nodiscard]] const SolveResult& solve(const QPData& qp) noexcept;

  // 설정 변경 (non-RT)
  void set_max_iter(int iter) noexcept;
  void set_eps_abs(double eps) noexcept;

  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

 private:
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  SolveResult result_;
  QPSolverConfig config_;
  bool initialized_{false};
  bool first_solve_{true};
  int prev_n_vars_{0};
  int prev_n_eq_{0};
  int prev_n_ineq_{0};
};

}  // namespace rtc::tsid
