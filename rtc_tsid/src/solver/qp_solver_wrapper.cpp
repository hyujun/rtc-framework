#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

#include <chrono>
#include <limits>

namespace rtc::tsid {

void QPSolverWrapper::Init(int max_n_vars, int max_n_eq, int max_n_ineq,
                           const QPSolverConfig& config) {
  config_ = config;
  max_n_vars_ = max_n_vars;
  max_n_eq_ = max_n_eq;
  max_n_ineq_ = max_n_ineq;

  // ProxSuite dense QP 객체 — max dimension으로 1회만 할당.
  // 이후 Solve() 는 update() 만 호출하여 RT path 에서 heap free/alloc 을 발생시키지 않는다.
  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(max_n_vars, max_n_eq, max_n_ineq);

  // Solver 설정
  qp_->settings.eps_abs = config_.eps_abs;
  qp_->settings.eps_rel = config_.eps_rel;
  qp_->settings.max_iter = config_.max_iter;
  qp_->settings.verbose = config_.verbose;
  qp_->settings.compute_timings = false;

  // 첫 호출은 equality-constrained initial guess (warm-start 데이터 없음)
  qp_->settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;

  // 결과 버퍼 pre-allocate
  result_.Init(max_n_vars);

  // Max-dim 자명 QP 로 초기화: H=I·1e-8 (PD), g=0, A=0, b=0, C=0, l=-inf, u=+inf.
  // 이후 Solve() 는 같은 dim 으로 update() 만 호출.
  Eigen::MatrixXd H0 = Eigen::MatrixXd::Identity(max_n_vars, max_n_vars) * 1e-8;
  Eigen::VectorXd g0 = Eigen::VectorXd::Zero(max_n_vars);
  Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(max_n_eq, max_n_vars);
  Eigen::VectorXd b0 = Eigen::VectorXd::Zero(max_n_eq);
  Eigen::MatrixXd C0 = Eigen::MatrixXd::Zero(max_n_ineq, max_n_vars);
  Eigen::VectorXd l0 =
      Eigen::VectorXd::Constant(max_n_ineq, -std::numeric_limits<double>::infinity());
  Eigen::VectorXd u0 =
      Eigen::VectorXd::Constant(max_n_ineq, std::numeric_limits<double>::infinity());

  if (max_n_eq > 0 && max_n_ineq > 0) {
    qp_->init(H0, g0, A0, b0, C0, l0, u0);
  } else if (max_n_eq > 0) {
    qp_->init(H0, g0, A0, b0, std::nullopt, std::nullopt, std::nullopt);
  } else if (max_n_ineq > 0) {
    qp_->init(H0, g0, std::nullopt, std::nullopt, C0, l0, u0);
  } else {
    qp_->init(H0, g0, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  }

  // 이후 모든 Solve() 호출은 warm-start
  qp_->settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

  initialized_ = true;
}

const SolveResult& QPSolverWrapper::Solve(const QPData& qp) noexcept {
  if (!initialized_) {
    result_.converged = false;
    return result_;
  }

  const auto t_start = std::chrono::steady_clock::now();

  // RT-safe: max-dim QP 를 1회만 alloc 했으므로 항상 update() 만 호출.
  // Caller (WQPFormulation) 는 qp.H/A/C 를 max_n_vars × max_n_vars 등으로 pre-size 하고,
  // active 영역만 채우고 inactive 는 zero / l=-inf / u=+inf 로 패딩한다.
  // n_vars / n_eq / n_ineq 필드는 결과 추출 시에만 사용 (solver dim 은 항상 max).
  const int n = max_n_vars_;
  const int n_eq = max_n_eq_;
  const int n_ineq = max_n_ineq_;

  auto H_view = qp.H.topLeftCorner(n, n);
  auto g_view = qp.g.head(n);

  if (n_eq > 0 && n_ineq > 0) {
    qp_->update(H_view, g_view, qp.A.topLeftCorner(n_eq, n), qp.b.head(n_eq),
                qp.C.topLeftCorner(n_ineq, n), qp.l.head(n_ineq), qp.u.head(n_ineq));
  } else if (n_eq > 0) {
    qp_->update(H_view, g_view, qp.A.topLeftCorner(n_eq, n), qp.b.head(n_eq), std::nullopt,
                std::nullopt, std::nullopt);
  } else if (n_ineq > 0) {
    qp_->update(H_view, g_view, std::nullopt, std::nullopt, qp.C.topLeftCorner(n_ineq, n),
                qp.l.head(n_ineq), qp.u.head(n_ineq));
  } else {
    qp_->update(H_view, g_view, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                std::nullopt);
  }

  qp_->solve();

  // 결과 추출 — caller 가 알려준 active dim (qp.n_vars) 만큼만 읽음
  result_.converged =
      (qp_->results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  result_.iterations = static_cast<int>(qp_->results.info.iter);

  const int active_n = (qp.n_vars > 0 && qp.n_vars <= n) ? qp.n_vars : n;
  if (result_.converged) {
    result_.x_opt.head(active_n) = qp_->results.x.head(active_n);
  }

  const auto t_end = std::chrono::steady_clock::now();
  result_.solve_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();

  return result_;
}

void QPSolverWrapper::SetMaxIter(int iter) noexcept {
  config_.max_iter = iter;
  if (qp_) {
    qp_->settings.max_iter = iter;
  }
}

void QPSolverWrapper::SetEpsAbs(double eps) noexcept {
  config_.eps_abs = eps;
  if (qp_) {
    qp_->settings.eps_abs = eps;
  }
}

}  // namespace rtc::tsid
