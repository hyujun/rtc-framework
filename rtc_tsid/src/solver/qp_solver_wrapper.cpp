#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

#include <chrono>

namespace rtc::tsid {

void QPSolverWrapper::init(int max_n_vars, int max_n_eq, int max_n_ineq,
                           const QPSolverConfig& config) {
  config_ = config;

  // ProxSuite dense QP 객체 생성 (max dimension, workspace pre-allocate)
  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
      max_n_vars, max_n_eq, max_n_ineq);

  // Solver 설정
  qp_->settings.eps_abs = config_.eps_abs;
  qp_->settings.eps_rel = config_.eps_rel;
  qp_->settings.max_iter = config_.max_iter;
  qp_->settings.verbose = config_.verbose;
  qp_->settings.compute_timings = false;

  // Warm-start: 이전 solve 결과를 초기 추정으로 사용
  qp_->settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

  // 결과 버퍼 pre-allocate
  result_.init(max_n_vars);

  initialized_ = true;
  first_solve_ = true;
  prev_n_vars_ = 0;
  prev_n_eq_ = 0;
  prev_n_ineq_ = 0;
}

const SolveResult& QPSolverWrapper::solve(const QPData& qp) noexcept {
  if (!initialized_) {
    result_.converged = false;
    return result_;
  }

  const auto t_start = std::chrono::steady_clock::now();

  const int n = qp.n_vars;
  const int n_eq = qp.n_eq;
  const int n_ineq = qp.n_ineq;

  // Active subview 추출 (pre-allocated buffer의 상위 n×n 영역)
  auto H_view = qp.H.topLeftCorner(n, n);
  auto g_view = qp.g.head(n);

  // Dimension 변경 시 QP 재구성
  const bool dims_changed =
      (n != prev_n_vars_) || (n_eq != prev_n_eq_) || (n_ineq != prev_n_ineq_);

  if (first_solve_ || dims_changed) {
    // 새 dimension으로 QP 재구성
    qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
        n, n_eq, n_ineq);

    qp_->settings.eps_abs = config_.eps_abs;
    qp_->settings.eps_rel = config_.eps_rel;
    qp_->settings.max_iter = config_.max_iter;
    qp_->settings.verbose = config_.verbose;
    qp_->settings.compute_timings = false;
    qp_->settings.initial_guess =
        proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

    // 첫 solve 또는 dimension 변경 시에는 equality constrained initial guess
    qp_->settings.initial_guess =
        proxsuite::proxqp::InitialGuessStatus::
            EQUALITY_CONSTRAINED_INITIAL_GUESS;

    if (n_eq > 0 && n_ineq > 0) {
      qp_->init(H_view, g_view, qp.A.topLeftCorner(n_eq, n), qp.b.head(n_eq),
                qp.C.topLeftCorner(n_ineq, n), qp.l.head(n_ineq),
                qp.u.head(n_ineq));
    } else if (n_eq > 0) {
      qp_->init(H_view, g_view, qp.A.topLeftCorner(n_eq, n), qp.b.head(n_eq),
                std::nullopt, std::nullopt,
                std::nullopt);
    } else if (n_ineq > 0) {
      qp_->init(H_view, g_view, std::nullopt,
                std::nullopt, qp.C.topLeftCorner(n_ineq, n),
                qp.l.head(n_ineq), qp.u.head(n_ineq));
    } else {
      qp_->init(H_view, g_view, std::nullopt,
                std::nullopt, std::nullopt,
                std::nullopt, std::nullopt);
    }

    qp_->solve();

    // 이후 warm-start 활성화
    qp_->settings.initial_guess =
        proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

    first_solve_ = false;
  } else {
    // Dimension 동일 → update + solve (warm-start)
    if (n_eq > 0 && n_ineq > 0) {
      qp_->update(H_view, g_view, qp.A.topLeftCorner(n_eq, n),
                  qp.b.head(n_eq), qp.C.topLeftCorner(n_ineq, n),
                  qp.l.head(n_ineq), qp.u.head(n_ineq));
    } else if (n_eq > 0) {
      qp_->update(H_view, g_view, qp.A.topLeftCorner(n_eq, n),
                  qp.b.head(n_eq), std::nullopt,
                  std::nullopt, std::nullopt);
    } else if (n_ineq > 0) {
      qp_->update(H_view, g_view, std::nullopt,
                  std::nullopt, qp.C.topLeftCorner(n_ineq, n),
                  qp.l.head(n_ineq), qp.u.head(n_ineq));
    } else {
      qp_->update(H_view, g_view, std::nullopt,
                  std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
    }

    qp_->solve();
  }

  // 결과 추출
  result_.converged =
      (qp_->results.info.status ==
       proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  result_.iterations = static_cast<int>(qp_->results.info.iter);

  if (result_.converged) {
    result_.x_opt.head(n) = qp_->results.x;
  }

  prev_n_vars_ = n;
  prev_n_eq_ = n_eq;
  prev_n_ineq_ = n_ineq;

  const auto t_end = std::chrono::steady_clock::now();
  result_.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();

  return result_;
}

void QPSolverWrapper::set_max_iter(int iter) noexcept {
  config_.max_iter = iter;
  if (qp_) {
    qp_->settings.max_iter = iter;
  }
}

void QPSolverWrapper::set_eps_abs(double eps) noexcept {
  config_.eps_abs = eps;
  if (qp_) {
    qp_->settings.eps_abs = eps;
  }
}

}  // namespace rtc::tsid
