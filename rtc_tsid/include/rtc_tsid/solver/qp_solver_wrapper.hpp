#pragma once

#include "rtc_tsid/types/qp_types.hpp"

#include <memory>

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
  // Inner (Newton) iteration cap per outer proximal step. ProxQP's default
  // (1500) lets a degenerate / infeasible QP grind for 0.5–1.4 s (max_iter ×
  // max_iter_in inner iters) — catastrophic at a 500 Hz RT tick. Bound it so an
  // infeasible solve fails FAST and the caller's qp_fail fallback engages
  // instead of overrunning the loop. Feasible, well-conditioned TSID QPs
  // converge in well under this many inner iters, so it does not regress the
  // happy path.
  int max_iter_in{100};
  bool verbose{false};
  // Re-run ProxQP's Ruiz equilibration on every Solve(). Init() equilibrates a
  // TRIVIAL placeholder problem (H = 1e-8·I, A = 0, seeded C), and ProxQP's
  // update() keeps that preconditioner unless told otherwise — so with the
  // default (false) every real problem is solved under the placeholder's
  // scaling. Default false keeps existing callers bit-for-bit; a caller whose
  // matrices change scale between solves (or that relies on ProxQP's
  // infeasibility verdict) should set it (dynamic_catching MPC E1-F01, #627).
  bool update_preconditioner{false};
  // ProxQP's KKT backend. Automatic picks PrimalLDLT when constraints outnumber
  // variables, and that backend allocates inside solve() on every active-set
  // change (hundreds of C-level mallocs per solve at 126 vars / 434 rows,
  // E1-F01 #627) and ran ~3× slower there than PrimalDualLDLT. Automatic keeps
  // existing callers unchanged.
  proxsuite::proxqp::DenseBackend dense_backend{proxsuite::proxqp::DenseBackend::Automatic};
  // Threshold of ProxQP's primal-infeasibility test (its own default). The
  // test accepts an APPROXIMATE Farkas certificate — ‖Aᵀδy + Cᵀδz‖ within this
  // fraction of ‖(δy, δz)‖ — so it also fires on feasible problems whose
  // multipliers grow fast (a QP with large linear penalties on elastic
  // variables, dynamic_catching E1-F13 #739: PRIMAL_INFEASIBLE on a QP that is
  // feasible by construction). 0 leaves only an EXACT certificate
  // (‖Aᵀδy + Cᵀδz‖ = 0): a QP that is infeasible without one runs to max_iter
  // instead of being reported early. For a caller whose QP cannot be
  // infeasible.
  double eps_primal_inf{1e-4};
};

// ────────────────────────────────────────────────
// ProxSuite dense QP solver 래핑
//
// - Init() 시 max dimension으로 QP 객체 생성 (1회 할당)
// - Solve() 시 Update() + Solve()로 warm-start 유지
// - compute 경로에서 동적 할당 없음 (ProxSuite 내부 workspace 재사용)
// ────────────────────────────────────────────────
class QPSolverWrapper {
 public:
  QPSolverWrapper() = default;

  // max dimension으로 ProxSuite QP 객체 생성
  void Init(int max_n_vars, int max_n_eq, int max_n_ineq,
            const QPSolverConfig& config = QPSolverConfig{});

  // QP solve (RT-safe: 사전 할당된 workspace만 사용)
  // qp.n_vars, n_eq, n_ineq가 이전 호출과 다르면 내부 re-init
  // 결과는 내부 result_에 저장, reference 반환
  // converged == true 이면 x_opt 는 유한하다. 해가 비유한이면 (예: NaN 이 섞인
  // g) ProxQP 가 SOLVED 를 내도 converged = false 이고 x_opt 는 직전 해를 유지하며,
  // 다음 solve 는 warm start 없이 시작한다 (비유한 해가 이후 solve 를 전부 막지 않게).
  [[nodiscard]] const SolveResult& Solve(const QPData& qp) noexcept;

  // Discard the warm start: the NEXT Solve() begins from x = y = z = 0 instead
  // of from the previous solve's iterates, after which warm-starting resumes.
  //
  // Warm starting is the right default for a controller, where consecutive
  // ticks solve almost the same QP. It is WRONG whenever consecutive solves are
  // DIFFERENT problems — an offline sweep, a candidate loop, a map — because
  // then each answer depends on which problem happened to be solved before it,
  // and a solver that is asked the same question twice can give two answers.
  // dynamic_catching's catch-pose IK is exactly that case: the offline
  // catchability map (S3.5a) and the runtime planner (S6.2) must agree, so
  // every candidate starts cold (L3 §4.2).
  //
  // Only a settings enum changes — no allocation, safe on the RT path. This is
  // the same mechanism the non-finite-iterate guard already uses (#546); this
  // makes it reachable deliberately rather than only as a fault response.
  void ResetWarmStart() noexcept;

  // Multipliers of the LAST Solve(), in the caller's units (ProxQP unscales its
  // results): y for the equality rows A x = b, z for the two-sided rows
  // l ≤ C x ≤ u. Sign convention — ProxQP's stationarity condition
  //
  //   H x + g + Aᵀ y + Cᵀ z = 0,
  //
  // so z_i > 0 on a row active at its UPPER bound, z_i < 0 at its LOWER bound
  // and z_i = 0 on an inactive one (pinned by test_qp_solver_wrapper).
  //
  // Sized to the solver's MAX dimensions (padded rows read 0), references into
  // the solver — no copy, no allocation — and valid until the next Solve().
  // Meaningful only when that Solve() reported converged: a failed solve
  // leaves whatever iterates it stopped at, and a non-finite one leaves NaN.
  // Empty before Init().
  [[nodiscard]] const Eigen::VectorXd& EqualityDual() const noexcept;
  [[nodiscard]] const Eigen::VectorXd& InequalityDual() const noexcept;

  // 설정 변경 (non-RT)
  void SetMaxIter(int iter) noexcept;
  void SetEpsAbs(double eps) noexcept;

  [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

 private:
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  SolveResult result_;
  QPSolverConfig config_;
  bool initialized_{false};
  int max_n_vars_{0};
  int max_n_eq_{0};
  int max_n_ineq_{0};

  // Persistent inequality placeholder, allocated once in Init() and reused for
  // (a) the init model and (b) any Solve() tick with zero active inequality rows.
  // c_seed_ is non-zero but inactive (l_inf_ = -inf, u_inf_ = +inf), so
  // ProxSuite's Debug-only assert(model.is_valid) never sees an all-zero C with
  // n_in > 0 at either init or update. Inert in NDEBUG/production builds.
  Eigen::MatrixXd c_seed_;
  Eigen::VectorXd l_inf_;
  Eigen::VectorXd u_inf_;
  // What the dual accessors return before Init() (never resized).
  Eigen::VectorXd no_dual_;
};

}  // namespace rtc::tsid
