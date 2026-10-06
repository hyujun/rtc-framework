#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace rtc::tsid {
namespace {

class QPSolverWrapperTest : public ::testing::Test {
 protected:
  QPSolverWrapper solver;
};

// 가장 단순한 QP: min 0.5 * x^T H x + g^T x, unconstrained
// H = I_2, g = [-1, -2] → x* = [1, 2]
TEST_F(QPSolverWrapperTest, UnconstrainedQP) {
  solver.Init(2, 0, 0);

  QPData qp;
  qp.Init(2, 0, 0);
  qp.n_vars = 2;
  qp.n_eq = 0;
  qp.n_ineq = 0;

  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  qp.g.head(2) << -1.0, -2.0;

  const auto& result = solver.Solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 1.0, 1e-6);
  EXPECT_NEAR(result.x_opt(1), 2.0, 1e-6);
  EXPECT_GT(result.solve_time_us, 0.0);
}

// Equality constrained QP:
// min 0.5 * ||x||^2  s.t. x0 + x1 = 1
// → x* = [0.5, 0.5]
TEST_F(QPSolverWrapperTest, EqualityConstrainedQP) {
  solver.Init(2, 1, 0);

  QPData qp;
  qp.Init(2, 1, 0);
  qp.n_vars = 2;
  qp.n_eq = 1;
  qp.n_ineq = 0;

  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  qp.g.head(2).setZero();

  // x0 + x1 = 1
  qp.A(0, 0) = 1.0;
  qp.A(0, 1) = 1.0;
  qp.b(0) = 1.0;

  const auto& result = solver.Solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 0.5, 1e-6);
  EXPECT_NEAR(result.x_opt(1), 0.5, 1e-6);
}

// Inequality constrained QP:
// min 0.5 * ||x||^2  s.t.  x0 >= 2, x1 >= 3
// → x* = [2, 3]
TEST_F(QPSolverWrapperTest, InequalityConstrainedQP) {
  solver.Init(2, 0, 2);

  QPData qp;
  qp.Init(2, 0, 2);
  qp.n_vars = 2;
  qp.n_eq = 0;
  qp.n_ineq = 2;

  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  qp.g.head(2).setZero();

  // C * x >= l  →  l <= C * x <= u
  // x0 >= 2:  C[0,:] = [1, 0],  l[0] = 2,   u[0] = +inf
  // x1 >= 3:  C[1,:] = [0, 1],  l[1] = 3,   u[1] = +inf
  qp.C(0, 0) = 1.0;
  qp.C(1, 1) = 1.0;
  qp.l(0) = 2.0;
  qp.l(1) = 3.0;
  qp.u(0) = 1e10;
  qp.u(1) = 1e10;

  const auto& result = solver.Solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 2.0, 1e-5);
  EXPECT_NEAR(result.x_opt(1), 3.0, 1e-5);
}

// Mixed equality + inequality:
// min 0.5 * ||x||^2  s.t.  x0 + x1 = 3,  x0 >= 2
// → x0 = 2, x1 = 1
TEST_F(QPSolverWrapperTest, MixedConstrainedQP) {
  solver.Init(2, 1, 1);

  QPData qp;
  qp.Init(2, 1, 1);
  qp.n_vars = 2;
  qp.n_eq = 1;
  qp.n_ineq = 1;

  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  qp.g.head(2).setZero();

  // x0 + x1 = 3
  qp.A(0, 0) = 1.0;
  qp.A(0, 1) = 1.0;
  qp.b(0) = 3.0;

  // x0 >= 2
  qp.C(0, 0) = 1.0;
  qp.l(0) = 2.0;
  qp.u(0) = 1e10;

  const auto& result = solver.Solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 2.0, 1e-5);
  EXPECT_NEAR(result.x_opt(1), 1.0, 1e-5);
}

// Warm-start 테스트: 동일 dimension에서 연속 solve
TEST_F(QPSolverWrapperTest, WarmStartConsecutiveSolves) {
  solver.Init(3, 0, 0);

  QPData qp;
  qp.Init(3, 0, 0);
  qp.n_vars = 3;

  // 첫 번째 solve: min 0.5*||x||^2 + [-1,-2,-3]^T x → x* = [1,2,3]
  qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
  qp.g.head(3) << -1.0, -2.0, -3.0;

  const auto& r1 = solver.Solve(qp);
  ASSERT_TRUE(r1.converged);
  EXPECT_NEAR(r1.x_opt(0), 1.0, 1e-6);
  EXPECT_NEAR(r1.x_opt(1), 2.0, 1e-6);
  EXPECT_NEAR(r1.x_opt(2), 3.0, 1e-6);

  // 두 번째 solve: g 약간 변경 → warm-start로 빠르게 수렴
  qp.g.head(3) << -1.1, -2.1, -3.1;

  const auto& r2 = solver.Solve(qp);
  ASSERT_TRUE(r2.converged);
  EXPECT_NEAR(r2.x_opt(0), 1.1, 1e-6);
  EXPECT_NEAR(r2.x_opt(1), 2.1, 1e-6);
  EXPECT_NEAR(r2.x_opt(2), 3.1, 1e-6);

  // warm-start 효과: 두 번째가 첫 번째보다 iteration 적거나 같아야
  // (단, 보장은 아니므로 이것은 soft check)
}

// Dimension 변경 테스트: n_vars가 바뀔 때 re-init
TEST_F(QPSolverWrapperTest, DimensionChange) {
  solver.Init(4, 2, 2);

  // 먼저 2D QP
  {
    QPData qp;
    qp.Init(4, 2, 2);
    qp.n_vars = 2;
    qp.n_eq = 0;
    qp.n_ineq = 0;
    qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
    qp.g.head(2) << -1.0, -2.0;

    const auto& r = solver.Solve(qp);
    ASSERT_TRUE(r.converged);
    EXPECT_NEAR(r.x_opt(0), 1.0, 1e-6);
  }

  // 3D QP로 변경
  {
    QPData qp;
    qp.Init(4, 2, 2);
    qp.n_vars = 3;
    qp.n_eq = 0;
    qp.n_ineq = 0;
    qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
    qp.g.head(3) << -4.0, -5.0, -6.0;

    const auto& r = solver.Solve(qp);
    ASSERT_TRUE(r.converged);
    EXPECT_NEAR(r.x_opt(0), 4.0, 1e-6);
    EXPECT_NEAR(r.x_opt(1), 5.0, 1e-6);
    EXPECT_NEAR(r.x_opt(2), 6.0, 1e-6);
  }
}

// 초기화 없이 solve 호출 시 실패
TEST_F(QPSolverWrapperTest, SolveWithoutInit) {
  QPData qp;
  qp.Init(2, 0, 0);
  qp.n_vars = 2;

  const auto& result = solver.Solve(qp);
  EXPECT_FALSE(result.converged);
}

// TSID와 유사한 차원의 QP (7 DoF + 3 contact forces = 10 vars)
TEST_F(QPSolverWrapperTest, TsidLikeDimension) {
  const int nv = 7;
  const int n_lambda = 3;
  const int n_vars = nv + n_lambda;
  const int n_eq = 3;     // contact accel = 0
  const int n_ineq = nv;  // torque limits

  solver.Init(n_vars, n_eq, n_ineq);

  QPData qp;
  qp.Init(n_vars, n_eq, n_ineq);
  qp.n_vars = n_vars;
  qp.n_eq = n_eq;
  qp.n_ineq = n_ineq;

  // Posture task: min ||a - a_des||^2 → H = [I 0; 0 0], g = [-a_des; 0]
  qp.H.topLeftCorner(nv, nv) = Eigen::MatrixXd::Identity(nv, nv);
  qp.g.head(nv) = -Eigen::VectorXd::Ones(nv) * 0.1;  // a_des = 0.1

  // Small regularization on lambda to make H PD
  for (int i = nv; i < n_vars; ++i) {
    qp.H(i, i) = 1e-4;
  }

  // Equality: random-ish contact Jacobian * a = -bias
  qp.A.topLeftCorner(n_eq, nv) =
      Eigen::MatrixXd::Random(n_eq, nv) * 0.1 + Eigen::MatrixXd::Identity(n_eq, nv);
  qp.b.head(n_eq) = -Eigen::VectorXd::Ones(n_eq) * 0.05;

  // Inequality: torque limits  -100 <= a_i <= 100
  for (int i = 0; i < n_ineq; ++i) {
    qp.C(i, i) = 1.0;
    qp.l(i) = -100.0;
    qp.u(i) = 100.0;
  }

  const auto& result = solver.Solve(qp);
  ASSERT_TRUE(result.converged);
  EXPECT_GT(result.iterations, 0);
  EXPECT_LT(result.solve_time_us, 10000.0);  // < 10ms
}

// A non-finite solve must report failure and must not latch the solver off.
// ProxQP returns PROXQP_SOLVED with NaN iterates for a NaN g (NaN residuals pass
// its convergence test), and every Solve() warm-starts from the previous
// iterates — before the fix the NaN solve came back converged with a NaN x_opt
// and every later finite solve failed from the NaN warm start.
TEST_F(QPSolverWrapperTest, RecoversAfterNonFiniteSolve) {
  solver.Init(3, 0, 3);

  QPData qp;
  qp.Init(3, 0, 3);
  qp.n_vars = 3;
  qp.n_ineq = 3;
  qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
  qp.C.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
  qp.l.head(3).setConstant(-0.5);
  qp.u.head(3).setConstant(0.5);

  // min ½‖x‖² + gᵀx over the box [-0.5, 0.5]³ → x* = clamp(−g, ±0.5).
  qp.g.head(3) << -0.2, -1.0, 0.3;
  ASSERT_TRUE(solver.Solve(qp).converged);

  // A NaN task reference spreads through Jᵀr into every entry of g (the CLIK
  // case); a single NaN entry can stay confined to a box-clamped component.
  qp.g.head(3).setConstant(std::numeric_limits<double>::quiet_NaN());
  const auto& bad = solver.Solve(qp);
  EXPECT_FALSE(bad.converged);
  EXPECT_TRUE(bad.x_opt.allFinite()) << "x_opt keeps the last good solution";

  qp.g.head(3) << -0.1, 0.2, 2.0;
  for (int k = 0; k < 5; ++k) {
    const auto& r = solver.Solve(qp);
    ASSERT_TRUE(r.converged) << "solve " << k << " after the non-finite tick";
    EXPECT_NEAR(r.x_opt(0), 0.1, 1e-6);
    EXPECT_NEAR(r.x_opt(1), -0.2, 1e-6);
    EXPECT_NEAR(r.x_opt(2), -0.5, 1e-6);
  }
}

// ResetWarmStart() must make the answer depend only on the QUESTION, not on
// what the solver was asked before it.
//
// Warm starting is right for a controller and wrong for a sweep: dynamic
// catching's offline catchability map and its runtime planner must return the
// same catch pose for the same candidate, and with a retained warm start the
// answer would depend on which candidate was tried first (L3 §4.2).
//
// The tolerance is loosened on purpose. ProxQP stops as soon as its residual
// test passes, so at a tight tolerance every start converges to the same point
// and the leak this test exists to detect would be invisible — a test that
// passes because the effect is too small to see is not a test. The positive
// control below MEASURES that the fixture can see it.
TEST_F(QPSolverWrapperTest, ResetWarmStartMakesTheAnswerOrderIndependent) {
  QPSolverConfig loose;
  loose.eps_abs = 1e-3;

  // Two different box QPs over the same shape.
  const auto fill = [](QPData& qp, double g0, double g1, double g2) {
    qp.Init(3, 0, 3);
    qp.n_vars = 3;
    qp.n_ineq = 3;
    qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
    qp.H(1, 1) = 1e-3;  // ill-conditioned, so the iterate path matters
    qp.H(2, 2) = 1e3;
    qp.C.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
    qp.l.head(3).setConstant(-2.0);
    qp.u.head(3).setConstant(2.0);
    qp.g.head(3) << g0, g1, g2;
  };
  QPData p;
  QPData other;
  fill(p, -0.7, -1.3, 0.9);
  fill(other, 1.9, 0.4, -1.7);

  // Reference: a solver that has seen nothing but P.
  QPSolverWrapper fresh;
  fresh.Init(3, 0, 3, loose);
  const Eigen::VectorXd reference = fresh.Solve(p).x_opt.head(3);
  ASSERT_TRUE(reference.allFinite());

  // Same question, asked after a different one, with the warm start discarded.
  QPSolverWrapper reset;
  reset.Init(3, 0, 3, loose);
  ASSERT_TRUE(reset.Solve(p).converged);
  ASSERT_TRUE(reset.Solve(other).converged);
  reset.ResetWarmStart();
  const Eigen::VectorXd after_reset = reset.Solve(p).x_opt.head(3);
  for (int i = 0; i < 3; ++i)
    EXPECT_EQ(after_reset(i), reference(i)) << "component " << i;

  // Positive control: the SAME sequence without the reset. If this ever stops
  // differing, the assertion above has become vacuous and the fixture — not the
  // API — is what needs fixing.
  QPSolverWrapper warm;
  warm.Init(3, 0, 3, loose);
  ASSERT_TRUE(warm.Solve(p).converged);
  ASSERT_TRUE(warm.Solve(other).converged);
  const Eigen::VectorXd without_reset = warm.Solve(p).x_opt.head(3);
  EXPECT_FALSE(without_reset(0) == reference(0) && without_reset(1) == reference(1) &&
               without_reset(2) == reference(2))
      << "warm start no longer leaks here — this fixture can no longer measure the reset";
}

// The dual accessors and their sign convention. A caller that builds a KKT
// residual or an exact-penalty weight from the multipliers (the catching SQP
// core) depends on BOTH the stationarity sign and "upper active ⇒ z > 0";
// neither is stated in ProxQP's public headers, so they are pinned here.
//
//   min ½‖x‖² + gᵀx   s.t.  x0 + x1 + x2 = 1,   x0 ≤ 0.1,   x1 ≥ 0.6,   |x2| ≤ 5
//
// The unconstrained optimum of the equality-only problem puts x0 above 0.1
// and x1 below 0.6, so the first row binds at its upper bound, the second at
// its lower bound and the third stays inactive. The fixture breaks the
// symmetry on purpose: one row per sign, different magnitudes.
TEST_F(QPSolverWrapperTest, DualsFollowTheStationaritySignConvention) {
  QPSolverConfig config;
  config.eps_abs = 1e-9;
  config.max_iter = 200;
  config.update_preconditioner = true;
  solver.Init(3, 1, 3, config);
  EXPECT_EQ(solver.EqualityDual().size(), 1);
  EXPECT_EQ(solver.InequalityDual().size(), 3);

  QPData qp;
  qp.Init(3, 1, 3);
  qp.n_vars = 3;
  qp.n_eq = 1;
  qp.n_ineq = 3;
  qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
  qp.g.head(3) << -2.0, 1.0, 0.0;
  qp.A.row(0) << 1.0, 1.0, 1.0;
  qp.b(0) = 1.0;
  qp.C.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
  const double inf = std::numeric_limits<double>::infinity();
  qp.l.head(3) << -inf, 0.6, -5.0;
  qp.u.head(3) << 0.1, inf, 5.0;

  const auto& result = solver.Solve(qp);
  ASSERT_TRUE(result.converged);
  const Eigen::Vector3d x = result.x_opt.head(3);
  const Eigen::VectorXd& y = solver.EqualityDual();
  const Eigen::VectorXd& z = solver.InequalityDual();
  ASSERT_EQ(y.size(), 1);
  ASSERT_EQ(z.size(), 3);

  // The rows the fixture means to bind do bind.
  EXPECT_NEAR(x(0), 0.1, 1e-7);
  EXPECT_NEAR(x(1), 0.6, 1e-7);
  EXPECT_NEAR(x(2), 0.3, 1e-7);

  // Stationarity with the documented signs.
  const Eigen::Vector3d grad = qp.H.topLeftCorner(3, 3) * x + qp.g.head(3) +
                               qp.A.topLeftCorner(1, 3).transpose() * y +
                               qp.C.topLeftCorner(3, 3).transpose() * z;
  EXPECT_LT(grad.cwiseAbs().maxCoeff(), 1e-7);
  // The opposite sign on z must NOT satisfy it — otherwise the residual above
  // would be blind to the convention it exists to pin.
  const Eigen::Vector3d grad_flipped = qp.H.topLeftCorner(3, 3) * x + qp.g.head(3) +
                                       qp.A.topLeftCorner(1, 3).transpose() * y -
                                       qp.C.topLeftCorner(3, 3).transpose() * z;
  EXPECT_GT(grad_flipped.cwiseAbs().maxCoeff(), 1.0);

  // Closed form: x2 free ⇒ y = −x2 = −0.3; z0 = 2 − 0.1 − y, z1 = −1 − 0.6 − y.
  EXPECT_NEAR(y(0), -0.3, 1e-6);
  EXPECT_NEAR(z(0), 2.2, 1e-6);
  EXPECT_NEAR(z(1), -1.3, 1e-6);
  EXPECT_NEAR(z(2), 0.0, 1e-6);
  EXPECT_GT(z(0), 0.0) << "upper bound active";
  EXPECT_LT(z(1), 0.0) << "lower bound active";
}

// eps_primal_inf is ProxQP's threshold for accepting an APPROXIMATE
// infeasibility certificate. 0 keeps only an exact one — and a QP with two
// contradictory rows has one (δz = (1, −1) gives Cᵀδz = 0 exactly), so it is
// still reported, never mistaken for solved. (Why a caller sets 0 — false
// reports on feasible QPs with large penalties — is pinned where it occurs:
// the docking core's suite has the control that fails without it.)
TEST_F(QPSolverWrapperTest, ZeroPrimalInfeasibilityThresholdStillRejectsAContradiction) {
  QPData qp;
  qp.Init(2, 0, 2);
  qp.n_vars = 2;
  qp.n_ineq = 2;
  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  // x0 ≤ −1 and x0 ≥ 1: no point satisfies both.
  qp.C(0, 0) = 1.0;
  qp.C(1, 0) = 1.0;
  const double inf = std::numeric_limits<double>::infinity();
  qp.l.head(2) << -inf, 1.0;
  qp.u.head(2) << -1.0, inf;
  constexpr int kPrimalInfeasible = 2;  // proxsuite::proxqp::QPSolverOutput

  QPSolverConfig exact_only;
  exact_only.max_iter = 50;
  exact_only.eps_primal_inf = 0.0;
  QPSolverWrapper wrapper;
  wrapper.Init(2, 0, 2, exact_only);
  const auto& result = wrapper.Solve(qp);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.status, kPrimalInfeasible);

  // The default threshold does not call it solved either.
  QPSolverConfig standard;
  standard.max_iter = 50;
  QPSolverWrapper reference;
  reference.Init(2, 0, 2, standard);
  EXPECT_FALSE(reference.Solve(qp).converged);

  // And a feasible QP solves the same under both.
  qp.l(1) = -3.0;  // x0 ≥ −3 with x0 ≤ −1
  qp.g.head(2) << 0.0, -2.0;
  QPSolverWrapper feasible_exact;
  feasible_exact.Init(2, 0, 2, exact_only);
  const auto& solved = feasible_exact.Solve(qp);
  ASSERT_TRUE(solved.converged);
  EXPECT_NEAR(solved.x_opt(0), -1.0, 1e-5);
  EXPECT_NEAR(solved.x_opt(1), 2.0, 1e-5);
}

TEST_F(QPSolverWrapperTest, DualsAreEmptyBeforeInit) {
  EXPECT_EQ(solver.EqualityDual().size(), 0);
  EXPECT_EQ(solver.InequalityDual().size(), 0);
}

}  // namespace
}  // namespace rtc::tsid
