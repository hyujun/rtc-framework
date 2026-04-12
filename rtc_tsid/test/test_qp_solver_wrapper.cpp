#include <gtest/gtest.h>

#include "rtc_tsid/solver/qp_solver_wrapper.hpp"

namespace rtc::tsid {
namespace {

class QPSolverWrapperTest : public ::testing::Test {
 protected:
  QPSolverWrapper solver;
};

// 가장 단순한 QP: min 0.5 * x^T H x + g^T x, unconstrained
// H = I_2, g = [-1, -2] → x* = [1, 2]
TEST_F(QPSolverWrapperTest, UnconstrainedQP) {
  solver.init(2, 0, 0);

  QPData qp;
  qp.init(2, 0, 0);
  qp.n_vars = 2;
  qp.n_eq = 0;
  qp.n_ineq = 0;

  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  qp.g.head(2) << -1.0, -2.0;

  const auto& result = solver.solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 1.0, 1e-6);
  EXPECT_NEAR(result.x_opt(1), 2.0, 1e-6);
  EXPECT_GT(result.solve_time_us, 0.0);
}

// Equality constrained QP:
// min 0.5 * ||x||^2  s.t. x0 + x1 = 1
// → x* = [0.5, 0.5]
TEST_F(QPSolverWrapperTest, EqualityConstrainedQP) {
  solver.init(2, 1, 0);

  QPData qp;
  qp.init(2, 1, 0);
  qp.n_vars = 2;
  qp.n_eq = 1;
  qp.n_ineq = 0;

  qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
  qp.g.head(2).setZero();

  // x0 + x1 = 1
  qp.A(0, 0) = 1.0;
  qp.A(0, 1) = 1.0;
  qp.b(0) = 1.0;

  const auto& result = solver.solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 0.5, 1e-6);
  EXPECT_NEAR(result.x_opt(1), 0.5, 1e-6);
}

// Inequality constrained QP:
// min 0.5 * ||x||^2  s.t.  x0 >= 2, x1 >= 3
// → x* = [2, 3]
TEST_F(QPSolverWrapperTest, InequalityConstrainedQP) {
  solver.init(2, 0, 2);

  QPData qp;
  qp.init(2, 0, 2);
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

  const auto& result = solver.solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 2.0, 1e-5);
  EXPECT_NEAR(result.x_opt(1), 3.0, 1e-5);
}

// Mixed equality + inequality:
// min 0.5 * ||x||^2  s.t.  x0 + x1 = 3,  x0 >= 2
// → x0 = 2, x1 = 1
TEST_F(QPSolverWrapperTest, MixedConstrainedQP) {
  solver.init(2, 1, 1);

  QPData qp;
  qp.init(2, 1, 1);
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

  const auto& result = solver.solve(qp);

  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.x_opt(0), 2.0, 1e-5);
  EXPECT_NEAR(result.x_opt(1), 1.0, 1e-5);
}

// Warm-start 테스트: 동일 dimension에서 연속 solve
TEST_F(QPSolverWrapperTest, WarmStartConsecutiveSolves) {
  solver.init(3, 0, 0);

  QPData qp;
  qp.init(3, 0, 0);
  qp.n_vars = 3;

  // 첫 번째 solve: min 0.5*||x||^2 + [-1,-2,-3]^T x → x* = [1,2,3]
  qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
  qp.g.head(3) << -1.0, -2.0, -3.0;

  const auto& r1 = solver.solve(qp);
  ASSERT_TRUE(r1.converged);
  EXPECT_NEAR(r1.x_opt(0), 1.0, 1e-6);
  EXPECT_NEAR(r1.x_opt(1), 2.0, 1e-6);
  EXPECT_NEAR(r1.x_opt(2), 3.0, 1e-6);

  // 두 번째 solve: g 약간 변경 → warm-start로 빠르게 수렴
  qp.g.head(3) << -1.1, -2.1, -3.1;

  const auto& r2 = solver.solve(qp);
  ASSERT_TRUE(r2.converged);
  EXPECT_NEAR(r2.x_opt(0), 1.1, 1e-6);
  EXPECT_NEAR(r2.x_opt(1), 2.1, 1e-6);
  EXPECT_NEAR(r2.x_opt(2), 3.1, 1e-6);

  // warm-start 효과: 두 번째가 첫 번째보다 iteration 적거나 같아야
  // (단, 보장은 아니므로 이것은 soft check)
}

// Dimension 변경 테스트: n_vars가 바뀔 때 re-init
TEST_F(QPSolverWrapperTest, DimensionChange) {
  solver.init(4, 2, 2);

  // 먼저 2D QP
  {
    QPData qp;
    qp.init(4, 2, 2);
    qp.n_vars = 2;
    qp.n_eq = 0;
    qp.n_ineq = 0;
    qp.H.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity();
    qp.g.head(2) << -1.0, -2.0;

    const auto& r = solver.solve(qp);
    ASSERT_TRUE(r.converged);
    EXPECT_NEAR(r.x_opt(0), 1.0, 1e-6);
  }

  // 3D QP로 변경
  {
    QPData qp;
    qp.init(4, 2, 2);
    qp.n_vars = 3;
    qp.n_eq = 0;
    qp.n_ineq = 0;
    qp.H.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
    qp.g.head(3) << -4.0, -5.0, -6.0;

    const auto& r = solver.solve(qp);
    ASSERT_TRUE(r.converged);
    EXPECT_NEAR(r.x_opt(0), 4.0, 1e-6);
    EXPECT_NEAR(r.x_opt(1), 5.0, 1e-6);
    EXPECT_NEAR(r.x_opt(2), 6.0, 1e-6);
  }
}

// 초기화 없이 solve 호출 시 실패
TEST_F(QPSolverWrapperTest, SolveWithoutInit) {
  QPData qp;
  qp.init(2, 0, 0);
  qp.n_vars = 2;

  const auto& result = solver.solve(qp);
  EXPECT_FALSE(result.converged);
}

// TSID와 유사한 차원의 QP (7 DoF + 3 contact forces = 10 vars)
TEST_F(QPSolverWrapperTest, TsidLikeDimension) {
  const int nv = 7;
  const int n_lambda = 3;
  const int n_vars = nv + n_lambda;
  const int n_eq = 3;    // contact accel = 0
  const int n_ineq = nv;  // torque limits

  solver.init(n_vars, n_eq, n_ineq);

  QPData qp;
  qp.init(n_vars, n_eq, n_ineq);
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
      Eigen::MatrixXd::Random(n_eq, nv) * 0.1 +
      Eigen::MatrixXd::Identity(n_eq, nv);
  qp.b.head(n_eq) = -Eigen::VectorXd::Ones(n_eq) * 0.05;

  // Inequality: torque limits  -100 <= a_i <= 100
  for (int i = 0; i < n_ineq; ++i) {
    qp.C(i, i) = 1.0;
    qp.l(i) = -100.0;
    qp.u(i) = 100.0;
  }

  const auto& result = solver.solve(qp);
  ASSERT_TRUE(result.converged);
  EXPECT_GT(result.iterations, 0);
  EXPECT_LT(result.solve_time_us, 10000.0);  // < 10ms
}

}  // namespace
}  // namespace rtc::tsid
