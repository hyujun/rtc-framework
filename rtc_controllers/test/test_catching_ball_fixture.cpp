// G0-A — sanity checks of the catching suites' ball flight fixture
// (docs/dynamic_catching/L0_core.md §4.4). Ported from the reference
// docs/dynamic_catching/test_l0.cpp with its thresholds unchanged. The fixture
// generates the reference trajectories every other catching suite compares
// against, so a wrong fixture would make those suites agree with a wrong answer.
#include "rtc_controllers/testing/catching_ball_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

using rtc::catching::fixture::BallMat7;
using rtc::catching::fixture::BallModel;
using rtc::catching::fixture::BallState;
using rtc::catching::fixture::F;
using rtc::catching::fixture::Jacobian;
using rtc::catching::fixture::Propagate;
using rtc::catching::fixture::Rk4;
using rtc::catching::fixture::Rk4WithStm;

// 1) With k = 0 the flight is the closed-form parabola.
TEST(CatchingBallFixture, MatchesAnalyticSolutionWithoutDrag) {
  const BallModel m;
  BallState x;
  x << 0, 0, 0, 3, -1, 6, 0.0;
  const double T = 0.8;
  BallState y = x;
  (void)Propagate(m, y, T, 2e-3, 1000);
  const Eigen::Vector3d p_an = x.head<3>() + x.segment<3>(3) * T + 0.5 * m.g * T * T;
  EXPECT_LT((y.head<3>() - p_an).norm(), 1e-9);
}

// 2) Mechanical energy never increases: Ė = −k‖v‖³ ≤ 0.
TEST(CatchingBallFixture, MechanicalEnergyNonIncreasing) {
  const BallModel m;
  BallState x;
  x << 0, 0, 0, 4, 2, 7, 0.0229;
  const auto energy = [&](const BallState& s) {
    return 0.5 * s.segment<3>(3).squaredNorm() - m.g.dot(s.head<3>());
  };
  double E = energy(x);
  double worst = 0.0;
  for (int i = 0; i < 500; ++i) {
    x = Rk4(m, x, 2e-3);
    const double E2 = energy(x);
    worst = std::max(worst, E2 - E);
    E = E2;
  }
  EXPECT_LT(worst, 1e-9);
}

// 3) Halving the step divides the global error by ≈16 (fourth order).
TEST(CatchingBallFixture, Rk4IsFourthOrder) {
  const BallModel m;
  BallState x0;
  x0 << 0, 0, 0, 5, 1, 8, 0.0229;
  const double T = 0.5;
  const auto run = [&](double h) {
    BallState y = x0;
    (void)Propagate(m, y, T, h, 1 << 20);
    return y;
  };
  const BallState ref = run(1e-5);
  const double e1 = (run(8e-3).head<3>() - ref.head<3>()).norm();
  const double e2 = (run(4e-3).head<3>() - ref.head<3>()).norm();
  const double ratio = e1 / e2;
  EXPECT_GT(ratio, 12.0);
  EXPECT_LT(ratio, 20.0);
}

// 4) Φ matches a central finite difference of the propagated state.
TEST(CatchingBallFixture, StmMatchesFiniteDifference) {
  const BallModel m;
  BallState x0;
  x0 << 0.2, -0.1, 1.0, 4, 2, 7, 0.0229;
  const double T = 0.5;
  const double h = 2e-3;
  const int n = static_cast<int>(T / h + 0.5);
  const auto prop = [&](BallState s) {
    for (int i = 0; i < n; ++i)
      s = Rk4(m, s, h);
    return s;
  };
  BallState x = x0;
  BallMat7 Phi = BallMat7::Identity();
  for (int i = 0; i < n; ++i)
    Rk4WithStm(m, x, Phi, h);
  BallMat7 Fd;
  const double d = 1e-6;
  for (Eigen::Index j = 0; j < 7; ++j) {
    BallState xp = x0;
    BallState xm = x0;
    xp(j) += d;
    xm(j) -= d;
    Fd.col(j) = (prop(xp) - prop(xm)) / (2.0 * d);
  }
  EXPECT_LT((Phi - Fd).cwiseAbs().maxCoeff(), 1e-6);
}

// 5) Regression C8: below v_eps, ∂v̇/∂k must use the same ‖v‖ as f(). Compared
//    against the closed form −‖v‖v directly — a finite difference would drown
//    in g = 9.81 and lose the digits this case is about.
TEST(CatchingBallFixture, DragJacobianConsistentWithFieldAtLowSpeed) {
  const BallModel m;
  BallState x;
  x << 0, 0, 0, 2e-4, 0.0, -1e-4, 0.05;  // ‖v‖ = 2.24e-4 ≪ v_eps
  const Eigen::Vector3d v = x.segment<3>(3);
  ASSERT_LT(v.norm(), m.v_eps);
  const Eigen::Vector3d exact = -v.norm() * v;
  const Eigen::Vector3d got = Jacobian(m, x).block<3, 1>(3, 6);
  EXPECT_LT((got - exact).norm() / exact.norm(), 1e-12);
  // And the field it must agree with really is the unclamped one.
  EXPECT_LT((F(m, x).segment<3>(3) - (m.g + 0.05 * exact)).norm(), 1e-15);
}

// 6) Regression C11: `truncated`, not the step count, reports the cap.
TEST(CatchingBallFixture, PropagateTruncatedContract) {
  const BallModel m;
  BallState x;
  x << 0, 0, 0, 4, 0, 6, 0.0229;
  BallState a = x;
  BallState b = x;
  const auto r1 = Propagate(m, a, 0.2, 2e-3, 100);  // exactly 100 steps, not capped
  const auto r2 = Propagate(m, b, 0.5, 2e-3, 100);  // needs 250 → capped at 100
  EXPECT_EQ(r1.steps, 100);
  EXPECT_FALSE(r1.truncated);
  EXPECT_EQ(r2.steps, 100);
  EXPECT_TRUE(r2.truncated);
  EXPECT_DOUBLE_EQ(r2.h_used, 0.005);
}

}  // namespace
