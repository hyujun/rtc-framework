// E1-F01 (#627): closed-form evaluation of a piecewise-constant-jerk joint
// trajectory (jerk_segment.hpp). The nodes below are built by propagating the
// exact triple-integrator step, so a correct evaluator reproduces every node
// from its predecessor — the C² claim is checked against independently
// propagated values, not against the evaluator's own output.
#include "rtc_controllers/catching/jerk_segment.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using rtc::catching::EvaluateJerkSegment;
using rtc::catching::SampleJerkTrajectory;

constexpr double kDt = 0.05;
constexpr int kNodes = 6;  // N segments, N+1 nodes
constexpr int kJoints = 2;

struct Nodes {
  Eigen::MatrixXd q{kJoints, kNodes + 1};
  Eigen::MatrixXd qd{kJoints, kNodes + 1};
  Eigen::MatrixXd qdd{kJoints, kNodes + 1};
};

// Exact propagation of q⃛ = u_k on each interval; joints get different,
// sign-changing jerks so no symmetry can hide an index or sign error.
Nodes Propagate() {
  Nodes n;
  n.q.col(0) << 0.3, -1.1;
  n.qd.col(0) << 0.8, -0.4;
  n.qdd.col(0) << -2.0, 1.5;
  for (int k = 0; k < kNodes; ++k) {
    for (int j = 0; j < kJoints; ++j) {
      const double u = (j == 0 ? 40.0 : -25.0) * std::cos(0.9 * k + j);
      const double q = n.q(j, k);
      const double v = n.qd(j, k);
      const double a = n.qdd(j, k);
      n.q(j, k + 1) = q + kDt * v + 0.5 * kDt * kDt * a + kDt * kDt * kDt * u / 6.0;
      n.qd(j, k + 1) = v + kDt * a + 0.5 * kDt * kDt * u;
      n.qdd(j, k + 1) = a + kDt * u;
    }
  }
  return n;
}

TEST(JerkSegment, JerkSegmentIsC2AtNodes) {
  const Nodes n = Propagate();
  Eigen::VectorXd q(kJoints), qd(kJoints), qdd(kJoints);
  for (int k = 0; k < kNodes; ++k) {
    // End of segment k equals node k+1 in q, q̇ and q̈ …
    ASSERT_TRUE(EvaluateJerkSegment(n.q.col(k), n.qd.col(k), n.qdd.col(k), n.qdd.col(k + 1), kDt,
                                    kDt, q, qd, qdd));
    EXPECT_LT((q - n.q.col(k + 1)).cwiseAbs().maxCoeff(), 1e-12) << "k=" << k;
    EXPECT_LT((qd - n.qd.col(k + 1)).cwiseAbs().maxCoeff(), 1e-12) << "k=" << k;
    EXPECT_LT((qdd - n.qdd.col(k + 1)).cwiseAbs().maxCoeff(), 1e-12) << "k=" << k;
    // … and the sampler agrees with the segment on both sides of the node.
    ASSERT_TRUE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, (k + 1) * kDt - 1e-9, q, qd, qdd));
    Eigen::VectorXd q2(kJoints), qd2(kJoints), qdd2(kJoints);
    ASSERT_TRUE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, (k + 1) * kDt + 1e-9, q2, qd2, qdd2));
    EXPECT_LT((q - q2).cwiseAbs().maxCoeff(), 1e-7);
    EXPECT_LT((qd - qd2).cwiseAbs().maxCoeff(), 1e-6);
    EXPECT_LT((qdd - qdd2).cwiseAbs().maxCoeff(), 1e-5);
  }
}

TEST(JerkSegment, MidSegmentMatchesExactPolynomial) {
  const Nodes n = Propagate();
  Eigen::VectorXd q(kJoints), qd(kJoints), qdd(kJoints);
  const int k = 3;
  const double tau = 0.37 * kDt;
  ASSERT_TRUE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, k * kDt + tau, q, qd, qdd));
  for (int j = 0; j < kJoints; ++j) {
    const double u = (n.qdd(j, k + 1) - n.qdd(j, k)) / kDt;
    EXPECT_NEAR(
        q[j],
        n.q(j, k) + n.qd(j, k) * tau + 0.5 * n.qdd(j, k) * tau * tau + u * tau * tau * tau / 6.0,
        1e-12);
    EXPECT_NEAR(qd[j], n.qd(j, k) + n.qdd(j, k) * tau + 0.5 * u * tau * tau, 1e-12);
    EXPECT_NEAR(qdd[j], n.qdd(j, k) + u * tau, 1e-12);
  }
}

TEST(JerkSegment, JerkSegmentRejectsNonPositiveDt) {
  const Nodes n = Propagate();
  Eigen::VectorXd q = Eigen::VectorXd::Constant(kJoints, 7.0);
  Eigen::VectorXd qd = q;
  Eigen::VectorXd qdd = q;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double dt : {0.0, -kDt, nan, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(SampleJerkTrajectory(n.q, n.qd, n.qdd, dt, 0.01, q, qd, qdd)) << dt;
    EXPECT_FALSE(EvaluateJerkSegment(n.q.col(0), n.qd.col(0), n.qdd.col(0), n.qdd.col(1), dt, 0.0,
                                     q, qd, qdd))
        << dt;
  }
  // Negative or non-finite time is a caller bug, not a clamp.
  EXPECT_FALSE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, -1e-6, q, qd, qdd));
  EXPECT_FALSE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, nan, q, qd, qdd));
  // τ outside [0, Δ] for a single segment.
  EXPECT_FALSE(EvaluateJerkSegment(n.q.col(0), n.qd.col(0), n.qdd.col(0), n.qdd.col(1), kDt,
                                   kDt * 1.01, q, qd, qdd));
  // Shape mismatch.
  Eigen::VectorXd short_q(kJoints - 1);
  EXPECT_FALSE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, 0.01, short_q, qd, qdd));
  // Outputs untouched by every failure above.
  EXPECT_TRUE((q.array() == 7.0).all());
  EXPECT_TRUE((qd.array() == 7.0).all());
  EXPECT_TRUE((qdd.array() == 7.0).all());
}

TEST(JerkSegment, JerkSegmentHoldsPastEnd) {
  const Nodes n = Propagate();
  Eigen::VectorXd q(kJoints), qd(kJoints), qdd(kJoints);
  for (const double t : {kNodes * kDt, kNodes * kDt + 1e-9, 10.0}) {
    ASSERT_TRUE(SampleJerkTrajectory(n.q, n.qd, n.qdd, kDt, t, q, qd, qdd));
    EXPECT_EQ(q, Eigen::VectorXd(n.q.col(kNodes)));
    EXPECT_EQ(qd, Eigen::VectorXd(n.qd.col(kNodes)));
    EXPECT_EQ(qdd, Eigen::VectorXd(n.qdd.col(kNodes)));
  }
}

}  // namespace
