// Ball prediction at a planner node (E1-F13, #739): the mean is SampleAt's, the
// covariance is F(Δ) Σ_i F(Δ)ᵀ from the NEAREST sample, and an unusable
// covariance is reported as such instead of becoming zero.
//
// The fixture breaks every symmetry the code could get wrong without a test
// noticing: the position and velocity blocks differ, the cross block is not
// symmetric, each sample carries a different covariance, and the wire matrix is
// asymmetric (so row-major versus column-major reading would show unless the
// symmetrisation hid it — the asymmetric part is therefore checked through a
// probe that is NOT symmetrised away, the cross block).
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/ball_node_samples.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace {

using rtc::catching::BallCovariance;
using rtc::catching::BallNodeSample;
using rtc::catching::BallTime;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::NowLead;
using rtc::catching::PropagateBallCovariance;
using rtc::catching::SampleAt;
using rtc::catching::SampleBallNode;
using rtc::catching::SampleEval;
using rtc::catching::TrajectorySnapshot;

constexpr std::int64_t kT0 = 1'000'000'000'000;
constexpr std::int64_t kStepNs = 50'000'000;  // the prediction's 50 ms spacing
constexpr int kN = 8;

// A ballistic prediction (constant acceleration, so every interpolation of
// the mean is exact) with a distinct, valid covariance per sample.
struct Fixture {
  TrajectorySnapshot traj;
  CovarianceSnapshot cov;
};

// A symmetric positive-definite 6×6 that differs per sample and has unequal
// blocks: Σ = A Aᵀ + diag, A filled from a per-sample pattern.
BallCovariance TruthCovariance(int i) {
  Eigen::Matrix<double, 6, 6> a;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      a(r, c) = 1e-3 * std::sin(0.7 * r + 1.3 * c + 0.4 * i) * (r < 3 ? 1.0 : 6.0);
    }
  }
  BallCovariance s = a * a.transpose();
  s.diagonal().array() += 1e-6;
  return s;
}

Fixture MakeFixture() {
  Fixture f;
  f.traj.valid = true;
  f.traj.n = kN;
  f.traj.token.snapshot_sequence = 7;
  f.cov.valid = true;
  f.cov.n = kN;
  f.cov.token = f.traj.token;
  const Eigen::Vector3d p0(1.8, -0.3, 1.1);
  const Eigen::Vector3d v0(-3.5, 0.4, 1.2);
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  for (int i = 0; i < kN; ++i) {
    const auto k = static_cast<std::size_t>(i);
    const double t = static_cast<double>(i) * 0.05;
    const Eigen::Vector3d p = p0 + v0 * t + 0.5 * t * t * g;
    const Eigen::Vector3d v = v0 + g * t;
    f.traj.s[k].t_ns = kT0 + i * kStepNs;
    for (std::size_t d = 0; d < 3; ++d) {
      f.traj.s[k].p[d] = p[static_cast<Eigen::Index>(d)];
      f.traj.s[k].v[d] = v[static_cast<Eigen::Index>(d)];
      f.traj.s[k].a[d] = g[static_cast<Eigen::Index>(d)];
    }
    const BallCovariance s = TruthCovariance(i);
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        f.cov.c[k][static_cast<std::size_t>(r * 6 + c)] = s(r, c);
      }
    }
  }
  return f;
}

Eigen::Matrix<double, 6, 6> TransitionMatrix(double delta) {
  Eigen::Matrix<double, 6, 6> f = Eigen::Matrix<double, 6, 6>::Identity();
  f.topRightCorner<3, 3>() = delta * Eigen::Matrix3d::Identity();
  return f;
}

TEST(BallNodeSamples, MeanIsTheSharedSampler) {
  const Fixture f = MakeFixture();
  const BallTime t{kT0 + 3 * kStepNs + 17'000'000};
  int hint = 0;
  const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, t, hint);
  const SampleEval ref = SampleAt(f.traj, NowLead{t.ns});
  ASSERT_TRUE(s.valid);
  ASSERT_TRUE(ref.valid);
  EXPECT_EQ(s.p, ref.p);
  EXPECT_EQ(s.v, ref.v);
  EXPECT_EQ(s.a, ref.a);
  EXPECT_FALSE(s.after_horizon);
}

TEST(BallNodeSamples, CovarianceIsPropagatedFromTheNearestSample) {
  const Fixture f = MakeFixture();
  // 17 ms past sample 3: nearest is 3, Δ = +17 ms.
  {
    const BallTime t{kT0 + 3 * kStepNs + 17'000'000};
    int hint = 0;
    const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, t, hint);
    ASSERT_TRUE(s.cov_valid);
    const Eigen::Matrix<double, 6, 6> fm = TransitionMatrix(0.017);
    const BallCovariance want = fm * TruthCovariance(3) * fm.transpose();
    EXPECT_LT((s.cov - want).cwiseAbs().maxCoeff(), 1e-18);
    EXPECT_EQ(s.cov, s.cov.transpose());
  }
  // 33 ms past sample 3: nearest is 4, Δ = −17 ms — a NEGATIVE step, and a
  // different sample's covariance (each sample's differs).
  {
    const BallTime t{kT0 + 3 * kStepNs + 33'000'000};
    int hint = 0;
    const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, t, hint);
    ASSERT_TRUE(s.cov_valid);
    const Eigen::Matrix<double, 6, 6> fm = TransitionMatrix(-0.017);
    const BallCovariance want = fm * TruthCovariance(4) * fm.transpose();
    EXPECT_LT((s.cov - want).cwiseAbs().maxCoeff(), 1e-18);
    const BallCovariance wrong =
        TransitionMatrix(0.033) * TruthCovariance(3) * TransitionMatrix(0.033).transpose();
    EXPECT_GT((s.cov - wrong).cwiseAbs().maxCoeff(), 1e-9)
        << "the fixture cannot tell the nearest sample from the interval's left end";
  }
  // On a sample: that sample's covariance, unpropagated.
  {
    const BallTime t{kT0 + 5 * kStepNs};
    int hint = 0;
    const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, t, hint);
    ASSERT_TRUE(s.cov_valid);
    EXPECT_LT((s.cov - TruthCovariance(5)).cwiseAbs().maxCoeff(), 1e-20);
  }
}

// Row-major reading and the [p; v] block order, on a wire matrix that is NOT
// symmetric: the block form must reproduce F ½(W + Wᵀ) Fᵀ of the matrix read
// row by row, and reading it column by column gives the same symmetric part —
// so the order is pinned through an entry-wise probe instead.
TEST(BallNodeSamples, WireLayoutIsRowMajorPositionThenVelocity) {
  Fixture f = MakeFixture();
  auto& c = f.cov.c[2];
  c.fill(0.0);
  // Σ_pp = diag(1, 2, 3)·1e-4, Σ_vv = diag(4, 5, 6)·1e-2, one cross term
  // Σ(p_x, v_y) = Σ(v_y, p_x) = 2e-4 written at both wire positions.
  for (std::size_t i = 0; i < 3; ++i) {
    c[i * 6 + i] = 1e-4 * static_cast<double>(i + 1);
    c[(i + 3) * 6 + (i + 3)] = 1e-2 * static_cast<double>(i + 4);
  }
  c[0 * 6 + 4] = 2e-4;
  c[4 * 6 + 0] = 2e-4;
  const BallTime t{kT0 + 2 * kStepNs};
  int hint = 0;
  const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, t, hint);
  ASSERT_TRUE(s.cov_valid);
  EXPECT_DOUBLE_EQ(s.cov(1, 1), 2e-4);
  EXPECT_DOUBLE_EQ(s.cov(4, 4), 5e-2);
  EXPECT_DOUBLE_EQ(s.cov(0, 4), 2e-4);
  EXPECT_DOUBLE_EQ(s.cov(4, 0), 2e-4);
  EXPECT_DOUBLE_EQ(s.cov(1, 3), 0.0);

  // And the propagation moves velocity variance into position, not the other
  // way round: 20 ms later Σ_pp(y) grows by Δ²·Σ_vv(y) (no pv term on y).
  const BallNodeSample later =
      SampleBallNode(f.traj, &f.cov, true, BallTime{t.ns + 20'000'000}, hint);
  ASSERT_TRUE(later.cov_valid);
  EXPECT_NEAR(later.cov(1, 1), 2e-4 + 0.02 * 0.02 * 5e-2, 1e-15);
  EXPECT_DOUBLE_EQ(later.cov(4, 4), 5e-2);
  // x has the cross term with v_y only, so Σ_pp(x) gains Δ²·Σ_vv(x) alone and
  // Σ(p_x, p_y) gains Δ·Σ(p_x, v_y).
  EXPECT_NEAR(later.cov(0, 1), 0.02 * 2e-4, 1e-15);
}

TEST(BallNodeSamples, PropagationMatchesTheMatrixProduct) {
  const BallCovariance s = TruthCovariance(1);
  for (const double delta : {-0.04, 0.0, 0.013, 0.2}) {
    const Eigen::Matrix<double, 6, 6> fm = TransitionMatrix(delta);
    const BallCovariance want = fm * s * fm.transpose();
    EXPECT_LT((PropagateBallCovariance(s, delta) - want).cwiseAbs().maxCoeff(), 1e-16)
        << "delta " << delta;
  }
}

TEST(BallNodeSamples, UnusableCovarianceIsReportedNotZeroed) {
  const Fixture good = MakeFixture();
  const BallTime t{kT0 + 3 * kStepNs + 5'000'000};
  const auto sample = [&t](const Fixture& f, const CovarianceSnapshot* cov, bool matched) {
    int hint = 0;
    return SampleBallNode(f.traj, cov, matched, t, hint);
  };
  // The control: this instant has a usable covariance.
  ASSERT_TRUE(sample(good, &good.cov, true).cov_valid);

  // Each failure leaves the MEAN valid.
  const auto expect_mean_only = [](const BallNodeSample& s, const char* what) {
    EXPECT_TRUE(s.valid) << what;
    EXPECT_FALSE(s.cov_valid) << what;
    EXPECT_EQ(s.cov, BallCovariance::Zero()) << what;
  };
  expect_mean_only(sample(good, nullptr, true), "no covariance");
  expect_mean_only(sample(good, &good.cov, false), "another prediction's covariance");
  {
    Fixture f = good;
    f.cov.valid = false;
    expect_mean_only(sample(f, &f.cov, true), "invalid snapshot");
  }
  {
    Fixture f = good;
    f.cov.n = 3;  // sample 3 (the nearest) has no entry
    expect_mean_only(sample(f, &f.cov, true), "covariance shorter than the trajectory");
  }
  {
    Fixture f = good;
    f.cov.c[3][4 * 6 + 5] = std::numeric_limits<double>::quiet_NaN();  // a velocity cross term
    expect_mean_only(sample(f, &f.cov, true), "NaN element");
  }
  {
    Fixture f = good;
    f.cov.c[3][2 * 6 + 2] = -1e-4;  // negative position variance
    expect_mean_only(sample(f, &f.cov, true), "negative variance");
  }
  {
    // A NaN in ANOTHER sample's covariance does not matter.
    Fixture f = good;
    f.cov.c[6][0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(sample(f, &f.cov, true).cov_valid);
  }
}

TEST(BallNodeSamples, AnUnsampleablePredictionIsInvalid) {
  Fixture f = MakeFixture();
  f.traj.valid = false;
  int hint = 0;
  const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, BallTime{kT0}, hint);
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(s.cov_valid);
}

TEST(BallNodeSamples, PastTheHorizonIsFlaggedAndUsesTheLastSample) {
  const Fixture f = MakeFixture();
  const BallTime t{kT0 + (kN - 1) * kStepNs + 30'000'000};
  int hint = 0;
  const BallNodeSample s = SampleBallNode(f.traj, &f.cov, true, t, hint);
  ASSERT_TRUE(s.valid);
  EXPECT_TRUE(s.after_horizon);
  ASSERT_TRUE(s.cov_valid);
  const Eigen::Matrix<double, 6, 6> fm = TransitionMatrix(0.030);
  const BallCovariance want = fm * TruthCovariance(kN - 1) * fm.transpose();
  EXPECT_LT((s.cov - want).cwiseAbs().maxCoeff(), 1e-18);
}

TEST(BallNodeSamples, SamplingAllocatesNothing) {
  const Fixture f = MakeFixture();
  int hint = 0;
  BallNodeSample s;
  {
    const rtc::testing::ScopedAllocGate gate;
    const rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 20; ++i) {
      s = SampleBallNode(f.traj, &f.cov, true, BallTime{kT0 + i * 13'000'000}, hint);
    }
    EXPECT_EQ(gate.count(), 0U);
    EXPECT_EQ(eigen_gate.violations(), 0U);
  }
  EXPECT_TRUE(s.cov_valid);
}

}  // namespace
