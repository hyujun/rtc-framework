// RiccatiFeedback unit tests.
//
// Covers:
//   * Uninitialised state returns zeros.
//   * Identity gain: u_fb = scale * Δx.
//   * Gain scale clamping to [0, 1].
//   * Δx magnitude cap clamps the output magnitude.
//   * accel_only mode only writes the first nv entries.
//   * Dimension mismatch produces zeros instead of undefined behaviour.

#include "rtc_mpc/feedback/riccati_feedback.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace rtc::mpc {
namespace {

constexpr int kNq = 3;
constexpr int kNv = 3;
constexpr int kNx = kNq + kNv;  // 6
constexpr int kNu = 3;
constexpr double kTol = 1e-12;

class RiccatiFeedbackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    riccati_.Init(kNv, kNu, kNx);
    q_curr_.setZero(kNq);
    v_curr_.setZero(kNv);
    q_ref_.setZero(kNq);
    v_ref_.setZero(kNv);
    u_fb_.setZero(kNu);
  }

  /// Build the nu × nx identity gain (top-left of an identity matrix).
  std::array<double, kNu * kNx> IdentityGain() const {
    std::array<double, kNu * kNx> data{};
    for (int i = 0; i < kNu; ++i) {
      data[static_cast<std::size_t>(i * kNx + i)] = 1.0;
    }
    return data;
  }

  RiccatiFeedback riccati_;
  Eigen::VectorXd q_curr_;
  Eigen::VectorXd v_curr_;
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd v_ref_;
  Eigen::VectorXd u_fb_;
};

TEST_F(RiccatiFeedbackTest, NoGainProducesZeroOutput) {
  EXPECT_FALSE(riccati_.HasGain());
  u_fb_.setConstant(42.0);
  riccati_.Compute(q_curr_, v_curr_, q_ref_, v_ref_, u_fb_);
  for (int i = 0; i < u_fb_.size(); ++i) {
    EXPECT_NEAR(u_fb_(i), 0.0, kTol);
  }
}

TEST_F(RiccatiFeedbackTest, IdentityGainRecoversDeltaQ) {
  auto data = IdentityGain();
  riccati_.SetGain(data.data(), kNu, kNx);
  EXPECT_TRUE(riccati_.HasGain());

  // Δq = [0.1, -0.2, 0.3]; Δv zero. accel_only → u_fb = Δq for i < nv.
  q_curr_ << 0.1, -0.2, 0.3;
  q_ref_.setZero();
  v_curr_.setZero();
  v_ref_.setZero();

  riccati_.Compute(q_curr_, v_curr_, q_ref_, v_ref_, u_fb_);
  EXPECT_NEAR(u_fb_(0), 0.1, kTol);
  EXPECT_NEAR(u_fb_(1), -0.2, kTol);
  EXPECT_NEAR(u_fb_(2), 0.3, kTol);
}

TEST_F(RiccatiFeedbackTest, GainScaleScalesOutput) {
  auto data = IdentityGain();
  riccati_.SetGain(data.data(), kNu, kNx);
  riccati_.SetGainScale(0.5);
  EXPECT_DOUBLE_EQ(riccati_.GainScale(), 0.5);

  q_curr_ << 1.0, 1.0, 1.0;
  riccati_.Compute(q_curr_, v_curr_, q_ref_, v_ref_, u_fb_);
  for (int i = 0; i < kNu; ++i) {
    EXPECT_NEAR(u_fb_(i), 0.5, kTol);
  }
}

TEST_F(RiccatiFeedbackTest, GainScaleClampedToUnitInterval) {
  riccati_.SetGainScale(-1.0);
  EXPECT_DOUBLE_EQ(riccati_.GainScale(), 0.0);
  riccati_.SetGainScale(2.0);
  EXPECT_DOUBLE_EQ(riccati_.GainScale(), 1.0);
}

TEST_F(RiccatiFeedbackTest, DeltaXMagnitudeCapClampsOutput) {
  auto data = IdentityGain();
  riccati_.SetGain(data.data(), kNu, kNx);
  riccati_.SetMaxDeltaXNorm(1.0);  // |Δx| cap

  // |Δq| = sqrt(3) * 10 ≈ 17.3; well above 1.
  q_curr_ << 10.0, 10.0, 10.0;
  riccati_.Compute(q_curr_, v_curr_, q_ref_, v_ref_, u_fb_);

  // After clamping, the effective |u_fb| magnitude should be ≤ max + ε.
  const double magnitude = u_fb_.norm();
  EXPECT_LE(magnitude, 1.0 + 1e-9) << "Clamped feedback must not exceed max_delta_x_norm";
  EXPECT_GT(magnitude, 0.0);
}

TEST_F(RiccatiFeedbackTest, AccelOnlyModeZerosExtraEntries) {
  auto data = IdentityGain();
  riccati_.SetGain(data.data(), kNu, kNx);
  riccati_.SetAccelOnly(true);

  // Provide an oversized u_fb buffer: only the first kNv entries should be
  // written, the rest zeroed.
  Eigen::VectorXd wide = Eigen::VectorXd::Constant(kNv + 3, 99.0);
  q_curr_ << 1.0, 2.0, 3.0;
  riccati_.Compute(q_curr_, v_curr_, q_ref_, v_ref_, wide);

  for (int i = 0; i < kNv; ++i) {
    EXPECT_NEAR(wide(i), static_cast<double>(i + 1), kTol);
  }
  for (int i = kNv; i < wide.size(); ++i) {
    EXPECT_NEAR(wide(i), 0.0, kTol) << "Tail of oversized u_fb must be zeroed in accel_only mode";
  }
}

TEST_F(RiccatiFeedbackTest, MismatchedDimsProduceZeroOutput) {
  auto data = IdentityGain();
  riccati_.SetGain(data.data(), kNu, kNx);

  // Feed wrong-sized q_ref: q_curr has 3, q_ref has 2.
  Eigen::VectorXd short_ref(2);
  short_ref.setZero();
  u_fb_.setConstant(7.0);
  riccati_.Compute(q_curr_, v_curr_, short_ref, v_ref_, u_fb_);
  for (int i = 0; i < u_fb_.size(); ++i) {
    EXPECT_NEAR(u_fb_(i), 0.0, kTol);
  }
}

TEST_F(RiccatiFeedbackTest, OutOfRangeSetGainIgnored) {
  auto data = IdentityGain();
  riccati_.SetGain(data.data(), kNu + 99, kNx);  // nu too large
  EXPECT_FALSE(riccati_.HasGain());
}

}  // namespace

// ── Kinodynamic gains: more rows than velocities, accel rows offset ─────

TEST(RiccatiFeedbackKinodynamic, GainWithMoreRowsThanVelocitiesIsInstalled) {
  // A free-flyer kinodynamic input has nu = Σ wrench + (nv − 6) > nv; sizing
  // the workspace by nv silently dropped every such gain.
  constexpr int kNvK = 3;
  constexpr int kNuK = 5;
  constexpr int kNdxK = 6;
  RiccatiFeedback riccati;
  riccati.Init(kNvK, kNuK, kNdxK);
  const Eigen::MatrixXd K = Eigen::MatrixXd::Identity(kNuK, kNdxK);
  std::array<double, kNuK * kNdxK> flat{};
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  riccati.SetGain(flat.data(), kNuK, kNdxK);
  EXPECT_TRUE(riccati.HasGain());
}

TEST(RiccatiFeedbackKinodynamic, AccelRowsSelectOnlyTheAccelerationBlock) {
  // u = [wrench(2); qdd(3)], ndx = 6. One-hot gain: wrench row 0 and accel
  // row 0 both respond to dx(0); accel-only output must carry only the
  // accel row.
  constexpr int kNvK = 3;
  constexpr int kNuK = 5;
  constexpr int kNdxK = 6;
  constexpr int kAccOffset = 2;
  RiccatiFeedback riccati;
  riccati.Init(kNvK, kNuK, kNdxK);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(kNuK, kNdxK);
  K(0, 0) = 100.0;             // a wrench row — must not leak into the output
  K(kAccOffset, 0) = 2.0;      // accel row 0
  K(kAccOffset + 2, 1) = 3.0;  // accel row 2
  std::array<double, kNuK * kNdxK> flat{};
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  riccati.SetGain(flat.data(), kNuK, kNdxK);
  ASSERT_TRUE(riccati.SetAccelRows(kAccOffset, kNvK));
  EXPECT_EQ(riccati.AccelRowOffset(), kAccOffset);
  EXPECT_EQ(riccati.AccelRows(), kNvK);

  Eigen::VectorXd dx = Eigen::VectorXd::Zero(kNdxK);
  dx(0) = 1.0;
  dx(1) = 1.0;
  Eigen::VectorXd u_fb = Eigen::VectorXd::Constant(kNvK + 1, 9.0);
  EXPECT_TRUE(riccati.ComputeFromDelta(dx, u_fb));
  EXPECT_DOUBLE_EQ(u_fb(0), 2.0);
  EXPECT_DOUBLE_EQ(u_fb(1), 0.0);
  EXPECT_DOUBLE_EQ(u_fb(2), 3.0);
  EXPECT_DOUBLE_EQ(u_fb(3), 0.0) << "entries past the accel rows are zeroed";

  // Full output (accel_only off) exposes the wrench row as well.
  riccati.SetAccelOnly(false);
  Eigen::VectorXd full = Eigen::VectorXd::Zero(kNuK);
  EXPECT_TRUE(riccati.ComputeFromDelta(dx, full));
  EXPECT_DOUBLE_EQ(full(0), 100.0);
  EXPECT_DOUBLE_EQ(full(kAccOffset), 2.0);

  // A block that does not fit the workspace is refused and leaves the rows alone.
  EXPECT_FALSE(riccati.SetAccelRows(kAccOffset, kNuK));
  EXPECT_FALSE(riccati.SetAccelRows(-1, 1));
  EXPECT_EQ(riccati.AccelRowOffset(), kAccOffset);
  EXPECT_EQ(riccati.AccelRows(), kNvK);
}

TEST(RiccatiFeedbackKinodynamic, ComputeFromDeltaHasTheSameSignAsCompute) {
  // dx = x ⊖ x_ref, u_fb = s · K · dx — a positive position error with a
  // positive gain entry gives a positive feedback, exactly as the Euclidean
  // path does.
  constexpr int kN = 3;
  RiccatiFeedback riccati;
  riccati.Init(kN, kN, 2 * kN);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(kN, 2 * kN);
  K(1, 1) = 4.0;
  std::array<double, kN * 2 * kN> flat{};
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  riccati.SetGain(flat.data(), kN, 2 * kN);

  Eigen::VectorXd q_curr = Eigen::VectorXd::Zero(kN);
  q_curr(1) = 0.5;
  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd from_compute = Eigen::VectorXd::Zero(kN);
  riccati.Compute(q_curr, zeros, zeros, zeros, from_compute);

  Eigen::VectorXd dx = Eigen::VectorXd::Zero(2 * kN);
  dx(1) = 0.5;
  Eigen::VectorXd from_delta = Eigen::VectorXd::Zero(kN);
  EXPECT_TRUE(riccati.ComputeFromDelta(dx, from_delta));
  EXPECT_DOUBLE_EQ(from_delta(1), 2.0);
  EXPECT_EQ(from_delta, from_compute);

  // Wrong tangent size → zeros, no exception.
  Eigen::VectorXd bad = Eigen::VectorXd::Constant(kN, 7.0);
  EXPECT_FALSE(riccati.ComputeFromDelta(Eigen::VectorXd::Zero(2 * kN + 1), bad));
  EXPECT_TRUE((bad.array() == 0.0).all());
}

TEST(RiccatiFeedbackKinodynamic, NonFiniteInputsNeverReachTheOutput) {
  constexpr int kN = 3;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  RiccatiFeedback riccati;
  riccati.Init(kN, kN, 2 * kN);
  Eigen::MatrixXd K = Eigen::MatrixXd::Identity(kN, 2 * kN);
  std::array<double, kN * 2 * kN> flat{};
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  riccati.SetGain(flat.data(), kN, 2 * kN);
  ASSERT_TRUE(riccati.HasGain());

  // NaN and Inf in Δx: zeros, not NaN, and the call reports the refusal.
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(2 * kN);
  dx(0) = nan;
  Eigen::VectorXd u = Eigen::VectorXd::Constant(kN, 1.0);
  EXPECT_FALSE(riccati.ComputeFromDelta(dx, u));
  EXPECT_TRUE((u.array() == 0.0).all());
  dx(0) = std::numeric_limits<double>::infinity();
  u.setConstant(1.0);
  EXPECT_FALSE(riccati.ComputeFromDelta(dx, u));
  EXPECT_TRUE((u.array() == 0.0).all());

  // A non-finite gain scale is ignored instead of latched.
  riccati.SetGainScale(0.5);
  riccati.SetGainScale(nan);
  EXPECT_DOUBLE_EQ(riccati.GainScale(), 0.5);

  // A gain with a NaN entry is not installed.
  K(1, 1) = nan;
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  riccati.SetGain(flat.data(), kN, 2 * kN);
  EXPECT_FALSE(riccati.HasGain());
}

TEST(RiccatiFeedbackKinodynamic, EuclideanComputeRefusesWhenNqPlusNvIsNotNdx) {
  // A configuration with one more coordinate than tangent entries (as a
  // continuous joint or a free-flyer gives) has no Euclidean Δq.
  constexpr int kNqC = 4;
  constexpr int kNvC = 3;
  RiccatiFeedback riccati;
  riccati.Init(kNvC, kNvC, 2 * kNvC);
  const Eigen::MatrixXd K = Eigen::MatrixXd::Identity(kNvC, 2 * kNvC);
  std::array<double, kNvC * 2 * kNvC> flat{};
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat.data(), flat.size()));
  riccati.SetGain(flat.data(), kNvC, 2 * kNvC);

  const Eigen::VectorXd q = Eigen::VectorXd::Ones(kNqC);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(kNvC);
  Eigen::VectorXd u_fb = Eigen::VectorXd::Constant(kNvC, 7.0);
  riccati.Compute(q, v, Eigen::VectorXd::Zero(kNqC), v, u_fb);
  EXPECT_TRUE((u_fb.array() == 0.0).all());
}
}  // namespace rtc::mpc
