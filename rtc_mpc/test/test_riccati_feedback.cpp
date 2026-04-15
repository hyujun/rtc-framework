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

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <array>

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
  EXPECT_LE(magnitude, 1.0 + 1e-9)
      << "Clamped feedback must not exceed max_delta_x_norm";
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
    EXPECT_NEAR(wide(i), 0.0, kTol)
        << "Tail of oversized u_fb must be zeroed in accel_only mode";
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
}  // namespace rtc::mpc
