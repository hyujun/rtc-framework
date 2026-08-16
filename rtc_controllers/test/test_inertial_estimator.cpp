// ── Layer 2B InertialEstimator (#455) ───────────────────────────────────────
//
// The regressor is built here in closed form rather than through pinocchio, and
// that is deliberate: this class is model-agnostic, and the ONE structural fact
// it is built around — a single pose can only ever pin 3 of the 4 parameters —
// is visible directly in the algebra.
//
// For a rigid body hanging at a frame under gravity alone the frame wrench is
//     f = m·g            τ = (m·c) × g = −skew(g)·(m·c)
// so the 6×4 body regressor block is
//     B(g) = [ g      0        ]
//            [ 0     −skew(g)  ]
// skew(g) has rank 2 (its null space is span{g}), so rank B = 3 at EVERY pose,
// and the direction it loses is exactly (m·c) ∥ g. Two non-parallel g's are
// needed to make it 4. Taking J = I lets Y equal B, so the fixture below is the
// physics with nothing in between.
#include "rtc_controllers/estimation/inertial_estimator.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace {

using rtc::estimation::InertialEstimator;
using rtc::estimation::InertialInvalidReason;

constexpr int kNv = 6;

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;
  s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return s;
}

/// Quasi-static payload regressor for frame-local gravity `g`: nv×10 with the
/// six inertia columns zero, exactly as RtModelHandle produces at v = a = 0.
Eigen::MatrixXd RegressorFor(const Eigen::Vector3d& g) {
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(kNv, 10);
  Y.block<3, 1>(0, 0) = g;
  Y.block<3, 3>(3, 1) = -Skew(g);
  return Y;
}

/// φ₄ = [m, m·c] for a known payload.
Eigen::Vector4d TrueParams(double mass, const Eigen::Vector3d& com) {
  Eigen::Vector4d phi;
  phi[0] = mass;
  phi.tail<3>() = mass * com;
  return phi;
}

std::vector<double> Residual(const Eigen::MatrixXd& Y, const Eigen::Vector4d& phi) {
  const Eigen::VectorXd r = Y.leftCols<4>() * phi;
  return {r.data(), r.data() + r.size()};
}

InertialEstimator::Config MakeConfig() {
  InertialEstimator::Config c;
  c.forgetting_factor = 1.0;
  c.min_param_sigma = 1e-6;
  c.min_mass = 1e-3;
  c.max_com_offset = 1.0;
  c.max_inertia_column = 1e-9;
  return c;
}

// 서로 평행하지 않은 프레임-로컬 중력 방향들 — 자세를 바꾼다는 것의 대역이다.
const Eigen::Vector3d kG1(0.0, 0.0, -9.81);
const Eigen::Vector3d kG2(0.0, -9.81, 0.0);
const Eigen::Vector3d kG3(-6.94, 0.0, -6.94);

class InertialEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override { est_.Init(kNv, MakeConfig()); }

  void Feed(const Eigen::Vector3d& g, const Eigen::Vector4d& phi) {
    const Eigen::MatrixXd Y = RegressorFor(g);
    est_.Update(Y, Residual(Y, phi));
  }

  InertialEstimator est_;
};

// ═══════════════════════════════════════════════════════════════════════════════
// [C2c] — 단일 자세로는 절대 안 된다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(InertialEstimatorTest, OnePoseNeverIdentifiesFourParameters) {
  const Eigen::Vector4d phi = TrueParams(1.7, {0.03, -0.05, 0.11});

  // 같은 자세를 500 tick 먹여도 rank 는 3 에서 안 올라간다 — 시간이 아니라 자세가 문제다.
  for (int i = 0; i < 500; ++i) {
    Feed(kG1, phi);
  }
  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kInsufficientPoseDiversity);
  EXPECT_EQ(est_.estimate().param_rank, 3);
}

TEST_F(InertialEstimatorTest, SecondPoseCompletesTheRankAndRecoversTheParameters) {
  const double mass = 1.7;
  const Eigen::Vector3d com(0.03, -0.05, 0.11);
  const Eigen::Vector4d phi = TrueParams(mass, com);

  Feed(kG1, phi);
  ASSERT_EQ(est_.estimate().param_rank, 3) << "one pose must not be enough";

  Feed(kG2, phi);
  ASSERT_TRUE(est_.valid()) << "reason=" << static_cast<int>(est_.invalid_reason());
  EXPECT_EQ(est_.estimate().param_rank, 4);

  EXPECT_NEAR(est_.estimate().mass, mass, 1e-9);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(est_.estimate().first_moment[static_cast<std::size_t>(i)], mass * com[i], 1e-9);
    EXPECT_NEAR(est_.estimate().com[static_cast<std::size_t>(i)], com[i], 1e-9);
  }
  EXPECT_LT(est_.estimate().fit_error, 1e-9);
}

// 놓친 방향은 임의가 아니라 g 방향이다 — 이 테스트가 그 사실을 박는다.
TEST_F(InertialEstimatorTest, TheUnobservedDirectionIsAlongGravity) {
  const double mass = 2.0;
  // 두 payload 는 m·c 가 kG1(=−Z) 방향으로만 다르다 → 한 자세에서 구분 불가.
  const Eigen::Vector4d phi_a = TrueParams(mass, {0.04, 0.02, 0.0});
  const Eigen::Vector4d phi_b = TrueParams(mass, {0.04, 0.02, 0.25});

  const Eigen::MatrixXd Y = RegressorFor(kG1);
  const std::vector<double> r_a = Residual(Y, phi_a);
  const std::vector<double> r_b = Residual(Y, phi_b);

  double diff = 0.0;
  for (std::size_t i = 0; i < r_a.size(); ++i) {
    diff = std::max(diff, std::abs(r_a[i] - r_b[i]));
  }
  EXPECT_LT(diff, 1e-12) << "두 payload 가 이 자세에서 같은 residual 을 낸다는 것이 요점";

  // 자세를 바꾸면 갈라진다.
  const Eigen::MatrixXd Y2 = RegressorFor(kG2);
  const std::vector<double> s_a = Residual(Y2, phi_a);
  const std::vector<double> s_b = Residual(Y2, phi_b);
  double diff2 = 0.0;
  for (std::size_t i = 0; i < s_a.size(); ++i) {
    diff2 = std::max(diff2, std::abs(s_a[i] - s_b[i]));
  }
  EXPECT_GT(diff2, 1.0);
}

TEST_F(InertialEstimatorTest, ThreePosesStillAgree) {
  const double mass = 0.8;
  const Eigen::Vector3d com(-0.02, 0.07, 0.04);
  const Eigen::Vector4d phi = TrueParams(mass, com);
  for (const auto& g : {kG1, kG2, kG3}) {
    Feed(g, phi);
  }
  ASSERT_TRUE(est_.valid());
  EXPECT_NEAR(est_.estimate().mass, mass, 1e-9);
  EXPECT_NEAR(est_.estimate().com[2], com[2], 1e-9);
}

// ═══════════════════════════════════════════════════════════════════════════════
// AC 10 — dynamic data 에 quasi-static 모델을 적용하지 않는다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(InertialEstimatorTest, RefusesTicksWithInertialExcitation) {
  const Eigen::Vector4d phi = TrueParams(1.7, {0.03, -0.05, 0.11});
  Feed(kG1, phi);
  Feed(kG2, phi);
  ASSERT_TRUE(est_.valid());

  // 관성 열이 살아 있는 tick — 즉 팔이 quasi-static 이 아니었다.
  Eigen::MatrixXd Y = RegressorFor(kG3);
  Y(0, 4) = 0.5;
  est_.Update(Y, Residual(Y, phi));

  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kDynamicExcitation);
}

TEST_F(InertialEstimatorTest, RejectedTickDoesNotPoisonTheAccumulator) {
  const double mass = 1.7;
  const Eigen::Vector3d com(0.03, -0.05, 0.11);
  const Eigen::Vector4d phi = TrueParams(mass, com);
  Feed(kG1, phi);
  Feed(kG2, phi);
  ASSERT_TRUE(est_.valid());

  Eigen::MatrixXd bad = RegressorFor(kG3);
  bad(0, 4) = 0.5;
  est_.Update(bad, Residual(bad, phi));
  ASSERT_FALSE(est_.valid());

  // 거부된 tick 이 누적기를 건드리지 않았다면 정상 tick 하나로 곧바로 복귀한다.
  Feed(kG3, phi);
  ASSERT_TRUE(est_.valid());
  EXPECT_NEAR(est_.estimate().mass, mass, 1e-9);
}

// ═══════════════════════════════════════════════════════════════════════════════
// [C3] physical consistency
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(InertialEstimatorTest, NegativeMassIsRejectedNotReported) {
  const Eigen::Vector4d phi = TrueParams(-0.9, {0.01, 0.02, 0.03});
  Feed(kG1, phi);
  Feed(kG2, phi);
  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kNonPositiveMass);
}

TEST_F(InertialEstimatorTest, ImplausibleComOffsetIsRejected) {
  // max_com_offset = 1.0 m. 2.5 m 짜리 지렛대는 payload 가 아니라 모델 오차다.
  const Eigen::Vector4d phi = TrueParams(0.5, {0.0, 0.0, 2.5});
  Feed(kG1, phi);
  Feed(kG2, phi);
  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kComOutOfBounds);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 입력 게이트 · 수명
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(InertialEstimatorTest, NonFiniteInputIsNamed) {
  const Eigen::Vector4d phi = TrueParams(1.0, {0.0, 0.0, 0.05});
  Eigen::MatrixXd Y = RegressorFor(kG1);
  Y(2, 0) = std::numeric_limits<double>::quiet_NaN();
  est_.Update(Y, Residual(RegressorFor(kG1), phi));
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kNonFiniteInput);
}

TEST_F(InertialEstimatorTest, WrongShapeIsNamed) {
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(kNv, 4);  // 10열이 아니다
  est_.Update(Y, std::vector<double>(kNv, 0.0));
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kShortInput);
}

TEST_F(InertialEstimatorTest, ResetDropsTheAccumulator) {
  const Eigen::Vector4d phi = TrueParams(1.7, {0.03, -0.05, 0.11});
  Feed(kG1, phi);
  Feed(kG2, phi);
  ASSERT_TRUE(est_.valid());

  est_.ResetRtState();
  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kInsufficientPoseDiversity);
  EXPECT_EQ(est_.estimate().param_rank, 0);

  // 그리고 자세 다양성을 처음부터 다시 요구한다.
  Feed(kG1, phi);
  EXPECT_FALSE(est_.valid());
}

TEST_F(InertialEstimatorTest, HoldCarriesTheCallersReason) {
  est_.Hold(InertialInvalidReason::kUpstreamInvalid);
  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kUpstreamInvalid);

  // kNone 으로 Hold 하면 "검사도 안 하고 valid" 가 되므로 kHeld 로 접힌다.
  est_.Hold(InertialInvalidReason::kNone);
  EXPECT_FALSE(est_.valid());
  EXPECT_EQ(est_.invalid_reason(), InertialInvalidReason::kHeld);
}

TEST(InertialEstimatorInitTest, RejectsUnusableThresholds) {
  InertialEstimator e;
  auto bad = [](auto mutate) {
    InertialEstimator::Config c = MakeConfig();
    mutate(c);
    return c;
  };
  EXPECT_THROW(e.Init(0, MakeConfig()), std::invalid_argument);
  EXPECT_THROW(e.Init(kNv, bad([](auto& c) { c.forgetting_factor = 0.0; })), std::invalid_argument);
  // 1 초과는 누적기를 발산시켜 자세 게이트를 나이 게이트로 바꾼다.
  EXPECT_THROW(e.Init(kNv, bad([](auto& c) { c.forgetting_factor = 1.5; })), std::invalid_argument);
  EXPECT_THROW(e.Init(kNv, bad([](auto& c) { c.min_param_sigma = 0.0; })), std::invalid_argument);
  EXPECT_THROW(e.Init(kNv, bad([](auto& c) {
                        c.max_com_offset = std::numeric_limits<double>::quiet_NaN();
                      })),
               std::invalid_argument);
  // 그리고 정상 설정은 통과해야 한다 (위 EXPECT_THROW 들이 다른 이유로 통과하는 것 방지).
  EXPECT_NO_THROW(e.Init(kNv, MakeConfig()));
}

TEST_F(InertialEstimatorTest, UninitializedUpdateIsNamed) {
  InertialEstimator fresh;
  const Eigen::MatrixXd Y = RegressorFor(kG1);
  fresh.Update(Y, std::vector<double>(kNv, 0.0));
  EXPECT_EQ(fresh.invalid_reason(), InertialInvalidReason::kNotInitialized);
}

}  // namespace
