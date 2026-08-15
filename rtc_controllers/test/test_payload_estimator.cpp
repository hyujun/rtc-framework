// Layer 2A of #135 — quasi-static payload estimation from the momentum-observer
// residual.
//
// THE ORACLE IS CONSTRUCTED FORWARD, NEVER SOLVED FOR. Every test states a
// payload it wants (mass, CoM offset, gravity direction), builds the wrench that
// payload applies to the robot, and maps it to a residual with r = Jᵀw — the
// same direction the physics runs. The estimator then has to invert it. Writing
// the test the other way round (pick r, assert whatever comes out) would pin the
// implementation to itself and stay green through a sign flip, which is the one
// error #135 explicitly asked to be pinned.
//
// The Jacobian is built from the geometric definition of a LOCAL_WORLD_ALIGNED
// frame Jacobian for revolute joints — column i is [z_i × (p_f − p_i); z_i] —
// rather than a hand-typed matrix, so a reader can check the fixture against a
// textbook instead of trusting seven rows of constants.

#include "rtc_controllers/estimation/payload_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

using rtc::estimation::PayloadEstimator;
using rtc::estimation::PayloadInvalidReason;

constexpr int kNv = 7;
const Eigen::Vector3d kGravityZ(0.0, 0.0, -9.81);

PayloadEstimator::Config MakeConfig() {
  PayloadEstimator::Config c;
  c.sigma0 = 1e-3;         // well below the fixture's σ_min → λ² = 0 → exact LS
  c.lambda_max = 0.05;     // the value every shipped §6.5 consumer drives
  c.min_sigma = 1e-6;
  c.max_fit_error = 1e-6;  // synthetic data reconstructs to machine precision
  c.min_gravity = 1e-3;
  return c;
}

/// Joint axes and origins of a deliberately non-degenerate 7-DoF arm, plus the
/// payload frame origin. Values are arbitrary but fixed; what matters is that
/// no two axes are parallel and the frame is off every axis, so J has full row
/// rank and the wrench is observable in all six directions.
struct Chain {
  std::vector<Eigen::Vector3d> axis;
  std::vector<Eigen::Vector3d> origin;
  Eigen::Vector3d frame{0.35, 0.12, 0.85};
};

Chain MakeChain() {
  Chain c;
  const double d[kNv][6] = {
      {0, 0, 1, 0.00, 0.00, 0.00}, {0, 1, 0, 0.00, 0.02, 0.15},
      {0, 0, 1, 0.01, 0.00, 0.34}, {0, -1, 0, 0.03, 0.01, 0.52},
      {0, 0, 1, 0.00, 0.02, 0.66}, {0, 1, 0, 0.05, 0.00, 0.74},
      {1, 0, 0, 0.10, 0.04, 0.80},
  };
  for (const auto& row : d) {
    c.axis.emplace_back(Eigen::Vector3d(row[0], row[1], row[2]).normalized());
    c.origin.emplace_back(row[3], row[4], row[5]);
  }
  return c;
}

/// J (6×nv), LOCAL_WORLD_ALIGNED at `chain.frame`: linear rows first.
Eigen::MatrixXd FrameJacobian(const Chain& chain) {
  Eigen::MatrixXd J(6, kNv);
  for (int i = 0; i < kNv; ++i) {
    const Eigen::Vector3d& z = chain.axis[static_cast<std::size_t>(i)];
    J.block<3, 1>(0, i) = z.cross(chain.frame - chain.origin[static_cast<std::size_t>(i)]);
    J.block<3, 1>(3, i) = z;
  }
  return J;
}

/// The wrench a payload of `mass` whose CoM sits at `com_offset` from the frame
/// origin applies TO THE ROBOT: a downward force plus the moment of that force
/// about the frame origin.
Eigen::Matrix<double, 6, 1> PayloadWrench(double mass, const Eigen::Vector3d& gravity,
                                          const Eigen::Vector3d& com_offset) {
  Eigen::Matrix<double, 6, 1> w;
  const Eigen::Vector3d f = mass * gravity;
  w.head<3>() = f;
  w.tail<3>() = com_offset.cross(f);
  return w;
}

/// The residual that wrench produces — the forward map [MO-2]/[WRENCH-A] share.
std::vector<double> ResidualFor(const Eigen::MatrixXd& J,
                                const Eigen::Matrix<double, 6, 1>& w) {
  const Eigen::VectorXd tau = J.transpose() * w;
  return std::vector<double>(tau.data(), tau.data() + tau.size());
}

}  // namespace

// ── AC1: the sign convention, pinned ────────────────────────────────────────
//
// A 2.5 kg payload hanging under gravity must come back as +2.5 kg. This is the
// assertion #135 asked for: it fails if the residual is ever reinterpreted as
// the torque the robot applies to the environment, because then every recovered
// mass is negative.
TEST(PayloadEstimatorTest, RecoversKnownHangingMass) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const double mass = 2.5;
  const auto w_true = PayloadWrench(mass, kGravityZ, Eigen::Vector3d::Zero());
  const auto r = ResidualFor(J, w_true);

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, kGravityZ);

  ASSERT_TRUE(est.valid()) << "reason " << static_cast<int>(est.invalid_reason());
  const auto& e = est.estimate();
  EXPECT_GT(e.mass, 0.0) << "a hanging payload must have POSITIVE estimated mass";
  EXPECT_NEAR(e.mass, mass, 1e-9);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(e.wrench[static_cast<std::size_t>(i)], w_true(i), 1e-9) << "component " << i;
  // The exactness above is only meaningful if the pose was far from singular,
  // i.e. the adaptive law contributed no shrinkage. State that, don't assume it.
  EXPECT_GT(e.sigma_min, MakeConfig().sigma0);
  EXPECT_DOUBLE_EQ(e.lambda_sq, 0.0);
  EXPECT_LT(e.fit_error, 1e-9);
}

// The other half of the sign contract: reverse the residual and the mass must
// go negative. Without this, a test suite that only ever sees +2.5 would pass on
// an implementation that returned |m|.
TEST(PayloadEstimatorTest, ReversedResidualYieldsNegativeMass) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const auto w_true = PayloadWrench(2.5, kGravityZ, Eigen::Vector3d::Zero());
  auto r = ResidualFor(J, w_true);
  for (double& v : r)
    v = -v;

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, kGravityZ);

  ASSERT_TRUE(est.valid());
  EXPECT_NEAR(est.estimate().mass, -2.5, 1e-9);
}

// A CoM offset shows up as a moment, and must NOT disturb the mass: [MASS-A]
// reads the force rows only. Pins that the force/moment split is not confused.
TEST(PayloadEstimatorTest, ComOffsetProducesMomentWithoutBiasingMass) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const Eigen::Vector3d com(0.04, -0.03, 0.02);
  const auto w_true = PayloadWrench(1.2, kGravityZ, com);
  const auto r = ResidualFor(J, w_true);

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, kGravityZ);

  ASSERT_TRUE(est.valid());
  EXPECT_NEAR(est.estimate().mass, 1.2, 1e-9);
  const auto& w = est.estimate().wrench;
  EXPECT_GT(std::abs(w[3]) + std::abs(w[4]) + std::abs(w[5]), 1e-3) << "moment must be non-zero";
  for (int i = 3; i < 6; ++i)
    EXPECT_NEAR(w[static_cast<std::size_t>(i)], w_true(i), 1e-9);
}

// ARCH-1 in test form: gravity comes from the model, so a URDF whose gravity is
// along −Y (or a weaker world) must need no code change. A hard-coded 9.81·−Z
// anywhere in [MASS-A] fails this.
TEST(PayloadEstimatorTest, NonZGravityRecoversSameMass) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const Eigen::Vector3d g_y(0.0, -3.71, 0.0);  // different axis AND magnitude
  const auto w_true = PayloadWrench(4.0, g_y, Eigen::Vector3d::Zero());
  const auto r = ResidualFor(J, w_true);

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, g_y);

  ASSERT_TRUE(est.valid());
  EXPECT_NEAR(est.estimate().mass, 4.0, 1e-9);
}

// ── AC3: the guards ─────────────────────────────────────────────────────────

// A residual no Cartesian wrench could have produced — the signature of an
// unmodelled joint-level torque (armature / damping / friction, [MO-1u]). The
// projection cannot absorb it, and the tick must be refused rather than
// reported as a payload. Built from the null space of J so it is orthogonal to
// everything Jᵀ can reach, which is exactly what a joint-local effect looks
// like.
TEST(PayloadEstimatorTest, ResidualOutsideJacobianRangeIsRejected) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const Eigen::VectorXd n = J.fullPivLu().kernel().col(0).normalized();
  ASSERT_LT((J * n).norm(), 1e-9) << "fixture: n must lie in the null space of J";
  const std::vector<double> r(n.data(), n.data() + n.size());

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, kGravityZ);

  EXPECT_FALSE(est.valid());
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kPoorFit);
}

TEST(PayloadEstimatorTest, RankDeficientJacobianIsRejected) {
  const Chain chain = MakeChain();
  Eigen::MatrixXd J = FrameJacobian(chain);
  J.row(5).setZero();  // one wrench direction leaves no trace in r at all

  PayloadEstimator est;
  auto cfg = MakeConfig();
  cfg.min_sigma = 1e-3;
  est.Init(kNv, cfg);
  est.Update(J, std::vector<double>(kNv, 0.1), kGravityZ);

  EXPECT_FALSE(est.valid());
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kRankDeficient);
}

TEST(PayloadEstimatorTest, DegenerateGravityIsRejected) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const auto r = ResidualFor(J, PayloadWrench(2.0, kGravityZ, Eigen::Vector3d::Zero()));

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, Eigen::Vector3d::Zero());

  EXPECT_FALSE(est.valid());
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kDegenerateGravity);
}

TEST(PayloadEstimatorTest, NonFiniteInputsAreRejected) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  auto r = ResidualFor(J, PayloadWrench(2.0, kGravityZ, Eigen::Vector3d::Zero()));

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());

  auto r_nan = r;
  r_nan[3] = std::nan("");
  est.Update(J, r_nan, kGravityZ);
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kNonFiniteInput);

  est.Update(J, r, Eigen::Vector3d(0.0, 0.0, std::nan("")));
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kNonFiniteInput);

  Eigen::MatrixXd J_nan = J;
  J_nan(2, 2) = std::numeric_limits<double>::infinity();
  est.Update(J_nan, r, kGravityZ);
  EXPECT_FALSE(est.valid()) << "a non-finite Jacobian must never reach the LLT";
}

TEST(PayloadEstimatorTest, ShortResidualIsRejected) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, std::vector<double>(kNv - 1, 0.0), kGravityZ);

  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kShortInput);
}

// ── Lifecycle / caller-gate contract ────────────────────────────────────────

// A rejected tick must not erase the last good estimate: "could not tell" and
// "no payload" are different claims, and a consumer that reads mass==0 after a
// closed gate would act on the wrong one.
TEST(PayloadEstimatorTest, HoldPreservesLastEstimateAndReportsReason) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const auto r = ResidualFor(J, PayloadWrench(3.0, kGravityZ, Eigen::Vector3d::Zero()));

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, kGravityZ);
  ASSERT_TRUE(est.valid());

  est.Hold(PayloadInvalidReason::kHandMoving);
  EXPECT_FALSE(est.valid());
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kHandMoving);
  EXPECT_NEAR(est.estimate().mass, 3.0, 1e-9) << "the estimate must FREEZE, not zero";
}

TEST(PayloadEstimatorTest, HoldWithNoneIsRecordedAsHeld) {
  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Hold(PayloadInvalidReason::kNone);
  EXPECT_FALSE(est.valid()) << "Hold() must never be able to assert validity";
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kHeld);
}

TEST(PayloadEstimatorTest, ResetDropsTheLatchedEstimate) {
  const Chain chain = MakeChain();
  const Eigen::MatrixXd J = FrameJacobian(chain);
  const auto r = ResidualFor(J, PayloadWrench(3.0, kGravityZ, Eigen::Vector3d::Zero()));

  PayloadEstimator est;
  est.Init(kNv, MakeConfig());
  est.Update(J, r, kGravityZ);
  ASSERT_TRUE(est.valid());

  est.ResetRtState();
  EXPECT_FALSE(est.valid());
  EXPECT_EQ(est.estimate().mass, 0.0);
}

TEST(PayloadEstimatorTest, UseBeforeInitIsRejected) {
  PayloadEstimator est;
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kNotInitialized);
  est.Hold(PayloadInvalidReason::kHandMoving);
  EXPECT_EQ(est.invalid_reason(), PayloadInvalidReason::kNotInitialized);
}

// Every threshold is load-bearing, so none may be silently disabled. NaN is
// listed explicitly because `<= 0.0` alone would let it through and leave the
// gate permanently open while reading as configured.
TEST(PayloadEstimatorTest, InitRejectsUnusableThresholds) {
  PayloadEstimator est;
  EXPECT_THROW(est.Init(0, MakeConfig()), std::invalid_argument);
  EXPECT_THROW(est.Init(kNv + 1000, MakeConfig()), std::invalid_argument);

  const double bad[] = {0.0, -1.0, std::nan(""), std::numeric_limits<double>::infinity()};
  for (double b : bad) {
    for (int which = 0; which < 5; ++which) {
      auto c = MakeConfig();
      switch (which) {
        case 0: c.sigma0 = b; break;
        case 1: c.lambda_max = b; break;
        case 2: c.min_sigma = b; break;
        case 3: c.max_fit_error = b; break;
        default: c.min_gravity = b; break;
      }
      EXPECT_THROW(est.Init(kNv, c), std::invalid_argument)
          << "threshold " << which << " accepted " << b;
    }
  }
}
