/// @file test_barrier_costs.cpp
/// @brief The soft limits of the kinodynamic OCP: the piecewise-polynomial
///        barrier against its closed form and at its knots, the joint-limit
///        barrier on the public G1 (which joints it covers, where its
///        gradient lands, its derivatives against central differences), and
///        the sphere-pair barrier (gradient against central differences, the
///        Gauss–Newton Hessian against an independent outer-product sum,
///        full stance and coincident points); the relaxed log barrier
///        against its closed form and at its knot, and the contact wrench
///        cone barrier (value against an independent sum over the residual,
///        gradient against central differences, the input Hessian against
///        central differences of the gradient, the whole Hessian against an
///        independent Gauss–Newton sum, swing rows empty).
///
/// Pre-registered: barrier μ 1200, δ 0.1 (joint limits) and δ 0.04 (sphere
/// pairs); closed form and knot continuity to rel 1e-12 (the zero band to
/// 1e-12·μ·max(1, δ²)); the G1 gate set (seed 20261114, 250 samples) with the
/// joints drawn within ±0.1 rad of a limit; central differences with
/// h = 1e-6 on the state tangent (`space.integrate`), element-wise
/// `|g − g_fd| ≤ 1e-6 + 1e-5·|g_fd|`, every sample (sphere pairs: samples
/// with a pair closer than 1e-3 m skipped); the sphere-pair Hessian against
/// `Σ B''·∇h ∇hᵀ` from `LOCAL_WORLD_ALIGNED` point Jacobians, rel 1e-12.
/// Relaxed log barrier: friction μ 0.2, δ 5 N and CoP μ 0.6, δ 0.03 N·m (the
/// reference's); closed form rel 1e-12; at `h = δ` the value, first and
/// second derivative jump ≤ 1e-12·μ·max(1, 1/δ²); the wrench cone barrier
/// on the gate set (every mode in turn): value rel 1e-12, gradient (state
/// tangent and input) and input Hessian against central differences with
/// the tolerances above plus 1e-9 times the differenced value (|B| or
/// |∇B|∞: the rounding of a difference of a large barrier value), the
/// whole Hessian rel 1e-12 of its largest entry;
/// a swing contact's rows add nothing (value, gradient, Hessian bit zero in
/// flight).

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/barrier_costs.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/jacobian_compare.hpp"
#include "test_utils/kinodynamic_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Eigenvalues>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::CollisionBarrierCost;
using rtc::mpc::ContactConeBarrierCost;
using rtc::mpc::ContactRectangle;
using rtc::mpc::ContactWrenchConeData;
using rtc::mpc::ContactWrenchConeResidual;
using rtc::mpc::FrictionCone;
using rtc::mpc::JointLimitBarrierCost;
using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::PolynomialBarrier;
using rtc::mpc::RelaxedLogBarrier;
using rtc::mpc::SpherePair;
using tu::ColumnErr;
using tu::CompareColumns;
using tu::KinoSample;
using Ode = KinodynamicDynamics::Ode;
using Space = KinodynamicDynamics::Space;

constexpr std::uint32_t kGateSeed = 20261114U;
constexpr double kH = 1e-6;
constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-5;
constexpr double kMu = 1200.0;
constexpr double kJointDelta = 0.1;
constexpr double kPairDelta = 0.04;

bool BitwiseZero(double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  return bits == 0U;
}

bool AllBitwiseZero(const Eigen::Ref<const Eigen::MatrixXd>& m) {
  for (Eigen::Index j = 0; j < m.cols(); ++j) {
    for (Eigen::Index i = 0; i < m.rows(); ++i) {
      if (!BitwiseZero(m(i, j))) {
        return false;
      }
    }
  }
  return true;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& w) {
  Eigen::Matrix3d s;
  s << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return s;
}

double RelDiff(double got, double exp) {
  return std::abs(got - exp) / std::max(1.0, std::abs(exp));
}

/// The header's closed form, branch by branch.
struct ClosedForm {
  double mu;
  double d;

  [[nodiscard]] double LowValue(double h) const {
    return mu * (0.5 * h * h - d * h / 2.0 + d * d / 6.0);
  }

  [[nodiscard]] double LowFirst(double h) const { return mu * (h - d / 2.0); }

  [[nodiscard]] double LowSecond(double /*h*/) const { return mu; }

  [[nodiscard]] double MidValue(double h) const {
    return mu * (-h * h * h / (6.0 * d) + 0.5 * h * h - d * h / 2.0 + d * d / 6.0);
  }

  [[nodiscard]] double MidFirst(double h) const { return mu * (-h * h / (2.0 * d) + h - d / 2.0); }

  [[nodiscard]] double MidSecond(double h) const { return mu * (1.0 - h / d); }

  [[nodiscard]] double Value(double h) const {
    return h >= d ? 0.0 : (h <= 0.0 ? LowValue(h) : MidValue(h));
  }

  [[nodiscard]] double First(double h) const {
    return h >= d ? 0.0 : (h <= 0.0 ? LowFirst(h) : MidFirst(h));
  }

  [[nodiscard]] double Second(double h) const {
    return h >= d ? 0.0 : (h <= 0.0 ? LowSecond(h) : MidSecond(h));
  }
};

/// Central differences of a scalar @p f(x) on the state tangent.
template <typename F>
Eigen::VectorXd TangentGradientFd(const Space& space, const Eigen::VectorXd& x, F&& f) {
  const int ndx = space.ndx();
  Eigen::VectorXd g(ndx);
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
  Eigen::VectorXd xp(x.size());
  Eigen::VectorXd xm(x.size());
  for (int j = 0; j < ndx; ++j) {
    dx[j] = kH;
    space.integrate(x, dx, xp);
    dx[j] = -kH;
    space.integrate(x, dx, xm);
    dx[j] = 0.0;
    g[j] = (f(xp) - f(xm)) / (2.0 * kH);
  }
  return g;
}

/// Central differences of a vector @p f(x) on the state tangent (columns).
template <typename F>
Eigen::MatrixXd TangentJacobianFd(const Space& space, const Eigen::VectorXd& x, F&& f) {
  const int ndx = space.ndx();
  Eigen::MatrixXd jac(ndx, ndx);
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
  Eigen::VectorXd xp(x.size());
  Eigen::VectorXd xm(x.size());
  for (int j = 0; j < ndx; ++j) {
    dx[j] = kH;
    space.integrate(x, dx, xp);
    dx[j] = -kH;
    space.integrate(x, dx, xm);
    dx[j] = 0.0;
    const Eigen::VectorXd gp = f(xp);
    jac.col(j) = (gp - f(xm)) / (2.0 * kH);
  }
  return jac;
}

ColumnErr CompareVectors(const Eigen::VectorXd& g, const Eigen::VectorXd& g_fd) {
  return CompareColumns(Eigen::MatrixXd(g), Eigen::MatrixXd(g_fd), 0, 1, kAbsTol, kRelTol);
}

// ── Barrier ─────────────────────────────────────────────────────────────────

TEST(PolynomialBarrierTest, MatchesTheClosedFormAndIsZeroFromDelta) {
  const PolynomialBarrier b{kMu, kJointDelta};
  const ClosedForm cf{kMu, kJointDelta};
  const double d = kJointDelta;
  double worst = 0.0;
  for (const double h : {-d, 0.0, d / 2.0}) {
    const double ev = h <= 0.0 ? cf.LowValue(h) : cf.MidValue(h);
    const double e1 = h <= 0.0 ? cf.LowFirst(h) : cf.MidFirst(h);
    const double e2 = h <= 0.0 ? cf.LowSecond(h) : cf.MidSecond(h);
    worst = std::max({worst, RelDiff(b.Value(h), ev), RelDiff(b.Derivative(h), e1),
                      RelDiff(b.SecondDerivative(h), e2)});
    EXPECT_LE(RelDiff(b.Value(h), ev), 1e-12) << "h " << h;
    EXPECT_LE(RelDiff(b.Derivative(h), e1), 1e-12) << "h " << h;
    EXPECT_LE(RelDiff(b.SecondDerivative(h), e2), 1e-12) << "h " << h;
  }
  const double zero_tol = 1e-12 * kMu * std::max(1.0, d * d);
  double worst_zero = 0.0;
  for (const double h : {d, 2.0 * d}) {
    worst_zero = std::max({worst_zero, std::abs(b.Value(h)), std::abs(b.Derivative(h)),
                           std::abs(b.SecondDerivative(h))});
    EXPECT_LE(std::abs(b.Value(h)), zero_tol) << "h " << h;
    EXPECT_LE(std::abs(b.Derivative(h)), zero_tol) << "h " << h;
    EXPECT_LE(std::abs(b.SecondDerivative(h)), zero_tol) << "h " << h;
  }
  std::printf("barrier vs closed form: worst rel %.3e; zero band worst %.3e\n", worst, worst_zero);
}

// C² at both knots: the two branch polynomials agree there (the closed form
// itself), and the implementation's branches either side of each knot agree
// (the value at the knot against the one at the next double past it).
TEST(PolynomialBarrierTest, ContinuousToTheSecondDerivativeAtBothKnots) {
  const ClosedForm cf{kMu, kJointDelta};
  const double d = kJointDelta;
  const double tol = 1e-12 * kMu * std::max(1.0, d * d);
  double worst = 0.0;
  auto check = [&](double a, double b, const char* what) {
    worst = std::max(worst, std::abs(a - b));
    EXPECT_LE(std::abs(a - b), tol) << what << " " << a << " vs " << b;
  };
  check(cf.LowValue(0.0), cf.MidValue(0.0), "closed form B at 0");
  check(cf.LowFirst(0.0), cf.MidFirst(0.0), "closed form B' at 0");
  check(cf.LowSecond(0.0), cf.MidSecond(0.0), "closed form B'' at 0");
  check(cf.MidValue(d), 0.0, "closed form B at delta");
  check(cf.MidFirst(d), 0.0, "closed form B' at delta");
  check(cf.MidSecond(d), 0.0, "closed form B'' at delta");

  const PolynomialBarrier b{kMu, kJointDelta};
  const double above_zero = std::nextafter(0.0, 1.0);
  const double below_delta = std::nextafter(d, 0.0);
  check(b.Value(0.0), b.Value(above_zero), "B at 0");
  check(b.Derivative(0.0), b.Derivative(above_zero), "B' at 0");
  check(b.SecondDerivative(0.0), b.SecondDerivative(above_zero), "B'' at 0");
  check(b.Value(below_delta), b.Value(d), "B at delta");
  check(b.Derivative(below_delta), b.Derivative(d), "B' at delta");
  check(b.SecondDerivative(below_delta), b.SecondDerivative(d), "B'' at delta");
  std::printf("barrier knot continuity: worst jump %.3e (tol %.3e)\n", worst, tol);
}

// Beyond the registered gate: the derivatives against central differences
// of the implementation over the band.
TEST(PolynomialBarrierTest, DerivativesMatchCentralDifferences) {
  const PolynomialBarrier b{kMu, kJointDelta};
  double worst = 0.0;
  for (int i = 0; i <= 400; ++i) {
    const double h = -0.15 + 0.4 * i / 400.0;
    if (std::abs(h) < 1e-5 || std::abs(h - kJointDelta) < 1e-5) {
      continue;  // a jump of B''' within the step
    }
    const double d1 = (b.Value(h + kH) - b.Value(h - kH)) / (2.0 * kH);
    const double d2 = (b.Derivative(h + kH) - b.Derivative(h - kH)) / (2.0 * kH);
    const double r1 = std::abs(b.Derivative(h) - d1) / (kAbsTol + kRelTol * std::abs(d1));
    const double r2 = std::abs(b.SecondDerivative(h) - d2) / (kAbsTol + kRelTol * std::abs(d2));
    worst = std::max({worst, r1, r2});
    EXPECT_LE(r1, 1.0) << "h " << h;
    EXPECT_LE(r2, 1.0) << "h " << h;
  }
  std::printf("barrier derivatives vs FD: worst err/tol %.3e\n", worst);
}

// ── Relaxed log barrier ─────────────────────────────────────────────────────

/// The reference's relaxed log barrier, written out.
struct LogClosedForm {
  double mu;
  double d;

  [[nodiscard]] double Value(double h) const {
    if (h > d) {
      return -mu * std::log(h);
    }
    return mu * (-std::log(d) + 0.5 * std::pow((h - 2.0 * d) / d, 2) - 0.5);
  }

  [[nodiscard]] double First(double h) const {
    return h > d ? -mu / h : mu * (h - 2.0 * d) / (d * d);
  }

  [[nodiscard]] double Second(double h) const { return h > d ? mu / (h * h) : mu / (d * d); }
};

constexpr RelaxedLogBarrier kFrictionBarrier{0.2, 5.0};
constexpr RelaxedLogBarrier kCopBarrier{0.6, 0.03};

TEST(RelaxedLogBarrierTest, MatchesTheClosedForm) {
  double worst = 0.0;
  for (const RelaxedLogBarrier& b : {kFrictionBarrier, kCopBarrier}) {
    const LogClosedForm cf{b.mu, b.delta};
    const double d = b.delta;
    for (const double h : {-10.0 * d, -d, 0.0, 0.5 * d, d, 1.5 * d, 10.0 * d}) {
      const double r0 = RelDiff(b.Value(h), cf.Value(h));
      const double r1 = RelDiff(b.Derivative(h), cf.First(h));
      const double r2 = RelDiff(b.SecondDerivative(h), cf.Second(h));
      worst = std::max({worst, r0, r1, r2});
      EXPECT_LE(r0, 1e-12) << "delta " << d << " h " << h;
      EXPECT_LE(r1, 1e-12) << "delta " << d << " h " << h;
      EXPECT_LE(r2, 1e-12) << "delta " << d << " h " << h;
    }
  }
  std::printf("relaxed log barrier vs closed form: worst rel %.3e\n", worst);
}

// C² at the knot: the log branch just above δ against the quadratic at δ.
TEST(RelaxedLogBarrierTest, ContinuousToTheSecondDerivativeAtDelta) {
  double worst = 0.0;
  for (const RelaxedLogBarrier& b : {kFrictionBarrier, kCopBarrier}) {
    const double d = b.delta;
    const double tol = 1e-12 * b.mu * std::max(1.0, 1.0 / (d * d));
    const double above = std::nextafter(d, 2.0 * d);
    const std::array<std::pair<const char*, double>, 3> jumps = {{
        {"B", b.Value(above) - b.Value(d)},
        {"B'", b.Derivative(above) - b.Derivative(d)},
        {"B''", b.SecondDerivative(above) - b.SecondDerivative(d)},
    }};
    for (const auto& [what, jump] : jumps) {
      worst = std::max(worst, std::abs(jump) / tol);
      EXPECT_LE(std::abs(jump), tol) << what << " at delta " << d;
    }
  }
  std::printf("relaxed log barrier knot: worst jump / tol %.3e\n", worst);
}

TEST(RelaxedLogBarrierTest, DerivativesMatchCentralDifferences) {
  double worst = 0.0;
  for (const RelaxedLogBarrier& b : {kFrictionBarrier, kCopBarrier}) {
    const double d = b.delta;
    const double step = kH * std::max(1.0, d);
    for (int i = 0; i <= 400; ++i) {
      const double h = -2.0 * d + 6.0 * d * i / 400.0;
      if (std::abs(h - d) < 10.0 * step) {
        continue;  // the third derivative jumps within the step
      }
      const double d1 = (b.Value(h + step) - b.Value(h - step)) / (2.0 * step);
      const double d2 = (b.Derivative(h + step) - b.Derivative(h - step)) / (2.0 * step);
      const double r1 = std::abs(b.Derivative(h) - d1) / (kAbsTol + kRelTol * std::abs(d1));
      const double r2 = std::abs(b.SecondDerivative(h) - d2) / (kAbsTol + kRelTol * std::abs(d2));
      worst = std::max({worst, r1, r2});
      EXPECT_LE(r1, 1.0) << "delta " << d << " h " << h;
      EXPECT_LE(r2, 1.0) << "delta " << d << " h " << h;
    }
  }
  std::printf("relaxed log barrier derivatives vs FD: worst err/tol %.3e\n", worst);
}

// ── G1 fixture ──────────────────────────────────────────────────────────────

/// A limited one-entry joint, found independently of the cost.
struct LimitedJoint {
  Eigen::Index idx_q;
  Eigen::Index idx_v;
  double lower;
  double upper;
};

std::vector<LimitedJoint> IndependentLimitedJoints(const pinocchio::Model& model) {
  std::vector<LimitedJoint> out;
  for (int j = 1; j < model.njoints; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    if (model.nqs[ju] != 1 || model.nvs[ju] != 1) {
      continue;
    }
    const int iq = model.idx_qs[ju];
    const double lo = model.lowerPositionLimit[iq];
    const double hi = model.upperPositionLimit[iq];
    if (!std::isfinite(lo) || !std::isfinite(hi)) {
      continue;
    }
    out.push_back({iq, model.idx_vs[ju], lo, hi});
  }
  return out;
}

class BarrierCostsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(dyn_.Init(handler_), rtc::mpc::KinodynamicError::kNoError);
    oracle_.emplace(model_, tu::G1SoleFrames(model_), dyn_.force_size());
    samples_ = tu::G1GateSamples(*oracle_, kGateSeed);
    joints_ = IndependentLimitedJoints(model_);
  }

  /// The state of @p s with every limited joint within ±0.1 rad of one of
  /// its limits (side and offset drawn).
  Eigen::VectorXd NearLimits(const KinoSample& s, std::mt19937& rng) const {
    std::uniform_real_distribution<double> off(-0.1, 0.1);
    std::bernoulli_distribution side(0.5);
    Eigen::VectorXd x = s.x;
    for (const LimitedJoint& j : joints_) {
      x[j.idx_q] = (side(rng) ? j.lower : j.upper) + off(rng);
    }
    return x;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<tu::KinoOracle> oracle_;
  std::vector<KinoSample> samples_;
  std::vector<LimitedJoint> joints_;
};

// ── Joint limits ────────────────────────────────────────────────────────────

TEST_F(BarrierCostsTest, JointLimitCoversTheLimitedOneEntryJoints) {
  const JointLimitBarrierCost cost(*dyn_.space(), dyn_.nu(), model_,
                                   PolynomialBarrier{kMu, kJointDelta});
  const int independent = static_cast<int>(joints_.size());
  EXPECT_EQ(cost.n_joints(), independent);
  EXPECT_EQ(JointLimitBarrierCost::CountLimitedJoints(model_), independent);
  // The public G1: every actuated joint is limited, the free-flyer is not.
  EXPECT_EQ(independent, 23);
  EXPECT_EQ(independent, model_.nv - 6);
  std::printf("G1 limited one-entry joints: %d\n", independent);
}

TEST_F(BarrierCostsTest, JointLimitGradientPushesInwardAndLandsAtTheJointIndex) {
  const JointLimitBarrierCost cost(*dyn_.space(), dyn_.nu(), model_,
                                   PolynomialBarrier{kMu, kJointDelta});
  const ClosedForm cf{kMu, kJointDelta};
  const Eigen::VectorXd stand = tu::G1Stand(model_);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(model_.nq + model_.nv);
  // Every limited joint at mid-range (out of the band on the G1).
  x.head(model_.nq) = stand;
  for (const LimitedJoint& j : joints_) {
    x[j.idx_q] = 0.5 * (j.lower + j.upper);
    ASSERT_GE(0.5 * (j.upper - j.lower), kJointDelta) << "joint range inside the band";
  }
  const Eigen::VectorXd u = Eigen::VectorXd::Zero(dyn_.nu());
  for (std::size_t n = 0; n < joints_.size(); ++n) {
    const LimitedJoint& j = joints_[n];
    for (const bool at_lower : {true, false}) {
      Eigen::VectorXd xj = x;
      xj[j.idx_q] = at_lower ? j.lower + 0.05 : j.upper - 0.05;
      auto d = cost.createData();
      cost.evaluate(xj, u, *d);
      cost.computeGradients(xj, u, *d);
      cost.computeHessians(xj, u, *d);
      const double expected = at_lower ? cf.First(0.05) - cf.First(j.upper - j.lower - 0.05)
                                       : cf.First(j.upper - j.lower - 0.05) - cf.First(0.05);
      // Descent (−∇) moves the joint inward.
      if (at_lower) {
        EXPECT_LT(d->Lx_[j.idx_v], 0.0) << "joint " << n;
      } else {
        EXPECT_GT(d->Lx_[j.idx_v], 0.0) << "joint " << n;
      }
      EXPECT_LE(RelDiff(d->Lx_[j.idx_v], expected), 1e-12) << "joint " << n;
      EXPECT_LE(RelDiff(d->value_, cf.Value(0.05)), 1e-12) << "joint " << n;
      EXPECT_LE(RelDiff(d->Lxx_(j.idx_v, j.idx_v), cf.Second(0.05)), 1e-12) << "joint " << n;
      // Nothing anywhere else.
      Eigen::VectorXd grad = d->grad_;
      grad[j.idx_v] = 0.0;
      Eigen::MatrixXd hess = d->hess_;
      hess(j.idx_v, j.idx_v) = 0.0;
      EXPECT_TRUE(AllBitwiseZero(grad)) << "joint " << n;
      EXPECT_TRUE(AllBitwiseZero(hess)) << "joint " << n;
    }
  }
  // The last limited joint's entry is at its own tangent index.
  const LimitedJoint& last = joints_.back();
  EXPECT_EQ(last.idx_v, model_.nv - 1);
}

TEST_F(BarrierCostsTest, JointLimitLeavesTheFreeFlyerEntriesZero) {
  const JointLimitBarrierCost cost(*dyn_.space(), dyn_.nu(), model_,
                                   PolynomialBarrier{kMu, kJointDelta});
  Eigen::VectorXd x = Eigen::VectorXd::Zero(model_.nq + model_.nv);
  x.head(model_.nq) = tu::G1Stand(model_);
  const Eigen::VectorXd u = Eigen::VectorXd::Zero(dyn_.nu());
  for (const bool pushed : {false, true}) {
    Eigen::VectorXd xs = x;
    if (pushed) {
      for (const LimitedJoint& j : joints_) {
        xs[j.idx_q] = j.lower + 0.05;
      }
    }
    auto d = cost.createData();
    cost.evaluate(xs, u, *d);
    cost.computeGradients(xs, u, *d);
    cost.computeHessians(xs, u, *d);
    EXPECT_TRUE(AllBitwiseZero(d->Lx_.head(6))) << "pushed " << pushed;
    EXPECT_TRUE(AllBitwiseZero(d->Lxx_.topRows(6))) << "pushed " << pushed;
    EXPECT_TRUE(AllBitwiseZero(d->Lxx_.leftCols(6))) << "pushed " << pushed;
    EXPECT_TRUE(AllBitwiseZero(d->Lx_.tail(model_.nv))) << "pushed " << pushed;
    EXPECT_TRUE(AllBitwiseZero(d->Lu_)) << "pushed " << pushed;
    if (pushed) {
      EXPECT_TRUE((d->Lx_.segment(6, model_.nv - 6).array() < 0.0).all());
    }
  }
}

TEST_F(BarrierCostsTest, JointLimitDerivativesMatchCentralDifferences) {
  const Space& space = *dyn_.space();
  const JointLimitBarrierCost cost(space, dyn_.nu(), model_, PolynomialBarrier{kMu, kJointDelta});
  const ClosedForm cf{kMu, kJointDelta};
  std::mt19937 rng(kGateSeed);
  ColumnErr worst_g;
  ColumnErr worst_h;
  double worst_value = 0.0;
  int pass_g = 0;
  int pass_h = 0;
  int in_band = 0;
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const KinoSample& s = samples_[i];
    const Eigen::VectorXd x = NearLimits(s, rng);
    auto d = cost.createData();
    cost.evaluate(x, s.u, *d);
    cost.computeGradients(x, s.u, *d);
    cost.computeHessians(x, s.u, *d);
    // Value against the sum over the independent joint list.
    double v = 0.0;
    for (const LimitedJoint& j : joints_) {
      v += cf.Value(x[j.idx_q] - j.lower) + cf.Value(j.upper - x[j.idx_q]);
      in_band += cf.Second(x[j.idx_q] - j.lower) + cf.Second(j.upper - x[j.idx_q]) > 0.0 ? 1 : 0;
    }
    worst_value = std::max(worst_value, RelDiff(d->value_, v));
    EXPECT_LE(RelDiff(d->value_, v), 1e-12) << "sample " << i;

    auto fd = cost.createData();
    const Eigen::VectorXd g_fd = TangentGradientFd(space, x, [&](const Eigen::VectorXd& xx) {
      cost.evaluate(xx, s.u, *fd);
      return fd->value_;
    });
    const Eigen::MatrixXd h_fd = TangentJacobianFd(space, x, [&](const Eigen::VectorXd& xx) {
      cost.computeGradients(xx, s.u, *fd);
      return Eigen::VectorXd(fd->Lx_);
    });
    const ColumnErr eg = CompareVectors(d->Lx_, g_fd);
    const Eigen::MatrixXd hess = d->Lxx_;
    const ColumnErr eh =
        CompareColumns(hess, h_fd, 0, static_cast<int>(hess.cols()), kAbsTol, kRelTol);
    worst_g.Fold(eg);
    worst_h.Fold(eh);
    pass_g += eg.ratio <= 1.0 ? 1 : 0;
    pass_h += eh.ratio <= 1.0 ? 1 : 0;
    EXPECT_LE(eg.ratio, 1.0) << "sample " << i << " abs " << eg.abs;
    EXPECT_LE(eh.ratio, 1.0) << "sample " << i << " abs " << eh.abs;
  }
  EXPECT_EQ(pass_g, static_cast<int>(samples_.size()));
  EXPECT_EQ(pass_h, static_cast<int>(samples_.size()));
  EXPECT_GT(in_band, 0);
  std::printf(
      "joint limit: value worst rel %.3e; gradient vs FD worst err/tol %.3e (abs %.3e); "
      "Hessian vs FD worst err/tol %.3e (abs %.3e); joint sides in band %d / %zu\n",
      worst_value, worst_g.ratio, worst_g.abs, worst_h.ratio, worst_h.abs, in_band,
      2 * joints_.size() * samples_.size());
  RecordProperty("fd_worst_ratio_joint_limit_gradient", std::to_string(worst_g.ratio));
  RecordProperty("fd_worst_ratio_joint_limit_hessian", std::to_string(worst_h.ratio));
}

// ── Sphere pairs ────────────────────────────────────────────────────────────

/// The synthetic pair geometry (radii set per sample).
struct PairGeometry {
  const char* frame_a;
  Eigen::Vector3d offset_a;
  const char* frame_b;
  Eigen::Vector3d offset_b;
};

const std::array<PairGeometry, 3> kPairs{{
    {"left_sole", Eigen::Vector3d(0.05, 0.02, 0.01), "right_sole",
     Eigen::Vector3d(-0.03, 0.01, 0.02)},
    {"left_knee_link", Eigen::Vector3d(0.0, 0.03, -0.1), "right_knee_link",
     Eigen::Vector3d(0.02, -0.03, -0.1)},
    {"left_rubber_hand", Eigen::Vector3d(0.02, 0.0, 0.0), "pelvis",
     Eigen::Vector3d(0.0, 0.05, -0.05)},
}};

/// World points and their q-tangent Jacobians from LOCAL_WORLD_ALIGNED frame
/// Jacobians (independent of the cost's LOCAL route).
struct PointKinematics {
  Eigen::Vector3d p;
  Eigen::MatrixXd jac;  // 3 × nv
};

PointKinematics PointOf(const pinocchio::Model& model, pinocchio::Data& data,
                        pinocchio::FrameIndex fid, const Eigen::Vector3d& offset) {
  PointKinematics out;
  const pinocchio::SE3& m = data.oMf[fid];
  const Eigen::Vector3d r = m.rotation() * offset;
  out.p = m.translation() + r;
  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(6, model.nv);
  pinocchio::getFrameJacobian(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, j);
  out.jac = j.topRows(3) - Skew(r) * j.bottomRows(3);
  return out;
}

std::vector<SpherePair> MakePairs(const pinocchio::Model& model, const std::vector<double>& ra,
                                  const std::vector<double>& rb) {
  std::vector<SpherePair> pairs;
  for (std::size_t i = 0; i < kPairs.size(); ++i) {
    SpherePair p;
    p.frame_a = model.getFrameId(kPairs[i].frame_a);
    p.offset_a = kPairs[i].offset_a;
    p.radius_a = ra[i];
    p.frame_b = model.getFrameId(kPairs[i].frame_b);
    p.offset_b = kPairs[i].offset_b;
    p.radius_b = rb[i];
    pairs.push_back(p);
  }
  return pairs;
}

TEST_F(BarrierCostsTest, CollisionDerivativesMatchCentralDifferencesAndTheGaussNewtonSum) {
  const Space& space = *dyn_.space();
  const int nv = model_.nv;
  const ClosedForm cf{kMu, kPairDelta};
  for (std::size_t i = 0; i < kPairs.size(); ++i) {
    ASSERT_LT(model_.getFrameId(kPairs[i].frame_a), model_.frames.size());
    ASSERT_LT(model_.getFrameId(kPairs[i].frame_b), model_.frames.size());
  }
  std::mt19937 rng(kGateSeed);
  std::uniform_real_distribution<double> margin(-0.03, 0.06);
  ColumnErr worst_g;
  double worst_h_elem = 0.0;
  double worst_h_scaled = 0.0;
  double worst_grad_indep = 0.0;
  double worst_value = 0.0;
  double min_eig_ratio = 0.0;
  int pass_g = 0;
  int pass_h = 0;
  int used = 0;
  int skipped = 0;
  int pairs_in_band = 0;
  int pairs_penetrating = 0;
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const KinoSample& s = samples_[i];
    const Eigen::VectorXd q = s.x.head(model_.nq);
    pinocchio::Data pd(model_);
    pinocchio::computeJointJacobians(model_, pd, q);
    pinocchio::updateFramePlacements(model_, pd);
    std::vector<PointKinematics> pa;
    std::vector<PointKinematics> pb;
    bool too_close = false;
    for (const PairGeometry& g : kPairs) {
      pa.push_back(PointOf(model_, pd, model_.getFrameId(g.frame_a), g.offset_a));
      pb.push_back(PointOf(model_, pd, model_.getFrameId(g.frame_b), g.offset_b));
      too_close = too_close || (pa.back().p - pb.back().p).norm() < 1e-3;
    }
    // Draw the margins even for a skipped sample, so the draw is fixed.
    std::vector<double> ra(kPairs.size());
    std::vector<double> rb(kPairs.size());
    for (std::size_t k = 0; k < kPairs.size(); ++k) {
      const double dist = (pa[k].p - pb[k].p).norm();
      const double r = std::max(0.0, 0.5 * (dist - margin(rng)));
      ra[k] = r;
      rb[k] = r;
    }
    if (too_close) {
      ++skipped;
      continue;
    }
    ++used;
    // Not full stance: 0 … n_modes − 2.
    const auto mode = static_cast<std::uint32_t>(i % (dyn_.n_modes() - 1U));
    const Ode ode = dyn_.MakeOde(mode).value();
    CollisionBarrierCost bound(space, ode, MakePairs(model_, ra, rb),
                               PolynomialBarrier{kMu, kPairDelta});
    ASSERT_TRUE(bound.Bind(&ode));
    auto d = bound.createData();
    bound.evaluate(s.x, s.u, *d);
    bound.computeGradients(s.x, s.u, *d);
    bound.computeHessians(s.x, s.u, *d);

    // Independent value, gradient and Gauss–Newton Hessian.
    double v = 0.0;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(nv);
    Eigen::MatrixXd hgn = Eigen::MatrixXd::Zero(nv, nv);
    for (std::size_t k = 0; k < kPairs.size(); ++k) {
      const Eigen::Vector3d diff = pa[k].p - pb[k].p;
      const double dist = diff.norm();
      const double h = dist - ra[k] - rb[k];
      pairs_in_band += h < kPairDelta ? 1 : 0;
      pairs_penetrating += h < 0.0 ? 1 : 0;
      const Eigen::RowVectorXd grad_h = (diff / dist).transpose() * (pa[k].jac - pb[k].jac);
      v += cf.Value(h);
      g += cf.First(h) * grad_h.transpose();
      hgn += cf.Second(h) * grad_h.transpose() * grad_h;
    }
    worst_value = std::max(worst_value, RelDiff(d->value_, v));
    EXPECT_LE(RelDiff(d->value_, v), 1e-12) << "sample " << i;
    double grad_err = 0.0;
    for (int j = 0; j < nv; ++j) {
      grad_err = std::max(grad_err, RelDiff(d->Lx_[j], g[j]));
    }
    worst_grad_indep = std::max(worst_grad_indep, grad_err);
    EXPECT_LE(grad_err, 1e-12) << "sample " << i;

    // Gradient against central differences of the value.
    auto fd = bound.createData();
    const Eigen::VectorXd g_fd = TangentGradientFd(space, s.x, [&](const Eigen::VectorXd& xx) {
      bound.evaluate(xx, s.u, *fd);
      return fd->value_;
    });
    const ColumnErr eg = CompareVectors(d->Lx_, g_fd);
    worst_g.Fold(eg);
    pass_g += eg.ratio <= 1.0 ? 1 : 0;
    EXPECT_LE(eg.ratio, 1.0) << "sample " << i << " abs " << eg.abs;
    EXPECT_TRUE(AllBitwiseZero(d->Lu_)) << "sample " << i;

    // The Hessian: the Gauss–Newton sum on the q block, zero elsewhere, PSD.
    const Eigen::MatrixXd hq = d->Lxx_.topLeftCorner(nv, nv);
    double elem = 0.0;
    for (int c = 0; c < nv; ++c) {
      for (int r = 0; r < nv; ++r) {
        elem = std::max(elem, RelDiff(hq(r, c), hgn(r, c)));
      }
    }
    const double scale = std::max(1.0, hgn.cwiseAbs().maxCoeff());
    const double scaled = (hq - hgn).cwiseAbs().maxCoeff() / scale;
    worst_h_elem = std::max(worst_h_elem, elem);
    worst_h_scaled = std::max(worst_h_scaled, scaled);
    pass_h += elem <= 1e-12 ? 1 : 0;
    EXPECT_LE(elem, 1e-12) << "sample " << i;
    Eigen::MatrixXd rest = d->hess_;
    rest.topLeftCorner(nv, nv).setZero();
    EXPECT_TRUE(AllBitwiseZero(rest)) << "sample " << i;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(0.5 * (hq + hq.transpose()));
    const double min_ratio = eig.eigenvalues().minCoeff() / scale;
    min_eig_ratio = std::min(min_eig_ratio, min_ratio);
    EXPECT_GE(min_ratio, -1e-12) << "sample " << i;
    EXPECT_LE((hq - hq.transpose()).cwiseAbs().maxCoeff(), 1e-12 * scale) << "sample " << i;
  }
  EXPECT_EQ(pass_g, used);
  EXPECT_EQ(pass_h, used);
  EXPECT_GE(used, 200);
  EXPECT_GT(pairs_in_band, 0);
  EXPECT_GT(pairs_penetrating, 0);
  std::printf(
      "collision: %d samples (%d skipped); pairs in band %d, penetrating %d of %zu; value worst "
      "rel %.3e; gradient vs independent worst rel %.3e; gradient vs FD worst err/tol %.3e (abs "
      "%.3e); Hessian vs GN sum worst rel element-wise %.3e, matrix-scaled %.3e; min "
      "eigenvalue / scale %.3e\n",
      used, skipped, pairs_in_band, pairs_penetrating,
      kPairs.size() * static_cast<std::size_t>(used), worst_value, worst_grad_indep, worst_g.ratio,
      worst_g.abs, worst_h_elem, worst_h_scaled, min_eig_ratio);
  RecordProperty("fd_worst_ratio_collision_gradient", std::to_string(worst_g.ratio));
}

// Full stance on the bound ODE (after an active evaluation on the same data):
// value, gradient and Hessian are all +0.
TEST_F(BarrierCostsTest, CollisionIsZeroInFullStance) {
  const Space& space = *dyn_.space();
  const KinoSample& s = samples_[0];
  Ode ode = dyn_.MakeOde(0b01).value();
  // Radii that put every pair in the band at this sample.
  const Eigen::VectorXd q = s.x.head(model_.nq);
  pinocchio::Data pd(model_);
  pinocchio::framesForwardKinematics(model_, pd, q);
  std::vector<double> r(kPairs.size());
  for (std::size_t k = 0; k < kPairs.size(); ++k) {
    const Eigen::Vector3d a = pd.oMf[model_.getFrameId(kPairs[k].frame_a)].act(kPairs[k].offset_a);
    const Eigen::Vector3d b = pd.oMf[model_.getFrameId(kPairs[k].frame_b)].act(kPairs[k].offset_b);
    r[k] = std::max(0.0, 0.5 * ((a - b).norm() - 0.01));
  }
  CollisionBarrierCost cost(space, ode, MakePairs(model_, r, r),
                            PolynomialBarrier{kMu, kPairDelta});
  ASSERT_TRUE(cost.Bind(&ode));
  auto d = cost.createData();
  cost.evaluate(s.x, s.u, *d);
  cost.computeGradients(s.x, s.u, *d);
  cost.computeHessians(s.x, s.u, *d);
  ASSERT_GT(d->value_, 0.0);
  ASSERT_FALSE(d->Lx_.isZero());
  ASSERT_FALSE(d->Lxx_.isZero());
  ASSERT_TRUE(dyn_.SetMode(ode, dyn_.n_modes() - 1U));
  cost.evaluate(s.x, s.u, *d);
  cost.computeGradients(s.x, s.u, *d);
  cost.computeHessians(s.x, s.u, *d);
  EXPECT_TRUE(BitwiseZero(d->value_));
  EXPECT_TRUE(AllBitwiseZero(d->grad_));
  EXPECT_TRUE(AllBitwiseZero(d->hess_));
  // And back.
  ASSERT_TRUE(dyn_.SetMode(ode, 0b10));
  cost.evaluate(s.x, s.u, *d);
  EXPECT_GT(d->value_, 0.0);
}

// Two spheres on the same point of the same frame: the margin is −(r_a + r_b),
// the value finite, the direction undefined and the gradient zero.
TEST_F(BarrierCostsTest, CoincidentPointsGiveAFiniteValueAndZeroGradient) {
  const Space& space = *dyn_.space();
  const Ode ode = dyn_.MakeOde(0b00).value();
  SpherePair p;
  p.frame_a = model_.getFrameId("left_sole");
  p.frame_b = p.frame_a;
  p.offset_a = Eigen::Vector3d(0.01, -0.02, 0.03);
  p.offset_b = p.offset_a;
  p.radius_a = 0.02;
  p.radius_b = 0.03;
  CollisionBarrierCost cost(space, ode, {p}, PolynomialBarrier{kMu, kPairDelta});
  ASSERT_TRUE(cost.Bind(&ode));
  const ClosedForm cf{kMu, kPairDelta};
  for (std::size_t i = 0; i < 10; ++i) {
    const KinoSample& s = samples_[i];
    auto d = cost.createData();
    cost.evaluate(s.x, s.u, *d);
    cost.computeGradients(s.x, s.u, *d);
    cost.computeHessians(s.x, s.u, *d);
    EXPECT_TRUE(std::isfinite(d->value_));
    EXPECT_LE(RelDiff(d->value_, cf.Value(-0.05)), 1e-12);
    EXPECT_TRUE(AllBitwiseZero(d->grad_));
    EXPECT_TRUE(AllBitwiseZero(d->hess_));
  }
}

TEST_F(BarrierCostsTest, CollisionBindingContracts) {
  const Space& space = *dyn_.space();
  const Ode ode = dyn_.MakeOde(0b00).value();
  CollisionBarrierCost cost(space, ode, {}, PolynomialBarrier{kMu, kPairDelta});
  EXPECT_EQ(cost.bound_ode(), nullptr);
  EXPECT_THROW((void)cost.createData(), std::logic_error);
  const Ode one(space, model_, model_.gravity.linear(), std::vector<bool>{false},
                std::vector<pinocchio::FrameIndex>{oracle_->contacts()[0]}, 6);
  EXPECT_FALSE(cost.Bind(&one));
  EXPECT_EQ(cost.bound_ode(), nullptr);
  ASSERT_TRUE(cost.Bind(&ode));
  EXPECT_EQ(cost.bound_ode(), &ode);
  auto d = cost.createData();
  ASSERT_TRUE(cost.Bind(nullptr));
  cost.evaluate(samples_[0].x, samples_[0].u, *d);
  EXPECT_TRUE(std::isnan(d->value_));
}

// ── Contact wrench cone barrier ─────────────────────────────────────────────

constexpr FrictionCone kCone{0.4, 0.0, 25.0};
constexpr double kRoundoff = 1e-9;  ///< FD absolute bound per unit of the differenced value
constexpr ContactRectangle kRect{-0.09, 0.09, -0.03, 0.03};

/// The cost's value, gradient and Hessian at one `(x, u)`.
struct ConeEval {
  double value{0.0};
  Eigen::VectorXd grad;
  Eigen::MatrixXd hess;
};

ConeEval EvalCone(const ContactConeBarrierCost& cost, const Eigen::VectorXd& x,
                  const Eigen::VectorXd& u) {
  auto d = cost.createData();
  cost.evaluate(x, u, *d);
  cost.computeGradients(x, u, *d);
  cost.computeHessians(x, u, *d);
  return {d->value_, d->grad_, d->hess_};
}

std::string DescribeErr(const ColumnErr& e) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "err/tol %.3e abs %.3e", e.ratio, e.abs);
  return buf;
}

// Value: Σ B(−c) over the stance contacts' rows of an independently
// evaluated residual. Gradient: central differences in the state tangent and
// the input. Input Hessian: central differences of the input gradient (the
// margins are linear in the CoP torques and forces at a fixed q, and the
// friction margin's √ curvature is kept, so the input block is exact). The
// whole Hessian: the Gauss–Newton sum over the residual's own Jacobian rows
// plus the friction √ curvature, assembled here.
TEST_F(BarrierCostsTest, ContactConeMatchesTheResidualAndCentralDifferences) {
  const Space& space = *dyn_.space();
  const int ndx = space.ndx();
  const int nu = dyn_.nu();
  const int fs = dyn_.force_size();
  const int nc = handler_.n_contacts();
  const LogClosedForm cff{kFrictionBarrier.mu, kFrictionBarrier.delta};
  const LogClosedForm cfc{kCopBarrier.mu, kCopBarrier.delta};
  double worst_value = 0.0;
  double worst_hess = 0.0;
  ColumnErr worst_gx;
  ColumnErr worst_gu;
  ColumnErr worst_huu;
  int pass_g = 0;
  int pass_h = 0;
  int friction_quadratic = 0;
  int cop_quadratic = 0;
  int stance_contacts = 0;
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const KinoSample& s = samples_[i];
    const auto mode = static_cast<std::uint32_t>(i % dyn_.n_modes());
    const Ode ode = dyn_.MakeOde(mode).value();
    ContactWrenchConeResidual res(ode, kCone, kRect);
    ASSERT_TRUE(res.Bind(&ode));
    const ContactConeBarrierCost cost(space, res, kFrictionBarrier, kCopBarrier);
    ASSERT_EQ(cost.bound_ode(), &ode);
    const ConeEval e = EvalCone(cost, s.x, s.u);

    // Independent value and Gauss–Newton Hessian from the residual.
    auto rd = res.createData();
    res.evaluate(s.x, s.u, *rd);
    res.computeJacobians(s.x, s.u, *rd);
    const auto& r = static_cast<const ContactWrenchConeData&>(*rd);
    const int rows = res.rows_per_contact();
    double v = 0.0;
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(ndx + nu, ndx + nu);
    for (int c = 0; c < nc; ++c) {
      if (((mode >> c) & 1U) == 0U) {
        continue;
      }
      ++stance_contacts;
      for (int k = 0; k < rows; ++k) {
        const int row = c * rows + k;
        const double h = -r.value_[row];
        const LogClosedForm& cf = k == 0 ? cff : cfc;
        (k == 0 ? friction_quadratic : cop_quadratic) += h <= cf.d ? 1 : 0;
        v += cf.Value(h);
        Eigen::RowVectorXd jac(ndx + nu);
        jac << -r.Jx_.row(row), -r.Ju_.row(row);
        hess += cf.Second(h) * jac.transpose() * jac;
        if (k == 0) {
          const Eigen::Vector2d f = s.u.segment<2>(c * fs);
          const double sq2 = f.squaredNorm() + kCone.regularization;
          const double sq = std::sqrt(sq2);
          const Eigen::Matrix2d hs = (Eigen::Matrix2d::Identity() - f * f.transpose() / sq2) / sq;
          hess.block<2, 2>(ndx + c * fs, ndx + c * fs) -= cf.First(h) * hs;
        }
      }
    }
    worst_value = std::max(worst_value, RelDiff(e.value, v));
    EXPECT_LE(RelDiff(e.value, v), 1e-12) << "sample " << i << " mode " << mode;
    const double scale = std::max(1.0, hess.cwiseAbs().maxCoeff());
    const double herr = (e.hess - hess).cwiseAbs().maxCoeff() / scale;
    worst_hess = std::max(worst_hess, herr);
    EXPECT_LE(herr, 1e-12) << "sample " << i << " mode " << mode;

    // Gradient and input Hessian against central differences.
    const auto value_at = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
      auto d = cost.createData();
      cost.evaluate(x, u, *d);
      return d->value_;
    };
    const Eigen::VectorXd gx_fd =
        TangentGradientFd(space, s.x, [&](const Eigen::VectorXd& x) { return value_at(x, s.u); });
    Eigen::VectorXd gu_fd(nu);
    Eigen::MatrixXd huu_fd(nu, nu);
    for (int j = 0; j < nu; ++j) {
      Eigen::VectorXd up = s.u;
      Eigen::VectorXd um = s.u;
      up[j] += kH;
      um[j] -= kH;
      gu_fd[j] = (value_at(s.x, up) - value_at(s.x, um)) / (2.0 * kH);
      huu_fd.col(j) =
          (EvalCone(cost, s.x, up).grad.tail(nu) - EvalCone(cost, s.x, um).grad.tail(nu)) /
          (2.0 * kH);
    }
    // The barrier's deep quadratic branch reaches |B| ~ 1e6 on this draw;
    // a central difference of it carries a rounding error of ~ε·|B|/h
    // (2e-10·|B|), so the absolute part of the bound grows with the value
    // differenced (the gradient's for the Hessian).
    const double tol_g = kAbsTol + kRoundoff * std::abs(e.value);
    const double tol_h = kAbsTol + kRoundoff * e.grad.cwiseAbs().maxCoeff();
    const ColumnErr ex = CompareColumns(Eigen::MatrixXd(e.grad.head(ndx)), Eigen::MatrixXd(gx_fd),
                                        0, 1, tol_g, kRelTol);
    const ColumnErr eu = CompareColumns(Eigen::MatrixXd(e.grad.tail(nu)), Eigen::MatrixXd(gu_fd), 0,
                                        1, tol_g, kRelTol);
    const ColumnErr eh = CompareColumns(Eigen::MatrixXd(e.hess.bottomRightCorner(nu, nu)), huu_fd,
                                        0, nu, tol_h, kRelTol);
    worst_gx.Fold(ex);
    worst_gu.Fold(eu);
    worst_huu.Fold(eh);
    pass_g += ex.ratio <= 1.0 && eu.ratio <= 1.0 ? 1 : 0;
    pass_h += eh.ratio <= 1.0 ? 1 : 0;
    EXPECT_LE(ex.ratio, 1.0) << "sample " << i << " mode " << mode << " grad x " << DescribeErr(ex);
    EXPECT_LE(eu.ratio, 1.0) << "sample " << i << " mode " << mode << " grad u " << DescribeErr(eu);
    EXPECT_LE(eh.ratio, 1.0) << "sample " << i << " mode " << mode << " hess uu "
                             << DescribeErr(eh);
  }
  std::printf(
      "contact cone barrier: %zu samples, %d stance contacts (quadratic branch: friction %d, "
      "CoP rows %d); value worst rel %.3e; Hessian vs Gauss-Newton sum worst %.3e; gradient vs "
      "FD %d/%zu (x %s; u %s); input Hessian vs FD %d/%zu (%s)\n",
      samples_.size(), stance_contacts, friction_quadratic, cop_quadratic, worst_value, worst_hess,
      pass_g, samples_.size(), DescribeErr(worst_gx).c_str(), DescribeErr(worst_gu).c_str(), pass_h,
      samples_.size(), DescribeErr(worst_huu).c_str());
}

TEST_F(BarrierCostsTest, ContactConeIsEmptyForASwingContactAndInFlight) {
  const Space& space = *dyn_.space();
  const KinoSample& s = samples_[0];
  Ode ode = dyn_.MakeOde(0b00).value();
  const ContactWrenchConeResidual res(ode, kCone, kRect);
  ContactConeBarrierCost cost(space, res, kFrictionBarrier, kCopBarrier);
  ASSERT_TRUE(cost.Bind(&ode));
  ConeEval e = EvalCone(cost, s.x, s.u);
  EXPECT_TRUE(BitwiseZero(e.value));
  EXPECT_TRUE(AllBitwiseZero(e.grad));
  EXPECT_TRUE(AllBitwiseZero(e.hess));
  // One contact in stance: the other's input columns carry nothing.
  ASSERT_TRUE(dyn_.SetMode(ode, 0b01));
  e = EvalCone(cost, s.x, s.u);
  EXPECT_NE(e.value, 0.0);
  const int ndx = space.ndx();
  const int nu = dyn_.nu();
  const int fs = dyn_.force_size();
  EXPECT_FALSE(e.grad.segment(ndx, fs).isZero());
  EXPECT_TRUE(AllBitwiseZero(e.grad.segment(ndx + fs, fs)));
  EXPECT_TRUE(AllBitwiseZero(e.hess.block(ndx + fs, 0, fs, ndx + nu)));
  EXPECT_TRUE(AllBitwiseZero(e.grad.tail(nu - 2 * fs)));
}

TEST_F(BarrierCostsTest, ContactConeBindingContracts) {
  const Space& space = *dyn_.space();
  const Ode ode = dyn_.MakeOde(0b11).value();
  const ContactWrenchConeResidual res(ode, kCone, kRect);
  ContactConeBarrierCost cost(space, res, kFrictionBarrier, kCopBarrier);
  EXPECT_EQ(cost.bound_ode(), nullptr);
  EXPECT_THROW((void)cost.createData(), std::logic_error);
  const Ode one(space, model_, model_.gravity.linear(), std::vector<bool>{true},
                std::vector<pinocchio::FrameIndex>{oracle_->contacts()[0]}, 6);
  EXPECT_FALSE(cost.Bind(&one));
  EXPECT_EQ(cost.bound_ode(), nullptr);
  ASSERT_TRUE(cost.Bind(&ode));
  EXPECT_EQ(cost.residual().bound_ode(), &ode);
  EXPECT_EQ(cost.friction_barrier().delta, kFrictionBarrier.delta);
  EXPECT_EQ(cost.cop_barrier().mu, kCopBarrier.mu);
  auto d = cost.createData();
  ASSERT_TRUE(cost.Bind(nullptr));
  cost.evaluate(samples_[0].x, samples_[0].u, *d);
  EXPECT_TRUE(std::isnan(d->value_));
}

}  // namespace
