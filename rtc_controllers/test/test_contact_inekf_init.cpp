// ── Contact InEKF core: initialisation, IMU→base helper, API contract ────────
//
// Stationary window accumulator, the two independent stationary gates, gravity
// alignment with the configured heading, the origin from stance points, the
// P₀ structure (Monte Carlo of the tilt / accel-bias correlation), the rigid
// IMU→base helper against an analytic trajectory, Init() refusals, call-order
// statuses, Reset(), the quaternion output and the slot noise scale.
#include "contact_inekf_testing.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace rtc::estimation::testing {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kYawCfg = 0.7;

ContactInekf Configured(int n = 2) {
  ContactInekf f;
  f.Init(FixtureConfig(), n);
  return f;
}

Mat3 Rpy(double roll, double pitch, double yaw) {
  return rtc::math::se3::RpyToRotationZyx(Vec3(roll, pitch, yaw));
}

/// Gravity-aligned specific force for a truth attitude.
Vec3 RestAccel(const Mat3& R) {
  return R.transpose() * Vec3(0.0, 0.0, kG0);
}

// ── Accumulator ──────────────────────────────────────────────────────────────

TEST(ContactInekfAccumulator, MeanMatchesLongDoubleAndRefusesNonFinite) {
  std::mt19937_64 rng(kSeed);
  std::normal_distribution<double> nd(0.0, 1.0);
  StationaryImuAccumulator acc;
  std::array<long double, 6> ref{};
  constexpr int kN = 1000;
  for (int k = 0; k < kN; ++k) {
    const Vec3 g(nd(rng) * 0.011, nd(rng) * 0.011, nd(rng) * 0.011);
    const Vec3 a(0.3 + nd(rng) * 0.22, -0.2 + nd(rng) * 0.22, 9.80 + nd(rng) * 0.22);
    ASSERT_TRUE(acc.Add(g, a));
    for (int i = 0; i < 3; ++i) {
      ref[static_cast<std::size_t>(i)] += g[i];
      ref[static_cast<std::size_t>(i + 3)] += a[i];
    }
  }
  double worst = 0.0;
  const Vec3 mg = acc.MeanGyro();
  const Vec3 ma = acc.MeanAccel();
  for (int i = 0; i < 3; ++i) {
    const long double rg = ref[static_cast<std::size_t>(i)] / kN;
    const long double ra = ref[static_cast<std::size_t>(i + 3)] / kN;
    worst =
        std::max(worst, static_cast<double>(std::abs(mg[i] - rg) / std::max(1.0L, std::abs(rg))));
    worst =
        std::max(worst, static_cast<double>(std::abs(ma[i] - ra) / std::max(1.0L, std::abs(ra))));
  }
  std::printf("[measure] accumulator mean |d|/max(1,|ref|) = %.3e (gate 1e-15)\n", worst);
  EXPECT_LE(worst, 1e-15);

  // A non-finite sample is refused and changes nothing.
  EXPECT_FALSE(acc.Add(Vec3(kNan, 0, 0), Vec3(0, 0, kG0)));
  EXPECT_FALSE(acc.Add(Vec3::Zero(), Vec3(0, kInf, 0)));
  EXPECT_EQ(acc.count(), kN);
  EXPECT_TRUE(acc.MeanGyro() == mg);
  EXPECT_TRUE(acc.MeanAccel() == ma);

  acc.Reset();
  EXPECT_EQ(acc.count(), 0);
  EXPECT_TRUE(acc.MeanAccel() == Vec3::Zero());
  EXPECT_TRUE(acc.Add(Vec3::Zero(), Vec3(0, 0, kG0)));
  EXPECT_TRUE(acc.MeanAccel() == Vec3(0, 0, kG0));
}

// ── Stationary gates (independent) ───────────────────────────────────────────

TEST(ContactInekfInitialize, StationaryGatesAreIndependent) {
  const Mat3 Rt = Rpy(0.1, -0.2, kYawCfg);
  const Vec3 up = RestAccel(Rt).normalized();
  const Vec3 still(0.0, 0.0, 0.0);

  struct Case {
    Vec3 gyro;
    Vec3 accel;
    InekfStatus want;
  };

  const std::vector<Case> cases{
      {still, up * (kG0 + 0.4), InekfStatus::kOk},
      {still, up * (kG0 - 0.4), InekfStatus::kOk},
      {still, up * (kG0 + 0.6), InekfStatus::kAccelNotStationary},
      {still, up * (kG0 - 0.6), InekfStatus::kAccelNotStationary},
      {Vec3(0.03, 0.02, -0.02), up * kG0, InekfStatus::kOk},
      {Vec3(0.0, 0.049, 0.0), up * kG0, InekfStatus::kOk},
      {Vec3(0.0, 0.0, 0.051), up * kG0, InekfStatus::kGyroNotStationary},
      {Vec3(0.2, 0.0, 0.0), up * kG0, InekfStatus::kGyroNotStationary},
      {still, Vec3(0.0, kNan, kG0), InekfStatus::kNonFiniteInput},
  };
  for (const Case& c : cases) {
    ContactInekf f = Configured();
    const Snapshot before = Capture(f);
    EXPECT_EQ(f.Initialize(c.gyro, c.accel, 2.0, kYawCfg, {}), c.want);
    if (c.want != InekfStatus::kOk) {
      EXPECT_TRUE(BitEqual(Capture(f), before));
    } else {
      EXPECT_TRUE(f.initialized());
    }
  }
}

// ── Gravity alignment, heading, origin ───────────────────────────────────────

TEST(ContactInekfInitialize, AlignsGravityAndKeepsConfiguredYaw) {
  double worst_up = 0.0;
  double worst_yaw = 0.0;
  for (int i = 0; i < 16; ++i) {
    const double tilt = (80.0 * static_cast<double>(i) / 15.0) * M_PI / 180.0;
    const double dir = 0.4 * static_cast<double>(i);
    const Mat3 Rt = Rpy(tilt * std::cos(dir), tilt * std::sin(dir), kYawCfg);
    const Vec3 a = RestAccel(Rt);
    ContactInekf f = Configured();
    ASSERT_EQ(f.Initialize(Vec3::Zero(), a, 2.0, kYawCfg, {}), InekfStatus::kOk) << i;
    const Mat3& R0 = f.rotation();
    worst_up = std::max(worst_up, (R0.transpose() * Vec3::UnitZ() - a.normalized()).norm());
    worst_yaw = std::max(worst_yaw, std::abs(std::atan2(R0(1, 0), R0(0, 0)) - kYawCfg));
  }
  std::printf("[measure] 16 tilts 0-80 deg: |R0^T e_z - a/|a|| %.3e  |yaw - cfg| %.3e (1e-14)\n",
              worst_up, worst_yaw);
  EXPECT_LE(worst_up, 1e-14);
  EXPECT_LE(worst_yaw, 1e-14);
}

TEST(ContactInekfInitialize, OriginIsMeanOfStanceAnchors) {
  std::mt19937_64 rng(kSeed);
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  double worst = 0.0;
  for (int trial = 0; trial < 64; ++trial) {
    const int n = 1 + trial % 4;
    std::vector<Vec3> stance;
    for (int i = 0; i < n; ++i) {
      stance.emplace_back(0.3 * ud(rng), 0.3 * ud(rng), -0.5 + 0.3 * ud(rng));
    }
    const Mat3 Rt = Rpy(0.3 * ud(rng), 0.3 * ud(rng), kYawCfg);
    ContactInekf f = Configured(4);
    ASSERT_EQ(f.Initialize(Vec3::Zero(), RestAccel(Rt), 2.0, kYawCfg, stance), InekfStatus::kOk);
    Vec3 mean = Vec3::Zero();
    for (const Vec3& r : stance) {
      mean += f.position() + f.rotation() * r;
    }
    worst = std::max(worst, (mean / n).cwiseAbs().maxCoeff());
    EXPECT_TRUE(f.velocity() == Vec3::Zero());
    EXPECT_TRUE(f.gyro_bias() == Vec3::Zero());
    EXPECT_TRUE(f.accel_bias() == Vec3::Zero());
    for (int s = 0; s < 4; ++s) {
      EXPECT_FALSE(f.contact_active(s));  // no anchors are created
    }
  }
  std::printf("[measure] origin: max |mean anchor| = %.3e m (gate 1e-15)\n", worst);
  EXPECT_LE(worst, 1e-15);
}

// ── P₀ ───────────────────────────────────────────────────────────────────────

TEST(ContactInekfInitialize, PriorBlocksAreTheConfiguredVariances) {
  const ContactInekf::Config cfg = FixtureConfig();
  ContactInekf f = Configured(2);
  ASSERT_EQ(f.Initialize(Vec3::Zero(), Vec3(0.0, 0.0, kG0), 2.0, kYawCfg, {}), InekfStatus::kOk);
  const Cov& P = f.covariance();
  const int bg = f.BiasOffset();
  const double vv = cfg.prior_velocity_sigma * cfg.prior_velocity_sigma;
  const double pp = cfg.prior_position_sigma * cfg.prior_position_sigma;
  const double gg = cfg.prior_gyro_bias_sigma * cfg.prior_gyro_bias_sigma;
  const double yy = cfg.prior_yaw_sigma * cfg.prior_yaw_sigma;
  EXPECT_TRUE((P.block<3, 3>(3, 3) == vv * Mat3::Identity()));
  EXPECT_TRUE((P.block<3, 3>(6, 6) == pp * Mat3::Identity()));
  EXPECT_TRUE((P.block<3, 3>(bg, bg) == gg * Mat3::Identity()));
  EXPECT_EQ(P(2, 2), yy);  // level: the world-z attitude row has no bias part
  EXPECT_TRUE(P.middleRows(ContactInekf::SlotOffset(0), 6).isZero(0.0));
  EXPECT_TRUE(P == P.transpose());
}

/// Alignment map of the preamble, rebuilt here from RpyToRotationZyx.
Mat3 AlignOracle(const Vec3& a, double yaw) {
  return Rpy(std::atan2(a.y(), a.z()), std::atan2(-a.x(), std::hypot(a.y(), a.z())), yaw);
}

TEST(ContactInekfInitialize, AlignmentJacobianMatchesFiniteDifference) {
  const ContactInekf::Config cfg = FixtureConfig();
  const double var_ba = cfg.prior_accel_bias_sigma * cfg.prior_accel_bias_sigma;
  double worst = 0.0;
  for (double tilt_deg : {0.0, 5.0, 20.0, 45.0, 70.0}) {
    const double tl = tilt_deg * M_PI / 180.0;
    const Vec3 a = RestAccel(Rpy(0.6 * tl, 0.8 * tl, kYawCfg)) + Vec3(0.01, -0.02, 0.015);
    ContactInekf f = Configured(2);
    ASSERT_EQ(f.Initialize(Vec3::Zero(), a, 2.0, kYawCfg, {}), InekfStatus::kOk);
    const Mat3 M = f.covariance().block<3, 3>(0, f.BiasOffset() + 3) / var_ba;
    const Mat3 R0 = AlignOracle(a, kYawCfg);
    Mat3 Mfd;
    const double h = 1e-6 * a.norm();
    for (int k = 0; k < 3; ++k) {
      const Vec3 e = Vec3::Unit(k) * h;
      const Vec3 xp = rtc::math::se3::log3(AlignOracle(a - e, kYawCfg) * R0.transpose());
      const Vec3 xm = rtc::math::se3::log3(AlignOracle(a + e, kYawCfg) * R0.transpose());
      Mfd.col(k) = (xp - xm) / (2.0 * h);
    }
    worst = std::max(worst, (M - Mfd).cwiseAbs().maxCoeff() / Mfd.cwiseAbs().maxCoeff());
  }
  std::printf("[measure] alignment Jacobian vs FD rel = %.3e (extra check, 1e-7)\n", worst);
  EXPECT_LE(worst, 1e-7);
}

double WhitenedStat(const std::vector<Eigen::Matrix<double, 6, 1>>& err,
                    const std::vector<Eigen::Matrix<double, 6, 6>>& cov) {
  Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
  for (std::size_t m = 0; m < err.size(); ++m) {
    const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(cov[m]);
    const Eigen::Matrix<double, 6, 1> w = llt.matrixL().solve(err[m]);
    C.noalias() += w * w.transpose();
  }
  C /= static_cast<double>(err.size());
  C -= Eigen::Matrix<double, 6, 6>::Identity();
  return Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>>(C)
      .eigenvalues()
      .cwiseAbs()
      .maxCoeff();
}

// Truth: attitude with roll/pitch from the tilt and ZYX yaw = configured + N(0, σ_yaw²),
// b_a ~ N(0, σ_ba²), a window of N = 1000 samples at 500 Hz with per-sample
// σ = 1e-2·√500 m/s². The filter's P₀ must describe (ξ_φ, ζ_a).
TEST(ContactInekfInitialize, PriorMatchesMonteCarloOfTiltAndAccelBias) {
  const ContactInekf::Config cfg = FixtureConfig();
  constexpr int kTrials = 4000;
  constexpr int kWindow = 1000;
  constexpr double kSamplePeriod = 2e-3;
  const double sample_sigma = cfg.accel_noise_density * std::sqrt(1.0 / kSamplePeriod);
  const double gyro_sigma = cfg.gyro_noise_density * std::sqrt(1.0 / kSamplePeriod);
  std::mt19937_64 rng(kSeed);
  std::normal_distribution<double> nd(0.0, 1.0);
  bool pass = true;
  for (double tilt_deg : {0.0, 5.0, 20.0, 45.0, 70.0}) {
    const double tl = tilt_deg * M_PI / 180.0;
    std::vector<Eigen::Matrix<double, 6, 1>> err;
    std::vector<Eigen::Matrix<double, 6, 6>> full;
    std::vector<Eigen::Matrix<double, 6, 6>> nocross;
    ContactInekf f = Configured(2);
    StationaryImuAccumulator acc;
    for (int m = 0; m < kTrials; ++m) {
      const Mat3 Rt = Rpy(0.6 * tl, 0.8 * tl, kYawCfg + cfg.prior_yaw_sigma * nd(rng));
      const Vec3 ba = Vec3(nd(rng), nd(rng), nd(rng)) * cfg.prior_accel_bias_sigma;
      const Vec3 f_true = RestAccel(Rt);
      acc.Reset();
      for (int k = 0; k < kWindow; ++k) {
        const Vec3 gn(nd(rng), nd(rng), nd(rng));
        const Vec3 an(nd(rng), nd(rng), nd(rng));
        if (!acc.Add(gn * gyro_sigma, f_true + ba + an * sample_sigma)) {
          FAIL() << "finite sample refused";
        }
      }
      f.Reset();
      ASSERT_EQ(f.Initialize(acc.MeanGyro(), acc.MeanAccel(), kWindow * kSamplePeriod, kYawCfg, {}),
                InekfStatus::kOk);
      Eigen::Matrix<double, 6, 1> e;
      e.head<3>() = rtc::math::se3::log3(Rt * f.rotation().transpose());
      e.tail<3>() = ba;  // b̂_a = 0
      err.push_back(e);
      std::array<int, 6> idx{0, 1, 2, f.BiasOffset() + 3, f.BiasOffset() + 4, f.BiasOffset() + 5};
      Eigen::Matrix<double, 6, 6> P;
      for (int c = 0; c < 6; ++c) {
        for (int r = 0; r < 6; ++r) {
          P(r, c) =
              f.covariance()(idx[static_cast<std::size_t>(r)], idx[static_cast<std::size_t>(c)]);
        }
      }
      full.push_back(P);
      P.block<3, 3>(0, 3).setZero();
      P.block<3, 3>(3, 0).setZero();
      nocross.push_back(P);
    }
    const double s_full = WhitenedStat(err, full);
    const double s_nocross = WhitenedStat(err, nocross);
    std::printf("[measure] tilt %4.1f deg: whitened %.3f (gate 0.17)  without cross term %.3f\n",
                tilt_deg, s_full, s_nocross);
    pass = pass && s_full <= 0.17;
    // The gate has teeth: dropping the tilt–bias correlation is caught.
    EXPECT_GT(s_nocross, 0.17) << tilt_deg;
  }
  EXPECT_TRUE(pass);
}

// ── IMU → base ───────────────────────────────────────────────────────────────

TEST(ContactInekfBaseHelper, MatchesAnalyticRigidTrajectory) {
  const Mat3 R_BI = Rpy(0.05, -0.1, 0.2);
  const Vec3 p_BI(0.045, 0.0, -0.083);
  const Vec3 p_IB = -(R_BI.transpose() * p_BI);
  const Mat3 Rc = Rpy(0.3, 0.2, -0.4);
  double worst_v = 0.0;
  double worst_p = 0.0;
  double max_rate = 0.0;
  bool rot_bit = true;
  for (const auto& [alpha, beta] : std::vector<std::pair<double, double>>{
           {0.0, 0.0}, {1.0, 0.5}, {-3.0, 7.0}, {20.0, -15.0}, {29.6, 18.6}}) {
    for (double t : {0.0, 0.13, 0.71, 2.3}) {
      auto Rz = [&](double ang) { return RotZ(ang); };
      auto Ry = [&](double ang) { return Rpy(0.0, ang, 0.0); };
      const Mat3 dRz = alpha * rtc::math::se3::hat(Vec3::UnitZ()) * Rz(alpha * t);
      const Mat3 dRy = beta * rtc::math::se3::hat(Vec3::UnitY()) * Ry(beta * t);
      const Mat3 R_WI = Rz(alpha * t) * Ry(beta * t) * Rc;
      const Mat3 dR_WI = dRz * Ry(beta * t) * Rc + Rz(alpha * t) * dRy * Rc;
      const Vec3 omega_I = rtc::math::se3::vee(R_WI.transpose() * dR_WI);
      max_rate = std::max(max_rate, omega_I.norm());
      const Vec3 p_WI(std::cos(t), std::sin(2.0 * t), 0.3 * t * t);
      const Vec3 v_WI(-std::sin(t), 2.0 * std::cos(2.0 * t), 0.6 * t);
      const BaseState b = ContactInekf::BaseFromImu(R_WI, p_WI, v_WI, omega_I, R_BI, p_BI);
      const Vec3 v_ref = v_WI + dR_WI * p_IB;  // d/dt (p_WI + R_WI p_IB)
      const Vec3 p_ref = p_WI + R_WI * p_IB;
      worst_v = std::max(worst_v, (b.v_WB - v_ref).norm() / (1.0 + v_ref.norm()));
      worst_p = std::max(worst_p, (b.p_WB - p_ref).norm() / (1.0 + p_ref.norm()));
      const Mat3 R_WB = R_WI * R_BI.transpose();
      rot_bit = rot_bit && b.R_WB == R_WB;
    }
  }
  std::printf("[measure] base helper: v rel %.3e  p rel %.3e (1e-12)  max |w| %.1f rad/s\n",
              worst_v, worst_p, max_rate);
  EXPECT_LE(max_rate, 35.0);
  EXPECT_GT(max_rate, 34.0);
  EXPECT_LE(worst_v, 1e-12);
  EXPECT_LE(worst_p, 1e-12);
  EXPECT_TRUE(rot_bit);
}

// ── Init() refusals ──────────────────────────────────────────────────────────

TEST(ContactInekfApi, InitRefusesEveryBadField) {
  double ContactInekf::Config::*fields[] = {
      &ContactInekf::Config::gravity,
      &ContactInekf::Config::gyro_noise_density,
      &ContactInekf::Config::accel_noise_density,
      &ContactInekf::Config::gyro_bias_walk,
      &ContactInekf::Config::accel_bias_walk,
      &ContactInekf::Config::contact_noise_density,
      &ContactInekf::Config::joint_position_sigma,
      &ContactInekf::Config::kinematic_sigma,
      &ContactInekf::Config::contact_gate_chi2,
      &ContactInekf::Config::prior_velocity_sigma,
      &ContactInekf::Config::prior_position_sigma,
      &ContactInekf::Config::prior_yaw_sigma,
      &ContactInekf::Config::prior_gyro_bias_sigma,
      &ContactInekf::Config::prior_accel_bias_sigma,
      &ContactInekf::Config::stationary_accel_tolerance,
      &ContactInekf::Config::stationary_gyro_tolerance,
  };
  for (auto field : fields) {
    for (double bad : {kNan, kInf, 0.0, -1.0, 1e-160, 1e160}) {  // squares: subnormal, Inf
      ContactInekf::Config c = FixtureConfig();
      c.*field = bad;
      ContactInekf f;
      EXPECT_THROW(f.Init(c, 2), std::invalid_argument);
      EXPECT_FALSE(f.configured());
    }
  }
  for (int n : {-1, kInekfMaxContacts + 1}) {
    ContactInekf f;
    EXPECT_THROW(f.Init(FixtureConfig(), n), std::invalid_argument);
  }
  for (int n = 0; n <= kInekfMaxContacts; ++n) {  // control: the same rig passes
    ContactInekf f;
    EXPECT_NO_THROW(f.Init(FixtureConfig(), n));
    EXPECT_EQ(f.dim(), 15 + 3 * n);
    EXPECT_EQ(f.BiasOffset(), 9 + 3 * n);
  }
}

// ── Call order, Reset, other refusals ────────────────────────────────────────

TEST(ContactInekfApi, CallOrderStatuses) {
  const Vec3 a(0.0, 0.0, kG0);
  ContactInekf raw;
  EXPECT_EQ(raw.Propagate(Vec3::Zero(), a, 2e-3), InekfStatus::kNotConfigured);
  EXPECT_EQ(raw.Initialize(Vec3::Zero(), a, 2.0, 0.0, {}), InekfStatus::kNotConfigured);
  EXPECT_EQ(raw.SetContactNoiseScale(0, 2.0), InekfStatus::kNotConfigured);

  ContactInekf f = Configured(2);
  EXPECT_EQ(f.Propagate(Vec3::Zero(), a, 2e-3), InekfStatus::kNotInitialized);
  ASSERT_EQ(f.Initialize(Vec3::Zero(), a, 2.0, 0.0, {}), InekfStatus::kOk);
  EXPECT_EQ(f.Initialize(Vec3::Zero(), a, 2.0, 0.0, {}), InekfStatus::kAlreadyInitialized);
  EXPECT_EQ(f.Propagate(Vec3::Zero(), a, 2e-3), InekfStatus::kOk);
  f.Reset();
  EXPECT_EQ(f.Propagate(Vec3::Zero(), a, 2e-3), InekfStatus::kNotInitialized);
}

TEST(ContactInekfApi, InitializeArgumentRefusals) {
  const Vec3 a(0.0, 0.0, kG0);
  const std::vector<Vec3> three(3, Vec3(0.0, 0.0, -0.7));

  struct Case {
    Vec3 accel;
    double window;
    double yaw;
    std::vector<Vec3> stance;
    InekfStatus want;
  };

  const std::vector<Case> cases{
      {a, 0.0, 0.0, {}, InekfStatus::kBadWindow},
      {a, -1.0, 0.0, {}, InekfStatus::kBadWindow},
      // Positive but subnormal: the window-mean variance overflows.
      {a, 1e-320, 0.0, {}, InekfStatus::kNonFiniteResult},
      {a, kInf, 0.0, {}, InekfStatus::kNonFiniteInput},
      {a, 2.0, kNan, {}, InekfStatus::kNonFiniteInput},
      {a, 2.0, 0.0, {Vec3(kNan, 0, 0)}, InekfStatus::kNonFiniteInput},
      {a, 2.0, 0.0, three, InekfStatus::kTooManyStancePoints},
      // Finite points whose sum overflows: the origin would be Inf.
      {a, 2.0, 0.0, {Vec3(1e308, 0, 0), Vec3(1e308, 0, 0)}, InekfStatus::kNonFiniteResult},
      {Vec3(kG0, 0.0, 0.0), 2.0, 0.0, {}, InekfStatus::kAlignmentSingular},
      {Vec3(-kG0, 1e-9, 0.0), 2.0, 0.0, {}, InekfStatus::kAlignmentSingular},
  };
  for (const Case& c : cases) {
    ContactInekf f = Configured(2);
    const Snapshot before = Capture(f);
    EXPECT_EQ(f.Initialize(Vec3::Zero(), c.accel, c.window, c.yaw, c.stance), c.want);
    EXPECT_TRUE(BitEqual(Capture(f), before));
  }
  // A window so long that its mean carries no noise makes tilt an exact image
  // of b_a: P₀ is singular in double and the Cholesky test refuses it.
  {
    ContactInekf f = Configured(2);
    const Snapshot before = Capture(f);
    EXPECT_EQ(f.Initialize(Vec3::Zero(), a, 1e290, 0.0, {}), InekfStatus::kNotPositiveDefinite);
    EXPECT_TRUE(BitEqual(Capture(f), before));
  }
  // The alignment guard sits at cos(pitch) = 0.1 (|pitch| = 84.26°).
  for (const auto& [pitch_deg, want] :
       std::vector<std::pair<double, InekfStatus>>{{84.0, InekfStatus::kOk},
                                                   {-84.0, InekfStatus::kOk},
                                                   {84.5, InekfStatus::kAlignmentSingular},
                                                   {-84.5, InekfStatus::kAlignmentSingular}}) {
    const double pr = pitch_deg * M_PI / 180.0;
    ContactInekf f = Configured(2);
    EXPECT_EQ(
        f.Initialize(Vec3::Zero(), kG0 * Vec3(-std::sin(pr), 0.0, std::cos(pr)), 2.0, 0.0, {}),
        want)
        << pitch_deg;
  }
}

TEST(ContactInekfApi, ResetClearsStateKeepsConfiguration) {
  ContactInekf f = Configured(3);
  const Vec3 a = RestAccel(Rpy(0.2, -0.1, 0.5));
  ASSERT_EQ(f.Initialize(Vec3::Zero(), a, 2.0, 0.5, {}), InekfStatus::kOk);
  ASSERT_EQ(f.SetContactNoiseScale(2, 5.0), InekfStatus::kOk);
  Access::SetActive(f, 1, true);
  f.Reset();
  ContactInekf fresh = Configured(3);
  EXPECT_TRUE(BitEqual(Capture(f), Capture(fresh)));
  EXPECT_EQ(f.num_slots(), 3);
  EXPECT_TRUE(f.configured());
  // The configuration survived: the same gates and priors apply.
  EXPECT_EQ(f.Initialize(Vec3::Zero(), a * 1.06, 2.0, 0.5, {}), InekfStatus::kAccelNotStationary);
  ASSERT_EQ(f.Initialize(Vec3::Zero(), a, 2.0, 0.5, {}), InekfStatus::kOk);
  ASSERT_EQ(fresh.Initialize(Vec3::Zero(), a, 2.0, 0.5, {}), InekfStatus::kOk);
  EXPECT_TRUE(BitEqual(Capture(f), Capture(fresh)));
}

TEST(ContactInekfApi, NoiseScaleRefusals) {
  ContactInekf f = Configured(2);
  EXPECT_EQ(f.SetContactNoiseScale(-1, 2.0), InekfStatus::kBadSlot);
  EXPECT_EQ(f.SetContactNoiseScale(2, 2.0), InekfStatus::kBadSlot);
  EXPECT_EQ(f.SetContactNoiseScale(0, 0.99), InekfStatus::kBadNoiseScale);
  EXPECT_EQ(f.SetContactNoiseScale(0, kNan), InekfStatus::kBadNoiseScale);
  EXPECT_EQ(f.SetContactNoiseScale(0, kInf), InekfStatus::kBadNoiseScale);
  EXPECT_EQ(f.SetContactNoiseScale(0, 1e300), InekfStatus::kBadNoiseScale);  // (s·σ_d)² = Inf
  EXPECT_EQ(f.contact_noise_scale(0), 1.0);
  EXPECT_EQ(f.SetContactNoiseScale(0, 1.0), InekfStatus::kOk);
  EXPECT_EQ(f.SetContactNoiseScale(1, 2.5), InekfStatus::kOk);
  EXPECT_EQ(f.contact_noise_scale(1), 2.5);
}

TEST(ContactInekfApi, QuaternionIsUnitAndMatchesRotation) {
  ContactInekf f = Configured(2);
  ASSERT_EQ(f.Initialize(Vec3::Zero(), RestAccel(Rpy(0.4, -0.3, 1.1)), 2.0, 1.1, {}),
            InekfStatus::kOk);
  double worst_norm = 0.0;
  double worst_R = 0.0;
  for (int k = 0; k < 2000; ++k) {
    const double t = 2e-3 * k;
    ASSERT_EQ(
        f.Propagate(Vec3(std::sin(t), 0.7 * std::cos(2.0 * t), -0.4), Vec3(0.3, -0.1, kG0), 2e-3),
        InekfStatus::kOk);
    const Eigen::Quaterniond q = f.orientation();
    ASSERT_GT(std::abs(q.x()) * std::abs(q.y()) * std::abs(q.z()), 0.0);
    worst_norm = std::max(worst_norm, std::abs(q.norm() - 1.0));
    worst_R = std::max(worst_R, (q.toRotationMatrix() - f.rotation()).cwiseAbs().maxCoeff());
  }
  std::printf("[measure] quaternion: |norm - 1| %.3e (1e-15)  |R(q) - R| %.3e\n", worst_norm,
              worst_R);
  EXPECT_LE(worst_norm, 1e-15);
}

TEST(ContactInekfApi, NoiseScaleEntersOnlyItsSlotBlockSquared) {
  std::array<Cov, 3> qd;
  const std::array<double, 3> scale{1.0, 2.0, 3.0};
  const std::vector<GridState> states = GridStates();
  for (std::size_t i = 0; i < 3; ++i) {
    ContactInekf f;
    LoadGridState(f, states[3]);  // three active slots
    ASSERT_EQ(f.SetContactNoiseScale(1, scale[i]), InekfStatus::kOk);
    ASSERT_EQ(f.Propagate(Vec3(0.3, -0.2, 0.5), Vec3(0.2, 0.1, kG0), 2e-3), InekfStatus::kOk);
    qd[i] = Access::Qd(f);
  }
  const int o = ContactInekf::SlotOffset(1);
  bool others_bit = true;
  for (Eigen::Index c = 0; c < qd[0].cols(); ++c) {
    for (Eigen::Index r = 0; r < qd[0].rows(); ++r) {
      const bool in_block = r >= o && r < o + 3 && c >= o && c < o + 3;
      if (!in_block) {
        others_bit = others_bit && qd[1](r, c) == qd[0](r, c) && qd[2](r, c) == qd[0](r, c);
      }
    }
  }
  const Mat3 d2 = qd[1].block<3, 3>(o, o) - qd[0].block<3, 3>(o, o);  // 3 B
  const Mat3 d3 = qd[2].block<3, 3>(o, o) - qd[0].block<3, 3>(o, o);  // 8 B
  const double rel = (d3 - (8.0 / 3.0) * d2).norm() / d3.norm();
  std::printf("[measure] noise scale: other blocks bit %d  s^2 law rel %.3e (1e-12)\n",
              static_cast<int>(others_bit), rel);
  EXPECT_TRUE(others_bit);
  EXPECT_GT(d2.norm(), 0.0);
  EXPECT_LE(rel, 1e-12);
}

}  // namespace
}  // namespace rtc::estimation::testing
