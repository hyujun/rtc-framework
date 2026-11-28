// ── Contact InEKF core: support height on flat known ground ─────────────────
//
// H of y = e_zᵀ d_i − h against central finite differences of the true anchor
// height, the update against a dense textbook Kalman step built here (gain,
// injection, Joseph covariance, the inflated variant), the world-yaw +
// horizontal-translation gauge (same innovation, the posterior moved by the
// same gauge; a vertical shift is not a gauge), the declared second-order term,
// the gate and every refusal leaving the filter bit-identical, and a long run of
// pins keeping P positive definite.
#include "contact_inekf_testing.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace rtc::estimation::testing {
namespace {

using Access = ContactInekfTestAccess;
constexpr double kDt = 2e-3;
constexpr double kSigma = 2e-3;  // [m]
constexpr double kGate = 10.83;  // χ²(1), 99.9 %

/// A correlated filter: `active` anchors at |p| ≈ `scale`, then propagation.
void Warm(ContactInekf& f, std::uint64_t seed, int active, double scale = 10.0) {
  StartFilter(f, seed, active, scale);
  std::mt19937_64 rng(seed + 11);
  std::normal_distribution<double> nd(0.0, 1.0);
  for (int k = 0; k < 20; ++k) {
    ASSERT_EQ(
        f.Propagate(Vec3(nd(rng), nd(rng), nd(rng)),
                    f.rotation().transpose() * Vec3(0, 0, kG0) + Vec3(nd(rng), nd(rng), nd(rng)),
                    kDt),
        InekfStatus::kOk);
  }
}

/// The analytic H of the ground row for `slot` (full layout).
Eigen::VectorXd GroundRow(const ContactInekf& f, int slot) {
  Eigen::VectorXd h = Eigen::VectorXd::Zero(f.dim());
  const Vec3& d = f.contact_position(slot);
  h(0) = d.y();
  h(1) = -d.x();
  h(ContactInekf::SlotOffset(slot) + 2) = 1.0;
  return h;
}

/// (R, [v, p, d_0 … d_3]) of the filter, every slot column present.
struct State {
  Mat3 R;
  std::vector<Vec3> cols;
  Vec3 bg, ba;
};

State Read(const ContactInekf& f) {
  State s{f.rotation(), {f.velocity(), f.position()}, f.gyro_bias(), f.accel_bias()};
  for (int i = 0; i < f.num_slots(); ++i) {
    s.cols.push_back(f.contact_position(i));
  }
  return s;
}

/// ξ over (φ, v, p, d_0 … d_3) of the full layout without the biases.
Eigen::VectorXd GroupPart(const Eigen::VectorXd& e, int num_slots) {
  return e.head(9 + 3 * num_slots);
}

double MaxAbs(const Eigen::Ref<const Eigen::MatrixXd>& m) {
  return m.cwiseAbs().maxCoeff();
}

/// √(H P Hᵀ + σ²): the innovation's standard deviation. The anchor height's
/// own uncertainty is H P Hᵀ — in these coordinates P_zz alone is not it: a
/// tilt error moves an anchor far from the origin vertically too.
double InnovationSd(const ContactInekf& f, int slot) {
  const Eigen::VectorXd h = GroundRow(f, slot);
  const Eigen::MatrixXd P = f.covariance();
  return std::sqrt(h.dot(P * h) + kSigma * kSigma);
}

/// Inactive slots' rows and columns to zero, as the filter keeps them.
void ZeroInactive(const ContactInekf& f, Eigen::MatrixXd& P) {
  for (int i = 0; i < f.num_slots(); ++i) {
    if (!f.contact_active(i)) {
      P.middleRows(ContactInekf::SlotOffset(i), 3).setZero();
      P.middleCols(ContactInekf::SlotOffset(i), 3).setZero();
    }
  }
}

// ── H ────────────────────────────────────────────────────────────────────────

TEST(ContactInekfGround, MeasurementJacobianMatchesFiniteDifference) {
  double worst = 0.0;
  for (int seed = 0; seed < 32; ++seed) {
    ContactInekf f;
    Warm(f, static_cast<std::uint64_t>(seed), 2);
    for (int slot = 0; slot < 2; ++slot) {
      const State hat = Read(f);
      const Eigen::VectorXd h = GroundRow(f, slot);
      const int n = 9 + 3 * f.num_slots();
      constexpr double kStep = 1e-6;
      for (int j = 0; j < n; ++j) {
        double y[2];
        for (int sgn = 0; sgn < 2; ++sgn) {
          Eigen::VectorXd xi = Eigen::VectorXd::Zero(n);
          xi(j) = sgn == 0 ? kStep : -kStep;
          State t = hat;
          ApplyExpLeft(xi, t.R, t.cols);
          // y(X) − y(X̂): the true anchor height minus the estimated one.
          y[sgn] = t.cols[2 + static_cast<std::size_t>(slot)].z() -
                   hat.cols[2 + static_cast<std::size_t>(slot)].z();
        }
        const double fd = (y[0] - y[1]) / (2.0 * kStep);
        worst = std::max(worst, std::abs(fd - h(j)) / (1.0 + std::abs(h(j))));
      }
    }
  }
  EXPECT_LT(worst, 1e-7);
}

// ── The update against a dense reference ─────────────────────────────────────

struct Expected {
  State x;
  Eigen::MatrixXd P;
  double nu, nis;
};

Expected DenseStep(const ContactInekf& f, int slot, double height, double var_extra) {
  const Eigen::MatrixXd P = f.covariance();
  const Eigen::VectorXd h = GroundRow(f, slot);
  const double nu = height - f.contact_position(slot).z();
  const double s0 = h.dot(P * h) + kSigma * kSigma;
  const double var = kSigma * kSigma + var_extra;
  const double s = h.dot(P * h) + var;
  const Eigen::VectorXd K = P * h / s;
  const Eigen::VectorXd delta = K * nu;
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(f.dim(), f.dim()) - K * h.transpose();
  Expected out{Read(f), IKH * P * IKH.transpose() + var * K * K.transpose(), nu, nu * nu / s0};
  ApplyExpLeft(GroupPart(delta, f.num_slots()), out.x.R, out.x.cols);
  out.x.bg += delta.segment<3>(f.BiasOffset());
  out.x.ba += delta.segment<3>(f.BiasOffset() + 3);
  return out;
}

void ExpectMatches(const ContactInekf& f, const Expected& e, int active) {
  EXPECT_LT(MaxAbs(f.rotation() - e.x.R), 1e-12);
  EXPECT_LT((f.velocity() - e.x.cols[0]).norm(), 1e-12);
  EXPECT_LT((f.position() - e.x.cols[1]).norm(), 1e-11);
  for (int i = 0; i < active; ++i) {
    EXPECT_LT((f.contact_position(i) - e.x.cols[2 + static_cast<std::size_t>(i)]).norm(), 1e-11);
  }
  EXPECT_LT((f.gyro_bias() - e.x.bg).norm(), 1e-14);
  EXPECT_LT((f.accel_bias() - e.x.ba).norm(), 1e-14);
  const Eigen::MatrixXd P = f.covariance();
  EXPECT_LT(MaxAbs(P - e.P) / MaxAbs(e.P), 1e-11);
}

TEST(ContactInekfGround, UpdateMatchesTheDenseKalmanStep) {
  for (int seed = 0; seed < 16; ++seed) {
    for (int slot = 0; slot < 2; ++slot) {
      ContactInekf f;
      Warm(f, static_cast<std::uint64_t>(100 + seed), 2);
      const double height = f.contact_position(slot).z() + 1e-3 * (seed % 5 - 2);
      const Expected e = DenseStep(f, slot, height, 0.0);
      GroundUpdateReport rep;
      ASSERT_EQ(f.UpdateGroundHeight(slot, height, kSigma, kGate, false, rep), InekfStatus::kOk);
      EXPECT_EQ(rep.result, GroundUpdateResult::kUsed);
      EXPECT_DOUBLE_EQ(rep.innovation, e.nu);
      EXPECT_NEAR(rep.nis, e.nis, 1e-12 * (1.0 + e.nis));
      ExpectMatches(f, e, 2);
    }
  }
}

TEST(ContactInekfGround, InflatedUpdateUsesTheInnovationAsExtraVarianceAndNoGate) {
  ContactInekf f;
  Warm(f, 7, 2);
  const double height = f.contact_position(1).z() + 20.0 * InnovationSd(f, 1);  // NIS 400
  GroundUpdateReport rep;
  ContactInekf gated = f;
  ASSERT_EQ(gated.UpdateGroundHeight(1, height, kSigma, kGate, false, rep), InekfStatus::kOk);
  ASSERT_EQ(rep.result, GroundUpdateResult::kGated);
  const double nu = height - f.contact_position(1).z();
  const Expected e = DenseStep(f, 1, height, nu * nu);
  ASSERT_EQ(f.UpdateGroundHeight(1, height, kSigma, kGate, true, rep), InekfStatus::kOk);
  EXPECT_EQ(rep.result, GroundUpdateResult::kInflated);
  EXPECT_GT(rep.nis, kGate);  // reported against σ² alone
  ExpectMatches(f, e, 2);
  // A partial step: the anchor moves toward the plane, by less than half the way
  // (the gain is H P Hᵀ / (H P Hᵀ + σ² + ν²) on the anchor's own height).
  const double moved = f.contact_position(1).z() - (height - nu);
  EXPECT_GT(moved, 0.0);
  EXPECT_LT(moved, 0.5 * nu);
}

// ── Gauge ────────────────────────────────────────────────────────────────────

/// f2 = G·f1 exactly (G = (Rz(ψ), t)), P2 = Ad_G P1 Ad_Gᵀ.
void GaugeCopy(const ContactInekf& f1, const Mat3& Rz, const Vec3& t, ContactInekf& f2) {
  f2 = f1;
  std::array<Vec3, 4> d = ZeroContacts();
  std::array<bool, 4> on{};
  for (int i = 0; i < f1.num_slots(); ++i) {
    const auto si = static_cast<std::size_t>(i);
    on[si] = f1.contact_active(i);
    if (on[si]) {
      d[si] = Rz * f1.contact_position(i) + t;
    }
  }
  const Cov A = GaugeAdjoint(f1, Rz, t);
  Cov P = A * f1.covariance() * A.transpose();
  P = (0.5 * (P + P.transpose())).eval();
  for (int i = 0; i < f1.num_slots(); ++i) {
    if (!f1.contact_active(i)) {
      P.middleRows(ContactInekf::SlotOffset(i), 3).setZero();
      P.middleCols(ContactInekf::SlotOffset(i), 3).setZero();
    }
  }
  Access::SetState(f2, Rz * f1.rotation(), Rz * f1.velocity(), Rz * f1.position() + t, d, on,
                   f1.gyro_bias(), f1.accel_bias(), P);
}

TEST(ContactInekfGround, AYawAndHorizontalShiftOfTheWorldIsAGauge) {
  double worst_nis = 0.0, worst_x = 0.0, worst_P = 0.0;
  for (int seed = 0; seed < 16; ++seed) {
    ContactInekf f1;
    Warm(f1, static_cast<std::uint64_t>(200 + seed), 2);
    const double psi = 0.4 * (seed - 8);
    const Mat3 Rz = RotZ(psi);
    const Vec3 t(3.0 * std::sin(seed), -2.0 * std::cos(seed), 0.0);
    ContactInekf f2;
    GaugeCopy(f1, Rz, t, f2);
    const double height = f1.contact_position(0).z() + 1.5e-3;
    GroundUpdateReport r1, r2;
    ASSERT_EQ(f1.UpdateGroundHeight(0, height, kSigma, kGate, false, r1), InekfStatus::kOk);
    ASSERT_EQ(f2.UpdateGroundHeight(0, height, kSigma, kGate, false, r2), InekfStatus::kOk);
    EXPECT_DOUBLE_EQ(r1.innovation, r2.innovation);
    worst_nis = std::max(worst_nis, std::abs(r1.nis - r2.nis) / r1.nis);
    worst_x = std::max(worst_x, (f2.position() - (Rz * f1.position() + t)).norm());
    for (int i = 0; i < 2; ++i) {
      worst_x =
          std::max(worst_x, (f2.contact_position(i) - (Rz * f1.contact_position(i) + t)).norm());
    }
    worst_x = std::max(worst_x, MaxAbs(f2.rotation() - Rz * f1.rotation()));
    const Cov A = GaugeAdjoint(f1, Rz, t);
    Eigen::MatrixXd moved = A * f1.covariance() * A.transpose();
    ZeroInactive(f1, moved);
    worst_P = std::max(worst_P, MaxAbs(f2.covariance() - moved) / MaxAbs(moved));
  }
  EXPECT_LT(worst_nis, 1e-10);
  EXPECT_LT(worst_x, 1e-11);
  EXPECT_LT(worst_P, 1e-10);
}

TEST(ContactInekfGround, AVerticalShiftOfTheWorldIsNotAGauge) {
  ContactInekf f1;
  Warm(f1, 300, 1);
  ContactInekf f2;
  GaugeCopy(f1, Mat3::Identity(), Vec3(0.0, 0.0, 0.01), f2);
  GroundUpdateReport r1, r2;
  ASSERT_EQ(f1.UpdateGroundHeight(0, 0.0, kSigma, kGate, true, r1), InekfStatus::kOk);
  ASSERT_EQ(f2.UpdateGroundHeight(0, 0.0, kSigma, kGate, true, r2), InekfStatus::kOk);
  EXPECT_NEAR(r2.innovation - r1.innovation, -0.01, 1e-15);
}

// ── The linearisation's declared second-order term ───────────────────────────

TEST(ContactInekfGround, SecondOrderTermIsHalfYawTimesTiltAlongTheAnchor) {
  // An anchor 10 m from the origin, a yaw error 0.05 and a tilt error 0.007
  // along it: e_zᵀ(exp(φ) d − d − φ × d) = ½ ψ (θ · d_h) + O(|φ|³ |d|).
  const Vec3 d(10.0, 0.0, 0.0);
  double worst = 0.0;
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 0.0, 1.0}) {
      const Vec3 phi(0.007 * sx, 0.007 * sy, 0.05);
      const Vec3 dt = rtc::math::se3::exp3(phi) * d;
      const double second = dt.z() - d.z() - phi.cross(d).z();
      const double declared = 0.5 * phi.z() * (phi.head<2>().dot(d.head<2>()));
      // The next term is −|φ|²/6 (φ × d)_z, bounded by |φ|² |φ_h| |d| / 6.
      const double third = phi.squaredNorm() * phi.head<2>().norm() * d.norm() / 6.0;
      worst = std::max(worst, std::abs(second - declared) / third);
      EXPECT_GT(std::abs(declared), 1e-3);  // ≈ 1.75 mm: the size the σ_g choice must cover
    }
  }
  EXPECT_LT(worst, 1.01);
}

// ── Gate and refusals ────────────────────────────────────────────────────────

TEST(ContactInekfGround, AGatedPinChangesNothingAndReportsIt) {
  ContactInekf f;
  Warm(f, 400, 2);
  const Snapshot before = Capture(f);
  GroundUpdateReport rep;
  const double height = f.contact_position(0).z() + 20.0 * InnovationSd(f, 0);
  ASSERT_EQ(f.UpdateGroundHeight(0, height, kSigma, kGate, false, rep), InekfStatus::kOk);
  EXPECT_EQ(rep.result, GroundUpdateResult::kGated);
  EXPECT_GT(rep.nis, kGate);
  EXPECT_DOUBLE_EQ(rep.innovation, height - f.contact_position(0).z());
  EXPECT_TRUE(BitEqual(before, Capture(f)));
  // An infinite gate never refuses.
  ASSERT_EQ(
      f.UpdateGroundHeight(0, height, kSigma, std::numeric_limits<double>::infinity(), false, rep),
      InekfStatus::kOk);
  EXPECT_EQ(rep.result, GroundUpdateResult::kUsed);
}

TEST(ContactInekfGround, EveryRefusalLeavesTheFilterBitIdentical) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  {
    ContactInekf f;
    GroundUpdateReport rep;
    EXPECT_EQ(f.UpdateGroundHeight(0, 0.0, kSigma, kGate, false, rep), InekfStatus::kNotConfigured);
    f.Init(FixtureConfig(), 2);
    EXPECT_EQ(f.UpdateGroundHeight(0, 0.0, kSigma, kGate, false, rep),
              InekfStatus::kNotInitialized);
  }
  ContactInekf f;
  Warm(f, 500, 2);
  ASSERT_EQ(f.RemoveContact(1), InekfStatus::kOk);
  const Snapshot before = Capture(f);

  struct Case {
    int slot;
    double height, sigma, gate;
    InekfStatus want;
  };

  const Case cases[] = {
      {-1, 0.0, kSigma, kGate, InekfStatus::kBadSlot},
      {4, 0.0, kSigma, kGate, InekfStatus::kBadSlot},
      {1, 0.0, kSigma, kGate, InekfStatus::kSlotInactive},
      {0, nan, kSigma, kGate, InekfStatus::kBadGroundModel},
      {0, inf, kSigma, kGate, InekfStatus::kBadGroundModel},
      {0, 0.0, 0.0, kGate, InekfStatus::kBadGroundModel},
      {0, 0.0, -kSigma, kGate, InekfStatus::kBadGroundModel},
      {0, 0.0, nan, kGate, InekfStatus::kBadGroundModel},
      {0, 0.0, inf, kGate, InekfStatus::kBadGroundModel},
      {0, 0.0, 1e-170, kGate, InekfStatus::kBadGroundModel},  // σ² underflows
      {0, 0.0, 1e200, kGate, InekfStatus::kBadGroundModel},   // σ² overflows
      {0, 0.0, kSigma, 0.0, InekfStatus::kBadGroundModel},
      {0, 0.0, kSigma, -1.0, InekfStatus::kBadGroundModel},
      {0, 0.0, kSigma, nan, InekfStatus::kBadGroundModel},
  };
  for (const Case& c : cases) {
    GroundUpdateReport rep;
    rep.innovation = 42.0;
    for (bool inflate : {false, true}) {
      EXPECT_EQ(f.UpdateGroundHeight(c.slot, c.height, c.sigma, c.gate, inflate, rep), c.want)
          << c.slot << ' ' << c.height << ' ' << c.sigma << ' ' << c.gate;
      EXPECT_EQ(rep.innovation, 42.0);  // the report is written only on kOk
    }
    EXPECT_TRUE(BitEqual(before, Capture(f)));
  }
  // A non-finite covariance entry on the row is refused, not used.
  Cov P = f.covariance();
  P(ContactInekf::SlotOffset(0) + 2, 0) = nan;
  Access::SetCovariance(f, P);
  const Snapshot poisoned = Capture(f);
  GroundUpdateReport rep;
  EXPECT_EQ(f.UpdateGroundHeight(0, 0.0, kSigma, kGate, false, rep), InekfStatus::kNonFiniteResult);
  EXPECT_TRUE(BitEqual(poisoned, Capture(f)));
}

// ── Repeated pins ────────────────────────────────────────────────────────────

TEST(ContactInekfGround, PinsBetweenPropagationsKeepPPositiveDefinite) {
  ContactInekf f;
  Warm(f, 600, 2);
  std::mt19937_64 rng(601);
  std::normal_distribution<double> nd(0.0, 1.0);
  double min_eig = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 2000; ++k) {
    ASSERT_EQ(f.Propagate(Vec3(nd(rng), nd(rng), nd(rng)) * 0.1,
                          f.rotation().transpose() * Vec3(0, 0, kG0) +
                              Vec3(nd(rng), nd(rng), nd(rng)) * 0.1,
                          kDt),
              InekfStatus::kOk);
    GroundUpdateReport rep;
    const int slot = k % 2;
    ASSERT_EQ(f.UpdateGroundHeight(slot, f.contact_position(slot).z() + 1e-4 * nd(rng), kSigma,
                                   std::numeric_limits<double>::infinity(), false, rep),
              InekfStatus::kOk);
    const std::vector<int> idx = ActiveIndex(f);
    const Eigen::MatrixXd Pa = Gather(f.covariance(), idx);
    min_eig = std::min(min_eig,
                       Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(Pa).eigenvalues().minCoeff());
  }
  EXPECT_GT(min_eig, 0.0);
  // The pinned heights are held to about σ (plus a tick of anchor walk).
  for (int slot = 0; slot < 2; ++slot) {
    EXPECT_LT(InnovationSd(f, slot), 2.0 * kSigma);
  }
}

}  // namespace
}  // namespace rtc::estimation::testing
