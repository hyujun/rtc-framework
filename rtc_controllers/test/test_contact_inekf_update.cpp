// ── Contact InEKF core: contact update ───────────────────────────────────────
//
// H against central finite differences of the true kinematics, the injection
// sign (first-order error map ξ⁺ = (I − KH)ξ₀), the insertion Jacobian, the
// world-yaw + translation gauge through a mixed propagate / insert / update /
// remove sequence (one step at a time and accumulated), covariance symmetry /
// positive definiteness with an update every tick, the χ² gate against a
// packed reference that sees only the surviving slots, and refusals.
#include "contact_inekf_packed_oracle.hpp"
#include "contact_inekf_testing.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <random>
#include <vector>

namespace rtc::estimation::testing {
namespace {

constexpr double kDt = 2e-3;
constexpr double kEps = 1e-6;

/// A correlated state: anchored slots, then a few ticks of propagation and
/// self-consistent updates.
void Warm(ContactInekf& f, std::uint64_t seed, int active) {
  StartFilter(f, seed, active);
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

struct XCols {
  Mat3 R;
  std::vector<Vec3> cols;  // v, p, d(active)
  std::vector<int> slots;
};

XCols Cols(const ContactInekf& f) {
  XCols x{f.rotation(), {f.velocity(), f.position()}, {}};
  for (int i = 0; i < f.num_slots(); ++i) {
    if (f.contact_active(i)) {
      x.cols.push_back(f.contact_position(i));
      x.slots.push_back(i);
    }
  }
  return x;
}

/// z_i = R̂ r_i + p̂ − d̂_i with r_i the TRUE kinematics of X = Exp(ξ)X̂.
Eigen::VectorXd TrueInnovation(const XCols& hat, const Eigen::VectorXd& xi) {
  XCols t = hat;
  ApplyExpLeft(xi, t.R, t.cols);
  const auto k = static_cast<Eigen::Index>(hat.slots.size());
  Eigen::VectorXd z(3 * k);
  for (Eigen::Index q = 0; q < k; ++q) {
    const auto qi = static_cast<std::size_t>(q);
    const Vec3 r = t.R.transpose() * (t.cols[2 + qi] - t.cols[1]);
    z.segment<3>(3 * q) = hat.R * r + hat.cols[1] - hat.cols[2 + qi];
  }
  return z;
}

// ── H ────────────────────────────────────────────────────────────────────────

TEST(ContactInekfUpdate, MeasurementJacobianMatchesFiniteDifference) {
  double worst = 0.0;
  for (int s = 0; s < 64; ++s) {
    ContactInekf f;
    Warm(f, static_cast<std::uint64_t>(s), 1 + s % 4);
    ContactInekf g = f;
    std::vector<ContactUpdateReport> rep(4);
    ASSERT_EQ(g.UpdateContacts(PredictedKinematics(f), rep), InekfStatus::kOk);
    const std::vector<int> idx = ActiveIndex(f);
    const Eigen::MatrixXd Hfull = Access::H(g);
    Eigen::MatrixXd H(Hfull.rows(), static_cast<Eigen::Index>(idx.size()));
    for (std::size_t c = 0; c < idx.size(); ++c) {
      H.col(static_cast<Eigen::Index>(c)) = Hfull.col(idx[c]);
    }
    const XCols hat = Cols(f);
    const auto nx = static_cast<Eigen::Index>(idx.size()) - 6;
    Eigen::MatrixXd Jfd(H.rows(), H.cols());
    for (Eigen::Index c = 0; c < H.cols(); ++c) {
      Eigen::VectorXd xp = Eigen::VectorXd::Zero(nx);
      Eigen::VectorXd xm = Eigen::VectorXd::Zero(nx);
      if (c < nx) {
        xp(c) = kEps;
        xm(c) = -kEps;
      }
      Jfd.col(c) = (TrueInnovation(hat, xp) - TrueInnovation(hat, xm)) / (2.0 * kEps);
    }
    worst =
        std::max(worst, (Jfd - H).cwiseAbs().maxCoeff() / std::max(1.0, H.cwiseAbs().maxCoeff()));
  }
  std::printf("[measure] H vs FD (64 states, 1-4 slots): %.3e (1e-7)\n", worst);
  EXPECT_LE(worst, 1e-7);
}

// ── Injection sign ───────────────────────────────────────────────────────────

TEST(ContactInekfUpdate, InjectionFollowsTheFirstOrderErrorMap) {
  double worst = 0.0;
  for (int s = 0; s < 64; ++s) {
    ContactInekf f;
    Warm(f, 100 + static_cast<std::uint64_t>(s), 1 + s % 4);
    const std::vector<int> idx = ActiveIndex(f);
    const auto Dp = static_cast<Eigen::Index>(idx.size());
    const Eigen::Index nx = Dp - 6;
    std::mt19937_64 rng(200 + static_cast<std::uint64_t>(s));
    std::normal_distribution<double> nd(0.0, 1.0);
    Eigen::VectorXd xi0(Dp);
    for (Eigen::Index i = 0; i < Dp; ++i) {
      xi0(i) = nd(rng);
    }
    xi0 *= 1e-4 / xi0.norm();

    const XCols hat = Cols(f);
    XCols truth = hat;
    ApplyExpLeft(xi0.head(nx), truth.R, truth.cols);
    const Vec3 bg_t = f.gyro_bias() + xi0.segment<3>(nx);
    const Vec3 ba_t = f.accel_bias() + xi0.segment<3>(nx + 3);
    std::vector<ContactKinematics> k(4);
    for (std::size_t q = 0; q < hat.slots.size(); ++q) {
      k[static_cast<std::size_t>(hat.slots[q])] =
          Kin(truth.R.transpose() * (truth.cols[2 + q] - truth.cols[1]));
    }
    std::vector<ContactUpdateReport> rep(4);
    ASSERT_EQ(f.UpdateContacts(k, rep), InekfStatus::kOk);
    const Eigen::MatrixXd K = Access::Gain(f);
    const Eigen::MatrixXd H = Access::H(f);
    ASSERT_EQ(H.rows(), static_cast<Eigen::Index>(3 * hat.slots.size()));

    Eigen::VectorXd full = Eigen::VectorXd::Zero(f.dim());
    for (Eigen::Index i = 0; i < Dp; ++i) {
      full(idx[static_cast<std::size_t>(i)]) = xi0(i);
    }
    const Eigen::VectorXd want_full = full - K * (H * full);
    Eigen::VectorXd want(Dp);
    for (Eigen::Index i = 0; i < Dp; ++i) {
      want(i) = want_full(idx[static_cast<std::size_t>(i)]);
    }
    const XCols post = Cols(f);
    Eigen::VectorXd got(Dp);
    got.head(nx) = LogError(truth.R, truth.cols, post.R, post.cols);
    got.segment<3>(nx) = bg_t - f.gyro_bias();
    got.segment<3>(nx + 3) = ba_t - f.accel_bias();
    worst = std::max(worst, (got - want).norm());
  }
  std::printf("[measure] |xi+ - (I-KH) xi0| at |xi0| = 1e-4: %.3e (1e-6)\n", worst);
  EXPECT_LE(worst, 1e-6);
}

// ── Insertion Jacobian ───────────────────────────────────────────────────────

TEST(ContactInekfUpdate, InsertionCovarianceMatchesFiniteDifference) {
  const ContactInekf::Config cfg = FixtureConfig();
  double worst = 0.0;
  for (int s = 0; s < 64; ++s) {
    ContactInekf f;
    Warm(f, 300 + static_cast<std::uint64_t>(s), s % 4);
    const int j = 3;  // at most three slots are active, so slot 3 is free
    std::mt19937_64 rng(400 + static_cast<std::uint64_t>(s));
    std::normal_distribution<double> nd(0.0, 1.0);
    Eigen::MatrixXd J(3, 6);
    for (Eigen::Index c = 0; c < 6; ++c) {
      J.col(c) = Vec3(nd(rng), nd(rng), nd(rng)) * 0.1;
    }
    const ContactKinematics kin =
        Kin(FootPoint(j) + Vec3(nd(rng), nd(rng), nd(rng)) * 0.05, true, J);
    const std::vector<int> idx = ActiveIndex(f);
    const Eigen::MatrixXd P0 = Gather(f.covariance(), idx);
    const XCols hat = Cols(f);
    const auto Dp = static_cast<Eigen::Index>(idx.size());
    const Eigen::Index nx = Dp - 6;
    const Vec3 d_hat = hat.cols[1] + hat.R * kin.position;

    Eigen::MatrixXd Mx = Eigen::MatrixXd::Zero(3, Dp);
    for (Eigen::Index c = 0; c < nx; ++c) {
      Vec3 out[2];
      for (int sg = 0; sg < 2; ++sg) {
        Eigen::VectorXd xi = Eigen::VectorXd::Zero(nx);
        xi(c) = sg == 0 ? kEps : -kEps;
        XCols t = hat;
        ApplyExpLeft(xi, t.R, t.cols);
        const Vec3 phi = xi.head<3>();
        const Vec3 d_t = t.cols[1] + t.R * kin.position;
        out[sg] =
            rtc::math::se3::leftJacobianInverse(phi) * (d_t - rtc::math::se3::exp3(phi) * d_hat);
      }
      Mx.col(c) = (out[0] - out[1]) / (2.0 * kEps);
    }
    Mat3 Mr;
    for (int c = 0; c < 3; ++c) {
      const Vec3 e = Vec3::Unit(c) * kEps;
      Mr.col(c) = ((hat.cols[1] + hat.R * (kin.position + e)) -
                   (hat.cols[1] + hat.R * (kin.position - e))) /
                  (2.0 * kEps);
    }
    const Eigen::MatrixXd want_cross = Mx * P0;
    const Mat3 want_dd = Mx * P0 * Mx.transpose() + Mr * BodyNoise(kin, cfg) * Mr.transpose();

    ASSERT_EQ(f.InsertContact(j, kin), InekfStatus::kOk);
    const Cov& P = f.covariance();
    const int o = ContactInekf::SlotOffset(j);
    Eigen::MatrixXd got_cross(3, Dp);
    for (Eigen::Index c = 0; c < Dp; ++c) {
      got_cross.col(c) = P.block<3, 1>(o, idx[static_cast<std::size_t>(c)]);
    }
    const Mat3 got_dd = P.block<3, 3>(o, o);
    const double scale = std::max(P0.cwiseAbs().maxCoeff(), got_dd.cwiseAbs().maxCoeff());
    worst = std::max({worst, (got_cross - want_cross).cwiseAbs().maxCoeff() / scale,
                      (got_dd - want_dd).cwiseAbs().maxCoeff() / scale});
  }
  std::printf("[measure] insertion cross/block vs FD, rel to max|P|: %.3e (1e-7)\n", worst);
  EXPECT_LE(worst, 1e-7);
}

// ── Gauge through a mixed sequence ───────────────────────────────────────────

void ZeroInactive(const ContactInekf& f, Cov& P) {
  for (int i = 0; i < f.num_slots(); ++i) {
    if (!f.contact_active(i)) {
      P.middleRows(ContactInekf::SlotOffset(i), 3).setZero();
      P.middleCols(ContactInekf::SlotOffset(i), 3).setZero();
    }
  }
}

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
  ZeroInactive(f1, P);
  Access::SetState(f2, Rz * f1.rotation(), Rz * f1.velocity(), Rz * f1.position() + t, d, on,
                   f1.gyro_bias(), f1.accel_bias(), P);
}

struct GaugeWorst {
  double nis = 0, zb = 0, b = 0, v = 0, P = 0;
  bool same_decisions = true;
};

void GaugeRun(int seed, double psi, bool one_step, GaugeWorst& w) {
  const ContactInekf::Config cfg = FixtureConfig();
  constexpr std::array<double, 5> kDts{2e-4, 1e-3, 2e-3, 1e-2, 2e-2};
  const double dt = kDts[static_cast<std::size_t>(seed % 5)];
  const double wmax = std::min(3.0, 0.6 / dt);
  std::mt19937_64 rng(static_cast<std::uint64_t>(seed) + 100);
  std::normal_distribution<double> nd(0.0, 1.0);
  std::uniform_real_distribution<double> ud(0.0, 6.28);
  ContactInekf f1;
  StartFilter(f1, static_cast<std::uint64_t>(seed), 2);
  const Vec3 t = Vec3(nd(rng), nd(rng), nd(rng)).normalized() * 10.0;
  const Mat3 Rz = RotZ(psi);
  ContactInekf f2;
  GaugeCopy(f1, Rz, t, f2);
  const Cov AdG = GaugeAdjoint(f1, Rz, t);
  const Cov AdGi = AdG.inverse();

  Mat3 Rt = f1.rotation() * rtc::math::se3::exp3(Vec3(1e-3, -2e-3, 1e-3));
  Vec3 vt = f1.velocity() + Vec3::Constant(1e-2);
  Vec3 pt = f1.position() + Vec3::Constant(1e-3);
  const Vec3 pc = pt;
  std::map<int, Vec3> dtrue;
  for (int i = 0; i < 2; ++i) {
    dtrue[i] = f1.contact_position(i) + Vec3::Constant(1e-3);
  }
  const Vec3 phase(ud(rng), ud(rng), ud(rng));
  const Vec3 g(0, 0, -kG0);
  for (int k = 0; k < 2000; ++k) {
    const double tk = k * dt;
    const bool rot = (k / 500) % 2 == 1;
    const Vec3 wm =
        rot ? Vec3(wmax * std::sin(2.1 * tk + phase.x()), wmax * std::sin(1.3 * tk + phase.y()),
                   wmax * std::sin(0.7 * tk + phase.z()))
            : Vec3(0.01 * std::sin(9 * tk), 0.01 * std::cos(7 * tk), 0.01 * std::sin(5 * tk));
    const Vec3 aw = 0.5 * Vec3(std::sin(1.7 * tk), std::cos(1.1 * tk), 0.3 * std::sin(2.3 * tk)) -
                    0.5 * vt - 0.05 * (pt - pc);
    const Vec3 am = Rt.transpose() * (aw - g);
    Mat3 R1;
    Vec3 v1;
    Vec3 p1;
    Access::MeanMap(Rt, vt, pt, wm, am, dt, kG0, R1, v1, p1);
    Rt = R1;
    vt = v1;
    pt = p1;

    if (one_step) {
      GaugeCopy(f1, Rz, t, f2);
    }
    ASSERT_EQ(f1.Propagate(wm, am, dt), InekfStatus::kOk);
    ASSERT_EQ(f2.Propagate(wm, am, dt), InekfStatus::kOk);
    if (k % 250 == 125) {
      const int j = (k / 250) % 2;
      const int jn = 2 + (k / 250) % 2;
      for (ContactInekf* f : {&f1, &f2}) {
        for (int s : {j, jn}) {
          if (f->contact_active(s)) {
            ASSERT_EQ(f->RemoveContact(s), InekfStatus::kOk);
          }
        }
      }
      dtrue.erase(j);
      dtrue.erase(jn);
      const Vec3 rb(0.1, jn % 2 == 0 ? 0.1 : -0.1, -0.7);
      dtrue[jn] = pt + Rt * rb;
      ASSERT_EQ(f1.InsertContact(jn, Kin(rb)), InekfStatus::kOk);
      ASSERT_EQ(f2.InsertContact(jn, Kin(rb)), InekfStatus::kOk);
    }
    std::vector<ContactKinematics> kin(4);
    for (const auto& [i, d] : dtrue) {
      kin[static_cast<std::size_t>(i)] =
          Kin(Rt.transpose() * (d - pt) + Vec3(nd(rng), nd(rng), nd(rng)) * 2e-3);
    }
    std::vector<ContactUpdateReport> r1(4);
    std::vector<ContactUpdateReport> r2(4);
    ASSERT_EQ(f1.UpdateContacts(kin, r1), InekfStatus::kOk);
    ASSERT_EQ(f2.UpdateContacts(kin, r2), InekfStatus::kOk);

    const double zscale = one_step ? 1.0 : cfg.kinematic_sigma;
    for (std::size_t i = 0; i < 4; ++i) {
      w.same_decisions = w.same_decisions && r1[i].result == r2[i].result;
      if (r1[i].result == ContactUpdateResult::kUsed) {
        w.nis = std::max(w.nis, std::abs(r1[i].mahalanobis2 - r2[i].mahalanobis2));
        w.zb = std::max(w.zb, (r1[i].innovation - r2[i].innovation).norm() / zscale);
      }
    }
    const int bo = f1.BiasOffset();
    const Eigen::Matrix<double, 6, 1> sb =
        f1.covariance().block<6, 6>(bo, bo).diagonal().cwiseSqrt();
    Eigen::Matrix<double, 6, 1> db;
    db << f2.gyro_bias() - f1.gyro_bias(), f2.accel_bias() - f1.accel_bias();
    w.b = std::max(w.b, db.cwiseQuotient(sb).cwiseAbs().maxCoeff());
    const Vec3 sv = (f1.rotation().transpose() * f1.covariance().block<3, 3>(3, 3) * f1.rotation())
                        .diagonal()
                        .cwiseSqrt();
    const Vec3 dv =
        f2.rotation().transpose() * f2.velocity() - f1.rotation().transpose() * f1.velocity();
    w.v = std::max(w.v, dv.cwiseQuotient(sv).cwiseAbs().maxCoeff());
    Cov back = AdGi * f2.covariance() * AdGi.transpose();
    ZeroInactive(f1, back);
    w.P = std::max(w.P, (back - f1.covariance()).norm() / f1.covariance().norm());
  }
}

TEST(ContactInekfUpdate, GaugeOneStepAtATime) {
  GaugeWorst w;
  for (int seed = 0; seed < 8; ++seed) {
    for (double psi : {0.3, -0.3, 2.9, -2.9}) {
      GaugeRun(seed, psi, true, w);
    }
  }
  std::printf(
      "[measure] gauge 1-step: |dNIS| %.3e (1e-7)  |dz_body| %.3e m (1e-12)  db/sb %.3e (1e-8)  "
      "dv/sv %.3e (1e-8)  P rel %.3e (1e-10)\n",
      w.nis, w.zb, w.b, w.v, w.P);
  EXPECT_TRUE(w.same_decisions);
  EXPECT_LE(w.nis, 1e-7);
  EXPECT_LE(w.zb, 1e-12);
  EXPECT_LE(w.b, 1e-8);
  EXPECT_LE(w.v, 1e-8);
  EXPECT_LE(w.P, 1e-10);
}

TEST(ContactInekfUpdate, GaugeAccumulatedOverTheSequence) {
  GaugeWorst w;
  for (int seed = 0; seed < 8; ++seed) {
    for (double psi : {0.3, -0.3, 2.9, -2.9}) {
      GaugeRun(seed, psi, false, w);
    }
  }
  std::printf(
      "[measure] gauge 2000 ticks: |dNIS| %.3e  dz/s_kin %.3e  db/sb %.3e  dv/sv %.3e (1e-5)  "
      "(report) P rel %.3e\n",
      w.nis, w.zb, w.b, w.v, w.P);
  EXPECT_TRUE(w.same_decisions);
  EXPECT_LE(w.nis, 1e-5);
  EXPECT_LE(w.zb, 1e-5);
  EXPECT_LE(w.b, 1e-5);
  EXPECT_LE(w.v, 1e-5);
}

// ── Long runs with an update every tick ──────────────────────────────────────

TEST(ContactInekfUpdate, CovarianceStaysSymmetricPositiveDefiniteWithUpdates) {
  int asym = 0;
  int llt_fail = 0;
  double min_eig = std::numeric_limits<double>::infinity();
  for (int seed = 0; seed < 32; ++seed) {
    ContactInekf f;
    StartFilter(f, 500 + static_cast<std::uint64_t>(seed), 2);
    std::mt19937_64 rng(600 + static_cast<std::uint64_t>(seed));
    std::normal_distribution<double> nd(0.0, 1.0);
    Mat3 Rt = f.rotation();
    Vec3 vt = f.velocity();
    Vec3 pt = f.position();
    const std::array<Vec3, 2> dt_true{f.contact_position(0), f.contact_position(1)};
    const double amp = seed % 2 == 0 ? 3.0 : 0.3;
    auto check = [&] {
      const Cov& P = f.covariance();
      if (!(P == P.transpose())) {
        ++asym;
      }
      if (Eigen::LLT<Eigen::MatrixXd>(Gather(P, ActiveIndex(f))).info() != Eigen::Success) {
        ++llt_fail;
      }
    };
    for (int k = 0; k < 5000; ++k) {
      const double t = k * kDt;
      const Vec3 wm(amp * std::sin(1.3 * t + seed), amp * std::cos(0.7 * t),
                    0.5 * amp * std::sin(2.1 * t));
      const Vec3 am = Rt.transpose() * Vec3(0, 0, kG0) - 0.2 * Rt.transpose() * vt;
      Mat3 R1;
      Vec3 v1;
      Vec3 p1;
      Access::MeanMap(Rt, vt, pt, wm, am, kDt, kG0, R1, v1, p1);
      Rt = R1;
      vt = v1;
      pt = p1;
      ASSERT_EQ(f.Propagate(wm, am, kDt), InekfStatus::kOk);
      check();
      std::vector<ContactKinematics> kin(4);
      for (std::size_t i = 0; i < 2; ++i) {
        kin[i] = Kin(Rt.transpose() * (dt_true[i] - pt) + Vec3(nd(rng), nd(rng), nd(rng)) * 2e-3);
      }
      std::vector<ContactUpdateReport> rep(4);
      ASSERT_EQ(f.UpdateContacts(kin, rep), InekfStatus::kOk);
      check();
      if (k % 500 == 499) {
        min_eig = std::min(min_eig, Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(
                                        Gather(f.covariance(), ActiveIndex(f)))
                                        .eigenvalues()(0));
      }
    }
  }
  std::printf("[measure] 32x5000 with updates: asym %d  llt fail %d  (report) min eig %.3e\n", asym,
              llt_fail, min_eig);
  EXPECT_EQ(asym, 0);
  EXPECT_EQ(llt_fail, 0);
}

// ── χ² gate ──────────────────────────────────────────────────────────────────

/// Kinematics that put slot `bad` 10σ out along its weakest S direction.
std::vector<ContactKinematics> WithOutliers(const ContactInekf& f, const std::vector<int>& bad) {
  const ContactInekf::Config cfg = FixtureConfig();
  std::vector<ContactKinematics> k = PredictedKinematics(f);
  for (int s : bad) {
    const int o = ContactInekf::SlotOffset(s);
    const Cov& P = f.covariance();
    const Mat3 N =
        f.rotation() * BodyNoise(k[static_cast<std::size_t>(s)], cfg) * f.rotation().transpose();
    const Mat3 S =
        P.block<3, 3>(6, 6) + P.block<3, 3>(o, o) - P.block<3, 3>(6, o) - P.block<3, 3>(o, 6) + N;
    const Eigen::SelfAdjointEigenSolver<Mat3> es(S);
    const Vec3 dz = 10.0 * std::sqrt(es.eigenvalues()(0)) * es.eigenvectors().col(0);
    k[static_cast<std::size_t>(s)].position =
        f.rotation().transpose() * (f.contact_position(s) - f.position() + dz);
  }
  return k;
}

TEST(ContactInekfUpdate, GatedSlotIsLeftOutAndTheRestMatchTheReference) {
  const ContactInekf::Config cfg = FixtureConfig();
  double worst = 0.0;
  for (int rep_i = 0; rep_i < 64; ++rep_i) {
    ContactInekf f;
    Warm(f, 700 + static_cast<std::uint64_t>(rep_i), 3);
    const std::vector<ContactKinematics> k = WithOutliers(f, {0});
    PackedFilter o = PackedFilter::FromCore(f);
    std::vector<ContactUpdateReport> rep(4);
    ASSERT_EQ(f.UpdateContacts(k, rep), InekfStatus::kOk);
    EXPECT_EQ(rep[0].result, ContactUpdateResult::kGated);
    EXPECT_NEAR(rep[0].mahalanobis2, 100.0, 1e-6);
    EXPECT_EQ(rep[1].result, ContactUpdateResult::kUsed);
    EXPECT_EQ(rep[2].result, ContactUpdateResult::kUsed);
    EXPECT_EQ(rep[3].result, ContactUpdateResult::kInactive);
    EXPECT_TRUE(f.contact_active(0));
    o.Update({{1, k[1].position, BodyNoise(k[1], cfg)}, {2, k[2].position, BodyNoise(k[2], cfg)}});
    const auto [dx, dP] = o.Distance(f);
    worst = std::max({worst, dx, dP});
  }
  std::printf("[measure] gated slot left out: core vs reference on the rest %.3e (1e-12)\n", worst);
  EXPECT_LE(worst, 1e-12);
}

TEST(ContactInekfUpdate, AllGatedIsANoOp) {
  ContactInekf f;
  Warm(f, 800, 3);
  const Snapshot before = Capture(f);
  std::vector<ContactUpdateReport> rep(4);
  ASSERT_EQ(f.UpdateContacts(WithOutliers(f, {0, 1, 2}), rep), InekfStatus::kOk);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(rep[static_cast<std::size_t>(i)].result, ContactUpdateResult::kGated);
  }
  EXPECT_TRUE(BitEqual(Capture(f), before));
  // Nothing requested: also a no-op.
  std::vector<ContactKinematics> none(4);
  ASSERT_EQ(f.UpdateContacts(none, rep), InekfStatus::kOk);
  EXPECT_EQ(rep[0].result, ContactUpdateResult::kNotRequested);
  EXPECT_EQ(rep[3].result, ContactUpdateResult::kInactive);
  EXPECT_TRUE(BitEqual(Capture(f), before));
}

TEST(ContactInekfUpdate, UnrequestedSlotKeepsItsAnchor) {
  const ContactInekf::Config cfg = FixtureConfig();
  ContactInekf f;
  Warm(f, 900, 3);
  std::vector<ContactKinematics> k = PredictedKinematics(f);
  k[0].use = false;
  k[1].position += Vec3(1e-3, 0, 0);
  PackedFilter o = PackedFilter::FromCore(f);
  std::vector<ContactUpdateReport> rep(4);
  ASSERT_EQ(f.UpdateContacts(k, rep), InekfStatus::kOk);
  EXPECT_EQ(rep[0].result, ContactUpdateResult::kNotRequested);
  EXPECT_TRUE(f.contact_active(0));
  o.Update({{1, k[1].position, BodyNoise(k[1], cfg)}, {2, k[2].position, BodyNoise(k[2], cfg)}});
  const auto [dx, dP] = o.Distance(f);
  std::printf("[measure] use=false slot: core vs reference %.3e / %.3e (1e-12)\n", dx, dP);
  EXPECT_LE(std::max(dx, dP), 1e-12);
}

TEST(ContactInekfUpdate, SlotInsertedThisStepIsNotUsedTwice) {
  const ContactInekf::Config cfg = FixtureConfig();
  ContactInekf f;
  Warm(f, 950, 2);  // slots 0, 1 active, both older than the last Propagate
  ASSERT_EQ(f.InsertContact(2, Kin(FootPoint(2))), InekfStatus::kOk);
  std::vector<ContactKinematics> k = PredictedKinematics(f);
  k[0].position += Vec3(1e-3, -1e-3, 0.0);
  PackedFilter o = PackedFilter::FromCore(f);
  std::vector<ContactUpdateReport> rep(4);
  ASSERT_EQ(f.UpdateContacts(k, rep), InekfStatus::kOk);
  EXPECT_EQ(rep[2].result, ContactUpdateResult::kInsertedThisStep);
  EXPECT_EQ(rep[0].result, ContactUpdateResult::kUsed);
  o.Update({{0, k[0].position, BodyNoise(k[0], cfg)}, {1, k[1].position, BodyNoise(k[1], cfg)}});
  const auto [dx, dP] = o.Distance(f);
  EXPECT_LE(std::max(dx, dP), 1e-12);
  // After the next step it is an ordinary slot.
  ASSERT_EQ(f.Propagate(Vec3::Zero(), f.rotation().transpose() * Vec3(0, 0, kG0), kDt),
            InekfStatus::kOk);
  ASSERT_EQ(f.UpdateContacts(PredictedKinematics(f), rep), InekfStatus::kOk);
  EXPECT_EQ(rep[2].result, ContactUpdateResult::kUsed);
}

// ── Refusals ─────────────────────────────────────────────────────────────────

TEST(ContactInekfUpdate, RefusalsLeaveTheStateUntouched) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<ContactUpdateReport> rep(4);
  {
    ContactInekf f;
    f.Init(FixtureConfig(), 4);
    EXPECT_EQ(f.UpdateContacts(std::vector<ContactKinematics>(4), rep),
              InekfStatus::kNotInitialized);
  }
  ContactInekf f;
  Warm(f, 1000, 2);  // slots 0, 1 active
  const Snapshot before = Capture(f);
  // A refused call writes neither the state nor the report.
  auto expect = [&](const std::vector<ContactKinematics>& k, std::vector<ContactUpdateReport>& r,
                    InekfStatus want) {
    for (ContactUpdateReport& x : r) {
      x.mahalanobis2 = -7.0;
    }
    EXPECT_EQ(f.UpdateContacts(k, r), want);
    EXPECT_TRUE(BitEqual(Capture(f), before));
    for (const ContactUpdateReport& x : r) {
      EXPECT_EQ(x.mahalanobis2, -7.0);
    }
  };
  std::vector<ContactUpdateReport> short_rep(3);
  expect(PredictedKinematics(f), short_rep, InekfStatus::kSizeMismatch);
  expect(std::vector<ContactKinematics>(3), rep, InekfStatus::kSizeMismatch);
  std::vector<ContactKinematics> k = PredictedKinematics(f);
  k[2] = Kin(FootPoint(2));  // inactive, requested
  expect(k, rep, InekfStatus::kSlotInactive);
  k = PredictedKinematics(f);
  k[0].position.x() = nan;
  expect(k, rep, InekfStatus::kNonFiniteInput);
  k = PredictedKinematics(f);
  k[1].jacobian = Eigen::MatrixXd::Constant(3, 2, nan);
  expect(k, rep, InekfStatus::kNonFiniteInput);
  k = PredictedKinematics(f);
  k[1].jacobian = Eigen::MatrixXd::Constant(3, 2, 1e200);  // Σ overflows
  expect(k, rep, InekfStatus::kNonFiniteResult);

  // Finite inputs whose innovation overflows (p near the double range).
  {
    ContactInekf g = f;
    std::array<Vec3, 4> d = ZeroContacts();
    d[0] = f.contact_position(0);
    d[1] = f.contact_position(1);
    Access::SetState(g, f.rotation(), f.velocity(), Vec3(1.5e308, 0, 0), d, {true, true},
                     f.gyro_bias(), f.accel_bias(), f.covariance());
    const Snapshot gb = Capture(g);
    std::vector<ContactKinematics> kk(4);
    kk[0] = Kin(f.rotation().transpose() * Vec3(1.5e308, 0, 0));
    std::vector<ContactUpdateReport> r(4);
    EXPECT_EQ(g.UpdateContacts(kk, r), InekfStatus::kNonFiniteResult);
    EXPECT_TRUE(BitEqual(Capture(g), gb));
  }

  // Seam-injected covariances that break each positive-definiteness test.
  const int o0 = ContactInekf::SlotOffset(0);
  const int o1 = ContactInekf::SlotOffset(1);
  auto with_P = [&](const Cov& P, InekfStatus want, bool only_slot0 = false) {
    ContactInekf g = f;
    Access::SetCovariance(g, P);
    const Snapshot gb = Capture(g);
    std::vector<ContactUpdateReport> r(4);
    std::vector<ContactKinematics> kk = PredictedKinematics(g);
    if (only_slot0) {
      kk[1].use = false;
    }
    EXPECT_EQ(g.UpdateContacts(kk, r), want);
    EXPECT_TRUE(BitEqual(Capture(g), gb));
  };
  {
    // Per-slot S not PD, and that slot is the only one requested: its m²
    // would be meaningless, so nothing downstream may see it.
    Cov P = f.covariance();
    P.block<3, 3>(o0, o0) -= 10.0 * Mat3::Identity();
    with_P(P, InekfStatus::kNotPositiveDefinite, true);
  }
  {
    Cov P = Cov::Identity(f.dim(), f.dim());  // stacked S not PD: anchors anti-correlated
    P.block<3, 3>(o0, o1) = 2.0 * Mat3::Identity();
    P.block<3, 3>(o1, o0) = 2.0 * Mat3::Identity();
    for (int s = 2; s < 4; ++s) {
      P.middleRows(ContactInekf::SlotOffset(s), 3).setZero();
      P.middleCols(ContactInekf::SlotOffset(s), 3).setZero();
    }
    with_P(P, InekfStatus::kNotPositiveDefinite);
  }
  {
    Cov P = f.covariance();  // velocity block not PD, contact blocks fine
    P.middleRows(3, 3).setZero();
    P.middleCols(3, 3).setZero();
    P.block<3, 3>(3, 3) = -Mat3::Identity();
    with_P(P, InekfStatus::kNotPositiveDefinite);
  }
}

}  // namespace
}  // namespace rtc::estimation::testing
