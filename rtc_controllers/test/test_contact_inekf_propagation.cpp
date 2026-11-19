// ── Contact InEKF core: IMU propagation ───────────────────────────────────────
//
// Mean, Γ series, Φ and Q_d against independent oracles over a fixed grid
// (5 steps × 8 rotation steps × 8 states = 320 cases), range and finiteness
// refusals, covariance symmetry / positive definiteness over long runs, the
// world-yaw + translation gauge, and determinism. Thresholds were fixed before
// the first measurement; every gate prints what it measured.
#include "contact_inekf_oracle.hpp"
#include "contact_inekf_testing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

namespace rtc::estimation::testing {
namespace {

using oracle::LD;
using oracle::Mat3L;
using oracle::MatL;
using oracle::Vec3L;

struct GridCase {
  double dt;
  double theta;
  std::size_t state;
  Vec3 gyro;
  Vec3 accel;
};

std::vector<GridCase> GridCases(const std::vector<GridState>& states) {
  std::vector<GridCase> out;
  for (double dt : kGridDt) {
    for (double th : kGridTheta) {
      for (std::size_t k = 0; k < states.size(); ++k) {
        const GridState& s = states[k];
        out.push_back({dt, th, k, Vec3(s.wdir * (th / dt) + s.bg), Vec3(s.specific_force + s.ba)});
      }
    }
  }
  return out;
}

std::vector<Vec3> ActiveCols(const GridState& s, const Vec3& v, const Vec3& p) {
  std::vector<Vec3> cols{v, p};
  for (std::size_t i = 0; i < 4; ++i) {
    if (s.active[i]) {
      cols.push_back(s.d[i]);
    }
  }
  return cols;
}

std::vector<Vec3L> ToL(const std::vector<Vec3>& c) {
  std::vector<Vec3L> out;
  for (const Vec3& x : c) {
    out.push_back(x.cast<LD>());
  }
  return out;
}

int ActiveCount(const GridState& s) {
  return static_cast<int>(std::count(s.active.begin(), s.active.end(), true));
}

// ── Γ series ──────────────────────────────────────────────────────────────────

TEST(ContactInekfPropagation, GammaSeriesMatchesMatrixSeries) {
  std::mt19937_64 rng(kSeed);
  std::normal_distribution<double> nd(0.0, 1.0);
  double worst = 0.0;
  auto check = [&](const Vec3& phi) {
    for (int m = 0; m <= 2; ++m) {
      const Mat3L ref = oracle::GammaOracle(m, phi.cast<LD>());
      const Mat3 g = m == 0   ? GammaSeries<0>(phi)
                     : m == 1 ? GammaSeries<1>(phi)
                              : GammaSeries<2>(phi);
      const Mat3L got = g.cast<LD>();
      worst = std::max(worst, static_cast<double>((got - ref).cwiseAbs().maxCoeff()));
    }
  };
  for (int i = 0; i < 64; ++i) {
    const Vec3 dir = Vec3(nd(rng), nd(rng), nd(rng)).normalized();
    check(dir * (0.7 * static_cast<double>(i) / 63.0));
  }
  for (double th : {1e-9, 1e-6, 1e-4, 1e-3}) {
    check(Vec3(0.3, -0.5, 0.81).normalized() * th);
  }
  std::printf("[measure] Gamma max |elem err| = %.3e (gate 1e-15)\n", worst);
  EXPECT_LE(worst, 1e-15);
}

// ── Mean ──────────────────────────────────────────────────────────────────────

TEST(ContactInekfPropagation, MeanMatchesQuadratureOracle) {
  const std::vector<GridState> states = GridStates();
  double worst = 0.0;
  for (const GridCase& c : GridCases(states)) {
    const GridState& s = states[c.state];
    ContactInekf f;
    LoadGridState(f, s);
    ASSERT_EQ(f.Propagate(c.gyro, c.accel, c.dt), InekfStatus::kOk);
    const Vec3 w = c.gyro - s.bg;
    const Vec3 a = c.accel - s.ba;
    const oracle::MeanL ref =
        oracle::MeanOracle(s.R.cast<LD>(), s.v.cast<LD>(), s.p.cast<LD>(), w.cast<LD>(),
                           a.cast<LD>(), static_cast<LD>(c.dt), static_cast<LD>(kG0));
    const double eR = static_cast<double>((f.rotation().cast<LD>() - ref.R).norm()) /
                      (1.0 + static_cast<double>(ref.R.norm()));
    const double ev = static_cast<double>((f.velocity().cast<LD>() - ref.v).norm()) /
                      (1.0 + static_cast<double>(ref.v.norm()));
    const double ep = static_cast<double>((f.position().cast<LD>() - ref.p).norm()) /
                      (1.0 + static_cast<double>(ref.p.norm()));
    worst = std::max({worst, eR, ev, ep});
  }
  std::printf("[measure] mean max ||d||/(1+||ref||) = %.3e over 320 (gate 1e-12)\n", worst);
  EXPECT_LE(worst, 1e-12);
}

// ── Φ vs Lie finite differences of the exact mean map ────────────────────────

TEST(ContactInekfPropagation, TransitionMatchesLieFiniteDifference) {
  const std::vector<GridState> states = GridStates();
  constexpr double kEps = 1e-6;
  double worst = 0.0;
  for (const GridCase& c : GridCases(states)) {
    const GridState& s = states[c.state];
    ContactInekf f;
    LoadGridState(f, s);
    ASSERT_EQ(f.Propagate(c.gyro, c.accel, c.dt), InekfStatus::kOk);
    const std::vector<int> idx = ActiveIndex(f);
    const Eigen::MatrixXd Phi = Gather(Access::Phi(f), idx);
    const int Dp = static_cast<int>(idx.size());
    const int nx = Dp - 6;

    const std::vector<Vec3> cols0 = ActiveCols(s, s.v, s.p);
    Mat3 R1h;
    Vec3 v1h;
    Vec3 p1h;
    Access::MeanMap(s.R, s.v, s.p, c.gyro - s.bg, c.accel - s.ba, c.dt, kG0, R1h, v1h, p1h);
    std::vector<Vec3> cols1h = cols0;
    cols1h[0] = v1h;
    cols1h[1] = p1h;

    Eigen::MatrixXd J(Dp, Dp);
    for (int i = 0; i < Dp; ++i) {
      Eigen::VectorXd e[2];
      for (int k = 0; k < 2; ++k) {
        Eigen::VectorXd xi = Eigen::VectorXd::Zero(Dp);
        xi(i) = (k == 0 ? 1.0 : -1.0) * kEps;
        Mat3 Rt = s.R;
        std::vector<Vec3> ct = cols0;
        ApplyExpLeft(xi.head(nx), Rt, ct);
        const Vec3 bg = s.bg + xi.segment<3>(nx);
        const Vec3 ba = s.ba + xi.segment<3>(nx + 3);
        Mat3 R1;
        Vec3 v1;
        Vec3 p1;
        Access::MeanMap(Rt, ct[0], ct[1], c.gyro - bg, c.accel - ba, c.dt, kG0, R1, v1, p1);
        ct[0] = v1;
        ct[1] = p1;
        e[k] = Eigen::VectorXd(Dp);
        e[k].head(nx) = LogError(R1, ct, R1h, cols1h);
        e[k].tail<6>() = xi.tail<6>();
      }
      J.col(i) = (e[0] - e[1]) / (2.0 * kEps);
    }
    const double scale = std::max(1.0, Phi.cwiseAbs().maxCoeff());
    worst = std::max(worst, (J - Phi).cwiseAbs().maxCoeff() / scale);
  }
  std::printf("[measure] Phi vs Lie FD max = %.3e over 320 (gate 1e-7)\n", worst);
  EXPECT_LE(worst, 1e-7);
}

// ── Φ and Q_d vs the long-double Van Loan oracle ─────────────────────────────

double QdGate(double dt) {
  if (dt < 5e-4) {
    return 1e-6;
  }
  if (dt < 1.5e-3) {
    return 2e-8;
  }
  if (dt < 5e-3) {
    return 1e-8;
  }
  return 3e-10;
}

TEST(ContactInekfPropagation, TransitionAndNoiseMatchVanLoan) {
  const std::vector<GridState> states = GridStates();
  const ContactInekf::Config cfg = FixtureConfig();

  struct Worst {
    double xx = 0.0, xb = 0.0, qd = 0.0;
  };

  std::array<Worst, kGridDt.size()> per_dt{};
  bool pass = true;
  for (const GridCase& c : GridCases(states)) {
    const GridState& s = states[c.state];
    ContactInekf f;
    LoadGridState(f, s);
    ASSERT_EQ(f.Propagate(c.gyro, c.accel, c.dt), InekfStatus::kOk);
    const std::vector<int> idx = ActiveIndex(f);
    const Eigen::MatrixXd Phi = Gather(Access::Phi(f), idx);
    const Eigen::MatrixXd Qd = Gather(Access::Qd(f), idx);
    const int k = ActiveCount(s);
    const int Dp = 15 + 3 * k;

    std::vector<LD> qc(static_cast<std::size_t>(Dp), 0.0L);
    auto fill = [&](int o, double sigma) {
      for (int i = 0; i < 3; ++i) {
        qc[static_cast<std::size_t>(o + i)] = static_cast<LD>(sigma) * static_cast<LD>(sigma);
      }
    };
    fill(0, cfg.gyro_noise_density);
    fill(3, cfg.accel_noise_density);
    for (int i = 0; i < k; ++i) {
      fill(9 + 3 * i, cfg.contact_noise_density);
    }
    fill(9 + 3 * k, cfg.gyro_bias_walk);
    fill(12 + 3 * k, cfg.accel_bias_walk);

    const Vec3 w = c.gyro - s.bg;
    const Vec3 a = c.accel - s.ba;
    const oracle::Discrete left =
        oracle::VanLoanLeft(w.cast<LD>(), a.cast<LD>(), static_cast<LD>(c.dt), qc, k);
    const std::vector<Vec3> cols0 = ActiveCols(s, s.v, s.p);
    const std::vector<Vec3> cols1 = ActiveCols(s, f.velocity(), f.position());
    const oracle::Discrete ri = oracle::ToRightInvariant(left, s.R.cast<LD>(), ToL(cols0),
                                                         f.rotation().cast<LD>(), ToL(cols1));

    double big = 0.0;
    for (const Vec3& x : cols0) {
      big = std::max(big, x.norm());
    }
    const double xx_bound = 1e-13 * (1.0 + big);
    const int nb = Dp / 3;
    const int ibg = nb - 2;
    const int iba = nb - 1;
    Worst& wd = per_dt[static_cast<std::size_t>(std::find(kGridDt.begin(), kGridDt.end(), c.dt) -
                                                kGridDt.begin())];
    for (int bi = 0; bi < nb; ++bi) {
      for (int bj = 0; bj < nb; ++bj) {
        const MatL ref = ri.Phi.block<3, 3>(3 * bi, 3 * bj);
        const MatL got = Phi.block<3, 3>(3 * bi, 3 * bj).cast<LD>();
        const double diff = static_cast<double>((got - ref).norm());
        const bool bias_col = bj >= ibg && bi < ibg;
        const bool zero_bias_block = bias_col && bj == iba && bi != 1 && bi != 2;
        if (bias_col && !zero_bias_block) {
          const double r = diff / static_cast<double>(ref.norm());
          wd.xb = std::max(wd.xb, r);
          pass = pass && r <= 1e-9;
        } else {
          wd.xx = std::max(wd.xx, diff / (1.0 + big));
          pass = pass && diff <= xx_bound;
        }
      }
    }
    const double wq = oracle::Whitened(Qd.cast<LD>(), ri.Qd);
    wd.qd = std::max(wd.qd, wq);
    pass = pass && wq <= QdGate(c.dt);
  }
  for (std::size_t i = 0; i < kGridDt.size(); ++i) {
    std::printf(
        "[measure] dt %.0e: Phi state-state |d|/(1+|x|) %.3e (1e-13)  bias-col rel %.3e (1e-9)  "
        "Qd whitened %.3e (%.0e)\n",
        kGridDt[i], per_dt[i].xx, per_dt[i].xb, per_dt[i].qd, QdGate(kGridDt[i]));
  }
  EXPECT_TRUE(pass);
}

// ── Refusals leave the state bit-identical ───────────────────────────────────

class ContactInekfRefusal : public ::testing::Test {
 protected:
  void SetUp() override {
    states_ = GridStates();
    LoadGridState(f_, states_[4]);  // four active slots
    before_ = Capture(f_);
  }

  void ExpectRefused(const Vec3& gyro, const Vec3& accel, double dt, InekfStatus want) {
    EXPECT_EQ(f_.Propagate(gyro, accel, dt), want);
    EXPECT_TRUE(BitEqual(Capture(f_), before_));
  }

  Vec3 Gyro(double rate_step, double dt) const {
    return states_[4].wdir * (rate_step / dt) + states_[4].bg;
  }

  std::vector<GridState> states_;
  ContactInekf f_;
  Snapshot before_;
};

TEST_F(ContactInekfRefusal, StepOutOfRangeOrNonFinite) {
  const Vec3 a = states_[4].specific_force + states_[4].ba;
  const Vec3 w = Gyro(0.1, 2e-3);
  ExpectRefused(w, a, 1e-4, InekfStatus::kStepOutOfRange);
  ExpectRefused(w, a, 2.1e-2, InekfStatus::kStepOutOfRange);
  ExpectRefused(w, a, 0.0, InekfStatus::kStepOutOfRange);
  ExpectRefused(w, a, -1e-3, InekfStatus::kStepOutOfRange);
  ExpectRefused(w, a, std::numeric_limits<double>::quiet_NaN(), InekfStatus::kNonFiniteInput);
}

TEST_F(ContactInekfRefusal, RotationStepOutOfRange) {
  const Vec3 a = states_[4].specific_force + states_[4].ba;
  ExpectRefused(Gyro(0.71, 2e-3), a, 2e-3, InekfStatus::kRotationOutOfRange);
  ExpectRefused(Gyro(0.7 * (1.0 + 1e-9), 2e-3), a, 2e-3, InekfStatus::kRotationOutOfRange);
  // The grid's edge point is inside the range.
  ContactInekf g;
  LoadGridState(g, states_[4]);
  EXPECT_EQ(g.Propagate(Gyro(0.7 * (1.0 - 1e-12), 2e-3), a, 2e-3), InekfStatus::kOk);
}

TEST_F(ContactInekfRefusal, NonFiniteImuSample) {
  const Vec3 a = states_[4].specific_force + states_[4].ba;
  const Vec3 w = Gyro(0.1, 2e-3);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  ExpectRefused(Vec3(nan, 0, 0), a, 2e-3, InekfStatus::kNonFiniteInput);
  ExpectRefused(Vec3(0, inf, 0), a, 2e-3, InekfStatus::kNonFiniteInput);
  ExpectRefused(w, Vec3(0, 0, nan), 2e-3, InekfStatus::kNonFiniteInput);
  ExpectRefused(w, Vec3(-inf, 0, 0), 2e-3, InekfStatus::kNonFiniteInput);
}

TEST_F(ContactInekfRefusal, FiniteInputNonFiniteResult) {
  ExpectRefused(Gyro(0.1, 2e-3), Vec3(1e200, -1e200, 1e200), 2e-3, InekfStatus::kNonFiniteResult);
}

TEST_F(ContactInekfRefusal, NonPositiveDefiniteCovariance) {
  Cov P = f_.covariance();
  P(0, 0) = -1.0;
  Access::SetCovariance(f_, P);
  before_ = Capture(f_);
  ExpectRefused(Gyro(0.1, 2e-3), states_[4].specific_force + states_[4].ba, 2e-3,
                InekfStatus::kNotPositiveDefinite);
}

// ── Long runs: symmetry, positive definiteness, orthonormality ───────────────

struct ImuProfile {
  Vec3 amp, freq, phase, acc_amp;

  Vec3 Gyro(double t) const {
    return Vec3(amp.x() * std::sin(freq.x() * t + phase.x()),
                amp.y() * std::sin(freq.y() * t + phase.y()),
                amp.z() * std::sin(freq.z() * t + phase.z()));
  }

  Vec3 Accel(double t) const {
    return Vec3(acc_amp.x() * std::cos(0.7 * freq.x() * t), acc_amp.y() * std::sin(1.3 * t),
                kG0 + acc_amp.z() * std::sin(0.9 * freq.z() * t + 0.4));
  }
};

ImuProfile RandomProfile(std::mt19937_64& rng, double max_rate) {
  std::uniform_real_distribution<double> ud(0.0, 1.0);
  ImuProfile p;
  p.amp = Vec3(ud(rng), ud(rng), ud(rng)) * max_rate;
  p.freq = Vec3(0.2 + 3.0 * ud(rng), 0.2 + 3.0 * ud(rng), 0.2 + 3.0 * ud(rng));
  p.phase = Vec3(ud(rng), ud(rng), ud(rng)) * 6.28;
  p.acc_amp = Vec3(ud(rng), ud(rng), ud(rng)) * 4.0;
  return p;
}

/// A random positive definite P on the active rows (correlated).
Cov RandomCovariance(const ContactInekf& f, std::mt19937_64& rng) {
  std::normal_distribution<double> nd(0.0, 1.0);
  const std::vector<int> idx = ActiveIndex(f);
  const auto n = static_cast<Eigen::Index>(idx.size());
  Eigen::MatrixXd A(n, n);
  for (Eigen::Index c = 0; c < n; ++c) {
    for (Eigen::Index r = 0; r < n; ++r) {
      A(r, c) = nd(rng) * 1e-3;
    }
  }
  Eigen::MatrixXd S = A * A.transpose();
  S.diagonal().array() += 1e-6;
  Cov P = Cov::Zero(f.dim(), f.dim());
  for (Eigen::Index c = 0; c < n; ++c) {
    for (Eigen::Index r = 0; r < n; ++r) {
      P(idx[static_cast<std::size_t>(r)], idx[static_cast<std::size_t>(c)]) =
          0.5 * (S(r, c) + S(c, r));
    }
  }
  return P;
}

void LoadRandom(ContactInekf& f, std::mt19937_64& rng, int num_slots, int active) {
  std::normal_distribution<double> nd(0.0, 1.0);
  f.Init(FixtureConfig(), num_slots);
  std::array<Vec3, 4> d = ZeroContacts();
  std::array<bool, 4> on{};
  for (int i = 0; i < active; ++i) {
    on[static_cast<std::size_t>(i)] = true;
    d[static_cast<std::size_t>(i)] = Vec3(nd(rng), nd(rng), nd(rng));
  }
  const Mat3 R = Renormalize(rtc::math::se3::exp3(Vec3(nd(rng), nd(rng), nd(rng))));
  Access::SetState(f, R, Vec3(nd(rng), nd(rng), nd(rng)), Vec3(nd(rng), nd(rng), nd(rng)), d, on,
                   Vec3(nd(rng), nd(rng), nd(rng)) * 1e-3, Vec3(nd(rng), nd(rng), nd(rng)) * 5e-2,
                   Cov::Zero(f.dim(), f.dim()));
  Access::SetCovariance(f, RandomCovariance(f, rng));
}

double OrthoError(const Mat3& R) {
  return (R.transpose() * R - Mat3::Identity()).cwiseAbs().maxCoeff();
}

TEST(ContactInekfLongRun, CovarianceStaysSymmetricPositiveDefinite) {
  constexpr int kSeeds = 32;
  constexpr int kTicks = 5000;
  constexpr double kDt = 2e-3;
  double worst_ortho = 0.0;
  double min_eig = std::numeric_limits<double>::infinity();
  int asym = 0;
  int llt_fail = 0;
  for (int seed = 0; seed < kSeeds; ++seed) {
    std::mt19937_64 rng(kSeed + static_cast<std::uint64_t>(seed));
    ContactInekf f;
    LoadRandom(f, rng, 4, seed % 5);
    const ImuProfile imu = RandomProfile(rng, seed % 2 == 0 ? 20.0 : 2.0);
    for (int k = 0; k < kTicks; ++k) {
      const double t = kDt * k;
      ASSERT_EQ(f.Propagate(imu.Gyro(t), imu.Accel(t), kDt), InekfStatus::kOk)
          << "seed " << seed << " tick " << k;
      const Cov& P = f.covariance();
      if (!(P == P.transpose())) {
        ++asym;
      }
      const Eigen::MatrixXd Pa = Gather(P, ActiveIndex(f));
      if (Eigen::LLT<Eigen::MatrixXd>(Pa).info() != Eigen::Success) {
        ++llt_fail;
      }
      if (k % 500 == 499) {
        min_eig =
            std::min(min_eig, Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(Pa).eigenvalues()(0));
      }
      worst_ortho = std::max(worst_ortho, OrthoError(f.rotation()));
    }
  }
  std::printf("[measure] 32x5000: asym %d  llt fail %d  ortho %.3e  (report) min eig %.3e\n", asym,
              llt_fail, worst_ortho, min_eig);
  EXPECT_EQ(asym, 0);
  EXPECT_EQ(llt_fail, 0);
  EXPECT_LE(worst_ortho, 1e-14);
}

TEST(ContactInekfLongRun, RotationStaysOrthonormal) {
  constexpr int kTicks = 200000;
  constexpr double kDt = 2e-3;
  std::mt19937_64 rng(kSeed);
  ContactInekf f;
  LoadRandom(f, rng, 2, 2);
  const ImuProfile imu = RandomProfile(rng, 20.0);
  double worst = 0.0;
  for (int k = 0; k < kTicks; ++k) {
    const double t = kDt * k;
    ASSERT_EQ(f.Propagate(imu.Gyro(t), imu.Accel(t), kDt), InekfStatus::kOk) << "tick " << k;
    worst = std::max(worst, OrthoError(f.rotation()));
  }
  std::printf("[measure] 2e5 ticks: max |R^T R - I| = %.3e (gate 1e-14)\n", worst);
  EXPECT_LE(worst, 1e-14);
}

// ── Gauge: world yaw about gravity + translation ─────────────────────────────

Cov GaugeAdjoint(const ContactInekf& f, const Mat3& Rz, const Vec3& t) {
  Cov A = Cov::Zero(f.dim(), f.dim());
  A.block<3, 3>(0, 0) = Rz;
  A.block<3, 3>(3, 3) = Rz;
  A.block<3, 3>(6, 0) = rtc::math::se3::hat(t) * Rz;
  A.block<3, 3>(6, 6) = Rz;
  for (int i = 0; i < f.num_slots(); ++i) {
    const int o = ContactInekf::SlotOffset(i);
    A.block<3, 3>(o, 0) = rtc::math::se3::hat(t) * Rz;
    A.block<3, 3>(o, o) = Rz;
  }
  A.block<6, 6>(f.BiasOffset(), f.BiasOffset()).setIdentity();
  return A;
}

TEST(ContactInekfGauge, PropagationIsYawAndTranslationInvariant) {
  constexpr int kTicks = 2000;
  constexpr double kDt = 2e-3;
  double w_body = 0.0;
  double w_P = 0.0;
  bool bias_bit = true;
  for (int seed = 0; seed < 8; ++seed) {
    for (double psi : {0.3, -0.3, 2.9, -2.9}) {
      std::mt19937_64 rng(kSeed + 100 + static_cast<std::uint64_t>(seed));
      std::normal_distribution<double> nd(0.0, 1.0);
      std::uniform_real_distribution<double> ud(0.0, 1.0);
      const int n = seed % 2 == 0 ? 4 : 2;
      ContactInekf f1;
      LoadRandom(f1, rng, n, n);
      const Vec3 t = Vec3(nd(rng), nd(rng), nd(rng)).normalized() * (10.0 * ud(rng));
      const Mat3 Rz = RotZ(psi);
      std::array<Vec3, 4> d2 = ZeroContacts();
      std::array<bool, 4> on{};
      for (int i = 0; i < n; ++i) {
        on[static_cast<std::size_t>(i)] = true;
        d2[static_cast<std::size_t>(i)] = Rz * f1.contact_position(i) + t;
      }
      ContactInekf f2;
      f2.Init(FixtureConfig(), n);
      const Cov AdG = GaugeAdjoint(f1, Rz, t);
      const Cov P2 = AdG * f1.covariance() * AdG.transpose();
      Access::SetState(f2, Rz * f1.rotation(), Rz * f1.velocity(), Rz * f1.position() + t, d2, on,
                       f1.gyro_bias(), f1.accel_bias(), P2);
      const ImuProfile imu = RandomProfile(rng, 10.0);
      const Cov AdGi = AdG.inverse();
      for (int k = 0; k < kTicks; ++k) {
        const double tk = kDt * k;
        ASSERT_EQ(f1.Propagate(imu.Gyro(tk), imu.Accel(tk), kDt), InekfStatus::kOk);
        ASSERT_EQ(f2.Propagate(imu.Gyro(tk), imu.Accel(tk), kDt), InekfStatus::kOk);
        const Vec3 b1 = f1.rotation().transpose() * f1.velocity();
        const Vec3 b2 = f2.rotation().transpose() * f2.velocity();
        w_body = std::max(w_body, (b1 - b2).norm() / b1.norm());
        for (int i = 0; i < n; ++i) {
          const Vec3 r1 = f1.rotation().transpose() * (f1.contact_position(i) - f1.position());
          const Vec3 r2 = f2.rotation().transpose() * (f2.contact_position(i) - f2.position());
          w_body = std::max(w_body, (r1 - r2).norm() / r1.norm());
        }
        bias_bit =
            bias_bit && f1.gyro_bias() == f2.gyro_bias() && f1.accel_bias() == f2.accel_bias();
        const Cov back = AdGi * f2.covariance() * AdGi.transpose();
        w_P = std::max(w_P, (back - f1.covariance()).norm() / f1.covariance().norm());
      }
    }
  }
  std::printf("[measure] gauge 32x2000: body rel %.3e  P rel %.3e  (gate 1e-9)  bias bit %d\n",
              w_body, w_P, static_cast<int>(bias_bit));
  EXPECT_LE(w_body, 1e-9);
  EXPECT_LE(w_P, 1e-9);
  EXPECT_TRUE(bias_bit);
}

// ── Determinism ──────────────────────────────────────────────────────────────

const std::array<Vec3, 2> kStance{Vec3(0.05, 0.1, -0.7), Vec3(0.05, -0.1, -0.7)};

void InitializeLevel(ContactInekf& f) {
  ASSERT_EQ(f.Initialize(Vec3(1e-4, -2e-4, 3e-4), Vec3(0.3, -0.2, 9.80), 2.0, 0.4, kStance),
            InekfStatus::kOk);
}

/// Activate the stance slots through the seam with a correlated P.
void ActivateStance(ContactInekf& f, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::array<Vec3, 4> d = ZeroContacts();
  std::array<bool, 4> on{};
  for (std::size_t i = 0; i < kStance.size(); ++i) {
    on[i] = true;
    d[i] = f.position() + f.rotation() * kStance[i];
  }
  Access::SetState(f, f.rotation(), f.velocity(), f.position(), d, on, f.gyro_bias(),
                   f.accel_bias(), Cov::Zero(f.dim(), f.dim()));
  Access::SetCovariance(f, RandomCovariance(f, rng));
}

void Drive(ContactInekf& f, int ticks, std::uint64_t seed, bool change_scale) {
  std::mt19937_64 rng(seed);
  const ImuProfile imu = RandomProfile(rng, 5.0);
  for (int k = 0; k < ticks; ++k) {
    if (change_scale && k == ticks / 2) {
      ASSERT_EQ(f.SetContactNoiseScale(1, 4.0), InekfStatus::kOk);
    }
    const double t = 2e-3 * k;
    ASSERT_EQ(f.Propagate(imu.Gyro(t), imu.Accel(t), 2e-3), InekfStatus::kOk);
  }
}

TEST(ContactInekfDeterminism, SameInputsSameBits) {
  ContactInekf a;
  ContactInekf b;
  for (ContactInekf* f : {&a, &b}) {
    f->Init(FixtureConfig(), 3);
    InitializeLevel(*f);
    ActivateStance(*f, 7);
    Drive(*f, 10000, 11, true);
  }
  EXPECT_TRUE(BitEqual(Capture(a), Capture(b)));

  // Reset + the same sequence again reproduces the first run.
  const Snapshot first = Capture(a);
  a.Reset();
  InitializeLevel(a);
  ActivateStance(a, 7);
  Drive(a, 10000, 11, true);
  EXPECT_TRUE(BitEqual(Capture(a), first));
}

TEST(ContactInekfDeterminism, ResetForgetsHistory) {
  ContactInekf used;
  used.Init(FixtureConfig(), 3);
  InitializeLevel(used);
  ActivateStance(used, 3);
  Drive(used, 500, 5, false);
  ASSERT_EQ(used.SetContactNoiseScale(2, 9.0), InekfStatus::kOk);
  Access::SetActive(used, 2, true);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(used.Propagate(Vec3(nan, 0, 0), Vec3(0, 0, kG0), 2e-3), InekfStatus::kOk);
  EXPECT_NE(used.Propagate(Vec3::Zero(), Vec3(0, 0, kG0), 5e-2), InekfStatus::kOk);
  used.Reset();

  ContactInekf fresh;
  fresh.Init(FixtureConfig(), 3);
  EXPECT_TRUE(BitEqual(Capture(used), Capture(fresh)));

  for (ContactInekf* f : {&used, &fresh}) {
    InitializeLevel(*f);
    ActivateStance(*f, 7);
    Drive(*f, 1000, 11, false);
  }
  EXPECT_TRUE(BitEqual(Capture(used), Capture(fresh)));
}

}  // namespace
}  // namespace rtc::estimation::testing
