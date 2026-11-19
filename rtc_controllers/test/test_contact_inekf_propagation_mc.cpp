// ── Contact InEKF core: propagated covariance vs a sub-stepped nonlinear truth ─
//
// Independent of the core's linearisation and of the Van Loan oracle. The truth
// is the continuous model: on tick k it sees the same held IMU sample as the
// filter, minus a random-walking bias and minus white noise realised by 20
// sub-steps (each a constant draw of variance density²/h); anchors and biases
// random-walk per sub-step; each sub-step is integrated exactly. With no
// updates, the filter's P after K ticks must describe the spread of the true
// errors: whitened statistic max|eig(C − I)| ≤ 0.25 at K = 5 and K = 500
// (M = 4000 trajectories, Δt = 2 ms, two active slots, dimension 21).
//
// The truth starts from X̂ perturbed by a draw from P₀ (P₀ = a small X prior +
// the bias prior), so the filter's Cholesky test sees a positive definite P.
#include "contact_inekf_testing.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <vector>

namespace rtc::estimation::testing {
namespace {

constexpr double kDt = 2e-3;
constexpr int kSub = 20;
constexpr int kTrials = 4000;
constexpr std::array<int, 2> kSnapshots{5, 500};
constexpr int kDim = 21;  // 15 + 3·2

using VecD = Eigen::Matrix<double, kDim, 1>;
using MatD = Eigen::Matrix<double, kDim, kDim>;

struct Imu {
  Vec3 gyro;
  Vec3 accel;
};

struct FilterSnap {
  Mat3 R;
  Vec3 v, p;
  std::array<Vec3, 2> d;
  MatD P;
};

// Γ₂ of a sub-step: |ω h| ≈ 1e-4, so the cubic tail is below 1e-17 relative.
Mat3 Gamma2Small(const Vec3& phi) {
  const Mat3 W = rtc::math::se3::hat(phi);
  return 0.5 * Mat3::Identity() + W / 6.0 + (W * W) / 24.0;
}

TEST(ContactInekfMonteCarlo, PropagatedCovarianceDescribesTrueErrors) {
  const ContactInekf::Config cfg = FixtureConfig();
  constexpr double kSigBg0 = 1e-3;
  constexpr double kSigBa0 = 5e-2;
  constexpr double kSigRot0 = 1e-4;
  constexpr double kSigTrans0 = 1e-3;

  // Filter: a fixed pose, two anchors under it, prior on X and on the biases.
  std::mt19937_64 rng(kSeed);
  std::normal_distribution<double> nd(0.0, 1.0);
  const Mat3 R0 = Renormalize(rtc::math::se3::exp3(Vec3(nd(rng), nd(rng), nd(rng)) * 0.5));
  const Vec3 v0(0.4, -0.2, 0.1);
  const Vec3 p0(0.3, -0.2, 0.9);
  std::array<Vec3, 4> d0 = ZeroContacts();
  d0[0] = p0 + R0 * Vec3(0.1, 0.1, -0.9);
  d0[1] = p0 + R0 * Vec3(0.1, -0.1, -0.9);
  ContactInekf f;
  f.Init(cfg, 2);
  Cov P0 = Cov::Zero(f.dim(), f.dim());
  for (int i = 0; i < 3; ++i) {
    P0(i, i) = kSigRot0 * kSigRot0;
  }
  for (int i = 3; i < 15; ++i) {
    P0(i, i) = kSigTrans0 * kSigTrans0;
  }
  for (int i = 15; i < 18; ++i) {
    P0(i, i) = kSigBg0 * kSigBg0;
  }
  for (int i = 18; i < 21; ++i) {
    P0(i, i) = kSigBa0 * kSigBa0;
  }
  Access::SetState(f, R0, v0, p0, d0, {true, true, false, false}, Vec3::Zero(), Vec3::Zero(), P0);

  std::vector<Imu> imu;
  std::map<int, FilterSnap> snap;
  const Vec3 g(0.0, 0.0, -kG0);
  for (int k = 0; k < kSnapshots.back(); ++k) {
    const double t = k * kDt;
    const Vec3 wm(std::sin(1.3 * t + 0.3), std::cos(0.9 * t), 0.6 * std::sin(2.1 * t + 1.0));
    const Vec3 ex(std::cos(1.1 * t), std::sin(0.7 * t), 0.5 * std::cos(1.9 * t));
    const Vec3 am = f.rotation().transpose() * (-g) + 3.0 * ex.normalized();
    imu.push_back({wm, am});
    ASSERT_EQ(f.Propagate(wm, am, kDt), InekfStatus::kOk);
    for (int K : kSnapshots) {
      if (k + 1 == K) {
        snap[K] = {f.rotation(),
                   f.velocity(),
                   f.position(),
                   {f.contact_position(0), f.contact_position(1)},
                   f.covariance()};
      }
    }
  }

  // Truth trajectories.
  const double h = kDt / kSub;
  const double sg = cfg.gyro_noise_density / std::sqrt(h);
  const double sa = cfg.accel_noise_density / std::sqrt(h);
  const double sd = cfg.contact_noise_density * std::sqrt(h);
  const double sbg = cfg.gyro_bias_walk * std::sqrt(h);
  const double sba = cfg.accel_bias_walk * std::sqrt(h);
  std::map<int, std::vector<VecD>> err;
  auto randn3 = [&] { return Vec3(nd(rng), nd(rng), nd(rng)); };
  for (int m = 0; m < kTrials; ++m) {
    Eigen::VectorXd xi0(15);
    xi0.head<3>() = randn3() * kSigRot0;
    for (int j = 0; j < 4; ++j) {
      xi0.segment<3>(3 + 3 * j) = randn3() * kSigTrans0;
    }
    Mat3 R = R0;
    std::vector<Vec3> cols{v0, p0, d0[0], d0[1]};
    ApplyExpLeft(xi0, R, cols);
    Vec3 v = cols[0];
    Vec3 p = cols[1];
    std::array<Vec3, 2> d{cols[2], cols[3]};
    Vec3 bg = randn3() * kSigBg0;
    Vec3 ba = randn3() * kSigBa0;
    for (int k = 0; k < kSnapshots.back(); ++k) {
      for (int s = 0; s < kSub; ++s) {
        const Vec3 w = imu[static_cast<std::size_t>(k)].gyro - bg - randn3() * sg;
        const Vec3 a = imu[static_cast<std::size_t>(k)].accel - ba - randn3() * sa;
        const Vec3 phi = w * h;
        const Vec3 a1 = rtc::math::se3::leftJacobian(phi) * a;
        const Vec3 a2 = Gamma2Small(phi) * a;
        p = p + v * h + R * a2 * (h * h) + 0.5 * g * (h * h);
        v = v + R * a1 * h + g * h;
        R = R * rtc::math::se3::exp3(phi);
        for (Vec3& x : d) {
          x += randn3() * sd;
        }
        bg += randn3() * sbg;
        ba += randn3() * sba;
      }
      R = Renormalize(R);
      for (int K : kSnapshots) {
        if (k + 1 == K) {
          const FilterSnap& fs = snap[K];
          VecD e;
          e.head<15>() = LogError(R, {v, p, d[0], d[1]}, fs.R, {fs.v, fs.p, fs.d[0], fs.d[1]});
          e.segment<3>(15) = bg;  // b̂ stays 0 without updates
          e.segment<3>(18) = ba;
          err[K].push_back(e);
        }
      }
    }
  }

  bool pass = true;
  for (int K : kSnapshots) {
    const Eigen::LLT<MatD> llt(snap[K].P);
    ASSERT_EQ(llt.info(), Eigen::Success);
    MatD C = MatD::Zero();
    for (const VecD& e : err[K]) {
      const VecD w = llt.matrixL().solve(e);
      C.noalias() += w * w.transpose();
    }
    C /= static_cast<double>(kTrials);
    C -= MatD::Identity();
    const double stat = Eigen::SelfAdjointEigenSolver<MatD>(C).eigenvalues().cwiseAbs().maxCoeff();
    std::printf("[measure] K = %3d: whitened %.3f (gate 0.25)\n", K, stat);
    pass = pass && stat <= 0.25;
  }
  EXPECT_TRUE(pass);
}

}  // namespace
}  // namespace rtc::estimation::testing
