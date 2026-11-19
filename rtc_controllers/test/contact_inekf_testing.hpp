// ── Shared fixture for the contact InEKF core tests (test-only) ──────────────
//
// The test seam (full state setter, last Φ / Q_d, the bare mean map), the
// common noise fixture, the propagation grid, SE_K(3) exp/log built on rtc_math
// (independent of the core's series), and bit-exact snapshots.
#pragma once

#include "rtc_controllers/estimation/contact_inekf.hpp"
#include "rtc_math/se3/so3.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace rtc::estimation {

struct ContactInekfTestAccess {
  using Cov = ContactInekf::CovMatrix;

  static void SetState(ContactInekf& f, const Eigen::Matrix3d& R, const Eigen::Vector3d& v,
                       const Eigen::Vector3d& p, const std::array<Eigen::Vector3d, 4>& d,
                       const std::array<bool, 4>& active, const Eigen::Vector3d& bg,
                       const Eigen::Vector3d& ba, const Cov& P) {
    f.R_ = R;
    f.v_ = v;
    f.p_ = p;
    f.d_ = d;
    f.active_ = active;
    f.bg_ = bg;
    f.ba_ = ba;
    f.P_ = P;
    f.initialized_ = true;
  }

  static void SetCovariance(ContactInekf& f, const Cov& P) { f.P_ = P; }

  static void SetActive(ContactInekf& f, int slot, bool on) {
    f.active_[static_cast<std::size_t>(slot)] = on;
  }

  static const Cov& Phi(const ContactInekf& f) { return f.phi_; }

  static const Cov& Qd(const ContactInekf& f) { return f.qd_; }

  /// H and K of the last computed stacked update (rows 3·used).
  static Eigen::MatrixXd H(const ContactInekf& f) { return f.h_; }

  static Eigen::MatrixXd Gain(const ContactInekf& f) { return f.gain_; }

  static void MeanMap(const Eigen::Matrix3d& R, const Eigen::Vector3d& v, const Eigen::Vector3d& p,
                      const Eigen::Vector3d& w, const Eigen::Vector3d& a, double dt, double g0,
                      Eigen::Matrix3d& R1, Eigen::Vector3d& v1, Eigen::Vector3d& p1) {
    const ContactInekf::Mean m = ContactInekf::PropagateMean(R, v, p, w, a, dt, g0);
    R1 = m.R;
    v1 = m.v;
    p1 = m.p;
  }
};

}  // namespace rtc::estimation

namespace rtc::estimation::testing {

using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;
using Cov = ContactInekf::CovMatrix;
using Access = ContactInekfTestAccess;

inline constexpr std::uint64_t kSeed = 20261119;
inline constexpr double kG0 = 9.81;

/// Common noise fixture (continuous densities).
inline ContactInekf::Config FixtureConfig() {
  ContactInekf::Config c;
  c.gravity = kG0;
  c.gyro_noise_density = 5e-4;
  c.accel_noise_density = 1e-2;
  c.gyro_bias_walk = 1e-5;
  c.accel_bias_walk = 1e-4;
  c.contact_noise_density = 1e-3;
  c.joint_position_sigma = 2e-3;
  c.kinematic_sigma = 2e-3;
  c.contact_gate_chi2 = 11.34;
  c.prior_velocity_sigma = 1e-2;
  c.prior_position_sigma = 1e-2;
  c.prior_yaw_sigma = 1e-3;
  c.prior_gyro_bias_sigma = 1e-3;
  c.prior_accel_bias_sigma = 5e-2;
  c.stationary_accel_tolerance = 0.5;
  c.stationary_gyro_tolerance = 0.05;
  return c;
}

/// Eigen vectors are not zeroed by value-initialisation of a std::array.
inline std::array<Vec3, 4> ZeroContacts() {
  std::array<Vec3, 4> d;
  for (Vec3& x : d) {
    x.setZero();
  }
  return d;
}

inline Mat3 Renormalize(const Mat3& R) {
  return 0.5 * (R * (3.0 * Mat3::Identity() - R.transpose() * R));
}

inline Mat3 RotZ(double psi) {
  return rtc::math::se3::RpyToRotationZyx(Vec3(0.0, 0.0, psi));
}

/// Active rows of the full layout, in packed order.
inline std::vector<int> ActiveIndex(const ContactInekf& f) {
  std::vector<int> idx;
  for (int i = 0; i < 9; ++i) {
    idx.push_back(i);
  }
  for (int s = 0; s < f.num_slots(); ++s) {
    if (f.contact_active(s)) {
      for (int i = 0; i < 3; ++i) {
        idx.push_back(ContactInekf::SlotOffset(s) + i);
      }
    }
  }
  for (int i = 0; i < 6; ++i) {
    idx.push_back(f.BiasOffset() + i);
  }
  return idx;
}

inline Eigen::MatrixXd Gather(const Cov& M, const std::vector<int>& idx) {
  const auto n = static_cast<Eigen::Index>(idx.size());
  Eigen::MatrixXd out(n, n);
  for (Eigen::Index c = 0; c < n; ++c) {
    for (Eigen::Index r = 0; r < n; ++r) {
      out(r, c) = M(idx[static_cast<std::size_t>(r)], idx[static_cast<std::size_t>(c)]);
    }
  }
  return out;
}

// ── SE_K(3) on rtc_math (independent of the core's series) ────────────────────

/// X ← Exp(ξ)·X on (R, cols): ξ = [φ, ξ_c…].
inline void ApplyExpLeft(const Eigen::VectorXd& xi, Mat3& R, std::vector<Vec3>& cols) {
  const Vec3 phi = xi.head<3>();
  const Mat3 dR = rtc::math::se3::exp3(phi);
  const Mat3 J = rtc::math::se3::leftJacobian(phi);
  R = dR * R;
  for (std::size_t j = 0; j < cols.size(); ++j) {
    cols[j] = dR * cols[j] + J * xi.segment<3>(3 + 3 * static_cast<Eigen::Index>(j));
  }
}

/// ξ with X_t = Exp(ξ)·X_h.
inline Eigen::VectorXd LogError(const Mat3& Rt, const std::vector<Vec3>& ct, const Mat3& Rh,
                                const std::vector<Vec3>& ch) {
  const Mat3 Re = Rt * Rh.transpose();
  const Vec3 phi = rtc::math::se3::log3(Re);
  const Mat3 Ji = rtc::math::se3::leftJacobianInverse(phi);
  Eigen::VectorXd out(3 + 3 * static_cast<Eigen::Index>(ct.size()));
  out.head<3>() = phi;
  for (std::size_t j = 0; j < ct.size(); ++j) {
    out.segment<3>(3 + 3 * static_cast<Eigen::Index>(j)) = Ji * (ct[j] - Re * ch[j]);
  }
  return out;
}

// ── Propagation grid ──────────────────────────────────────────────────────────

inline constexpr std::array<double, 5> kGridDt{2e-4, 1e-3, 2e-3, 1e-2, 2e-2};
inline constexpr std::array<double, 8> kGridTheta{0.0,  1e-9, 1e-6, 1e-4,
                                                  1e-2, 0.1,  0.35, 0.7 * (1.0 - 1e-12)};
inline constexpr std::array<int, 8> kGridActive{0, 1, 2, 3, 4, 4, 2, 1};
inline constexpr std::array<double, 8> kGridScale{0.5, 3.0, 7.0, 10.0, 1.0, 10.0, 5.0, 10.0};

struct GridState {
  Mat3 R;
  Vec3 v;
  Vec3 p;
  std::array<Vec3, 4> d = ZeroContacts();
  std::array<bool, 4> active{};
  Vec3 bg;
  Vec3 ba;
  Vec3 specific_force;  // true, bias-free
  Vec3 wdir;
};

inline std::vector<GridState> GridStates() {
  std::mt19937_64 rng(kSeed);
  std::normal_distribution<double> nd(0.0, 1.0);
  std::uniform_real_distribution<double> ud(0.0, 1.0);
  auto unit = [&] {
    const Vec3 x(nd(rng), nd(rng), nd(rng));
    return Vec3(x.normalized());
  };
  std::vector<GridState> out;
  for (std::size_t k = 0; k < kGridActive.size(); ++k) {
    GridState s;
    s.R = Renormalize(rtc::math::se3::exp3(unit() * (3.0 * ud(rng))));
    const double sc = kGridScale[k];
    s.v = unit() * sc;
    s.p = unit() * sc;
    for (int i = 0; i < 4; ++i) {
      const auto si = static_cast<std::size_t>(i);
      s.active[si] = i < kGridActive[k];
      s.d[si] = s.active[si] ? Vec3(unit() * sc) : Vec3::Zero();
    }
    s.bg = Vec3(nd(rng), nd(rng), nd(rng)) * 1e-3;
    s.ba = Vec3(nd(rng), nd(rng), nd(rng)) * 5e-2;
    s.specific_force = s.R.transpose() * Vec3(0.0, 0.0, kG0) + unit() * (10.0 * ud(rng));
    s.wdir = unit();
    out.push_back(s);
  }
  return out;
}

/// A filter (4 slots) holding `s` with P = diag(1e-4) on the active rows.
inline void LoadGridState(ContactInekf& f, const GridState& s) {
  f.Init(FixtureConfig(), 4);
  Cov P = Cov::Zero(f.dim(), f.dim());
  ContactInekfTestAccess::SetState(f, s.R, s.v, s.p, s.d, s.active, s.bg, s.ba, P);
  for (int i : ActiveIndex(f)) {
    P(i, i) = 1e-4;
  }
  ContactInekfTestAccess::SetCovariance(f, P);
}

// ── Bit-exact snapshot of everything the accessors expose ─────────────────────

struct Snapshot {
  std::vector<double> values;
  std::vector<unsigned char> flags;
};

inline void Push(std::vector<double>& v, const Eigen::Ref<const Eigen::MatrixXd>& m) {
  for (Eigen::Index c = 0; c < m.cols(); ++c) {
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
      v.push_back(m(r, c));
    }
  }
}

inline Snapshot Capture(const ContactInekf& f) {
  Snapshot s;
  Push(s.values, f.rotation());
  Push(s.values, f.velocity());
  Push(s.values, f.position());
  Push(s.values, f.gyro_bias());
  Push(s.values, f.accel_bias());
  Push(s.values, f.angular_velocity());
  for (int i = 0; i < kInekfMaxContacts; ++i) {
    Push(s.values, f.contact_position(i));
    s.values.push_back(f.contact_noise_scale(i));
    s.flags.push_back(static_cast<unsigned char>(f.contact_active(i)));
  }
  Push(s.values, f.covariance());
  s.flags.push_back(static_cast<unsigned char>(f.initialized()));
  s.flags.push_back(static_cast<unsigned char>(f.configured()));
  return s;
}

inline bool BitEqual(const Snapshot& a, const Snapshot& b) {
  return a.values.size() == b.values.size() && a.flags == b.flags &&
         std::memcmp(a.values.data(), b.values.data(), a.values.size() * sizeof(double)) == 0;
}

}  // namespace rtc::estimation::testing

namespace rtc::estimation::testing {

/// Contact kinematics with an optional Jacobian (zero columns by default).
inline ContactKinematics Kin(const Vec3& r, bool use = true,
                             const Eigen::MatrixXd& J = Eigen::MatrixXd(3, 0)) {
  ContactKinematics k;
  k.position = r;
  k.jacobian = J;
  k.use = use;
  return k;
}

/// Σ = σ_q² J Jᵀ + σ_kin² I, computed here independently of the core.
inline Mat3 BodyNoise(const ContactKinematics& k, const ContactInekf::Config& c) {
  const Eigen::MatrixXd J = k.jacobian;
  Mat3 S = c.joint_position_sigma * c.joint_position_sigma * (J * J.transpose());
  S += c.kinematic_sigma * c.kinematic_sigma * Mat3::Identity();
  return S;
}

/// Ad of the world gauge G = (Rz, t) on the full layout (every slot moved by t).
inline Cov GaugeAdjoint(const ContactInekf& f, const Mat3& Rz, const Vec3& t) {
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

/// Each active slot's body-frame point as the filter predicts it, requested.
inline std::vector<ContactKinematics> PredictedKinematics(const ContactInekf& f) {
  std::vector<ContactKinematics> k(static_cast<std::size_t>(f.num_slots()));
  for (int i = 0; i < f.num_slots(); ++i) {
    if (f.contact_active(i)) {
      k[static_cast<std::size_t>(i)] =
          Kin(f.rotation().transpose() * (f.contact_position(i) - f.position()));
    }
  }
  return k;
}

/// Nominal foot point of slot j in the IMU frame.
inline Vec3 FootPoint(int j) {
  return Vec3(0.1 * ((j % 2) == 0 ? 1.0 : -1.0), 0.1 * (j - 1.5), -0.7);
}

/// Initialized 4-slot filter, moved to |p| = scale, |v| = min(scale, 2), with
/// `active` slots anchored through InsertContact (P built only by operations).
inline void StartFilter(ContactInekf& f, std::uint64_t seed, int active, double scale = 10.0) {
  std::mt19937_64 rng(seed + 7);
  std::normal_distribution<double> nd(0.0, 1.0);
  std::uniform_real_distribution<double> ud(-3.0, 3.0);
  f.Init(FixtureConfig(), 4);
  const Vec3 a(0.3, -0.2, 9.8);
  if (f.Initialize(Vec3::Zero(), a, 2.0, ud(rng), {}) != InekfStatus::kOk) {
    return;
  }
  const Vec3 p = Vec3(nd(rng), nd(rng), nd(rng)).normalized() * scale;
  const Vec3 v = Vec3(nd(rng), nd(rng), nd(rng)).normalized() * std::min(scale, 2.0);
  ContactInekfTestAccess::SetState(f, f.rotation(), v, p, ZeroContacts(), {}, f.gyro_bias(),
                                   f.accel_bias(), f.covariance());
  for (int j = 0; j < active; ++j) {
    if (f.InsertContact(j, Kin(FootPoint(j))) != InekfStatus::kOk) {
      return;
    }
  }
}

}  // namespace rtc::estimation::testing
