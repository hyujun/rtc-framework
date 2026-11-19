#include "rtc_controllers/estimation/contact_inekf.hpp"

#include "rtc_math/se3/so3.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc::estimation {

namespace {

using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;
using DiagVector = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, kInekfMaxDim, 1>;

// Terms of the Γ coefficient series. At θ² ≤ 0.49 the first dropped term is
// below 0.49¹²/24! ≈ 3e-28 — far under one ulp of the leading 1/(m+1)!.
constexpr int kSeriesTerms = 12;
constexpr int kMaxFactorial = 2 * (kSeriesTerms - 1) + 4;

constexpr std::array<double, kMaxFactorial + 1> MakeInverseFactorials() {
  std::array<double, kMaxFactorial + 1> f{};
  double v = 1.0;
  f[0] = 1.0;
  for (int k = 1; k <= kMaxFactorial; ++k) {
    v /= static_cast<double>(k);
    f[static_cast<std::size_t>(k)] = v;
  }
  return f;
}

constexpr std::array<double, kMaxFactorial + 1> kInverseFactorial = MakeInverseFactorials();

// Σ_j (−θ²)ʲ / (2j + start)!  by Horner.
double CoefficientSeries(double theta2, int start) noexcept {
  double acc = kInverseFactorial[static_cast<std::size_t>(2 * (kSeriesTerms - 1) + start)];
  for (int j = kSeriesTerms - 2; j >= 0; --j) {
    acc = kInverseFactorial[static_cast<std::size_t>(2 * j + start)] - theta2 * acc;
  }
  return acc;
}

// 5-point Gauss–Legendre on [0, 1].
constexpr std::array<double, 5> kQuadNode{
    0.046910077030668003601186560850303517, 0.230765344947158454481842789649895598, 0.5,
    0.769234655052841545518157210350104402, 0.953089922969331996398813439149696483};
constexpr std::array<double, 5> kQuadWeight{
    0.118463442528094543757132020359958681, 0.239314335249683234020645757417819096,
    0.284444444444444444444444444444444444, 0.239314335249683234020645757417819096,
    0.118463442528094543757132020359958681};

// Smallest accepted cos(pitch) = √(ā_y² + ā_z²)/‖ā‖ (|pitch| ≤ 84.3°). The
// alignment Jacobian grows as 1/cos(pitch) and the prior's only margin along
// one attitude direction is σ_yaw² cos²(pitch), so closer to gravity along the
// IMU x axis the ZYX heading is not a usable initial condition.
constexpr double kAlignmentGuard = 0.1;

// x and x² both finite and normal: a σ whose square underflows or overflows
// would be accepted here and refused on every later tick.
bool UsableScale(double x) noexcept {
  const double x2 = x * x;
  return std::isfinite(x2) && x > 0.0 && x2 >= std::numeric_limits<double>::min();
}

void NeumaierAdd(double& sum, double& comp, double x) noexcept {
  const double t = sum + x;
  if (std::abs(sum) >= std::abs(x)) {
    comp += (sum - t) + x;
  } else {
    comp += (x - t) + sum;
  }
  sum = t;
}

}  // namespace

template <int M>
Eigen::Matrix3d GammaSeries(const Eigen::Vector3d& phi) noexcept {
  static_assert(M >= 0 && M <= 2);
  const double theta2 = phi.squaredNorm();
  const Mat3 W = rtc::math::se3::hat(phi);
  const double c1 = CoefficientSeries(theta2, M + 1);
  const double c2 = CoefficientSeries(theta2, M + 2);
  Mat3 out = (c2 * W) * W;
  out.noalias() += c1 * W;
  out.diagonal().array() += kInverseFactorial[static_cast<std::size_t>(M)];
  return out;
}

template Eigen::Matrix3d GammaSeries<0>(const Eigen::Vector3d& phi) noexcept;
template Eigen::Matrix3d GammaSeries<1>(const Eigen::Vector3d& phi) noexcept;
template Eigen::Matrix3d GammaSeries<2>(const Eigen::Vector3d& phi) noexcept;

// ── StationaryImuAccumulator ──────────────────────────────────────────────────

bool StationaryImuAccumulator::Add(const Eigen::Vector3d& gyro,
                                   const Eigen::Vector3d& accel) noexcept {
  if (!gyro.allFinite() || !accel.allFinite()) {
    return false;
  }
  for (int k = 0; k < 3; ++k) {
    const auto i = static_cast<std::size_t>(k);
    NeumaierAdd(sum_[i], comp_[i], gyro[k]);
    NeumaierAdd(sum_[i + 3], comp_[i + 3], accel[k]);
  }
  ++count_;
  return true;
}

void StationaryImuAccumulator::Reset() noexcept {
  sum_.fill(0.0);
  comp_.fill(0.0);
  count_ = 0;
}

Eigen::Vector3d StationaryImuAccumulator::MeanAt(std::size_t offset) const noexcept {
  Vec3 out = Vec3::Zero();
  if (count_ > 0) {
    const double n = static_cast<double>(count_);
    for (std::size_t k = 0; k < 3; ++k) {
      out[static_cast<Eigen::Index>(k)] = (sum_[offset + k] + comp_[offset + k]) / n;
    }
  }
  return out;
}

// ── ContactInekf: configuration ───────────────────────────────────────────────

void ContactInekf::Init(const Config& config, int num_slots) {
  const struct {
    const char* name;
    double value;
  } fields[] = {
      {"gravity", config.gravity},
      {"gyro_noise_density", config.gyro_noise_density},
      {"accel_noise_density", config.accel_noise_density},
      {"gyro_bias_walk", config.gyro_bias_walk},
      {"accel_bias_walk", config.accel_bias_walk},
      {"contact_noise_density", config.contact_noise_density},
      {"joint_position_sigma", config.joint_position_sigma},
      {"kinematic_sigma", config.kinematic_sigma},
      {"contact_gate_chi2", config.contact_gate_chi2},
      {"prior_velocity_sigma", config.prior_velocity_sigma},
      {"prior_position_sigma", config.prior_position_sigma},
      {"prior_yaw_sigma", config.prior_yaw_sigma},
      {"prior_gyro_bias_sigma", config.prior_gyro_bias_sigma},
      {"prior_accel_bias_sigma", config.prior_accel_bias_sigma},
      {"stationary_accel_tolerance", config.stationary_accel_tolerance},
      {"stationary_gyro_tolerance", config.stationary_gyro_tolerance},
  };

  for (const auto& f : fields) {
    if (!UsableScale(f.value)) {
      throw std::invalid_argument(std::string("ContactInekf::Init: ") + f.name +
                                  " must be > 0 with a finite, normal square");
    }
  }
  if (num_slots < 0 || num_slots > kInekfMaxContacts) {
    throw std::invalid_argument("ContactInekf::Init: num_slots must be in [0, " +
                                std::to_string(kInekfMaxContacts) + "]");
  }
  config_ = config;
  num_slots_ = num_slots;
  const int n = dim();
  phi_.setZero(n, n);
  qd_.setZero(n, n);
  ad_.setZero(n, n);
  left_.setZero(n, n);
  work_.setZero(n, n);
  next_.setZero(n, n);
  packed_.setZero(n, n);
  Reset();
}

void ContactInekf::Reset() noexcept {
  initialized_ = false;
  R_.setIdentity();
  v_.setZero();
  p_.setZero();
  bg_.setZero();
  ba_.setZero();
  omega_.setZero();
  for (auto& d : d_) {
    d.setZero();
  }
  active_.fill(false);
  noise_scale_.fill(1.0);
  P_.setZero(dim(), dim());
}

// ── ContactInekf: initialisation ──────────────────────────────────────────────

InekfStatus ContactInekf::Initialize(const Eigen::Vector3d& mean_gyro,
                                     const Eigen::Vector3d& mean_accel, double window_duration,
                                     double yaw,
                                     std::span<const Eigen::Vector3d> stance_points) noexcept {
  if (!configured()) {
    return InekfStatus::kNotConfigured;
  }
  if (initialized_) {
    return InekfStatus::kAlreadyInitialized;
  }
  if (!mean_gyro.allFinite() || !mean_accel.allFinite() || !std::isfinite(window_duration) ||
      !std::isfinite(yaw)) {
    return InekfStatus::kNonFiniteInput;
  }
  for (const Vec3& r : stance_points) {
    if (!r.allFinite()) {
      return InekfStatus::kNonFiniteInput;
    }
  }
  if (!(window_duration > 0.0)) {
    return InekfStatus::kBadWindow;
  }
  if (stance_points.size() > static_cast<std::size_t>(num_slots_)) {
    return InekfStatus::kTooManyStancePoints;
  }
  const double norm_a = mean_accel.norm();
  if (!(std::abs(norm_a - config_.gravity) <= config_.stationary_accel_tolerance)) {
    return InekfStatus::kAccelNotStationary;
  }
  if (!(mean_gyro.norm() <= config_.stationary_gyro_tolerance)) {
    return InekfStatus::kGyroNotStationary;
  }
  const double ax = mean_accel.x();
  const double ay = mean_accel.y();
  const double az = mean_accel.z();
  const double h = std::hypot(ay, az);
  if (!(h > kAlignmentGuard * norm_a)) {
    return InekfStatus::kAlignmentSingular;
  }

  const double roll = std::atan2(ay, az);
  const double pitch = std::atan2(-ax, h);
  const Mat3 R0 = rtc::math::se3::RpyToRotationZyx(Vec3(roll, pitch, yaw));

  // Alignment Jacobian M (preamble): ξ_φ ≈ M (b_a + n̄).
  const Vec3 yaw_y(-std::sin(yaw), std::cos(yaw), 0.0);  // Rz e_y
  const Vec3 roll_axis = R0.col(0);                      // Rz Ry e_x
  const double h2 = h * h;
  const double n2 = norm_a * norm_a;
  const Vec3 d_roll(0.0, az / h2, -ay / h2);
  const Vec3 d_pitch(-h / n2, ax * ay / (h * n2), ax * az / (h * n2));
  Mat3 M = yaw_y * d_pitch.transpose();
  M.noalias() += roll_axis * d_roll.transpose();
  M = -M;

  const double var_ba = config_.prior_accel_bias_sigma * config_.prior_accel_bias_sigma;
  const double var_mean =
      config_.accel_noise_density * config_.accel_noise_density / window_duration;
  const int D = dim();
  const int bg = BiasOffset();
  const int ba = bg + 3;
  work_.setZero(D, D);
  Mat3 att = (var_ba + var_mean) * (M * M.transpose());
  att(2, 2) += config_.prior_yaw_sigma * config_.prior_yaw_sigma;
  work_.block<3, 3>(0, 0) = att;
  const Mat3 cross = var_ba * M;
  work_.block<3, 3>(0, ba) = cross;
  work_.block<3, 3>(ba, 0) = cross.transpose();
  work_.block<3, 3>(3, 3).diagonal().setConstant(config_.prior_velocity_sigma *
                                                 config_.prior_velocity_sigma);
  work_.block<3, 3>(6, 6).diagonal().setConstant(config_.prior_position_sigma *
                                                 config_.prior_position_sigma);
  work_.block<3, 3>(bg, bg).diagonal().setConstant(config_.prior_gyro_bias_sigma *
                                                   config_.prior_gyro_bias_sigma);
  work_.block<3, 3>(ba, ba).diagonal().setConstant(var_ba);
  Tidy(work_);
  if (!work_.allFinite()) {
    return InekfStatus::kNonFiniteResult;
  }
  if (!ActiveBlockPositiveDefinite(work_)) {
    return InekfStatus::kNotPositiveDefinite;
  }

  Vec3 origin = Vec3::Zero();
  if (!stance_points.empty()) {
    Vec3 sum = Vec3::Zero();
    for (const Vec3& r : stance_points) {
      sum += r;
    }
    origin = -(R0 * (sum / static_cast<double>(stance_points.size())));
  }
  if (!origin.allFinite()) {
    return InekfStatus::kNonFiniteResult;
  }

  R_ = R0;
  v_.setZero();
  p_ = origin;
  bg_.setZero();
  ba_.setZero();
  omega_ = mean_gyro;
  P_ = work_;
  initialized_ = true;
  return InekfStatus::kOk;
}

// ── ContactInekf: propagation ─────────────────────────────────────────────────

ContactInekf::Mean ContactInekf::PropagateMean(const Eigen::Matrix3d& R, const Eigen::Vector3d& v,
                                               const Eigen::Vector3d& p, const Eigen::Vector3d& w,
                                               const Eigen::Vector3d& a, double dt,
                                               double gravity) noexcept {
  const Vec3 phi = w * dt;
  const Mat3 G0 = GammaSeries<0>(phi);
  const Mat3 G1 = GammaSeries<1>(phi);
  const Mat3 G2 = GammaSeries<2>(phi);
  const Vec3 g(0.0, 0.0, -gravity);
  const Vec3 a1 = G1 * a;
  const Vec3 a2 = G2 * a;

  Mean out;
  const Mat3 Rn = R * G0;
  const Mat3 gram = Rn.transpose() * Rn;
  out.R = 0.5 * (Rn * (3.0 * Mat3::Identity() - gram));
  out.v = v + (R * a1) * dt + g * dt;
  out.p = p + v * dt + (R * a2) * (dt * dt) + (0.5 * (dt * dt)) * g;
  return out;
}

void ContactInekf::LeftTransition(const Eigen::Vector3d& w, const Eigen::Vector3d& a, double u,
                                  CovMatrix& out) const noexcept {
  const int D = dim();
  const int bg = BiasOffset();
  const int ba = bg + 3;
  const Vec3 wu = w * u;
  const Mat3 G0u = GammaSeries<0>(wu);
  const Mat3 G1u = GammaSeries<1>(wu);
  const Mat3 G2u = GammaSeries<2>(wu);
  const Mat3 E = G0u.transpose();
  const Vec3 a1 = u * (G1u * a);
  const Vec3 a2 = (u * u) * (G2u * a);

  // ∫₀ᵘ f(s) ds and ∫₀ᵘ (u − s) f(s) ds, f(s) = [Γ₀(ω̄s) ā]× s Γ₁(ω̄s).
  Mat3 iv = Mat3::Zero();
  Mat3 ip = Mat3::Zero();
  for (std::size_t q = 0; q < kQuadNode.size(); ++q) {
    const double s = kQuadNode[q] * u;
    const Vec3 ws = w * s;
    const Vec3 rs_a = GammaSeries<0>(ws) * a;
    const Mat3 f = rtc::math::se3::hat(rs_a) * (s * GammaSeries<1>(ws));
    iv.noalias() += kQuadWeight[q] * f;
    ip.noalias() += (kQuadWeight[q] * (u - s)) * f;
  }
  iv *= u;
  ip *= u;

  const Mat3 bias_rot = -u * G1u.transpose();
  out.setZero(D, D);
  out.block<3, 3>(0, 0) = E;
  out.block<3, 3>(0, bg) = bias_rot;
  out.block<3, 3>(3, 0) = -(E * rtc::math::se3::hat(a1));
  out.block<3, 3>(3, 3) = E;
  out.block<3, 3>(3, bg) = E * iv;
  out.block<3, 3>(3, ba) = bias_rot;
  out.block<3, 3>(6, 0) = -(E * rtc::math::se3::hat(a2));
  out.block<3, 3>(6, 3) = u * E;
  out.block<3, 3>(6, 6) = E;
  out.block<3, 3>(6, bg) = E * ip;
  out.block<3, 3>(6, ba) = -(u * u) * (E * G2u);
  for (int i = 0; i < num_slots_; ++i) {
    out.block<3, 3>(SlotOffset(i), SlotOffset(i)) = E;
  }
  out.block<6, 6>(bg, bg).setIdentity();
}

void ContactInekf::Adjoint(const Eigen::Matrix3d& R, const Eigen::Vector3d& v,
                           const Eigen::Vector3d& p, CovMatrix& out) const noexcept {
  const int D = dim();
  const int bg = BiasOffset();
  out.setZero(D, D);
  out.block<3, 3>(0, 0) = R;
  out.block<3, 3>(3, 0) = rtc::math::se3::hat(v) * R;
  out.block<3, 3>(3, 3) = R;
  out.block<3, 3>(6, 0) = rtc::math::se3::hat(p) * R;
  out.block<3, 3>(6, 6) = R;
  for (int i = 0; i < num_slots_; ++i) {
    const int o = SlotOffset(i);
    out.block<3, 3>(o, 0) = rtc::math::se3::hat(d_[static_cast<std::size_t>(i)]) * R;
    out.block<3, 3>(o, o) = R;
  }
  out.block<6, 6>(bg, bg).setIdentity();
}

InekfStatus ContactInekf::Propagate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel,
                                    double dt) noexcept {
  if (!configured()) {
    return InekfStatus::kNotConfigured;
  }
  if (!initialized_) {
    return InekfStatus::kNotInitialized;
  }
  if (!gyro.allFinite() || !accel.allFinite() || !std::isfinite(dt)) {
    return InekfStatus::kNonFiniteInput;
  }
  if (!(dt >= kMinPropagationStep && dt <= kMaxPropagationStep)) {
    return InekfStatus::kStepOutOfRange;
  }
  const Vec3 w = gyro - bg_;
  const Vec3 a = accel - ba_;
  if (!(w.norm() * dt <= kMaxRotationStep)) {
    return InekfStatus::kRotationOutOfRange;
  }
  // A non-finite mean needs no test of its own: it reaches Q_d through
  // Ad(X̂⁺), so the finiteness test on P below refuses it.
  const Mean m = PropagateMean(R_, v_, p_, w, a, dt, config_.gravity);

  const int D = dim();
  const int bg = BiasOffset();

  // Q_c (diagonal).
  DiagVector qc = DiagVector::Zero(D);
  qc.segment<3>(0).setConstant(config_.gyro_noise_density * config_.gyro_noise_density);
  qc.segment<3>(3).setConstant(config_.accel_noise_density * config_.accel_noise_density);
  for (int i = 0; i < num_slots_; ++i) {
    const auto si = static_cast<std::size_t>(i);
    if (active_[si]) {
      const double sd = noise_scale_[si] * config_.contact_noise_density;
      qc.segment<3>(SlotOffset(i)).setConstant(sd * sd);
    }
  }
  qc.segment<3>(bg).setConstant(config_.gyro_bias_walk * config_.gyro_bias_walk);
  qc.segment<3>(bg + 3).setConstant(config_.accel_bias_walk * config_.accel_bias_walk);

  // Q_d = Ad(X̂⁺) [Σ_j w_j Δt Φ_L(u_j) Q_c Φ_L(u_j)ᵀ] Ad(X̂⁺)ᵀ.
  work_.setZero(D, D);
  for (std::size_t j = 0; j < kQuadNode.size(); ++j) {
    LeftTransition(w, a, kQuadNode[j] * dt, left_);
    next_.noalias() = left_ * qc.asDiagonal();
    work_.noalias() += (kQuadWeight[j] * dt) * (next_ * left_.transpose());
  }
  Adjoint(m.R, m.v, m.p, ad_);
  next_.noalias() = ad_ * work_;
  qd_.noalias() = next_ * ad_.transpose();

  // Φ: closed state–state part, Ad(X̂⁺) Φ_L(Δt) on the bias columns.
  LeftTransition(w, a, dt, left_);
  const Mat3 g_hat = rtc::math::se3::hat(Vec3(0.0, 0.0, -config_.gravity));
  phi_.setIdentity(D, D);
  phi_.block<3, 3>(3, 0) = g_hat * dt;
  phi_.block<3, 3>(6, 0) = (0.5 * (dt * dt)) * g_hat;
  phi_.block<3, 3>(6, 3) = dt * Mat3::Identity();
  phi_.middleCols(bg, 6).noalias() = ad_ * left_.middleCols(bg, 6);

  next_.noalias() = phi_ * P_;
  work_.noalias() = next_ * phi_.transpose();
  work_ += qd_;
  Tidy(work_);
  if (!work_.allFinite()) {
    return InekfStatus::kNonFiniteResult;
  }
  if (!ActiveBlockPositiveDefinite(work_)) {
    return InekfStatus::kNotPositiveDefinite;
  }

  R_ = m.R;
  v_ = m.v;
  p_ = m.p;
  omega_ = w;
  P_ = work_;
  return InekfStatus::kOk;
}

InekfStatus ContactInekf::SetContactNoiseScale(int slot, double scale) noexcept {
  if (!configured()) {
    return InekfStatus::kNotConfigured;
  }
  if (!SlotInRange(slot)) {
    return InekfStatus::kBadSlot;
  }
  const double sd = scale * config_.contact_noise_density;
  if (!std::isfinite(scale) || !(scale >= 1.0) || !std::isfinite(sd * sd)) {
    return InekfStatus::kBadNoiseScale;
  }
  noise_scale_[static_cast<std::size_t>(slot)] = scale;
  return InekfStatus::kOk;
}

// ── ContactInekf: covariance hygiene ──────────────────────────────────────────

void ContactInekf::Tidy(CovMatrix& P) const noexcept {
  const int D = dim();
  for (int i = 0; i < num_slots_; ++i) {
    if (!active_[static_cast<std::size_t>(i)]) {
      P.middleRows(SlotOffset(i), 3).setZero();
      P.middleCols(SlotOffset(i), 3).setZero();
    }
  }
  for (int c = 1; c < D; ++c) {
    for (int r = 0; r < c; ++r) {
      const double mean = 0.5 * (P(r, c) + P(c, r));
      P(r, c) = mean;
      P(c, r) = mean;
    }
  }
}

bool ContactInekf::ActiveBlockPositiveDefinite(const CovMatrix& P) noexcept {
  std::array<int, kInekfMaxDim> index{};
  int k = 0;
  for (int i = 0; i < 9; ++i) {
    index[static_cast<std::size_t>(k++)] = i;
  }
  for (int s = 0; s < num_slots_; ++s) {
    if (active_[static_cast<std::size_t>(s)]) {
      for (int i = 0; i < 3; ++i) {
        index[static_cast<std::size_t>(k++)] = SlotOffset(s) + i;
      }
    }
  }
  for (int i = 0; i < 6; ++i) {
    index[static_cast<std::size_t>(k++)] = BiasOffset() + i;
  }
  packed_.resize(k, k);
  for (int c = 0; c < k; ++c) {
    for (int r = 0; r < k; ++r) {
      packed_(r, c) = P(index[static_cast<std::size_t>(r)], index[static_cast<std::size_t>(c)]);
    }
  }
  llt_.compute(packed_);
  return llt_.info() == Eigen::Success;
}

// ── ContactInekf: accessors ───────────────────────────────────────────────────

Eigen::Quaterniond ContactInekf::orientation() const noexcept {
  Eigen::Quaterniond q(R_);
  q.normalize();
  return q;
}

const Eigen::Vector3d& ContactInekf::contact_position(int slot) const noexcept {
  if (!SlotInRange(slot)) {
    return d_[0];
  }
  return d_[static_cast<std::size_t>(slot)];
}

bool ContactInekf::contact_active(int slot) const noexcept {
  if (!SlotInRange(slot)) {
    return false;
  }
  return active_[static_cast<std::size_t>(slot)];
}

double ContactInekf::contact_noise_scale(int slot) const noexcept {
  if (!SlotInRange(slot)) {
    return 1.0;
  }
  return noise_scale_[static_cast<std::size_t>(slot)];
}

BaseState ContactInekf::BaseFromImu(const Eigen::Matrix3d& R_WI, const Eigen::Vector3d& p_WI,
                                    const Eigen::Vector3d& v_WI, const Eigen::Vector3d& omega_I,
                                    const Eigen::Matrix3d& R_BI,
                                    const Eigen::Vector3d& p_BI) noexcept {
  BaseState out;
  out.R_WB = R_WI * R_BI.transpose();
  out.p_WB = p_WI - out.R_WB * p_BI;
  const Vec3 p_IB = -(R_BI.transpose() * p_BI);
  out.v_WB = v_WI + R_WI * omega_I.cross(p_IB);
  return out;
}

}  // namespace rtc::estimation
