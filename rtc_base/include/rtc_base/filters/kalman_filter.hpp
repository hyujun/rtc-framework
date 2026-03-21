#ifndef RTC_BASE_FILTERS_KALMAN_FILTER_HPP_
#define RTC_BASE_FILTERS_KALMAN_FILTER_HPP_

// Discrete-time Kalman filter for N independent scalar channels.
//
// Motion model: constant-velocity (kinematic integration)
//   state xₖ = [position, velocity]ᵀ  (2×1 per channel)
//
// Transition matrix F (dt = sample period):
//   F = | 1  dt |
//       | 0   1 |
//
// Measurement model: position-only observation
//   H = [1  0]
//
// Process noise covariance Q (diagonal):
//   Q = | q_pos    0    |
//       |    0   q_vel  |
//
// Measurement noise covariance R (scalar per channel):
//   R = r
//
// Why this model for robot joint control?
//   The UR5e encoder provides noisy position readings.  Integrating a
//   constant-velocity kinematic model gives a filtered position estimate and
//   a clean velocity estimate, both without differentiation artifacts.
//   The Kalman gain adapts automatically to the Q/R ratio:
//     • Large r (noisy sensor) → trust model more → smoother output
//     • Large q_vel            → trust sensor more → faster tracking
//
// Prediction step (at every control tick):
//   x̂⁻ₖ = F · x̂ₖ₋₁
//   P⁻ₖ  = F · Pₖ₋₁ · Fᵀ + Q
//
// Update step (when a new measurement z arrives):
//   S  = H · P⁻ₖ · Hᵀ + R          (innovation covariance, scalar)
//   K  = P⁻ₖ · Hᵀ / S              (Kalman gain, 2×1)
//   x̂ₖ = x̂⁻ₖ + K · (z − H · x̂⁻ₖ)
//   Pₖ = (I − K·H) · P⁻ₖ           (Joseph form not needed for scalar H)
//
// RT safety:
//   - Init() performs parameter validation and may throw.
//   - Predict() / Update() / UpdateScalar() are noexcept — safe on the
//     500 Hz RT path.  No heap allocation after Init().
//
// Template parameter N: number of independent channels.
//
// Usage example:
//   KalmanFilterN<6> kf;
//   kf.Init(0.001, 0.01, 0.1, 0.002);   // q_pos, q_vel, r, dt=2ms
//
//   // Inside the 500 Hz ControlLoop():
//   kf.Predict();                        // propagate model (always call)
//   std::array<double,6> pos = kf.Update(raw_positions);   // fuse measurement
//
//   // Or in a single call (Predict + Update combined):
//   std::array<double,6> pos = kf.PredictAndUpdate(raw_positions);
//
//   // Read filtered velocity estimate:
//   double vel_j0 = kf.velocity(0);

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace rtc {

template <std::size_t N>
class KalmanFilterN {
 public:
  // ── Parameters ────────────────────────────────────────────────────────────

  struct Params {
    double q_pos{1e-3};   // process noise — position  [unit²]
    double q_vel{1e-2};   // process noise — velocity  [(unit/s)²]
    double r{1e-1};       // measurement noise — position [unit²]
    double dt{0.002};     // sample period [s] (default: 500 Hz)
  };

  // ── Initialisation ────────────────────────────────────────────────────────

  // Initialise with explicit parameters.
  //
  // q_pos : process noise variance for position [rad²] or [m²]
  // q_vel : process noise variance for velocity [(rad/s)² or (m/s)²]
  // r     : measurement noise variance [rad²] or [m²]
  // dt    : sample period [s], must be > 0
  //
  // Throws std::invalid_argument on invalid parameters.
  // Resets all channel states to zero.
  void Init(double q_pos, double q_vel, double r, double dt) {
    if (q_pos < 0.0 || q_vel < 0.0) {
      throw std::invalid_argument("KalmanFilter: process noise must be non-negative");
    }
    if (r <= 0.0) {
      throw std::invalid_argument("KalmanFilter: measurement noise must be positive");
    }
    if (dt <= 0.0) {
      throw std::invalid_argument("KalmanFilter: dt must be positive");
    }

    params_.q_pos = q_pos;
    params_.q_vel = q_vel;
    params_.r     = r;
    params_.dt    = dt;
    initialized_  = true;

    Reset();
  }

  // Convenience overload accepting a Params struct.
  void Init(const Params& p) { Init(p.q_pos, p.q_vel, p.r, p.dt); }

  // ── State management ──────────────────────────────────────────────────────

  // Reset all channels to zero state with high initial uncertainty.
  // Call this after E-STOP or controller restart.
  void Reset() noexcept {
    for (auto& s : state_) {
      s.pos = 0.0;
      s.vel = 0.0;
      s.p00 = 1.0;   // initial position uncertainty
      s.p01 = 0.0;
      s.p11 = 1.0;   // initial velocity uncertainty
    }
  }

  // Seed initial position without resetting covariance.
  // Useful to avoid large transients when the filter is first enabled.
  void SetInitialPositions(const std::array<double, N>& positions) noexcept {
    for (std::size_t i = 0; i < N; ++i) {
      state_[i].pos = positions[i];
      state_[i].vel = 0.0;
    }
  }

  // ── Prediction step ────────────────────────────────────────────────────────
  //
  // Propagate the state estimate forward by one time step using the
  // constant-velocity model.  Always call once per control tick, regardless
  // of whether a new measurement is available.
  //
  // noexcept — safe on the 500 Hz RT path.
  void Predict() noexcept {
    const double dt   = params_.dt;
    const double dt2  = dt * dt;
    const double qp   = params_.q_pos;
    const double qv   = params_.q_vel;

    for (std::size_t i = 0; i < N; ++i) {
      ChannelState& s = state_[i];

      // State propagation:  x̂⁻ = F · x̂
      const double pos_pred = s.pos + s.vel * dt;
      const double vel_pred = s.vel;

      // Covariance propagation:  P⁻ = F·P·Fᵀ + Q
      //
      //   F·P·Fᵀ  =  | p00 + dt·p01 + dt·p10 + dt²·p11  ,  p01 + dt·p11 |
      //               | p10 + dt·p11                       ,  p11          |
      //
      // (p01 == p10 by symmetry)
      const double fp00 = s.p00 + 2.0 * dt * s.p01 + dt2 * s.p11;
      const double fp01 = s.p01 + dt * s.p11;
      const double fp11 = s.p11;

      s.pos = pos_pred;
      s.vel = vel_pred;
      s.p00 = fp00 + qp;
      s.p01 = fp01;
      s.p11 = fp11 + qv;
    }
  }

  // ── Update step ────────────────────────────────────────────────────────────
  //
  // Fuse an N-channel position measurement and return the filtered positions.
  // The Kalman gain blends model prediction and sensor reading optimally.
  //
  // Call after Predict() each tick.  Returns filtered position array.
  // noexcept — safe on the 500 Hz RT path.
  [[nodiscard]] std::array<double, N> Update(
      const std::array<double, N>& measurements) noexcept {
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
      out[i] = UpdateChannel(i, measurements[i]);
    }
    return out;
  }

  // Single-channel convenience overload.
  [[nodiscard]] double UpdateScalar(double z,
                                    std::size_t channel = 0) noexcept {
    return UpdateChannel(channel, z);
  }

  // ── Combined Predict + Update ─────────────────────────────────────────────
  //
  // Equivalent to calling Predict() then Update() in sequence.
  // Preferred in the common case where a new measurement arrives every tick.
  [[nodiscard]] std::array<double, N> PredictAndUpdate(
      const std::array<double, N>& measurements) noexcept {
    Predict();
    return Update(measurements);
  }

  // ── State accessors ────────────────────────────────────────────────────────

  // Filtered position estimate for channel i.
  [[nodiscard]] double position(std::size_t i) const noexcept {
    return state_[i].pos;
  }

  // Filtered velocity estimate for channel i  (differentiation-free!).
  [[nodiscard]] double velocity(std::size_t i) const noexcept {
    return state_[i].vel;
  }

  // All filtered positions as an array.
  [[nodiscard]] std::array<double, N> positions() const noexcept {
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) { out[i] = state_[i].pos; }
    return out;
  }

  // All filtered velocity estimates as an array.
  [[nodiscard]] std::array<double, N> velocities() const noexcept {
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) { out[i] = state_[i].vel; }
    return out;
  }

  // Position variance (uncertainty) for channel i.  Converges to a small
  // value once the filter is warmed up.
  [[nodiscard]] double position_variance(std::size_t i) const noexcept {
    return state_[i].p00;
  }

  // Kalman gain for the position state of channel i (computed after the
  // last Update() call).  K → 1 means "trust sensor"; K → 0 means "trust model".
  [[nodiscard]] double kalman_gain(std::size_t i) const noexcept {
    return gain_[i];
  }

  [[nodiscard]] bool         initialized() const noexcept { return initialized_; }
  [[nodiscard]] const Params& params()     const noexcept { return params_; }

 private:
  // ── Per-channel state ─────────────────────────────────────────────────────
  //
  // State vector:  x = [pos, vel]ᵀ
  // Covariance P (symmetric 2×2) stored as three scalars:
  //   | p00  p01 |
  //   | p01  p11 |
  struct ChannelState {
    double pos{0.0};
    double vel{0.0};
    double p00{1.0};  // position variance
    double p01{0.0};  // position-velocity covariance
    double p11{1.0};  // velocity variance
  };

  // ── Core update for a single channel ─────────────────────────────────────
  //
  //   S  = p00⁻ + R                   (innovation covariance, scalar)
  //   K  = [p00⁻, p01⁻]ᵀ / S         (Kalman gain, 2×1)
  //   ν  = z − pos⁻                   (innovation)
  //   x̂  = x̂⁻ + K · ν
  //   P  = (I − K·H) · P⁻
  //
  // H = [1, 0], so (I − K·H) = | 1−K₀   0 |
  //                              | −K₁    1 |
  [[nodiscard]] double UpdateChannel(std::size_t i, double z) noexcept {
    ChannelState& s = state_[i];
    const double  r = params_.r;

    const double S   = s.p00 + r;              // innovation covariance
    const double k0  = s.p00 / S;              // Kalman gain — position row
    const double k1  = s.p01 / S;              // Kalman gain — velocity row
    const double nu  = z - s.pos;              // innovation

    // State update
    s.pos += k0 * nu;
    s.vel += k1 * nu;

    // Covariance update  P = (I − K·H) · P⁻
    const double p00_new = (1.0 - k0) * s.p00;
    const double p01_new = (1.0 - k0) * s.p01;
    const double p11_new = -k1 * s.p01 + s.p11;

    s.p00 = p00_new;
    s.p01 = p01_new;
    s.p11 = p11_new;

    gain_[i] = k0;  // store for diagnostics
    return s.pos;
  }

  // ── Member data ───────────────────────────────────────────────────────────
  Params                     params_{};
  std::array<ChannelState, N> state_{};
  std::array<double, N>       gain_{};   // last computed Kalman gain (position row)
  bool                        initialized_{false};
};

// ── Convenience aliases ────────────────────────────────────────────────────
using KalmanFilter6  = KalmanFilterN<6>;   // 6-DOF robot joints
using KalmanFilter11 = KalmanFilterN<11>;  // 11 sensor values per fingertip (8 baro + 3 ToF)
using KalmanFilter1  = KalmanFilterN<1>;   // single-channel scalar use

}  // namespace rtc

#endif  // RTC_BASE_FILTERS_KALMAN_FILTER_HPP_
