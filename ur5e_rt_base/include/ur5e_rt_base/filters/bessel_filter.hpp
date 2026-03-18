#ifndef UR5E_RT_BASE_FILTERS_BESSEL_FILTER_HPP_
#define UR5E_RT_BASE_FILTERS_BESSEL_FILTER_HPP_

// 4th-order Bessel low-pass filter implemented as two cascaded biquad sections
// using the bilinear transform with frequency prewarping.
//
// Why Bessel for robot control?
//   Bessel filters have maximally flat group delay (linear phase), meaning all
//   frequency components experience the same propagation delay.  This preserves
//   the shape of the filtered signal and prevents controller phase distortion —
//   a critical property for smooth, stable motion.
//
// Design:
//   Analog prototype poles (4th-order Bessel, -3 dB normalised to ω = 1 rad/s):
//     Conjugate pair 1:  –1.370063 ± 0.410258j  →  ω₀ = 1.4301691,  Q = 0.5219356
//     Conjugate pair 2:  –0.995214 ± 1.257102j  →  ω₀ = 1.6033575,  Q = 0.8055342
//   These are factored into two second-order sections (SOS / biquads) and mapped
//   to the digital domain via the bilinear transform with cutoff prewarping.
//
// RT safety:
//   - Init() performs all computation and may throw on invalid parameters.
//   - Apply() / ApplyScalar() are noexcept — safe to call on the 500 Hz RT path.
//   - No heap allocation after Init().
//
// Template parameter N: number of independent channels (e.g. 6 for robot joints).
//
// Usage example:
//   BesselFilterN<6> filter;
//   filter.Init(100.0, 500.0);          // 100 Hz cutoff, 500 Hz sample rate
//
//   // Inside the 500 Hz ControlLoop():
//   std::array<double,6> filtered = filter.Apply(raw_positions);

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>

namespace ur5e_rt_controller {

template <std::size_t N>
class BesselFilterN {
 public:
  // Coefficients of one second-order IIR section.
  // Transfer function:
  //   H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)
  struct BiquadCoeffs {
    double b0{}, b1{}, b2{};  // feedforward
    double a1{}, a2{};        // feedback (see ApplyBiquad for sign convention)
  };

  // Initialise filter coefficients for a given cutoff and sample rate.
  //
  // cutoff_hz      : desired -3 dB frequency [Hz] (must be < sample_rate_hz / 2)
  // sample_rate_hz : control loop rate [Hz], e.g. 500.0
  //
  // Throws std::invalid_argument on invalid parameters.
  // Resets internal state to zero.
  void Init(double cutoff_hz, double sample_rate_hz) {
    if (cutoff_hz <= 0.0 || sample_rate_hz <= 0.0) {
      throw std::invalid_argument("BesselFilter: frequencies must be positive");
    }
    if (cutoff_hz >= sample_rate_hz * 0.5) {
      throw std::invalid_argument(
          "BesselFilter: cutoff_hz must be below Nyquist (sample_rate_hz / 2)");
    }

    // Bilinear-transform constant and prewarped analog cutoff.
    // Prewarping ensures the digital -3 dB point lands exactly at cutoff_hz.
    const double k       = 2.0 * sample_rate_hz;
    const double omega_p = k * std::tan(std::numbers::pi * cutoff_hz / sample_rate_hz);

    section1_ = ComputeBiquad(kProtoOmega0_1, kProtoQ_1, omega_p, k);
    section2_ = ComputeBiquad(kProtoOmega0_2, kProtoQ_2, omega_p, k);

    cutoff_hz_      = cutoff_hz;
    sample_rate_hz_ = sample_rate_hz;
    initialized_    = true;

    Reset();
  }

  // Reset all delay elements to zero (e.g. at controller start or after E-STOP).
  void Reset() noexcept {
    for (auto& s : state1_) { s.d1 = s.d2 = 0.0; }
    for (auto& s : state2_) { s.d1 = s.d2 = 0.0; }
  }

  // Filter an N-channel input sample.  Returns the filtered output array.
  // noexcept — safe to call on the 500 Hz RT path.
  [[nodiscard]] std::array<double, N> Apply(
      const std::array<double, N>& input) noexcept {
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
      const double y1 = ApplyBiquad(section1_, state1_[i], input[i]);
      out[i]          = ApplyBiquad(section2_, state2_[i], y1);
    }
    return out;
  }

  // Convenience overload for single-channel use (channel index 0..N-1).
  [[nodiscard]] double ApplyScalar(double x,
                                   std::size_t channel = 0) noexcept {
    const double y1 = ApplyBiquad(section1_, state1_[channel], x);
    return ApplyBiquad(section2_, state2_[channel], y1);
  }

  // Accessors
  [[nodiscard]] bool   initialized()     const noexcept { return initialized_; }
  [[nodiscard]] double cutoff_hz()       const noexcept { return cutoff_hz_; }
  [[nodiscard]] double sample_rate_hz()  const noexcept { return sample_rate_hz_; }
  [[nodiscard]] const BiquadCoeffs& section1() const noexcept { return section1_; }
  [[nodiscard]] const BiquadCoeffs& section2() const noexcept { return section2_; }

 private:
  // ── Analog prototype constants ────────────────────────────────────────────
  // 4th-order Bessel filter, -3 dB normalised to ω = 1 rad/s.
  //
  // Starting from the Bessel polynomial B₄(s) = s⁴+10s³+45s²+105s+105,
  // factored into two SOS with group-delay normalisation:
  //   (s²+5.7924s+9.1401)(s²+4.2076s+11.4878)
  //
  // The -3 dB frequency of the group-delay normalised filter is
  //   ω₋₃dB = 2.1139176749 rad/s
  // Dividing all poles by this constant yields the -3 dB normalised prototype.
  //
  // Resulting poles (normalised to -3 dB at ω = 1 rad/s):
  //   p₁,₂ = -1.370063 ± 0.410258j  →  ω₀ = 1.4301691433,  Q = 0.5219356105
  //   p₃,₄ = -0.995214 ± 1.257102j  →  ω₀ = 1.6033574829,  Q = 0.8055342053
  static constexpr double kProtoOmega0_1 = 1.4301691433;  // √(1.370063²+0.410258²)
  static constexpr double kProtoQ_1      = 0.5219356105;  // ω₀₁/(2·1.370063)
  static constexpr double kProtoOmega0_2 = 1.6033574829;  // √(0.995214²+1.257102²)
  static constexpr double kProtoQ_2      = 0.8055342053;  // ω₀₂/(2·0.995214)

  // ── Per-biquad delay state (Direct Form II Transposed) ───────────────────
  struct BiquadState {
    double d1{0.0};  // first  delay element
    double d2{0.0};  // second delay element
  };

  // ── Coefficient computation ───────────────────────────────────────────────
  //
  // Maps one analog prototype 2nd-order section to the digital domain:
  //
  //   H_a(s) = ω₀² / (s² + (ω₀/Q)·s + ω₀²)   (prototype, normalised to 1 rad/s)
  //
  // Steps:
  //   1. Scale prototype natural frequency to the prewarped cutoff:
  //        ω₀_actual = ω₀_proto · ω_p
  //   2. Apply bilinear transform  s → k·(z−1)/(z+1)  analytically:
  //        D     = k² + (ω₀/Q)·k + ω₀²
  //        b0=b2 = ω₀² / D
  //        b1    = 2·b0
  //        a1    = (2·ω₀² − 2·k²) / D
  //        a2    = (k² − (ω₀/Q)·k + ω₀²) / D
  //
  // Q is scale-invariant and is passed through unchanged.
  [[nodiscard]] static BiquadCoeffs ComputeBiquad(double omega0_proto,
                                                  double q_proto,
                                                  double omega_p,
                                                  double k) noexcept {
    const double omega0    = omega0_proto * omega_p;
    const double omega0_sq = omega0 * omega0;
    const double k_sq      = k * k;
    const double bw        = omega0 / q_proto;  // ω₀/Q  (bandwidth term)
    const double D         = k_sq + bw * k + omega0_sq;

    BiquadCoeffs c;
    c.b0 = omega0_sq / D;
    c.b1 = 2.0 * c.b0;
    c.b2 = c.b0;
    c.a1 = (2.0 * omega0_sq - 2.0 * k_sq) / D;
    c.a2 = (k_sq - bw * k + omega0_sq) / D;
    return c;
  }

  // ── Single biquad evaluation — Direct Form II Transposed ─────────────────
  //
  //   y    =  b0·x + d1
  //   d1  ←  b1·x − a1·y + d2
  //   d2  ←  b2·x − a2·y
  //
  // TDF-II is preferred over Direct Form I for better numerical properties
  // (lower coefficient sensitivity) and simpler state management.
  [[nodiscard]] static double ApplyBiquad(const BiquadCoeffs& c,
                                          BiquadState& s,
                                          double x) noexcept {
    const double y = c.b0 * x + s.d1;
    s.d1           = c.b1 * x - c.a1 * y + s.d2;
    s.d2           = c.b2 * x - c.a2 * y;
    return y;
  }

  // ── Member data ───────────────────────────────────────────────────────────
  BiquadCoeffs section1_{};
  BiquadCoeffs section2_{};

  std::array<BiquadState, N> state1_{};
  std::array<BiquadState, N> state2_{};

  bool   initialized_{false};
  double cutoff_hz_{0.0};
  double sample_rate_hz_{0.0};
};

// ── Convenience aliases ────────────────────────────────────────────────────
using BesselFilter6  = BesselFilterN<6>;   // 6-DOF robot joints
using BesselFilter11 = BesselFilterN<11>;  // 11-DOF hand joints
using BesselFilter1  = BesselFilterN<1>;   // single-channel scalar use

// TOF sensor filtering (kMaxFingertips × kTofCount = 8 × 3 = 24 channels)
inline constexpr std::size_t kMaxTofChannels = 24;
using BesselFilterTof = BesselFilterN<kMaxTofChannels>;

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_FILTERS_BESSEL_FILTER_HPP_
