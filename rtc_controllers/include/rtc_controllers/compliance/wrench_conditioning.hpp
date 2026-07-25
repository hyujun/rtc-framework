// ── External wrench conditioning chain (spec §3.2.1, §3.3) ──────────────────
// The stage ORDER is normative and load-bearing:
//
//     bias removal → static gravity compensation → deadband → saturation → filter
//
// and it all happens in the SENSOR frame; the Ad^{-T} transform to the task
// frame is the LAST step and lives in the controller (design §8.2 step 8),
// because only the controller holds the model that supplies the two frames.
// Each stage is a free function so it can be pinned by an independent test
// instead of only through the composed Apply().
//
// Why the order matters (each swap is a real bug, not a preference):
//   • bias before gravity — the bias is the sensor's own electrical offset; the
//     gravity term is a modelled quantity. Removing gravity from a biased signal
//     leaves the bias in, and re-deriving it later double-counts.
//   • deadband before saturation — a deadband applied to an already-clipped
//     signal shrinks the clip level; the reverse leaves the declared |w|max the
//     true actuator-facing bound.
//   • filter last — filtering before the deadband smears the deadband edge over
//     the filter's impulse response, so the "no contact ⇒ exactly zero" property
//     the contact FSM depends on stops holding.
//
// Deadband is the SOFT (shrink) form  y = sign(x)·max(0, |x| − d), not a hard
// zeroing band. A hard band steps by d at the boundary, and §10.6 spends an
// entire MUST on not injecting torque discontinuities through this path.
//
// Filter: repo's BesselFilterN (4th order) rather than the spec's 2nd-order
// Butterworth (D11 — P5 forbids forking an equivalent filter; the deviation is
// recorded in docs/compliance-conventions.md). §3.3 asks for different cutoffs
// on translation and rotation, so it is TWO BesselFilterN<3> and not one <6>.
// BesselFilterN::Apply() before Init() silently returns 0 (all coefficients are
// zero) — Configure() is therefore mandatory, and the controller calls it from
// its constructor as well as from LoadConfig so no path can reach Apply()
// uninitialised.
#pragma once

#include "rtc_controllers/compliance/external_wrench.hpp"
#include <rtc_base/filters/bessel_filter.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::compliance {

// ── Stage 1: bias ───────────────────────────────────────────────────────────
inline void RemoveBias(Wrench6& w, const Wrench6& bias) noexcept {
  for (std::size_t i = 0; i < kWrenchDim; ++i)
    w[i] -= bias[i];
}

// ── Stage 2: static gravity / payload compensation (§3.2.1) ─────────────────
//
// Minimal implementation per the spec: the distal payload's weight, expressed at
// the sensor origin in the SENSOR frame. Dynamic terms (payload inertia × accel)
// are dropped — the quasi-static assumption this whole slice already makes.
//
//   f_g = ᵂR_Sᵀ · (m_p · ᵂg)          (world gravity rotated into the sensor)
//   τ_g = ˢr_c × f_g                   (lever arm from the sensor origin to CoM)
//
// `g_world` comes from the robot model (pinocchio Model::gravity), never from a
// hard-coded 9.81 — the URDF decides which axis gravity points along, and a
// non-Z-gravity model is an existing test case in this package.
[[nodiscard]] inline Wrench6 StaticGravityWrench(const Eigen::Matrix3d& R_world_sensor,
                                                 const Eigen::Vector3d& g_world,
                                                 double payload_mass,
                                                 const Eigen::Vector3d& com_sensor) noexcept {
  Wrench6 out{};
  if (payload_mass == 0.0)
    return out;
  const Eigen::Vector3d f = R_world_sensor.transpose() * (payload_mass * g_world);
  const Eigen::Vector3d t = com_sensor.cross(f);
  for (int i = 0; i < 3; ++i) {
    out[static_cast<std::size_t>(i)] = f(i);
    out[static_cast<std::size_t>(i) + 3] = t(i);
  }
  return out;
}

inline void RemoveGravity(Wrench6& w, const Wrench6& gravity_wrench) noexcept {
  for (std::size_t i = 0; i < kWrenchDim; ++i)
    w[i] -= gravity_wrench[i];
}

// ── Stage 3: deadband (soft shrink, component-wise) ─────────────────────────
inline void ApplyDeadband(Wrench6& w, const Wrench6& deadband) noexcept {
  for (std::size_t i = 0; i < kWrenchDim; ++i) {
    const double d = deadband[i];
    if (d <= 0.0)
      continue;
    const double a = std::abs(w[i]);
    w[i] = (a <= d) ? 0.0 : std::copysign(a - d, w[i]);
  }
}

// ── Stage 4: saturation (component-wise absolute bound) ─────────────────────
/// Returns true when any component was clipped (→ diagnostics / quality).
inline bool SaturateWrench(Wrench6& w, const Wrench6& max_abs) noexcept {
  bool clipped = false;
  for (std::size_t i = 0; i < kWrenchDim; ++i) {
    const double lim = max_abs[i];
    if (lim <= 0.0)
      continue;
    if (w[i] > lim) {
      w[i] = lim;
      clipped = true;
    } else if (w[i] < -lim) {
      w[i] = -lim;
      clipped = true;
    }
  }
  return clipped;
}

// ── Configuration (off-RT) ──────────────────────────────────────────────────
struct WrenchConditioningConfig {
  Wrench6 deadband{};                                        ///< per component, ≥0 (0 = off)
  Wrench6 max_abs{{500.0, 500.0, 500.0, 50.0, 50.0, 50.0}};  ///< |w|max per component
  double payload_mass{0.0};                                  ///< m_p [kg], 0 = no compensation
  Eigen::Vector3d payload_com{Eigen::Vector3d::Zero()};      ///< ˢr_c [m], sensor frame
  double filter_cutoff_force_hz{20.0};                       ///< §3.3 translation default
  double filter_cutoff_torque_hz{15.0};                      ///< §3.3 rotation default
  bool filter_enabled{true};
  int bias_samples{100};  ///< N averaged during BIAS_CALIBRATING (§3.2.1 MUST)
};

/// Owns the bias estimate and the filter state; stages above stay free functions.
/// Configure() is off-RT and may throw; every other method is RT-safe.
class WrenchConditioner {
 public:
  /// @param sample_rate_hz  control loop rate — the filter is designed for the
  ///        tick it will actually run at; a mismatch shifts the real cutoff.
  /// @throws std::runtime_error on a cutoff at/above Nyquist or a negative bound.
  ///
  /// ALL-OR-NOTHING: every check runs against the ARGUMENT, before a single byte
  /// of member state moves. A rejected call therefore leaves the live conditioner
  /// — config, bias, filter coefficients and filter history — exactly as it was.
  /// The controller's LoadConfig is built on that: it resolves the sensor frame
  /// and calls this last, precisely so a bad YAML cannot leave a running
  /// controller half-reconfigured. (Validating some fields and committing others
  /// first would silently swap deadband/saturation/payload under an unchanged
  /// filter — the exact half-state that ordering exists to prevent.)
  void Configure(const WrenchConditioningConfig& cfg, double sample_rate_hz) {
    for (std::size_t i = 0; i < kWrenchDim; ++i) {
      if (cfg.deadband[i] < 0.0)
        throw std::runtime_error("WrenchConditioner: wrench_deadband must be >= 0");
      if (cfg.max_abs[i] < 0.0)
        throw std::runtime_error("WrenchConditioner: wrench_max must be >= 0");
    }
    if (cfg.payload_mass < 0.0)
      throw std::runtime_error("WrenchConditioner: payload_mass must be >= 0");
    if (cfg.bias_samples < 0)
      throw std::runtime_error("WrenchConditioner: bias_calibration_samples must be >= 0");
    if (sample_rate_hz <= 0.0)
      throw std::runtime_error("WrenchConditioner: control rate must be > 0");
    if (cfg.filter_enabled) {
      // BesselFilterN::Init throws below/above its own bounds; translate the
      // message so a YAML author sees which knob is wrong. These two checks are
      // exactly Init's own preconditions, so the Init calls below cannot throw —
      // which is what makes the commit sequence that follows unconditional.
      if (cfg.filter_cutoff_force_hz <= 0.0 || cfg.filter_cutoff_torque_hz <= 0.0)
        throw std::runtime_error("WrenchConditioner: wrench filter cutoffs must be > 0");
      if (cfg.filter_cutoff_force_hz >= 0.5 * sample_rate_hz ||
          cfg.filter_cutoff_torque_hz >= 0.5 * sample_rate_hz)
        throw std::runtime_error(
            "WrenchConditioner: wrench filter cutoff must be below Nyquist (control_rate/2), got " +
            std::to_string(cfg.filter_cutoff_force_hz) + "/" +
            std::to_string(cfg.filter_cutoff_torque_hz) + " Hz at " +
            std::to_string(sample_rate_hz) + " Hz");
    }

    // ── Commit (nothing below throws) ─────────────────────────────────────────
    cfg_ = cfg;
    if (cfg_.filter_enabled) {
      filter_force_.Init(cfg_.filter_cutoff_force_hz, sample_rate_hz);
      filter_torque_.Init(cfg_.filter_cutoff_torque_hz, sample_rate_hz);
    }
    Reset();
  }

  /// Drop the bias estimate, the accumulator and the filter history. Called on
  /// (re)activation together with the controller's own re-seed.
  void Reset() noexcept {
    bias_ = Wrench6{};
    accum_ = Wrench6{};
    accum_count_ = 0;
    saturated_ = false;
    filter_force_.Reset();
    filter_torque_.Reset();
  }

  /// RT: fold one sample into the bias average (§3.2.1 "정지 상태에서 N 샘플
  /// 평균"). `gravity_wrench` is subtracted first so the stored bias is the pure
  /// sensor offset — that keeps Apply()'s documented stage order (bias, THEN
  /// gravity) self-consistent: at the calibration pose the chain output is 0.
  /// Returns true once `bias_samples` have been accumulated (bias committed).
  bool AccumulateBias(const Wrench6& raw, const Wrench6& gravity_wrench) noexcept {
    if (cfg_.bias_samples <= 0) {
      bias_ = Wrench6{};
      return true;
    }
    for (std::size_t i = 0; i < kWrenchDim; ++i)
      accum_[i] += raw[i] - gravity_wrench[i];
    if (++accum_count_ < cfg_.bias_samples)
      return false;
    const double inv = 1.0 / static_cast<double>(accum_count_);
    for (std::size_t i = 0; i < kWrenchDim; ++i)
      bias_[i] = accum_[i] * inv;
    accum_ = Wrench6{};
    accum_count_ = 0;
    return true;
  }

  /// RT: the full chain, in the sensor frame. `gravity_wrench` is the §3.2.1
  /// static payload term (StaticGravityWrench), passed in so the controller —
  /// which owns the model — supplies it and this class stays model-free.
  [[nodiscard]] Wrench6 Apply(const Wrench6& raw, const Wrench6& gravity_wrench) noexcept {
    Wrench6 w = raw;
    RemoveBias(w, bias_);
    RemoveGravity(w, gravity_wrench);
    ApplyDeadband(w, cfg_.deadband);
    saturated_ = SaturateWrench(w, cfg_.max_abs);
    if (cfg_.filter_enabled) {
      std::array<double, 3> f{w[0], w[1], w[2]};
      std::array<double, 3> t{w[3], w[4], w[5]};
      const std::array<double, 3> fo = filter_force_.Apply(f);
      const std::array<double, 3> to = filter_torque_.Apply(t);
      for (std::size_t i = 0; i < 3; ++i) {
        w[i] = fo[i];
        w[i + 3] = to[i];
      }
    }
    return w;
  }

  /// RT: load the filter delay lines with the DC steady state for `w` so the
  /// first Apply() after activation does not emit a step response (§3.3 MUST:
  /// "on_activate에서 현재 측정값으로 filter state를 채워 넣을 것").
  void SeedFilter(const Wrench6& w) noexcept {
    if (!cfg_.filter_enabled)
      return;
    filter_force_.Seed({w[0], w[1], w[2]});
    filter_torque_.Seed({w[3], w[4], w[5]});
  }

  /// RT: seed the filter from a RAW sample by running the algebraic stages only.
  /// This is what §3.3 actually asks for — the steady state of the signal the
  /// filter is about to see. Seeding with plain zero would be right only when
  /// the arm is unloaded at activation; with a standing load (holding an object)
  /// it reintroduces the very step response the seeding exists to avoid.
  /// Does NOT advance the delay lines, so it cannot ring.
  void SeedFromSample(const Wrench6& raw, const Wrench6& gravity_wrench) noexcept {
    Wrench6 w = raw;
    RemoveBias(w, bias_);
    RemoveGravity(w, gravity_wrench);
    ApplyDeadband(w, cfg_.deadband);
    (void)SaturateWrench(w, cfg_.max_abs);
    SeedFilter(w);
  }

  [[nodiscard]] const Wrench6& bias() const noexcept { return bias_; }

  [[nodiscard]] bool saturated() const noexcept { return saturated_; }

  [[nodiscard]] const WrenchConditioningConfig& config() const noexcept { return cfg_; }

 private:
  WrenchConditioningConfig cfg_{};
  Wrench6 bias_{};
  Wrench6 accum_{};
  int accum_count_{0};
  bool saturated_{false};
  BesselFilterN<3> filter_force_;
  BesselFilterN<3> filter_torque_;
};

}  // namespace rtc::compliance
