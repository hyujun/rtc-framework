#ifndef RTC_MPC_REFERENCE_SWING_TRAJECTORY_HPP_
#define RTC_MPC_REFERENCE_SWING_TRAJECTORY_HPP_

/// @file swing_trajectory.hpp
/// @brief The vertical swing-foot reference of one contact over one swing
///        stretch (a @ref SwingInterval), and its impact-proximity weight.
///
/// Only z is planned: a swing foot's x · y references are the builder's
/// business (it uses the measured foot pose). Two cubic Hermite segments meet
/// at the apex, `t_mid = (t_lo + t_td) / 2`, with zero velocity there.
///
/// Let `z_lo = terrain`, `z_td = terrain + touch_down_offset`, `h = height`
/// and `s = min(1, (t_td − t_lo) / time_scale)` — short swings are scaled
/// down. The four cases of an interval (@ref BuildSwing):
///  - **closed**: `(t_lo, z_lo, s·v_lo)` → apex `min(z_lo, z_td) + s·h` →
///    `(t_td, z_td, s·v_td)`;
///  - **open_end** (touch-down beyond the window): `(t_lo, z_lo, v_lo)` →
///    apex `z_lo + h` → `(t_end, z_lo + h, 0)`, unscaled;
///  - **open_start** (lift-off before the window): `(t0, z_td + h, 0)` →
///    apex `z_td + h` → `(t_td, z_td, v_td)`, unscaled;
///  - **both**: the constant `z_td + h`.
/// The impact proximity runs on the same machine: closed `1 → mid → 1` with
/// end rates `s·v_lo^ip`, `s·v_td^ip` (`mid` itself is not scaled); open_end
/// `(1, v_lo^ip) → mid → (mid, 0)`; open_start `(mid, 0) → mid → (1, v_td^ip)`;
/// both the constant `mid`. In stance z = terrain, ż = z̈ = 0, proximity 1.
///
/// @ref SwingTrajectory::Evaluate reads the swing on the half-open
/// `[t_lo, t_td)` (an open end holds the curve past `t_end`) and stance
/// outside it, so touch-down is the stance value and lift-off the curve's
/// start (= terrain, bitwise). A non-finite `t` reads as stance. The impact
/// proximity is clamped to `[0, 1]` with its rate 0 while clamped (end rates
/// steep enough to overshoot the cubic are not refused by the loader).
///
/// Per segment (coefficients of the original CubicSpline, normalised time
/// `τ = (t − t0) / Δt`): `c0 = p0`, `c1 = v0·Δt`, `c2 = −(3v0 + Δv)·Δt + 3Δp`,
/// `c3 = (2v0 + Δv)·Δt − 2Δp`, `z = c3τ³ + c2τ² + c1τ + c0`, `ż = dz/dτ / Δt`,
/// `z̈ = d²z/dτ² / Δt²`. Outside its segment a curve holds its end values.
///
/// Known limits (the original planner's, kept — W8 may clip): the open cases
/// are not scaled, so a lift-off a few µs before the window end plans the
/// full `h` over that sliver (finite, but ż · z̈ of 1e5 · 1e12 and beyond);
/// and with a negative offset a very short closed swing (`s·h < |offset|`)
/// dips below the terrain before rising (`min(z_lo, z_td) + s·h`).
///
/// Everything after the YAML loader is noexcept and heap-free (MPC thread).

#include "rtc_mpc/reference/contact_schedule.hpp"

#include <yaml-cpp/yaml.h>

namespace rtc::mpc {

/// @brief Shortest swing stretch @ref BuildSwing accepts [s] (NUM-2: each
///        segment divides by half of it).
inline constexpr double kMinSwingDuration = 1e-9;

/// @brief Failure modes of @ref LoadSwingConfig, @ref ValidateSwingConfig and @ref BuildSwing.
enum class SwingError {
  kNoError = 0,
  kNotAMap,              ///< the section is absent or not a map
  kUnknownKey,           ///< a key outside the schema (a misspelt key must not fall back)
  kMissingKey,           ///< a schema key without a value — every key is required
  kWrongType,            ///< a value of the wrong kind
  kNonFinite,            ///< a NaN / ±inf value (config, interval or terrain)
  kInvalidHeight,        ///< `height ≤ 0`
  kInvalidTimeScale,     ///< `time_scale ≤ 0` (NUM-2)
  kInvalidImpactMid,     ///< `impact_proximity.mid ∉ [0, 1]`
  kDegenerateInterval,   ///< `touch_down − lift_off ≤ kMinSwingDuration`, or the midpoint
                         ///< rounds onto an end (a 1-ulp stretch at a large `t`)
  kCoefficientOverflow,  ///< finite inputs whose Hermite coefficients overflow
};

/// @brief The impact-proximity curve's end points (dimensionless weight in [0, 1]).
struct ImpactProximityConfig {
  double mid{0.0};                  ///< the weight at the apex, in [0, 1]
  double lift_off_velocity{0.0};    ///< [1/s] rate at lift-off (closed / open_end)
  double touch_down_velocity{0.0};  ///< [1/s] rate at touch-down (closed / open_start)
};

/// @brief Swing configuration. YAML schema (`swing:`, every key required):
/// ```yaml
/// height: 0.08              # [m] > 0 — apex above the lower of lift-off / touch-down
/// lift_off_velocity: 0.05   # [m/s] ż at lift-off (scaled by s when closed)
/// touch_down_velocity: 0.0  # [m/s] ż at touch-down (scaled by s when closed)
/// touch_down_offset: -0.001 # [m] touch-down height relative to the terrain
/// time_scale: 0.4           # [s] > 0 — swings shorter than this are scaled by T / time_scale
/// impact_proximity:
///   mid: 0.005              # in [0, 1]
///   lift_off_velocity: -0.15   # [1/s]
///   touch_down_velocity: 0.3   # [1/s]
/// ```
struct SwingConfig {
  double height{0.0};
  double lift_off_velocity{0.0};
  double touch_down_velocity{0.0};
  double touch_down_offset{0.0};
  double time_scale{0.0};
  ImpactProximityConfig impact_proximity{};
};

/// @brief Parse the `swing:` map into `out`. `out` is written only on
///        `kNoError`, which also implies @ref ValidateSwingConfig passed.
[[nodiscard]] SwingError LoadSwingConfig(const YAML::Node& node, SwingConfig& out) noexcept;

/// @brief The value checks of the schema above (everything but key / type).
[[nodiscard]] SwingError ValidateSwingConfig(const SwingConfig& cfg) noexcept;

/// @brief One cubic Hermite segment on `[t0, t0 + dt]` (file comment), built
///        only by @ref TwoSegmentCubic::Build.
struct CubicSegment {
  double t0{0.0};
  double dt{1.0};
  double inv_dt{1.0};   ///< `1 / dt`
  double inv_dt2{1.0};  ///< `1 / dt²`
  double c0{0.0}, c1{0.0}, c2{0.0}, c3{0.0};

  /// @brief Value · rate · acceleration at `t`, clamped to the segment
  ///        (`t` NaN reads as `t0`).
  void Evaluate(double t, double& value, double& rate, double& acceleration) const noexcept;
};

/// @brief Two segments meeting at `t_mid = (t0 + t1) / 2` with zero rate.
class TwoSegmentCubic {
 public:
  /// @brief Build through `(t0, p0, v0)`, `(t_mid, p_mid, 0)`, `(t1, p1, v1)`.
  ///        `kDegenerateInterval` when `t1 − t0 ≤ kMinSwingDuration` or `t_mid`
  ///        rounds onto `t0` / `t1`; `kNonFinite` for a non-finite input,
  ///        `kCoefficientOverflow` for a coefficient. On error nothing changes.
  [[nodiscard]] SwingError Build(double t0, double p0, double v0, double p_mid, double t1,
                                 double p1, double v1) noexcept;

  /// @brief Left segment for `t < t_mid` (a NaN `t` too: its start), right for
  ///        `t ≥ t_mid`; each clamps to its own range, so the curve holds its
  ///        end values outside `[t0, t1]`.
  void Evaluate(double t, double& value, double& rate, double& acceleration) const noexcept;

 private:
  CubicSegment left_{};
  CubicSegment right_{};
  double t_mid_{0.0};
};

/// @brief One contact's vertical reference at one instant.
struct SwingSample {
  double z{0.0};           ///< [m]
  double z_dot{0.0};       ///< [m/s]
  double z_ddot{0.0};      ///< [m/s²]
  double impact{1.0};      ///< proximity weight, clamped to [0, 1] (1 = at / near a contact)
  double impact_dot{0.0};  ///< [1/s], 0 while `impact` is clamped
};

/// @brief One contact's swing over one interval, or stance (file comment).
class SwingTrajectory {
 public:
  /// @brief Stance everywhere: `z = terrain`, ż = z̈ = 0, proximity 1.
  [[nodiscard]] static SwingTrajectory Stance(double terrain_z) noexcept;

  [[nodiscard]] SwingSample Evaluate(double t) const noexcept;

  [[nodiscard]] bool swinging() const noexcept { return swinging_; }

  [[nodiscard]] double touch_down() const noexcept { return t_td_; }

  [[nodiscard]] bool open_start() const noexcept { return open_start_; }  ///< the start is t0

  [[nodiscard]] bool open_end() const noexcept { return open_end_; }

  [[nodiscard]] double scaling() const noexcept { return scaling_; }  ///< `s` (1 when open)

 private:
  friend SwingError BuildSwing(const SwingInterval&, const SwingConfig&, double,
                               SwingTrajectory&) noexcept;
  TwoSegmentCubic z_{};
  TwoSegmentCubic impact_{};
  double terrain_z_{0.0};
  double t_lo_{0.0};
  double t_td_{0.0};
  double scaling_{1.0};
  bool swinging_{false};
  bool open_start_{false};
  bool open_end_{false};
};

/// @brief Plan `interval` of a contact standing on `terrain_z` (the surface
///        height under that contact). `out` is written only on `kNoError`.
[[nodiscard]] SwingError BuildSwing(const SwingInterval& interval, const SwingConfig& cfg,
                                    double terrain_z, SwingTrajectory& out) noexcept;

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_SWING_TRAJECTORY_HPP_
