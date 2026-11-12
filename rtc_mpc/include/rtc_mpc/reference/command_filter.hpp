#ifndef RTC_MPC_REFERENCE_COMMAND_FILTER_HPP_
#define RTC_MPC_REFERENCE_COMMAND_FILTER_HPP_

/// @file command_filter.hpp
/// @brief Velocity command filter for a legged base: limit, smooth and integrate
///        a Twist-shaped command into a planar velocity + base height target.
///
/// The filter is a **non-RT** stage. Its caller (a consumer timer) calls
/// @ref CommandFilter::Step once per tick at the fixed `sample_rate_hz` it was
/// configured with — every tick, also when the input is stale or non-finite,
/// because the Bessel stage assumes a fixed sample period. Delivery of the
/// output to the RT side (SeqLock), the timer and the RT-side staleness check
/// (by `sequence` and a tick count — the RT side does not read a clock) belong
/// to the consumer.
///
/// Per-tick chain, axis `i` of `u = [v_x, v_y, ω_z, ḣ]` (Twist `linear.x`,
/// `linear.y`, `angular.z`, `linear.z` = base height rate [m/s]):
///  1. stale: `age = now_ns − raw_stamp_ns` (signed; an overflowing difference
///     counts as stale). `|age| > t_stale` ⇒ every raw axis is 0 and the
///     accepted values are cleared, so a non-finite axis after a stale stretch
///     holds 0, not a value from before it. A stamp more than `t_stale` in the
///     future is stale too — a clock running ahead must not keep a dead command
///     alive. `stale_count` counts such ticks.
///  2. finite gate (NUM-7, fresh ticks only): a non-finite axis keeps its last
///     accepted raw value; the other axes pass untouched. `nonfinite_count`
///     counts such ticks.
///  3. clamp `|u_i| ≤ u_max_i` (`u_max = [v_max…, hdot_max]`).
///  4. optional slew `|Δu_i| ≤ accel_max_i / f_s` (@ref SlewLimit; 0 = off).
///  5. `rtc::BesselFilterN<4>` at (`cutoff_hz`, `sample_rate_hz`).
///  6. final clamp `|y_i| ≤ u_max_i` (absorbs the ~1 % Bessel overshoot).
///  7. `base_height ← clamp(base_height + y_ḣ / f_s, h_min, h_max)` — the
///     integrator state is the clamped height itself, so it cannot wind up.
///
/// `angular.x` / `angular.y` are not used; a tick on which either is non-zero
/// (or non-finite) bumps `ignored_axis_count` and changes no output.
///
/// Thread contract: `Init`, `Reset` and `Step` run on one thread. No method
/// allocates after construction, and none throws.

#include "rtc_base/filters/bessel_filter.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

/// @brief Number of filtered command axes: `v_x, v_y, ω_z, ḣ`.
inline constexpr std::size_t kCommandAxes = 4;

/// @brief Upper bound of `t_stale` [s] — keeps `t_stale` in nanoseconds far inside int64.
inline constexpr double kMaxStaleTimeout = 1e6;

/// @brief Failure modes of @ref LoadCommandFilterConfig, @ref ValidateCommandFilterConfig,
///        @ref CommandFilter::Init and @ref CommandFilter::Reset.
enum class CommandFilterError {
  kNoError = 0,
  kNotAMap,              ///< the `command:` node is absent or not a map
  kUnknownKey,           ///< a key outside the schema (a misspelt key must not fall back)
  kMissingKey,           ///< a schema key without a value — every key is required
  kWrongType,            ///< a value of the wrong kind or sequence length
  kNonFinite,            ///< a NaN / ±inf value
  kInvalidSampleRate,    ///< `sample_rate_hz ≤ 0` (NUM-2)
  kInvalidCutoff,        ///< `cutoff_hz ≤ 0` or `cutoff_hz ≥ sample_rate_hz / 2`
  kNegativeLimit,        ///< a `v_max` / `hdot_max` / `accel_max` entry `< 0`
  kInvalidStaleTimeout,  ///< `t_stale ∉ [1 / sample_rate_hz, kMaxStaleTimeout]`
  kInvalidHeightRange,   ///< `h_min > h_max`, or `base_height ∉ [h_min, h_max]`
  kNotInitialised,       ///< `Reset` before a successful `Init`
  kHeightOutOfRange,     ///< `Reset(h0)` with `h0 ∉ [h_min, h_max]` (or non-finite)
};

/// @brief Filter configuration. YAML schema (`command:`, every key required):
/// ```yaml
/// sample_rate_hz: 100.0          # [Hz] > 0 — the rate Step is called at
/// cutoff_hz: 2.4                 # [Hz] in (0, sample_rate_hz / 2) — Bessel −3 dB
/// v_max: [2.0, 1.2, 1.0]         # [m/s, m/s, rad/s] ≥ 0 — |v_x|, |v_y|, |ω_z| bound
/// hdot_max: 0.1                  # [m/s] ≥ 0 — |ḣ| bound
/// accel_max: [0.0, 0.0, 0.0, 0.0]  # [m/s², m/s², rad/s², m/s²] ≥ 0 — slew, 0 = off
/// t_stale: 0.5                   # [s] in [1 / sample_rate_hz, 1e6] — older input reads as 0
/// base_height: 0.8               # [m] in [h_min, h_max] — height after Init
/// h_min: 0.5                     # [m]
/// h_max: 1.0                     # [m] ≥ h_min (equal pins the height)
/// ```
struct CommandFilterConfig {
  double sample_rate_hz{0.0};
  double cutoff_hz{0.0};
  std::array<double, kCommandAxes> u_max{};      ///< `[v_max…, hdot_max]`
  std::array<double, kCommandAxes> accel_max{};  ///< per axis, 0 = slew off
  double t_stale{0.0};
  double base_height{0.0};
  double h_min{0.0};
  double h_max{0.0};
};

/// @brief Parse the `command:` map into `out`. `out` is written only on
///        `kNoError`, which also implies @ref ValidateCommandFilterConfig passed.
[[nodiscard]] CommandFilterError LoadCommandFilterConfig(const YAML::Node& node,
                                                         CommandFilterConfig& out) noexcept;

/// @brief The value checks of the schema above (everything but key / type).
[[nodiscard]] CommandFilterError ValidateCommandFilterConfig(
    const CommandFilterConfig& cfg) noexcept;

/// @brief Twist-shaped raw command (no ROS dependency). `linear_z` is the base
///        height **rate** [m/s]; `angular_x` / `angular_y` are ignored.
struct RawVelocityCommand {
  double linear_x{0.0};   ///< v_x [m/s]
  double linear_y{0.0};   ///< v_y [m/s]
  double linear_z{0.0};   ///< ḣ [m/s]
  double angular_x{0.0};  ///< ignored
  double angular_y{0.0};  ///< ignored
  double angular_z{0.0};  ///< ω_z [rad/s]
};

/// @brief One filter output (trivially copyable — crosses to the RT side whole).
///        Counters count ticks since the last `Reset`.
struct FilteredCommand {
  double v_x{0.0};            ///< [m/s], |v_x| ≤ u_max[0]
  double v_y{0.0};            ///< [m/s], |v_y| ≤ u_max[1]
  double omega_z{0.0};        ///< [rad/s], |ω_z| ≤ u_max[2]
  double base_height{0.0};    ///< [m] in [h_min, h_max]
  std::int64_t stamp_ns{0};   ///< `now_ns` of the producing Step (producer clock, diagnostic only)
  std::uint64_t sequence{0};  ///< +1 per Step, 0 right after Reset
  std::uint64_t nonfinite_count{0};     ///< ticks with a non-finite used axis
  std::uint64_t stale_count{0};         ///< ticks the input was stale
  std::uint64_t ignored_axis_count{0};  ///< ticks with a non-zero angular.x / angular.y
  bool valid{false};  ///< a fresh sample arrived since the last Reset (latched — not "live")
  bool fresh{false};  ///< this tick's input was fresh; false ⇒ the output decays to rest
};

static_assert(std::is_trivially_copyable_v<FilteredCommand>);

/// @brief One slew step: move `prev` toward `target` by at most `max_step`
///        (`max_step ≤ 0` = off, returns `target`).
[[nodiscard]] constexpr double SlewLimit(double prev, double target, double max_step) noexcept {
  if (max_step <= 0.0) {
    return target;
  }
  const double d = target - prev;
  return prev + (d > max_step ? max_step : (d < -max_step ? -max_step : d));
}

/// @brief The filter (see file comment for the chain and the thread contract).
class CommandFilter {
 public:
  /// @brief Validate `cfg`, design the Bessel stage, then `Reset(cfg.base_height)`.
  ///        On failure the filter keeps its previous state.
  [[nodiscard]] CommandFilterError Init(const CommandFilterConfig& cfg) noexcept;

  /// @brief Zero the filter state and the accepted raw values, set the height
  ///        to `h0`, zero `sequence` and the counters, `valid = false`.
  [[nodiscard]] CommandFilterError Reset(double h0) noexcept;

  /// @brief Run one tick (see file comment). Before a successful `Init` it
  ///        returns the default output untouched.
  const FilteredCommand& Step(const RawVelocityCommand& raw, std::int64_t raw_stamp_ns,
                              std::int64_t now_ns) noexcept;

  [[nodiscard]] const FilteredCommand& output() const noexcept { return out_; }

  /// @brief The clamped, slew-limited sample the Bessel stage last saw (diagnostic).
  [[nodiscard]] const std::array<double, kCommandAxes>& limited_input() const noexcept {
    return limited_;
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

  [[nodiscard]] const CommandFilterConfig& config() const noexcept { return cfg_; }

 private:
  CommandFilterConfig cfg_{};
  rtc::BesselFilterN<kCommandAxes> bessel_{};
  std::array<double, kCommandAxes> accepted_{};  ///< last finite raw per axis
  std::array<double, kCommandAxes> limited_{};   ///< Bessel input (after clamp + slew)
  std::int64_t t_stale_ns_{0};
  FilteredCommand out_{};
  bool initialized_{false};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_COMMAND_FILTER_HPP_
