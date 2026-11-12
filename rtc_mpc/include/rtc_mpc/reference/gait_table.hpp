#ifndef RTC_MPC_REFERENCE_GAIT_TABLE_HPP_
#define RTC_MPC_REFERENCE_GAIT_TABLE_HPP_

/// @file gait_table.hpp
/// @brief Gait templates as contact bitmasks, baked from YAML into fixed arrays.
///
/// A gait is a periodic sequence of contact modes. A mode is a bitmask over the
/// contacts in the **caller's order** (bit i = contact i of the list passed to
/// @ref LoadGaitTable, normally the model handler's contact order). No side or
/// frame name lives here (ARCH-1); YAML names the contacts, the caller maps
/// names to bits. Mode 0 (no contact, flight) is allowed.
///
/// The table also carries the velocity thresholds of the automatic transition
/// policy run by @ref GaitScheduler (the formulas are on @ref GaitTemplate).
/// Gaits are ordered slowest first by `sequence`.
///
/// YAML (`gait:`, every key required, an unknown key refused; units and
/// ranges in rtc_mpc/config/reference_default.yaml):
/// ```yaml
/// sequence: [stance, walk]   # slowest → fastest, each gait once
/// min_dwell: 0.2             # [s] ≥ 0
/// switch_lead: 0.6           # [s] ≥ 0
/// gaits:
///   stance:
///     modes: [[contact_a, contact_b]]   # contact names per phase, [] = flight
///     switching_times: [0.0, 0.5]       # [s] 0, increasing, last = period
///     faster_above: [0.1, 0.1]          # [m/s, rad/s] ≥ 0
///     slower_below: [0.0, 0.0]          # [m/s, rad/s] ≥ 0
///     measured_tolerance: [10.0, 10.0]  # [m/s, rad/s] ≥ 0
/// ```
/// `faster_above` of each gait must exceed `slower_below` of the next one on both
/// axes (else the two would chatter — refused as a hysteresis inversion).
///
/// Loading allocates (non-RT, configure time); the baked table is trivially
/// copyable and the scheduler reads only its arrays.

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

namespace rtc::mpc {

inline constexpr std::size_t kMaxGaits = 8;
inline constexpr std::size_t kMaxGaitPhases = 8;
inline constexpr std::size_t kMaxContactBits = 32;  ///< a mode is a uint32_t

/// @brief Threshold axes: 0 = linear (applies to |v_x| and |v_y|) [m/s],
///        1 = yaw rate [rad/s].
inline constexpr std::size_t kLinearAxis = 0;
inline constexpr std::size_t kAngularAxis = 1;

/// @brief One gait. Phase j has `mode[j]` on `[switching_time[j],
///        switching_time[j+1])` of each period.
///
/// Transition policy on this gait, with `c` the command and `m` the measured
/// base velocity (axes v_x, v_y on the linear thresholds, ω_z on the angular):
/// - faster (to the next gait) when **any** `|c_i| > faster_above_i` and
///   **any** `|m_i| > faster_above_i − measured_tolerance_i`;
/// - slower (to the previous gait) when **every** `|c_i| < slower_below_i` and
///   **every** `|m_i| < slower_below_i + measured_tolerance_i`.
struct GaitTemplate {
  std::array<std::uint32_t, kMaxGaitPhases> mode{};
  std::array<double, kMaxGaitPhases + 1> switching_time{};  ///< `[0, …, period]`
  std::array<double, 2> faster_above{};
  std::array<double, 2> slower_below{};
  std::array<double, 2> measured_tolerance{};
  double period{0.0};  ///< == `switching_time[n_phases]`
  std::uint32_t n_phases{0};
};

/// @brief The OR of every phase mode — the gait's full support.
[[nodiscard]] constexpr std::uint32_t ModeUnion(const GaitTemplate& g) noexcept {
  std::uint32_t u = 0;
  for (std::size_t j = 0; j < g.n_phases && j < kMaxGaitPhases; ++j) {
    u |= g.mode[j];
  }
  return u;
}

/// @brief The baked table (gaits in `sequence` order).
struct GaitTable {
  std::array<GaitTemplate, kMaxGaits> gaits{};
  double min_dwell{0.0};    ///< [s]
  double switch_lead{0.0};  ///< [s]
  std::uint32_t n_gaits{0};
  std::uint32_t n_contacts{0};
};

static_assert(std::is_trivially_copyable_v<GaitTable>);

enum class GaitTableError {
  kNoError = 0,
  kNotAMap,              ///< `gait:` (or a gait entry) absent or not a map
  kUnknownKey,           ///< a key outside the schema
  kMissingKey,           ///< a schema key without a value
  kWrongType,            ///< a value of the wrong kind or sequence length
  kNonFinite,            ///< a NaN / ±inf value
  kNegativeValue,        ///< a threshold, `min_dwell` or `switch_lead` `< 0`
  kBadContactList,       ///< no contact, more than kMaxContactBits, or a name twice
  kUnknownContact,       ///< a mode names a contact outside the caller's list
  kRepeatedContact,      ///< a mode names the same contact twice
  kBadSequence,          ///< empty, a gait twice, a gait undefined or defined but unlisted
  kTooManyGaits,         ///< more than kMaxGaits
  kNoPhase,              ///< a gait with no mode
  kTooManyPhases,        ///< more than kMaxGaitPhases
  kLengthMismatch,       ///< `switching_times` is not one longer than `modes`
  kBadSwitchingTimes,    ///< first ≠ 0, not strictly increasing, or last ≠ period
  kModeOutOfRange,       ///< a mode bit at or above `n_contacts`
  kHysteresisInversion,  ///< `faster_above` of a gait ≤ `slower_below` of the next
};

/// @brief Parse `gait:` with `contact_names` giving bit order. `out` (and
///        `gait_names`, in sequence order) are written only on `kNoError`.
[[nodiscard]] GaitTableError LoadGaitTable(const YAML::Node& node,
                                           std::span<const std::string> contact_names,
                                           GaitTable& out,
                                           std::vector<std::string>* gait_names = nullptr) noexcept;

/// @brief The value checks of a baked table (for one built in code).
[[nodiscard]] GaitTableError ValidateGaitTable(const GaitTable& table) noexcept;

/// @brief Shortest phase over every gait [s] (0 for an empty table).
[[nodiscard]] double MinPhase(const GaitTable& table) noexcept;

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_GAIT_TABLE_HPP_
