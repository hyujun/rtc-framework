// ── Dynamics feedforward configuration (G2 schema layer) ────────────────────
// The POD and the YAML schema behind joint/dynamics_feedforward_law.hpp. Same
// split as the sibling params/ files: the LAW takes spans and knows no YAML;
// this file is a plain POD plus a yaml-cpp parser a binding calls once at
// configure time (non-RT, may throw).
//
// The block is named by the BINDING (it passes the key for error messages),
// because what the block means is the binding's business: a controller that
// ships gravity alone calls its block `gravity_ff`, and this parser does not
// care. The Coriolis gain has no key here on purpose — the law carries the
// slot, the schema grows the key when a binding ships the term (the inertia
// gain is that case: `inertia_gain`, optional, default 0 = the term off).
#pragma once

#include <yaml-cpp/yaml.h>

#include <string_view>

namespace rtc::params {

/// The feedforward lane's configuration. `max_abs` has NO defensible default:
/// a torque bound that quietly defaults to "whatever the device rates" is the
/// bound a hand with a placeholder rating does not have — so the parser
/// requires the key.
struct DynamicsFeedforwardParams {
  bool enabled{true};        ///< false: the lane writes 0 N·m every tick
  double gravity_gain{1.0};  ///< k_g, dimensionless, [0, kDynamicsFeedforwardGainMax]
  /// k_M of the inertia term (M·q̈_ref), dimensionless, [0, kDynamicsFeedforwardGainMax].
  /// Default 0: a block without the key ships gravity alone.
  double inertia_gain{0.0};
  double max_abs{0.0};  ///< τ_max [N·m], > 0 and finite; REQUIRED
};

/// Largest gain (gravity or inertia) the schema accepts. Above 1 only for
/// tuning (the law is then over-compensating on purpose); 2 is where that
/// stops being tuning.
inline constexpr double kDynamicsFeedforwardGainMax = 2.0;

/// Parse the feedforward block into @p out. Keys present overwrite @p out's
/// seed; `max_abs` must be present.
///
/// @param block  the block node (e.g. `cfg["gravity_ff"]`); must be a map
/// @param key    the block's YAML key, for error messages only
/// @param out    seeded with the caller's defaults
///
/// @throws std::runtime_error naming @p key when the block is absent or not a
///         map, `max_abs` is absent / non-finite / ≤ 0, or `gain` /
///         `inertia_gain` is non-finite / outside [0, kDynamicsFeedforwardGainMax]. yaml-cpp's own
///         exceptions (also std::runtime_error) on a wrong scalar type.
///
/// Non-RT: configure time only.
void ParseDynamicsFeedforwardParams(const YAML::Node& block, std::string_view key,
                                    DynamicsFeedforwardParams& out);

}  // namespace rtc::params
