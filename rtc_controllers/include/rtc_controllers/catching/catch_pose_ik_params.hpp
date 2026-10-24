// ── Catch-pose IK YAML schema ────────────────────────────────────────────────
// The YAML parser for `CatchPoseIkOptions`. It reads `planner.search.grid.ik.*`
// plus the two `planner.search.grid.catchability.*` keys that decide the gate,
// and produces exactly that struct. Non-RT: the same shape as its siblings
// under `params/` — POD + a free `ParseXxxParams`, yaml-cpp only, rclcpp-free,
// called once at configure time.
//
// TWO CONSUMERS, ONE FUNCTION. The offline catchability map and the runtime
// planner must reach a bit-identical q* from the same YAML (catch_pose_ik.hpp),
// which they can only do if they resolve the options the same way — so this is
// ordinary public API of the package, not a helper of either caller.
//
// ── Why one function and one aggregate, not two ─────────────────────────────
// `planner.search.grid.catchability.definition` SELECTS which of the two thresholds
// `CatchPoseIkOptions::manipulability_min` has to carry (they are different
// quantities in different units). Handing the thresholds back from a second,
// independent call would let a caller pair an `arm_6row` definition with the
// `arm_5row` number and get a plausible, wrong gate. Binding the two here means
// that pairing cannot be expressed.
//
// ── Scope, and the sibling this is NOT part of ──────────────────────────────
// `catching_params.hpp` deliberately declares `planner.search.grid.ik.*` out of its scope
// (its subset is the cross-layer constraint table). This is its sibling,
// not an extension of it: nothing here is added to `CatchingParams`. The two
// DO overlap on one key,
// `planner.search.grid.catchability.manipulability_min.arm_5row` (+ its invented
// `provisional` flag, whose rationale lives in params/catching_params.cpp) —
// unavoidable, because that key is both a cross-layer constraint and the gate
// threshold `CatchPoseIkOptions` must carry. Precisely what the two share and
// what they do not:
//   - SHARED — how the key is READ. Both go through the one private helper
//     (`params_detail::ReadTbdScalar`, src/params/catching_yaml_read.hpp): an
//     absent key takes the default, the literal `TBD` and a non-finite number
//     are still-TBD, a finite number is resolved, and anything else is refused
//     with `std::invalid_argument`. The default is the same number.
//   - NOT SHARED — how an OUT-OF-RANGE value is reported. `arm_5row: -0.1`
//     makes `ParseCatchPoseIkParams` THROW, because `options` must be ready to
//     hand to `Solve`; `ParseCatchingParams` PARSES it and
//     `ValidateCatchingParams` reports `kRangeViolation` against the key,
//     because that layer's contract is a full report rather than a first
//     failure. Both refuse the value; neither lets it run.
// The SharedKey* cases in test/test_catch_pose_ik_params.cpp pin both halves,
// so the documented difference cannot drift into an undocumented one.
//
// ── Keys, defaults, ranges ──────────────────────────────────────────────────
// Every field of `CatchPoseIkOptions` is wired to a key (including the
// central-difference step h of ∇log w₅, `planner.search.grid.ik.fd_step`,
// finite > 0). The key names, units, defaults and ranges are NOT restated here
// (AP-DOC-1): each struct field names its own key, unit and default, and the
// range checks in params/catch_pose_ik_params.cpp are the code's single copy
// of the ranges. The defaults case in test/test_catch_pose_ik_params.cpp
// asserts field by field that an absent section resolves to the struct's own
// values. `alpha_max` defaults to the provisional 0.26 rad (≈15°) and stays a
// `TbdDouble` (CatchPoseIkConfig::alpha_max) until evidence closes it.
//
// ── Rejected, defaulted, reported: three different things ───────────────────
//   - REJECTED (`std::invalid_argument`): a wrong type, a value outside its
//     range, an unparseable scalar, an unknown key under `planner.search.grid.ik`.
//     The typo case is deliberate, and stricter than a permissive parser: a
//     misspelt key under a section this parser wholly OWNS can only be a typo,
//     and defaulting it would silently run the map with a tuning value the
//     YAML plainly meant to change. (`planner.*` at large is NOT policed that
//     way — that tree holds ~30 keys belonging to other consumers, so an
//     unknown-key sweep there would reject other people's configuration.)
//   - DEFAULTED: an absent key, and an absent `planner.search.grid.ik` / `planner` /
//     `planner.search.grid.catchability` section. The result then equals the in-code
//     `CatchPoseIkOptions` default for every key, `alpha_max` included.
//   - REPORTED: the two retired keys (`planner.search.grid.ik.lambda`,
//     `planner.search.grid.ik.manip_min`). A deployed config may still carry them and
//     their meaning is known, so they are not typos: they are handed back in
//     `CatchPoseIkRetiredKeys` for the caller to log, which is the convention
//     the rest of `params/` uses (rtc_controllers/README.md `params/`).
#pragma once

#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catching_params.hpp"

#include <yaml-cpp/yaml.h>

#include <string_view>

namespace rtc::catching {

/// Retired keys that a deployed config may still carry. `lambda` was replaced
/// by the σ_min-adaptive pair `sigma0`/`lambda_max` and `manip_min` by
/// `planner.search.grid.catchability.manipulability_min`.
struct CatchPoseIkRetiredKeys {
  bool lambda{false};     ///< `planner.search.grid.ik.lambda` — superseded by sigma0 + lambda_max
  bool manip_min{false};  ///< `planner.search.grid.ik.manip_min` — superseded by catchability.*
};

/// Everything the `planner.search.grid.ik.*` / `planner.search.grid.catchability.*` keys
/// resolve to: the tuning struct `CatchPoseIk::Solve` takes, plus the TBD-capable record of the
/// keys that may still be open.
///
/// `options` is what runs. The `TbdDouble` members are the PARSE RECORD of the
/// keys that are (or may be) still open — `options` cannot express "still
/// TBD", so a caller that has to distinguish a measured value from a
/// provisional stand-in reads them, and a validator that has to refuse an
/// unfilled active key reports them.
///
/// The two TBD keys fail in different directions on purpose:
///
///   - `alpha_max` TBD leaves `options.alpha_max` at its in-code provisional
///     value (≈15°, declared as such in catch_pose_ik.hpp). The number is a
///     stand-in nobody has closed with
///     evidence yet; `alpha_max.tbd` is what says the config did not decide it.
///   - A TBD ACTIVE threshold instead makes `options.manipulability_min`
///     non-finite, so `CatchPoseIk::Solve` refuses the options outright
///     (`kOptionsInvalid`). There is no provisional stand-in to fall back on:
///     w₅ and w₆ are mixed-unit quantities of DIFFERENT dimension, which is
///     why each has its own key, so the
///     struct's `arm_5row` number is not a conservative substitute for an
///     `arm_6row` gate — it is a number about a different quantity.
///     Fail-closed is the only reading that cannot be mistaken for an answer.
///     This can only happen when the YAML says `TBD` outright or selects
///     `arm_6row`, whose default IS TBD — an absent section still
///     resolves to the struct default (arm_5row, 0.1).
struct CatchPoseIkConfig {
  /// Ready to hand to `CatchPoseIk::Solve`. Absent keys keep their in-code
  /// defaults.
  CatchPoseIkOptions options{};

  /// `planner.search.grid.ik.alpha_max` [rad], 0–π/2. Default 0.26 (provisional),
  /// which is `CatchPoseIkOptions::alpha_max`'s own default. This stays a
  /// `TbdDouble` — default-constructed, i.e. "not given" — because an explicit
  /// `TBD` in a config must remain representable, and an absent key must stay
  /// distinguishable from a decided one.
  TbdDouble alpha_max{};

  /// `planner.search.grid.catchability.manipulability_min.arm_5row`, ≥ 0. The default is
  /// read off `CatchPoseIkOptions` rather than repeating 0.1: the struct's
  /// `manipulability_min` default is that row (its `definition` default is
  /// `arm_5row`), so there is one number, not two that can drift.
  TbdDouble manipulability_min_arm_5row{
      TbdDouble::Resolved(CatchPoseIkOptions{}.manipulability_min)};

  /// `planner.search.grid.catchability.manipulability_min.arm_6row`, ≥ 0. Default
  /// `TBD` — the value is to be proposed from the offline catchability map itself.
  TbdDouble manipulability_min_arm_6row{};

  /// `planner.search.grid.catchability.manipulability_min.provisional` — the invented key
  /// params/catching_params.cpp owns and documents. Fail-closed default: true.
  bool manipulability_min_provisional{true};
};

/// The threshold `options.definition` selects, as a TBD-capable value.
///
/// `options.manipulability_min` is the same number once resolved; this is how
/// a caller asks whether it was resolved AT ALL, without re-deriving which row
/// the definition picked.
[[nodiscard]] inline const TbdDouble& ActiveManipulabilityMin(
    const CatchPoseIkConfig& cfg) noexcept {
  return cfg.options.definition == ManipDefinition::kArm6Row ? cfg.manipulability_min_arm_6row
                                                             : cfg.manipulability_min_arm_5row;
}

/// Parse `planner.search.grid.ik.*` + `planner.search.grid.catchability.*` out of the `catching:`
/// map.
///
/// @param node the `catching:` tree root — the SAME node `ParseCatchingParams`
///        takes, so one config load feeds both. Must be a map.
/// @param retired optional; receives which retired keys were present.
/// @param search_map the search whose keys these are, as the name of its map
///        under `planner.search` — the same sub-schema (`ik.*`,
///        `catchability.*`) is read from `planner.search.<search_map>`, and
///        every message names its paths there. The defaults and ranges do not
///        depend on it.
///
/// Throws `std::invalid_argument`, and only that (every `YAML::Exception` is
/// translated), on: a missing/non-map root; a present `planner`,
/// `planner.search.grid.ik`, `planner.search.grid.catchability` or `...manipulability_min` that is
/// not a map; a value of the wrong YAML type; a value outside its range; an unknown key under
/// `planner.search.grid.ik`; a `definition` that is neither
/// `"arm_5row"` nor `"arm_6row"`. Every message names the full dotted key
/// path. Non-RT: called from LoadConfig / on_configure or from an offline
/// tool, never from a tick.
[[nodiscard]] CatchPoseIkConfig ParseCatchPoseIkParams(const YAML::Node& node,
                                                       CatchPoseIkRetiredKeys* retired = nullptr,
                                                       std::string_view search_map = "grid");

}  // namespace rtc::catching
