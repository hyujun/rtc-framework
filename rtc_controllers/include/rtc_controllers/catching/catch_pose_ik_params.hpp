// ── Catch-pose IK YAML schema (dynamic_catching S3.5a) ──────────────────────
// The parser `catch_pose_ik.hpp` said did not exist yet ("No YAML parser yet
// (Q4): the planner binding that gets one later has to produce exactly this
// struct"). It reads `planner.ik.*` plus the two `planner.catchability.*` keys
// that decide the gate, and produces exactly that struct. Non-RT: the same G2
// schema-layer shape as its siblings under `params/` — POD + a free
// `ParseXxxParams`, yaml-cpp only, rclcpp-free, called once at configure time.
//
// TWO CONSUMERS, ONE FUNCTION. The offline catchability map (S3.5a) and the
// runtime planner (S6.2) must reach a bit-identical q* from the same YAML
// (catch_pose_ik.hpp, plan §11), which they can only do if they resolve the
// options the same way — so this is ordinary public API of the package, not a
// helper of either caller.
//
// ── Why one function and one aggregate, not two ─────────────────────────────
// `planner.catchability.definition` SELECTS which of the two thresholds
// `CatchPoseIkOptions::manipulability_min` has to carry (they are different
// quantities in different units — L3 §6: "차원이 달라 따로 둔다"). Handing the
// thresholds back from a second, independent call would let a caller pair an
// `arm_6row` definition with the `arm_5row` number and get a plausible, wrong
// gate. Binding the two here means that pairing cannot be expressed.
//
// ── Scope, and the sibling this is NOT part of ──────────────────────────────
// `catching_params.hpp` deliberately declares `planner.ik.*` out of its scope
// (its G0-C subset is the cross-layer constraint table). This is its sibling,
// not an extension of it: nothing here is added to `CatchingParams`. The two
// DO overlap on one key,
// `planner.catchability.manipulability_min.arm_5row` (+ its invented
// `provisional` flag, whose rationale lives in params/catching_params.cpp) —
// unavoidable, because that key is both a G0-C cross-constraint and the gate
// threshold `CatchPoseIkOptions` must carry. Both read it the same way and
// with the same default, and the L3 §6 table is the single source they agree
// against.
//
// ── Fields of `CatchPoseIkOptions` NOT reachable from YAML ──────────────────
// L3 §6 names no key for them, and this parser does not invent one; they keep
// their in-code default and a caller that needs to move them sets the struct
// field directly:
//   - `fd_step` — the central-difference step h for ∇log w₅.
// Every other field of the struct has a row in L3 §6 and is wired. The key
// names, units, defaults and ranges are NOT restated here (AP-DOC-1): L3 §6 is
// the authority, the range checks in params/catch_pose_ik_params.cpp are the
// code's single copy of it, and each struct field already names its own key.
//
// RESOLVED DISCREPANCY (doc vs code default), the one this parser found:
//   - `planner.ik.alpha_max` — L3 §6 gave the default as `TBD` while
//     `CatchPoseIkOptions::alpha_max` was already 0.26 rad (≈15°, marked
//     provisional in its own comment). Reconciled toward the CODE on
//     2026-09-21: L3 §6 now records `0.26 (provisional)`, so the two agree and
//     nothing here changed. What is still open is the EVIDENCE that closes it —
//     the S3.5a map's `theta` distribution, i.e. whether this cone actually
//     binds — so `CatchPoseIkConfig::alpha_max` stays a `TbdDouble`: an
//     explicit `TBD` in a config is still representable and still does not
//     silently become a number.
// Every OTHER default in those L3 §6 rows agrees with the struct exactly, as
// of this parser's audit of the table. The numbers are not copied here to say
// so — the agreement is pinned instead by the defaults case in
// test/test_catch_pose_ik_params.cpp, which asserts field by field that an
// absent section resolves to the struct's own values.
//
// ── Rejected, defaulted, reported: three different things ───────────────────
//   - REJECTED (`std::invalid_argument`): a wrong type, a value outside the
//     L3 §6 range, an unparseable scalar, an unknown key under `planner.ik`.
//     The typo case is deliberate, and stricter than a permissive parser: a
//     misspelt key under a section this parser wholly OWNS can only be a typo,
//     and defaulting it would silently run the map with a tuning value the
//     YAML plainly meant to change. (`planner.*` at large is NOT policed that
//     way — L3 §6 gives that tree ~30 keys belonging to other consumers, so an
//     unknown-key sweep there would reject other people's configuration.)
//   - DEFAULTED: an absent key, and an absent `planner.ik` / `planner` /
//     `planner.catchability` section. The result then equals the in-code
//     `CatchPoseIkOptions` default, which is the L3 §6 default for every key
//     but `alpha_max` (see the DISCREPANCY note below).
//   - REPORTED: the two keys L3 §6 marks removed in v0.5 (`planner.ik.lambda`,
//     `planner.ik.manip_min`). A deployed config may still carry them and
//     their meaning is known, so they are not typos: they are handed back in
//     `CatchPoseIkRetiredKeys` for the caller to log, which is the convention
//     the rest of `params/` uses (rtc_controllers/README.md `params/`).
#pragma once

#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catching_params.hpp"

#include <yaml-cpp/yaml.h>

namespace rtc::catching {

/// Keys L3 §6 no longer honours but that a deployed config may still carry.
/// `lambda` was replaced by the σ_min-adaptive pair `sigma0`/`lambda_max`
/// (D-7d) and `manip_min` by `planner.catchability.manipulability_min` (§4.5).
struct CatchPoseIkRetiredKeys {
  bool lambda{false};     ///< `planner.ik.lambda` — superseded by sigma0 + lambda_max
  bool manip_min{false};  ///< `planner.ik.manip_min` — superseded by catchability.*
};

/// Everything the L3 §6 `planner.ik.*` / `planner.catchability.*` rows resolve
/// to: the tuning struct `CatchPoseIk::Solve` takes, plus the TBD-capable
/// record of the keys the docs leave open.
///
/// `options` is what runs. The `TbdDouble` members are the PARSE RECORD of the
/// keys L3 §6 leaves (or may leave) open — `options` cannot express "still
/// TBD", so a caller that has to distinguish a measured value from a
/// provisional stand-in reads them, and a validator that has to refuse an
/// unfilled active key reports them.
///
/// The two TBD keys fail in different directions on purpose:
///
///   - `alpha_max` TBD leaves `options.alpha_max` at its in-code provisional
///     value (≈15°, declared as such in catch_pose_ik.hpp). The code owns a
///     number here that the doc has not fixed; `alpha_max.tbd` is what says so.
///   - A TBD ACTIVE threshold instead makes `options.manipulability_min`
///     non-finite, so `CatchPoseIk::Solve` refuses the options outright
///     (`kOptionsInvalid`). There is no provisional stand-in to fall back on:
///     w₅ and w₆ are mixed-unit quantities of DIFFERENT dimension, which is
///     why L3 §6 keeps a row for each ("차원이 달라 따로 둔다", C-3), so the
///     struct's `arm_5row` number is not a conservative substitute for an
///     `arm_6row` gate — it is a number about a different quantity.
///     Fail-closed is the only reading that cannot be mistaken for an answer.
///     This can only happen when the YAML says `TBD` outright or selects
///     `arm_6row`, whose L3 §6 default IS TBD — an absent section still
///     resolves to the struct default (arm_5row, 0.1).
struct CatchPoseIkConfig {
  /// Ready to hand to `CatchPoseIk::Solve`. Absent keys keep their in-code
  /// defaults.
  CatchPoseIkOptions options{};

  /// `planner.ik.alpha_max` [rad], 0–π/2. L3 §6 default is `TBD`.
  TbdDouble alpha_max{};

  /// `planner.catchability.manipulability_min.arm_5row`, ≥ 0. The default is
  /// read off `CatchPoseIkOptions` rather than repeating 0.1: the struct's
  /// `manipulability_min` default is that row (its `definition` default is
  /// `arm_5row`), so there is one number, not two that can drift.
  TbdDouble manipulability_min_arm_5row{
      TbdDouble::Resolved(CatchPoseIkOptions{}.manipulability_min)};

  /// `planner.catchability.manipulability_min.arm_6row`, ≥ 0. L3 §6 default is
  /// `TBD` — the value is to be proposed from the S3.5a/b map itself.
  TbdDouble manipulability_min_arm_6row{};

  /// `planner.catchability.manipulability_min.provisional` — the invented key
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

/// Parse `planner.ik.*` + `planner.catchability.*` out of the `catching:` map.
///
/// @param node the `catching:` tree root — the SAME node `ParseCatchingParams`
///        takes, so one config load feeds both. Must be a map.
/// @param retired optional; receives which retired keys were present.
///
/// Throws `std::invalid_argument`, and only that (every `YAML::Exception` is
/// translated), on: a missing/non-map root; a present `planner`,
/// `planner.ik`, `planner.catchability` or `...manipulability_min` that is not
/// a map; a value of the wrong YAML type; a value outside its L3 §6 range; an
/// unknown key under `planner.ik`; a `definition` that is neither
/// `"arm_5row"` nor `"arm_6row"`. Every message names the full dotted key
/// path. Non-RT: called from LoadConfig / on_configure or from an offline
/// tool, never from a tick.
[[nodiscard]] CatchPoseIkConfig ParseCatchPoseIkParams(const YAML::Node& node,
                                                       CatchPoseIkRetiredKeys* retired = nullptr);

}  // namespace rtc::catching
