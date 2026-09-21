// ── Shared YAML READING for the two `catching:` parsers (private) ───────────
// `catching_params.cpp` (ParseCatchingParams, the G0-C subset) and
// `catch_pose_ik_params.cpp` (ParseCatchPoseIkParams, `planner.ik.*` +
// `planner.catchability.*`) read the same tree, and overlap on one key. They
// used to carry a private copy each of the helpers below, which is how two
// parsers of one tree come to disagree about what a node MEANS (P5: generalise,
// never fork).
//
// What is shared is strictly how a node is READ — which YAML shapes count as
// "absent", which scalar is the `TBD` literal, that a non-finite number is
// still-TBD (L0 §5.3). What is NOT shared, on purpose, is how a problem is
// REPORTED: each helper returns a classification and never throws, so each
// parser keeps its own wording and its own mechanism (ParseCatchingParams
// defers a range violation to ValidateCatchingParams' report;
// ParseCatchPoseIkParams throws). Nothing here knows a key name, a range or a
// message prefix.
//
// Private to `src/params/`: never installed, included by relative path.
#pragma once

#include "rtc_controllers/catching/catching_params.hpp"  // TbdDouble

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <string>

namespace rtc::catching::params_detail {

// ── Section access ──────────────────────────────────────────────────────────

enum class SectionKind {
  kAbsent,   ///< parent absent/not a map, or `key` absent, or an empty `key:`
  kMap,      ///< `key` is a map — `node` is it
  kNotAMap,  ///< `key` is present but is a scalar/sequence — the caller refuses
};

struct SectionRead {
  SectionKind kind{SectionKind::kAbsent};
  /// The section for `kMap`; otherwise an EMPTY node, so that reading a key
  /// under an absent section yields that key's default. Subscript it only
  /// through a CONST reference: the const overload of `operator[]` hands back
  /// an undefined (falsy) node, whereas a default-constructed `YAML::Node` is
  /// itself DEFINED. Going through this rather than `parent[key][...]` also
  /// matters: yaml-cpp throws `YAML::InvalidNode`, not `std::invalid_argument`,
  /// when a missing node is subscripted.
  YAML::Node node{};
};

[[nodiscard]] inline SectionRead ReadSectionNode(const YAML::Node& parent, const char* key) {
  if (!parent.IsMap()) {
    return {};  // the parent section itself was absent
  }
  const YAML::Node child = parent[key];
  if (!child || child.IsNull()) {
    return {};
  }
  if (!child.IsMap()) {
    return {SectionKind::kNotAMap, YAML::Node()};
  }
  return {SectionKind::kMap, child};
}

// ── TBD-capable double ──────────────────────────────────────────────────────

enum class TbdReadKind {
  kAbsent,      ///< no such key — the caller applies its fallback
  kRead,        ///< `value` holds the result (resolved, literal TBD, or NaN-TBD)
  kNotANumber,  ///< present, neither a number nor the literal `TBD` — refused
};

struct TbdRead {
  TbdReadKind kind{TbdReadKind::kAbsent};
  TbdDouble value{};
  YAML::Node node{};  ///< the key's node when present, for the caller's message
};

/// Read a scalar the schema may leave open: the literal string `TBD`, or (per
/// L0 §5.3) a non-finite number, both come back as `tbd = true`. A finite
/// number comes back resolved and UNCHECKED — range is the caller's business.
[[nodiscard]] inline TbdRead ReadTbdScalar(const YAML::Node& sec, const char* key) {
  const YAML::Node v = sec[key];
  if (!v) {
    return {};
  }
  if (v.IsScalar() && v.Scalar() == "TBD") {
    return {TbdReadKind::kRead, TbdDouble{}, v};  // NaN, tbd = true
  }
  double d = std::numeric_limits<double>::quiet_NaN();
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    return {TbdReadKind::kNotANumber, TbdDouble{}, v};
  }
  if (!std::isfinite(d)) {
    return {TbdReadKind::kRead, TbdDouble{d, true}, v};  // NaN sentinel: still TBD (L0 §5.3)
  }
  return {TbdReadKind::kRead, TbdDouble{d, false}, v};
}

// ── Message support ─────────────────────────────────────────────────────────

/// The value as the YAML file spells it, for a message that quotes what was
/// written rather than a reformatted double (1e-10 through `std::to_string` is
/// "0.000000", which reads as a different complaint than the one being made).
[[nodiscard]] inline std::string Spelling(const YAML::Node& v) {
  if (v.IsScalar()) {
    return "'" + v.Scalar() + "'";
  }
  if (v.IsSequence()) {
    return "a sequence";
  }
  if (v.IsMap()) {
    return "a map";
  }
  return "an empty value";
}

}  // namespace rtc::catching::params_detail
