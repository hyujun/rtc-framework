// ── The TBD-capable scalar of the `catching:` parsers (private) ─────────────
// `catching_params.cpp` (ParseCatchingParams, the validator subset) and
// `catch_pose_ik_params.cpp` (ParseCatchPoseIkParams, `planner.search.grid.ik.*` +
// `planner.search.grid.catchability.*`) read the same tree, and overlap on one key. They
// used to carry a private copy each of the helper below, which is how two
// parsers of one tree come to disagree about what a node MEANS (P5: generalise,
// never fork).
//
// What is shared is how a node is READ: which scalar is the `TBD` literal, that a
// non-finite number is still-TBD. How a problem is REPORTED stays with each parser
// (ParseCatchingParams defers a range violation to ValidateCatchingParams' report;
// ParseCatchPoseIkParams throws). The generic readers — section access, unknown
// keys, spelling — are the public `rtc_controllers/params/yaml_read.hpp`
// (`rtc::params`); only the reader bound to `TbdDouble` lives here.
//
// Private to `src/params/`: never installed, included by relative path.
#pragma once

#include "rtc_controllers/catching/catching_params.hpp"  // TbdDouble

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>

namespace rtc::catching::params_detail {

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

/// Read a scalar the schema may leave open: the literal string `TBD`, or a
/// non-finite number, both come back as `tbd = true`. A finite
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
    return {TbdReadKind::kRead, TbdDouble{d, true}, v};  // NaN sentinel: still TBD
  }
  return {TbdReadKind::kRead, TbdDouble{d, false}, v};
}

}  // namespace rtc::catching::params_detail
