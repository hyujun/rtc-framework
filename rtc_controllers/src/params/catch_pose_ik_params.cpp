#include "rtc_controllers/catching/catch_pose_ik_params.hpp"

#include "catching_yaml_read.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc::catching {

namespace {

// ── The L3 §6 ranges, in code, once ─────────────────────────────────────────
// This block is the code's single copy of the table's 범위 column for the rows
// this parser owns; the header deliberately does not restate it (AP-DOC-1).
// `hi` is inclusive; `lo_open` distinguishes the table's ">0" rows from its
// "≥0" rows, which is a real difference here — `lambda_max`, `k_null`,
// `k_manip` and `manip_grad_tol` are legitimately 0 (k_manip = 0 is what
// recovers the pre-D-25 behaviour) while `mu` = 0 makes the task QP's Hessian
// singular and `v_eps` = 0 removes the NUM-7 speed floor entirely.
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPiOver2 = 1.5707963267948966;  // π/2, the alpha_max upper bound

struct Range {
  double lo{0.0};
  double hi{kInf};
  bool lo_open{false};  ///< true for ">lo", false for "≥lo"
};

constexpr Range kPositive{0.0, kInf, true};      // ">0"
constexpr Range kNonNegative{0.0, kInf, false};  // "≥0"

/// `planner.ik.eps_pos` is the one row whose 범위 column is "–". The bound used
/// here is not invented: `OptionsUsable` in catching/catch_pose_ik.cpp — the
/// only other authority on a usable value — requires `eps_pos > 0`, and a
/// non-positive acceptance tolerance would make the IK unable to accept any
/// pose at all. Nothing narrower is imposed, so every value the doc leaves
/// open still parses.
constexpr Range kEpsPos = kPositive;

[[noreturn]] void RejectMsg(const std::string& msg) {
  throw std::invalid_argument("catching: " + msg);
}

/// Full dotted path of a `planner.ik` key, for every message below.
std::string IkKey(const char* key) {
  return std::string("planner.ik.") + key;
}

using params_detail::Spelling;

/// Child section `key` of `parent`, named by its full dotted `path` in any
/// rejection. An absent (or empty `key:`) section reads as an empty node, so
/// every key under it takes its in-code default; a present section that is not
/// a map is refused. The READING is `params_detail::ReadSectionNode`, shared
/// with catching_params.cpp; only the refusal's wording is this parser's.
YAML::Node ReadSection(const YAML::Node& parent, const char* key, const std::string& path) {
  const params_detail::SectionRead sec = params_detail::ReadSectionNode(parent, key);
  if (sec.kind == params_detail::SectionKind::kNotAMap) {
    RejectMsg("section '" + path + "' must be a map");
  }
  return sec.node;
}

void CheckRange(const std::string& path, const YAML::Node& v, double d, const Range& r) {
  const bool lo_ok = r.lo_open ? (d > r.lo) : (d >= r.lo);
  if (!std::isfinite(d) || !lo_ok || d > r.hi) {
    RejectMsg("'" + path + "' = " + Spelling(v) + " is outside the L3 §6 range " +
              (r.lo_open ? "(" : "[") + std::to_string(r.lo) + ", " +
              (r.hi == kInf ? std::string("inf") : std::to_string(r.hi)) + "]");
  }
}

/// Read one double key. An absent key returns `fallback` untouched; a present
/// key must be a scalar that parses as a finite number inside `r`.
///
/// `sec` may be the empty node `ReadSection` returns for an absent section.
/// Subscripting it through a CONST reference is what makes that work: the
/// const overload hands back an undefined node (falsy), whereas an explicitly
/// default-constructed `YAML::Node` is DEFINED and would be fed to `as<>()`.
double ReadDouble(const YAML::Node& sec, const char* key, double fallback, const Range& r) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  const std::string path = IkKey(key);
  double d = std::numeric_limits<double>::quiet_NaN();
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    RejectMsg("'" + path + "' must be a number, got " + Spelling(v));
  }
  CheckRange(path, v, d, r);
  return d;
}

/// Read one int key. `as<int>()` requires the whole scalar to convert, so
/// "3.5" and "twenty" are both refused rather than truncated.
int ReadInt(const YAML::Node& sec, const char* key, int fallback, int lo, int hi) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  const std::string path = IkKey(key);
  int i = 0;
  try {
    i = v.as<int>();
  } catch (const YAML::Exception&) {
    RejectMsg("'" + path + "' must be an integer, got " + Spelling(v));
  }
  if (i < lo || i > hi) {
    RejectMsg("'" + path + "' = " + Spelling(v) + " is outside the L3 §6 range [" +
              std::to_string(lo) + ", " + std::to_string(hi) + "]");
  }
  return i;
}

/// Read a scalar the schema may leave open as the literal string "TBD" (or, per
/// L0 §5.3, an unparseable/non-finite number — treated the same way). The
/// READING is `params_detail::ReadTbdScalar`, the very function
/// `ReadTbdDouble` in params/catching_params.cpp uses, so the two parsers
/// cannot disagree about what a node means. What differs is what happens NEXT,
/// by design: this parser range-checks a resolved number and THROWS, whereas
/// ParseCatchingParams lets it through for ValidateCatchingParams to report.
TbdDouble ReadTbd(const YAML::Node& sec, const char* key, const std::string& path,
                  TbdDouble fallback, const Range& r) {
  const params_detail::TbdRead read = params_detail::ReadTbdScalar(sec, key);
  switch (read.kind) {
    case params_detail::TbdReadKind::kAbsent:
      return fallback;
    case params_detail::TbdReadKind::kNotANumber:
      RejectMsg("'" + path + "' must be a number or the literal 'TBD', got " + Spelling(read.node));
    case params_detail::TbdReadKind::kRead:
      break;
  }
  if (!read.value.tbd) {
    CheckRange(path, read.node, read.value.value, r);
  }
  return read.value;
}

bool ReadBool(const YAML::Node& sec, const char* key, const std::string& path, bool fallback) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  try {
    return v.as<bool>();
  } catch (const YAML::Exception&) {
    RejectMsg("'" + path + "' must be a bool, got " + Spelling(v));
  }
}

// ── Typo protection under `planner.ik` ──────────────────────────────────────
// This parser owns that section whole, so a key it does not know is a typo (or
// a key from a doc revision this build has not caught up with) and is refused.
// The list is the accept-list, not documentation: it must contain exactly the
// keys read below, which the UnknownKey cases in the test suite check from the
// outside by asserting that each known key is accepted.
constexpr const char* kIkKeys[] = {
    "max_iter", "eps_pos",     "alpha_max",   "rho",    "sigma0",  "lambda_max",
    "mu",       "qp_eps_abs",  "qp_max_iter", "k_null", "k_manip", "manip_grad_tol",
    "v_eps",    "dq_step_max",
};

/// Keys L3 §6 marks removed in v0.5. Present-but-retired is REPORTED, not
/// refused: the meaning is known, so a deployed config carrying one is a
/// migration case and the caller logs it (rtc_controllers/README.md `params/`).
void SweepIkKeys(const YAML::Node& ik, CatchPoseIkRetiredKeys& retired) {
  if (!ik.IsMap()) {
    return;
  }
  for (const auto& kv : ik) {
    if (!kv.first.IsScalar()) {
      RejectMsg("keys of section 'planner.ik' must be scalars");
    }
    const std::string name = kv.first.Scalar();
    if (name == "lambda") {
      retired.lambda = true;
      continue;
    }
    if (name == "manip_min") {
      retired.manip_min = true;
      continue;
    }
    bool known = false;
    for (const char* k : kIkKeys) {
      if (name == k) {
        known = true;
        break;
      }
    }
    if (!known) {
      RejectMsg("unknown key '" + IkKey(name.c_str()) +
                "' — L3 §6 defines no such parameter (typo?)");
    }
  }
}

}  // namespace

CatchPoseIkConfig ParseCatchPoseIkParams(const YAML::Node& node, CatchPoseIkRetiredKeys* retired) {
  if (retired != nullptr) {
    *retired = CatchPoseIkRetiredKeys{};
  }
  if (!node || !node.IsMap()) {
    RejectMsg("must be a map (the `catching:` tree, CATCHING_MASTER.md §6)");
  }
  CatchPoseIkConfig out;
  CatchPoseIkRetiredKeys local_retired;

  const YAML::Node planner = ReadSection(node, "planner", "planner");
  const YAML::Node ik = ReadSection(planner, "ik", "planner.ik");

  SweepIkKeys(ik, local_retired);
  if (retired != nullptr) {
    *retired = local_retired;
  }

  CatchPoseIkOptions& o = out.options;
  o.max_iter = ReadInt(ik, "max_iter", o.max_iter, 1, 100);
  o.eps_pos = ReadDouble(ik, "eps_pos", o.eps_pos, kEpsPos);
  o.rho = ReadDouble(ik, "rho", o.rho, Range{0.01, 1.0, false});
  o.sigma0 = ReadDouble(ik, "sigma0", o.sigma0, kPositive);
  o.lambda_max = ReadDouble(ik, "lambda_max", o.lambda_max, kNonNegative);
  o.dq_step_max = ReadDouble(ik, "dq_step_max", o.dq_step_max, kPositive);
  o.mu = ReadDouble(ik, "mu", o.mu, kPositive);
  o.qp_eps_abs = ReadDouble(ik, "qp_eps_abs", o.qp_eps_abs, kPositive);
  o.qp_max_iter = ReadInt(ik, "qp_max_iter", o.qp_max_iter, 1, std::numeric_limits<int>::max());
  o.k_null = ReadDouble(ik, "k_null", o.k_null, kNonNegative);
  o.k_manip = ReadDouble(ik, "k_manip", o.k_manip, kNonNegative);
  o.manip_grad_tol = ReadDouble(ik, "manip_grad_tol", o.manip_grad_tol, kNonNegative);
  o.v_eps = ReadDouble(ik, "v_eps", o.v_eps, kPositive);

  // alpha_max: L3 §6 records `0.26 (provisional)`, which is the struct's own
  // default (see the header's RESOLVED DISCREPANCY note). It is still read as a
  // TBD-capable value because an explicit `TBD` in a config must stay
  // representable: a TBD (or absent) key leaves the struct's provisional 0.26
  // in `options` and records the openness in `out.alpha_max`, so nothing here
  // turns a TBD into a decided number without saying so.
  out.alpha_max =
      ReadTbd(ik, "alpha_max", IkKey("alpha_max"), out.alpha_max, Range{0.0, kPiOver2, false});
  if (!out.alpha_max.tbd) {
    o.alpha_max = out.alpha_max.value;
  }

  // ── planner.catchability.* ────────────────────────────────────────────────
  const YAML::Node catchability = ReadSection(planner, "catchability", "planner.catchability");
  const YAML::Node manip_min =
      ReadSection(catchability, "manipulability_min", "planner.catchability.manipulability_min");
  out.manipulability_min_arm_5row =
      ReadTbd(manip_min, "arm_5row", "planner.catchability.manipulability_min.arm_5row",
              out.manipulability_min_arm_5row, kNonNegative);
  out.manipulability_min_arm_6row =
      ReadTbd(manip_min, "arm_6row", "planner.catchability.manipulability_min.arm_6row",
              out.manipulability_min_arm_6row, kNonNegative);
  out.manipulability_min_provisional = ReadBool(
      manip_min, "provisional", "planner.catchability.manipulability_min.provisional", true);

  if (const YAML::Node def = catchability["definition"]) {
    const std::string path = "planner.catchability.definition";
    if (!def.IsScalar()) {
      RejectMsg("'" + path + "' must be \"arm_5row\" or \"arm_6row\", got " + Spelling(def));
    }
    const std::string s = def.Scalar();
    if (s == "arm_5row") {
      o.definition = ManipDefinition::kArm5Row;
    } else if (s == "arm_6row") {
      o.definition = ManipDefinition::kArm6Row;
    } else {
      RejectMsg("'" + path + "' must be \"arm_5row\" or \"arm_6row\", got " + Spelling(def));
    }
  }

  // The gate threshold is the row `definition` selects. An unresolved one is
  // left non-finite rather than filled from the other row: the two are
  // different quantities in different units (L3 §6, catch_pose_ik.hpp note 2),
  // and `Solve` then refuses the options (kOptionsInvalid) instead of gating
  // against a number measured for the other definition. Under the defaults
  // this cannot fire — `arm_5row` resolves to the struct's own value.
  const TbdDouble& active = ActiveManipulabilityMin(out);
  o.manipulability_min = active.tbd ? std::numeric_limits<double>::quiet_NaN() : active.value;

  return out;
}

}  // namespace rtc::catching
