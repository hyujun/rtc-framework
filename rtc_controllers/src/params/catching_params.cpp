#include "rtc_controllers/catching/catching_params.hpp"

#include <rtc_base/types/types.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc::catching {

namespace {

// ── Invented YAML keys ───────────────────────────────────────────────────
// None of these are named by any dynamic_catching doc as a schema field —
// each place a doc marks a value "provisional" (L0 §5.3), it does so in
// prose or a table 근거 column, not with a boolean key, EXCEPT the catch
// frame (D-17, plan §10), which already has one (`provisional: true`) but
// lives in the robot's `urdf:` tree, a different YAML root than `catching:` —
// CheckCatchFrameProvisional (S2.3a) applies the same rule to it from the
// model config. For the three provisional groups this validator DOES own,
// the chosen key sits as a sibling of the group's other fields, mirroring
// the catch frame's own placement:
//   - `core.ball.provisional`                              (D-12 공 사양)
//   - `planner.catchability.manipulability_min.provisional` (D-18)
//   - `robot.hand.provisional`                              (L6 §6, whole profile)
// All three default to `true` (fail-closed: unconfirmed until the YAML says
// otherwise), matching how every other provisional flag in this repo defaults
// to blocking the real arm rather than trusting silence.

template <typename... Parts>
[[noreturn]] void Reject(Parts&&... parts) {
  std::string msg = "catching: ";
  (msg.append(std::forward<Parts>(parts)), ...);
  throw std::invalid_argument(msg);
}

template <typename T>
T ReadOptional(const YAML::Node& node, const char* key, T fallback) {
  const YAML::Node v = node[key];
  if (!v) {
    return fallback;
  }
  try {
    return v.as<T>();
  } catch (const YAML::Exception&) {
    Reject("'", key, "' is present but does not parse");
  }
}

/// Child section `key` of `parent`. An absent (or empty `key:`) section reads
/// as an empty node, so every key under it takes its doc default — itself TBD
/// where the doc leaves it open, which the validator then refuses in the
/// active configuration (fail-closed). A present section that is not a map is
/// refused. Going through this helper rather than `parent[key][...]` matters:
/// yaml-cpp throws `YAML::InvalidNode`, not `std::invalid_argument`, when a
/// missing node is subscripted.
YAML::Node ReadSection(const YAML::Node& parent, const char* key) {
  if (!parent.IsMap()) {
    return YAML::Node();  // the parent section itself was absent
  }
  const YAML::Node child = parent[key];
  if (!child || child.IsNull()) {
    return YAML::Node();
  }
  if (!child.IsMap()) {
    Reject("section '", key, "' must be a map");
  }
  return child;
}

/// Read a scalar that may be the literal string "TBD" (or, per L0 §5.3, an
/// unparseable/non-finite number — treated the same way). An absent key takes
/// `fallback` (the doc default, itself possibly still-TBD); a present key
/// that is neither a number nor "TBD" is refused.
TbdDouble ReadTbdDouble(const YAML::Node& node, const char* key, TbdDouble fallback) {
  const YAML::Node v = node[key];
  if (!v) {
    return fallback;
  }
  if (v.IsScalar() && v.Scalar() == "TBD") {
    return TbdDouble{};  // NaN, tbd = true
  }
  double d = std::numeric_limits<double>::quiet_NaN();
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    Reject("'", key, "' is present but is neither a number nor the literal 'TBD'");
  }
  if (!std::isfinite(d)) {
    return TbdDouble{d, true};  // NaN sentinel: still TBD (L0 §5.3)
  }
  return TbdDouble{d, false};
}

/// Parse `robot.hand.q_pre`/`q_close` (+ optional `caging_mask`). Both arrays
/// must be TBD together or given together — a doc has no notion of "half a
/// hand profile" (L6 §6). `caging_mask` defaults to "every joint is checked"
/// when absent, the fail-closed reading of L6 §4.2 (it names no default).
HandProfile ReadHandProfile(const YAML::Node& hand_node) {
  HandProfile out;
  out.rho_eps = ReadOptional(hand_node, "rho_eps", out.rho_eps);
  out.provisional = ReadOptional(hand_node, "provisional", true);

  const YAML::Node q_pre = hand_node["q_pre"];
  const YAML::Node q_close = hand_node["q_close"];
  const bool pre_is_seq = q_pre && q_pre.IsSequence();
  const bool close_is_seq = q_close && q_close.IsSequence();

  if (!pre_is_seq && !close_is_seq) {
    out.tbd = true;
    out.dof = 0;
    return out;
  }
  if (pre_is_seq != close_is_seq) {
    Reject("robot.hand.q_pre and q_close must both be 'TBD' or both given as arrays");
  }
  if (q_pre.size() != q_close.size()) {
    Reject("robot.hand.q_pre and q_close must have the same length");
  }
  if (q_pre.size() == 0) {
    Reject("robot.hand.q_pre/q_close must not be empty (use 'TBD' for an unset profile)");
  }
  if (q_pre.size() > kMaxHandDof) {
    Reject("robot.hand.q_pre/q_close length exceeds kMaxHandDof (", std::to_string(kMaxHandDof),
           ")");
  }
  const auto dof = static_cast<std::size_t>(q_pre.size());

  const YAML::Node caging = hand_node["caging_mask"];
  const bool caging_is_seq = caging && caging.IsSequence();
  if (caging && !caging_is_seq) {
    Reject("robot.hand.caging_mask must be a sequence when present");
  }
  if (caging_is_seq && caging.size() != dof) {
    Reject("robot.hand.caging_mask must have the same length as q_pre/q_close");
  }

  for (std::size_t i = 0; i < dof; ++i) {
    double pre = std::numeric_limits<double>::quiet_NaN();
    double close = std::numeric_limits<double>::quiet_NaN();
    try {
      pre = q_pre[i].as<double>();
      close = q_close[i].as<double>();
    } catch (const YAML::Exception&) {
      Reject("robot.hand.q_pre/q_close[", std::to_string(i), "] does not parse as a number");
    }
    if (!std::isfinite(pre) || !std::isfinite(close)) {
      Reject("robot.hand.q_pre/q_close[", std::to_string(i), "] must be finite");
    }
    out.q_pre[i] = pre;
    out.q_close[i] = close;
    if (caging_is_seq) {
      try {
        out.caging_mask[i] = caging[i].as<bool>();
      } catch (const YAML::Exception&) {
        Reject("robot.hand.caging_mask[", std::to_string(i), "] does not parse as a bool");
      }
    } else {
      out.caging_mask[i] = true;  // fail-closed default: check every joint
    }
  }
  out.dof = static_cast<int>(dof);
  out.tbd = false;
  return out;
}

BallSpec ReadBallSpec(const YAML::Node& ball_node) {
  BallSpec out;
  out.diameter = ReadTbdDouble(ball_node, "diameter", out.diameter);
  out.mass = ReadTbdDouble(ball_node, "mass", out.mass);
  out.restitution = ReadTbdDouble(ball_node, "restitution", out.restitution);
  out.provisional = ReadOptional(ball_node, "provisional", true);
  return out;
}

}  // namespace

CatchingParams ParseCatchingParams(const YAML::Node& node) {
  if (!node || !node.IsMap()) {
    Reject("must be a map (the `catching:` tree, CATCHING_MASTER.md §6)");
  }
  CatchingParams out;

  const YAML::Node reference = ReadSection(node, "reference");
  out.reference_omega = ReadTbdDouble(reference, "omega", out.reference_omega);
  out.reference_zeta = ReadTbdDouble(reference, "zeta", out.reference_zeta);
  out.reference_v_max = ReadTbdDouble(reference, "v_max", out.reference_v_max);
  out.reference_a_max = ReadTbdDouble(reference, "a_max", out.reference_a_max);

  const YAML::Node planner = ReadSection(node, "planner");
  const YAML::Node gamma = ReadSection(planner, "gamma");
  out.planner_gamma_eta_v = ReadTbdDouble(gamma, "eta_v", out.planner_gamma_eta_v);

  const YAML::Node catchability = ReadSection(planner, "catchability");
  const YAML::Node manip_min = ReadSection(catchability, "manipulability_min");
  out.planner_catchability_manip_min_arm5row =
      ReadTbdDouble(manip_min, "arm_5row", out.planner_catchability_manip_min_arm5row);
  out.planner_catchability_manip_min_provisional = ReadOptional(manip_min, "provisional", true);

  const YAML::Node supervisor = ReadSection(node, "supervisor");
  const YAML::Node decel = ReadSection(supervisor, "decel");
  out.supervisor_decel_a_dec = ReadTbdDouble(decel, "a_dec", out.supervisor_decel_a_dec);

  const YAML::Node core = ReadSection(node, "core");
  out.ball = ReadBallSpec(ReadSection(core, "ball"));

  const YAML::Node sim = ReadSection(node, "sim");
  out.sim_ball_drag_k = ReadTbdDouble(ReadSection(sim, "ball"), "drag_k", out.sim_ball_drag_k);

  const YAML::Node robot = ReadSection(node, "robot");
  out.hand = ReadHandProfile(ReadSection(robot, "hand"));

  return out;
}

namespace {

// ── Validation ────────────────────────────────────────────────────────────
// L4 §4.7 discrete stability boundary s = omega*h: unstable at/above
// 2*sqrt(2) - 2, accuracy recommendation is s <= 0.05. Written as literals
// (not std::sqrt) so the bound stays a compile-time constant without relying
// on a constexpr math function the standard does not guarantee pre-C++23.
inline constexpr double kSqrt2 = 1.4142135623730951;
inline constexpr double kDiscreteStabilityLimit = 2.0 * kSqrt2 - 2.0;  // ~0.8284271 (L4 §4.7)
inline constexpr double kDiscreteAccuracyWarnLimit = 0.05;             // L4 §4.7

void AddFailure(CatchingValidationReport& report, CatchingValidationReason reason, const char* key,
                int index = -1) noexcept {
  report.armable = false;
  if (report.failure_count < CatchingValidationReport::kMaxFailures) {
    report.failures[report.failure_count++] = CatchingValidationEntry{reason, key, index};
  }
}

void AddWarning(CatchingValidationReport& report, CatchingValidationReason reason, const char* key,
                int index = -1) noexcept {
  if (report.warning_count < CatchingValidationReport::kMaxWarnings) {
    report.warnings[report.warning_count++] = CatchingValidationEntry{reason, key, index};
  }
}

/// Active-config TBD gate (G0-C). Returns true if the caller should go on to
/// check `v.value` (i.e. `v` is active and resolved).
bool CheckActiveTbd(CatchingValidationReport& report, const TbdDouble& v, const char* key,
                    bool active) noexcept {
  if (!active) {
    return false;  // inactive-config key: TBD or not, it is not evaluated (G0-C)
  }
  if (v.tbd) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, key);
    return false;
  }
  return true;
}

void CheckRange(CatchingValidationReport& report, const char* key, double value, double lo,
                double hi) noexcept {
  if (!std::isfinite(value) || value < lo || value > hi) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, key);
  }
}

void CheckPositive(CatchingValidationReport& report, const char* key, double value) noexcept {
  if (!std::isfinite(value) || !(value > 0.0)) {
    AddFailure(report, CatchingValidationReason::kRangeViolation, key);
  }
}

/// Applies a provisional flag's L0 §5.3 rule: sim warns, real-arm blocks.
void CheckProvisional(CatchingValidationReport& report, const char* key, bool provisional,
                      bool real_arm_config) noexcept {
  if (!provisional) {
    return;
  }
  if (real_arm_config) {
    AddFailure(report, CatchingValidationReason::kProvisionalOnRealArm, key);
  } else {
    AddWarning(report, CatchingValidationReason::kProvisionalWarning, key);
  }
}

}  // namespace

void CheckCatchFrameProvisional(CatchingValidationReport& report, bool catch_frame_provisional,
                                bool real_arm_config) noexcept {
  CheckProvisional(report, kCatchFrameProvisionalKey, catch_frame_provisional, real_arm_config);
}

CatchingValidationReport ValidateCatchingParams(const CatchingParams& params,
                                                double control_rate_hz,
                                                bool real_arm_config) noexcept {
  CatchingValidationReport report;

  // control_rate (rtc_base-owned key; range is invariants.md §RT Path SSoT).
  const bool rate_ok = std::isfinite(control_rate_hz) &&
                       control_rate_hz >= rtc::kMinControlRateHz &&
                       control_rate_hz <= rtc::kMaxControlRateHz;
  if (!rate_ok) {
    AddFailure(report, CatchingValidationReason::kControlRateOutOfRange, "control_rate");
  }

  // reference.omega / zeta / v_max / a_max — active in every configuration.
  if (CheckActiveTbd(report, params.reference_omega, "reference.omega", true)) {
    CheckRange(report, "reference.omega", params.reference_omega.value, 1.0, 25.0);
  }
  if (CheckActiveTbd(report, params.reference_zeta, "reference.zeta", true)) {
    if (params.reference_zeta.value != 1.0) {
      AddFailure(report, CatchingValidationReason::kZetaNotCriticallyDamped, "reference.zeta");
    }
  }
  if (CheckActiveTbd(report, params.reference_v_max, "reference.v_max", true)) {
    CheckPositive(report, "reference.v_max", params.reference_v_max.value);
  }
  const bool a_max_ok = CheckActiveTbd(report, params.reference_a_max, "reference.a_max", true);
  if (a_max_ok) {
    CheckPositive(report, "reference.a_max", params.reference_a_max.value);
  }

  // planner.gamma.eta_v — D-9: 0 < eta_v <= 1.
  if (CheckActiveTbd(report, params.planner_gamma_eta_v, "planner.gamma.eta_v", true)) {
    const double eta_v = params.planner_gamma_eta_v.value;
    if (!std::isfinite(eta_v) || !(eta_v > 0.0) || !(eta_v <= 1.0)) {
      AddFailure(report, CatchingValidationReason::kEtaVOutOfRange, "planner.gamma.eta_v");
    }
  }

  // planner.catchability.manipulability_min.arm_5row (D-18) + its provisional flag.
  const bool manip_resolved =
      CheckActiveTbd(report, params.planner_catchability_manip_min_arm5row,
                     "planner.catchability.manipulability_min.arm_5row", true);
  if (manip_resolved) {
    CheckRange(report, "planner.catchability.manipulability_min.arm_5row",
               params.planner_catchability_manip_min_arm5row.value, 0.0,
               std::numeric_limits<double>::infinity());
    CheckProvisional(report, "planner.catchability.manipulability_min",
                     params.planner_catchability_manip_min_provisional, real_arm_config);
  }

  // supervisor.decel.a_dec — > 0 and <= reference.a_max (L7 §4.3, single-source key).
  const bool a_dec_ok =
      CheckActiveTbd(report, params.supervisor_decel_a_dec, "supervisor.decel.a_dec", true);
  if (a_dec_ok) {
    CheckPositive(report, "supervisor.decel.a_dec", params.supervisor_decel_a_dec.value);
    if (a_max_ok && params.supervisor_decel_a_dec.value > params.reference_a_max.value) {
      AddFailure(report, CatchingValidationReason::kDecelExceedsAMax, "supervisor.decel.a_dec");
    }
  }

  // core.ball.* — active in every configuration (L0 §6: "제어 경로에도 필요").
  const bool diameter_ok = CheckActiveTbd(report, params.ball.diameter, "core.ball.diameter", true);
  if (diameter_ok) {
    CheckRange(report, "core.ball.diameter", params.ball.diameter.value, 0.02, 0.3);
  }
  const bool mass_ok = CheckActiveTbd(report, params.ball.mass, "core.ball.mass", true);
  if (mass_ok) {
    CheckRange(report, "core.ball.mass", params.ball.mass.value, 0.005, 1.0);
  }
  const bool restitution_ok =
      CheckActiveTbd(report, params.ball.restitution, "core.ball.restitution", true);
  if (restitution_ok) {
    CheckRange(report, "core.ball.restitution", params.ball.restitution.value, 0.0, 1.0);
  }
  if (diameter_ok && mass_ok && restitution_ok) {
    CheckProvisional(report, "core.ball", params.ball.provisional, real_arm_config);
  }

  // sim.ball.drag_k — active ONLY in the sim configuration (fixture-only, L0 §1/§6).
  if (CheckActiveTbd(report, params.sim_ball_drag_k, "sim.ball.drag_k", !real_arm_config)) {
    CheckRange(report, "sim.ball.drag_k", params.sim_ball_drag_k.value, 0.0, 0.2);
  }

  // robot.hand.* — active in every configuration.
  if (params.hand.tbd) {
    AddFailure(report, CatchingValidationReason::kActiveConfigTbd, "robot.hand.q_pre/q_close");
  } else {
    CheckPositive(report, "robot.hand.rho_eps", params.hand.rho_eps);
    for (int i = 0; i < params.hand.dof; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      if (!params.hand.caging_mask[idx]) {
        continue;
      }
      const double gap = std::abs(params.hand.q_close[idx] - params.hand.q_pre[idx]);
      if (!(gap > params.hand.rho_eps)) {
        AddFailure(report, CatchingValidationReason::kHandCagingGapTooSmall, "robot.hand.q_close",
                   i);
      }
    }
    CheckProvisional(report, "robot.hand", params.hand.provisional, real_arm_config);
  }

  // ζ·ω·h (`dt` 기준, L4 §4.7): only meaningful once control_rate and omega
  // both resolved to a real number.
  if (rate_ok && !params.reference_omega.tbd) {
    const double h = 1.0 / control_rate_hz;
    const double s = params.reference_omega.value * h;
    if (s >= kDiscreteStabilityLimit) {
      AddFailure(report, CatchingValidationReason::kUnstableDiscretization, "reference.omega");
    } else if (s > kDiscreteAccuracyWarnLimit) {
      AddWarning(report, CatchingValidationReason::kDiscretizationAccuracy, "reference.omega");
    }
  }

  return report;
}

}  // namespace rtc::catching
