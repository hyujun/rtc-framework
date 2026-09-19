// ── Catching parameter validator (dynamic_catching S1.7) ─────────────────────
// The YAML half of the L0 §5.3 / §9 G0-C validator: a POD that carries the
// subset of the `catching:` schema the cross-layer checks need, a parser, and
// a fixed-capacity, allocation-free ValidateCatchingParams(). Non-RT
// (`on_configure`), same repo idiom as params/reach_gate_params.hpp.
//
// SCOPE. This is NOT the full controller config — that is frozen in S5 (plan
// §4.4 S5.4, "S5~S9 필드 superset 동결"). It carries only the keys G0-C
// (L0_core.md §9) and the CATCHING_MASTER.md §6 cross-constraint table
// reference:
//   - reference.omega/zeta/v_max/a_max      (L4 §6)
//   - planner.gamma.eta_v                   (L3 §6, D-9)
//   - planner.catchability.manipulability_min.arm_5row (L3 §6, D-18)
//   - supervisor.decel.a_dec                (L7 §6)
//   - core.ball.diameter/mass/restitution   (L0 §6, D-12)
//   - sim.ball.drag_k                       (L0 §6 — sim-fixture-only, see below)
//   - robot.hand.q_pre/q_close/caging_mask/rho_eps (L6 §6, §4.2)
// `planner.ik.*`, `planner.hand.d_eff/r_cap`, catch frame (D-17), and the D-16
// joint-accel box are out of scope: none of them appear in G0-C or the §6
// cross-constraint table, and their owning steps (S2.3a, S2.5) have not
// landed. See the .cpp for the YAML keys this file had to invent (the docs
// name the physical relationship but not a schema field) and why.
//
// ACTIVE CONFIGURATION. G0-C requires "TBD in an active-config key blocks
// arming; TBD in an inactive-config key passes" (L0 §5.3: "sim/실기 ... 에
// 따라 검사 집합을 결정"). Every field above is active in BOTH the sim and the
// real-arm configuration except `sim.ball.drag_k`, which L0 §1/§6 marks
// fixture-only (the k-identification tool, S3.8) — it is active only when
// `real_arm_config` is false, and skipped entirely (TBD or not) otherwise.
// This is the concrete active/inactive pair the tests exercise; there is no
// second one in scope (no `lead_enable`-gated key lives in this subset).
#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rtc::catching {

// ── TBD-capable scalar ────────────────────────────────────────────────────
// A YAML scalar the schema may leave open as the literal string "TBD" (or an
// unparseable/non-finite number, which L0 §5.3 treats the same way) until a
// decision fills it in. `value` is only meaningful when `tbd` is false.
struct TbdDouble {
  double value{std::numeric_limits<double>::quiet_NaN()};
  bool tbd{true};

  [[nodiscard]] static constexpr TbdDouble Resolved(double v) noexcept {
    return TbdDouble{v, false};
  }
};

// Hand joint-array capacity. Mirrors the `kMaxHandDof = 16` the L0 §5.2
// common-types header is planned to define once S1.2 lands (the widest
// supported hand is 16 DoF, L0 §5.2). Duplicated here rather than depended on
// because that header does not exist yet; unify when it does.
inline constexpr std::size_t kMaxHandDof = 16;

/// One hand profile (L6 §6 `robot.hand.*`, scope limited to what the L6 §4.2
/// caging check needs). `dof` is the number of valid entries in the arrays;
/// `tbd` is true while the whole profile is still the `TBD` placeholder
/// (S4.1 has not produced a robot-specific profile yet). `provisional` reads
/// an invented `robot.hand.provisional` key (see .cpp) because L6 §6 marks
/// "손 프로파일 값은 전부 provisional 이다" in prose, with no schema field.
struct HandProfile {
  std::array<double, kMaxHandDof> q_pre{};
  std::array<double, kMaxHandDof> q_close{};
  std::array<bool, kMaxHandDof> caging_mask{};  // which joints L6 §4.2 checks
  int dof{0};
  double rho_eps{0.02};  // L6 §6 default [rad]
  bool tbd{true};
  bool provisional{true};
};

/// D-12 공 사양 (L0 §6 `core.ball.*`). `provisional` reads an invented
/// `core.ball.provisional` key (see .cpp) — L0 §6 marks diameter/mass/
/// restitution "(provisional)" in its 근거 column only, no schema field.
struct BallSpec {
  TbdDouble diameter;     // m,  [0.02, 0.3]  (L0 §6)
  TbdDouble mass;         // kg, [0.005, 1.0] (L0 §6)
  TbdDouble restitution;  // –,  [0, 1]       (L0 §6)
  bool provisional{true};
};

/// Parsed subset of the `catching:` YAML tree the S1.7 validator needs. See
/// the file header for scope and the .cpp for every invented key.
struct CatchingParams {
  // reference: (L4 §6)
  TbdDouble reference_omega{TbdDouble::Resolved(10.0)};  // rad/s, [1, 25]
  TbdDouble reference_zeta{TbdDouble::Resolved(1.0)};    // –, v1 requires exactly 1
  TbdDouble reference_v_max;                             // m/s, > 0
  TbdDouble reference_a_max;                             // m/s², > 0

  // planner: (L3 §6)
  TbdDouble planner_gamma_eta_v{TbdDouble::Resolved(0.9)};                     // –, (0, 1] (D-9)
  TbdDouble planner_catchability_manip_min_arm5row{TbdDouble::Resolved(0.1)};  // –, >= 0 (D-18)
  bool planner_catchability_manip_min_provisional{true};  // invented key, see .cpp

  // supervisor: (L7 §6)
  TbdDouble supervisor_decel_a_dec;  // m/s², > 0 and <= reference_a_max (L7 §4.3)

  // core / sim: (L0 §6)
  BallSpec ball;
  TbdDouble sim_ball_drag_k;  // 1/m, [0, 0.2] — sim-fixture-only (active iff !real_arm_config)

  // robot: (L6 §6)
  HandProfile hand;
};

/// Parse the `catching:` map (CATCHING_MASTER.md §6 top-level tree). An absent
/// section (e.g. no `sim:` tree in a real-arm config) is not an error: its keys
/// take their doc defaults (TBD where open), which `ValidateCatchingParams`
/// refuses only when the key is active. Throws `std::invalid_argument` (and
/// only that) on: a missing/non-map root; a present section that is not a
/// map; a present scalar that is neither a finite number nor the literal
/// `"TBD"`; hand `q_pre`/`q_close` given inconsistently (one TBD, the other an
/// array; arrays of different length; empty arrays; a `caging_mask` of the
/// wrong length); a hand array longer than `kMaxHandDof`. A present-but-malformed key is refused
/// rather than defaulted — defaulting would read a typo as "still TBD".
[[nodiscard]] CatchingParams ParseCatchingParams(const YAML::Node& node);

// ── Validation report ────────────────────────────────────────────────────
enum class CatchingValidationReason : std::uint8_t {
  kActiveConfigTbd,          // an active-config key is still the TBD placeholder
  kControlRateOutOfRange,    // control_rate_hz outside [kMinControlRateHz, kMaxControlRateHz]
  kRangeViolation,           // a resolved value is outside its L0/L3/L4/L6/L7 §6 range
  kZetaNotCriticallyDamped,  // reference.zeta != 1 (v1 requires the closed-form solution)
  kEtaVOutOfRange,           // planner.gamma.eta_v not in (0, 1] (D-9)
  kDecelExceedsAMax,         // supervisor.decel.a_dec > reference.a_max (L7 §4.3)
  kUnstableDiscretization,   // omega*h >= 2*sqrt(2)-2 (L4 §4.7 discrete stability boundary)
  kDiscretizationAccuracy,   // omega*h > 0.05 (L4 §4.7 accuracy recommendation) — WARNING only
  kHandCagingGapTooSmall,    // |q_close[i] - q_pre[i]| <= rho_eps on a caging joint (L6 §4.2)
  kProvisionalOnRealArm,     // a provisional value blocks the real-arm configuration (L0 §5.3)
  kProvisionalWarning,       // same value, but the sim configuration only warns — WARNING only
};

/// One report line: which rule fired, on which key, and (for the per-joint
/// hand check) which array index — or -1 when the check is not per-index.
/// `key` always points at a string literal (static storage): the report never
/// allocates or formats.
struct CatchingValidationEntry {
  CatchingValidationReason reason{CatchingValidationReason::kRangeViolation};
  const char* key{""};
  int index{-1};
};

/// Fixed-capacity validation result (L0 §5.3: "ValidationReport{bool armable;
/// 고정 용량 실패 키 목록; 경고 목록}"). No heap: `failures`/`warnings` are
/// `std::array`s with saturating counts — a check beyond capacity is dropped
/// rather than allocated (capacity comfortably exceeds every case this
/// validator can currently raise, including one entry per hand joint).
struct CatchingValidationReport {
  static constexpr std::size_t kMaxFailures = 32;
  static constexpr std::size_t kMaxWarnings = 16;

  bool armable{true};
  std::array<CatchingValidationEntry, kMaxFailures> failures{};
  std::size_t failure_count{0};
  std::array<CatchingValidationEntry, kMaxWarnings> warnings{};
  std::size_t warning_count{0};
};

/// G0-C. `control_rate_hz` is the base controller's already-resolved
/// `control_rate` (rtc_base owns that key; this validator only consumes the
/// number). `real_arm_config` selects the active-configuration set (see file
/// header) and gates every `provisional` flag (L0 §5.3: sim warns, real-arm
/// blocks). Allocation-free and noexcept: every input is already a POD, and a
/// full report is not something we would refuse to construct.
[[nodiscard]] CatchingValidationReport ValidateCatchingParams(const CatchingParams& params,
                                                              double control_rate_hz,
                                                              bool real_arm_config) noexcept;

/// Key reported for the catch frame's provisional flag (D-17).
inline constexpr const char* kCatchFrameProvisionalKey =
    "urdf.extra_frames.<catch_frame>.provisional";

/// Applies the L0 §5.3 provisional rule to the catch frame (D-17): a
/// provisional frame warns in sim and blocks a real-arm configuration. The
/// flag lives in the robot config (`urdf.extra_frames.<name>.provisional`,
/// carried by rtc_urdf_bridge::ExtraFrameConfig), not in the `catching:`
/// section, so the caller reads it from the model config and adds it to the
/// report ValidateCatchingParams produced. Allocation-free, noexcept.
void CheckCatchFrameProvisional(CatchingValidationReport& report, bool catch_frame_provisional,
                                bool real_arm_config) noexcept;

}  // namespace rtc::catching
