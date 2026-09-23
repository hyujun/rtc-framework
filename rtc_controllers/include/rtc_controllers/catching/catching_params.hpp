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
//   - robot.hand.q_open/q_pre/q_close/caging_mask/rho_eps (L6 §6, §4.2)
//   - robot.hand.eta_close/T_close_e2e       (L6 §6, added by S4.1)
// `planner.ik.*`, `planner.hand.d_eff/r_cap`, catch frame (D-17), and the D-16
// joint-accel box are out of scope: none of them appear in G0-C or the §6
// cross-constraint table, and their owning steps (S2.3a, S2.5) have not
// landed. See the .cpp for the YAML keys this file had to invent (the docs
// name the physical relationship but not a schema field) and why.
//
// S4a SCOPE (2026-09-20). The three L6 §5.1 fields the hand-timing step needs
// — `q_open` (the pose homing opens the hand to), `eta_close` (the §4.2
// closure threshold η) and `T_close_e2e` (the identified end-to-end closure
// time) — are parsed and validated here.
//
// S7.1 SCOPE (2026-09-23). The hand sequencer's own keys join: `T_hold`,
// `T_close_timeout`, `hold.mode` / `hold.delta_rad` (L6 §4.4's hold rule) and
// the arrival test `q_tol` / `qd_tol`. `T_pre` does NOT: the hand waits at
// `q_pre` from the moment the arm reaches its wait pose (#537 S7 decision Q4),
// so there is no time-based preshape, and a leftover `T_pre` key is refused by
// name rather than silently ignored.
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
/// `joint_cmd.accel_constraint` (decision K). Mirrors
/// rtc::tsid::ClikReferenceGenerator::AccelConstraint so this parameter header
/// does not pull in the CLIK header (pinocchio + proxsuite); the binding maps
/// one onto the other in a switch.
enum class CatchingAccelConstraint : std::uint8_t { kBox, kKinematic, kDynamic };

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

/// `robot.hand.T_close_timeout` when the profile omits it: this many
/// T_close_e2e (#537 S7 D-S7-1, provisional — S8). Twice the p99 closure time
/// is late enough that a nominal close never trips it and early enough that a
/// hand that stalled short of η is noticed within one more closure.
inline constexpr double kCloseTimeoutPerE2e = 2.0;

/// L6 §4.4's hold rule (`robot.hand.hold.mode`): what the hand is commanded to
/// once closure is reached. `kCloseTarget` keeps commanding `q_close` (the
/// posture pair already includes "a little squeeze" past the stall); the
/// `kMeasuredOffset` form commands the measured pose at Hold entry plus
/// `hold.delta_rad` toward `q_close` on every caging joint, for a hand whose
/// `q_close` would squeeze too hard once it is a real one.
enum class HandHoldMode : std::uint8_t { kCloseTarget, kMeasuredOffset };

/// One hand profile (L6 §6 `robot.hand.*`, scope limited to what the L6 §4.2
/// caging check and the S4a timing measurement need). `dof` is the number of
/// valid entries in the arrays; `tbd` is true while the `q_pre`/`q_close` pair
/// is still the `TBD` placeholder. `q_open_tbd` tracks the third pose array
/// separately: it is not part of the §4.2 caging pair, so a profile that gives
/// the pair but omits `q_open` parses (and the validator reports the one key)
/// rather than being refused wholesale. `provisional` reads an invented
/// `robot.hand.provisional` key (see .cpp) because L6 §6 marks "손 프로파일
/// 값은 전부 provisional 이다" in prose, with no schema field.
struct HandProfile {
  std::array<double, kMaxHandDof> q_open{};  // homing pose (L6 §5.1, #537 S7 Q4)
  std::array<double, kMaxHandDof> q_pre{};
  std::array<double, kMaxHandDof> q_close{};
  std::array<bool, kMaxHandDof> caging_mask{};  // which joints L6 §4.2 checks
  int dof{0};
  double rho_eps{0.02};   // L6 §6 default [rad]
  TbdDouble eta_close;    // –, [0.5, 1]   — L6 §4.2 closure threshold η
  TbdDouble T_close_e2e;  // s, >= 0       — L6 §4.2 end-to-end closure time
  // ── Sequencer (S7.1, L6 §5.3/§6) ─────────────────────────────────────────
  /// How long HOLD lasts before RETREAT [s], [0, 5]. 0.5 s, not L6's original
  /// 1.0: a trial cycle is dominated by this wait and the ball is caged well
  /// before it (provisional, S8).
  TbdDouble T_hold{TbdDouble::Resolved(0.5)};
  /// Close → Hold without reaching η [s], > T_close_e2e. Absent from the YAML
  /// it is DERIVED at parse, kCloseTimeoutPerE2e × T_close_e2e — a stored
  /// default would outlive the measurement it was derived from.
  TbdDouble T_close_timeout;
  HandHoldMode hold_mode{HandHoldMode::kCloseTarget};
  double hold_delta_rad{0.0};  // rad, >= 0 — read only by kMeasuredOffset
  /// "At the target": max |q − target| ≤ q_tol AND max |q̇| ≤ qd_tol. The
  /// velocity half is what makes it SETTLED rather than passing through —
  /// the contact baseline is learned from a hand at rest.
  double q_tol{0.01};   // rad, > 0
  double qd_tol{0.05};  // rad/s, > 0
  bool tbd{true};
  bool q_open_tbd{true};
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
  /// L0 §5.3 flag for the whole L4 reference block, read from an invented
  /// `reference.provisional` key (see the .cpp header for why these keys are
  /// invented and why they default to true).
  ///
  /// It exists because `reference.a_max` is a DERIVED bound whose derivation
  /// L4 §6 defers to the D-16 revision, while `v_max` is decided — so the two
  /// cannot share a TBD. A profile that left `a_max` at TBD instead would be
  /// refused outright in sim, and because CM latches `bring_up_failed` on any
  /// controller's configure failure that takes EVERY controller on the robot
  /// down with it — observed on a shipped sim profile 2026-09-22 (the robot and
  /// the session are named in plan §7.3 A-S5-11, which owns the incident; this
  /// header is robot-agnostic and stays that way). The provisional rule says
  /// the right thing instead: sim warns, a real arm is blocked.
  bool reference_provisional{true};

  // planner: (L3 §6)
  TbdDouble planner_gamma_eta_v{TbdDouble::Resolved(0.9)};                     // –, (0, 1] (D-9)
  TbdDouble planner_catchability_manip_min_arm5row{TbdDouble::Resolved(0.1)};  // –, >= 0 (D-18)
  bool planner_catchability_manip_min_provisional{true};  // invented key, see .cpp

  // supervisor: (L7 §6)
  TbdDouble supervisor_decel_a_dec;  // m/s², > 0 and <= reference_a_max (L7 §4.3)

  // io: (L1 §6) — vision ingress. Consumed from S5.2.
  //
  // `io_n_min` is a COUNT, so it is an int with 0 meaning "absent" rather than
  // a TbdDouble: the schema's TBD placeholder exists for values a decision has
  // not produced yet, and a point count that arrived as 10.5 is a malformed
  // key, not an open one. 0 is reported as an active TBD.
  int io_n_min{0};           // points, [2, kCap]
  TbdDouble io_t_stale;      // s, [0.02, 0.2]  — steady RECEIVE age limit
  TbdDouble io_future_tol;   // s, [1e-4, 1e-2] — real-arm clock-sync budget
  TbdDouble io_horizon_min;  // s, > 0        — D-15 usable-window requirement
  TbdDouble io_track_eval_offset{TbdDouble::Resolved(0.05)};  // s, [0, 0.3]
  TbdDouble io_track_j_warn;  // m, > 0 — DIAGNOSTIC threshold, never a gate
  /// The sim configuration's own `future_tol` (A-S5-2). Active only when
  /// `real_arm_config` is false, exactly like `sim.ball.drag_k`.
  ///
  /// It exists because the two numbers are two orders of magnitude apart and
  /// the controller YAML is shared between sim and hardware: the sim ball
  /// lane's stamps ride the SIM time axis and legitimately lead wall by the
  /// in-flight phase error (D-3, `rtc_mujoco_sim` §Projectile Ball stamp),
  /// while a camera on real hardware stamps at capture and may lead wall only
  /// by the clock-sync error. One key with one range could serve only one of
  /// them, and the sim value inside the real-arm range would silently accept a
  /// 100 ms clock offset on hardware.
  TbdDouble sim_io_future_tol;  // s, (0, 0.5]

  // prediction: (L1 §6 / L2) — what the vision profile is expected to produce.
  TbdDouble prediction_dt_expected{TbdDouble::Resolved(0.05)};  // s, (0, 1]

  // joint_cmd: (L5 §6) — the CLIK step. Consumed from S5.3.
  //
  // The weights keep the documented ordering w_task >> w_arm >> damping_sq
  // (L5 §4.3): the task must win, the posture must only resolve the redundant
  // degree of freedom, and the damping must only regularise. The validator
  // checks the ORDERING, not just the ranges — three individually sensible
  // weights in the wrong order produce a controller that tracks posture and
  // treats the catch point as a suggestion.
  TbdDouble joint_cmd_k_p{TbdDouble::Resolved(20.0)};         // 1/s, [1, 100]
  TbdDouble joint_cmd_k_axis{TbdDouble::Resolved(8.0)};       // 1/s, (0, 30]
  TbdDouble joint_cmd_k_posture{TbdDouble::Resolved(1.0)};    // 1/s, [0, 10]
  TbdDouble joint_cmd_w_task{TbdDouble::Resolved(1.0)};       // –, > 0
  TbdDouble joint_cmd_w_axis{TbdDouble::Resolved(0.5)};       // –, > 0
  TbdDouble joint_cmd_w_arm{TbdDouble::Resolved(1e-2)};       // –, >= 0
  TbdDouble joint_cmd_w_smooth{TbdDouble::Resolved(1e-3)};    // –, >= 0
  TbdDouble joint_cmd_damping_sq{TbdDouble::Resolved(1e-4)};  // –, > 0
  int joint_cmd_max_iter{20};                                 // –, >= 1
  /// `joint_cmd.accel_constraint` (decision K, S6-C2): which acceleration
  /// constraint the CLIK QP carries — `box` (the D-16 per-joint window, the
  /// default and the S5 behaviour), `kinematic` (task acceleration J·v̇ + J̇·v
  /// of the tracked rows), `dynamic` (arm torque M·v̇ + h ≤ η_τ·τ_max, τ_max =
  /// the arm device's `joint_limits.max_torque`). Each form reads only its own
  /// keys below; a key of another form is refused at parse.
  CatchingAccelConstraint joint_cmd_accel_constraint{CatchingAccelConstraint::kBox};
  TbdDouble joint_cmd_task_accel_max_linear{};   // m/s², > 0 — kinematic
  TbdDouble joint_cmd_task_accel_max_angular{};  // rad/s², > 0 — kinematic
  /// η_τ of the torque rows. The D-16 derivation's margin (0.8, user decision
  /// 2026-09-20) is the default: the same torque budget, now spent per tick.
  TbdDouble joint_cmd_eta_tau{TbdDouble::Resolved(0.8)};  // –, (0, 1] — dynamic
  /// L5 §4.4/§4.5. **sim is 0** (2026-09-20: no lag is injected), and the
  /// lead axis then coincides with the real one. A non-zero value is the
  /// hardware identification (S10) or the axis-confusion fixture.
  TbdDouble joint_cmd_lag_t_arm{TbdDouble::Resolved(0.0)};  // s, >= 0
  bool joint_cmd_lag_lead_enable{false};

  // robot.arm: (L5 §6) — the boxes CLIK is given.
  /// How far INSIDE the device's own position limits the CLIK box sits. The
  /// backend clamps commands to the device limits; if CLIK were given the same
  /// box, its solution and the command actually written would differ whenever
  /// it touched a bound, and the difference would surface as a tracking error
  /// nobody can attribute (L5 §4.3 "중복 방지").
  TbdDouble robot_arm_limit_margin{TbdDouble::Resolved(0.05)};  // rad, [0, 0.3]

  // supervisor: (L7 §6) — the two keys the joint command layer reports to.
  TbdDouble supervisor_track_err_abort;  // rad, > 0 — L7 owns this key, L5 only reads it
  int supervisor_n_qp{0};                // consecutive QP failures before FAULT, >= 1

  // supervisor: (L7 §6) — the S7.2 driver's keys. Defaults are the #537 S7
  // decisions (2026-09-23, D-S7-2 / Q6 / Q8), all provisional until S8.
  /// How long COMMITTED/CLOSING may run on a stale prediction past io.t_stale
  /// before it is BALL_STALE_LONG (A-6) [s], [0, 1].
  TbdDouble supervisor_stale_committed_max_s{TbdDouble::Resolved(0.10)};
  /// REF_SATURATED: this many consecutive saturated reference ticks (Q6).
  /// 100 (0.2 s at 500 Hz), not the 5 first proposed: a reference that starts
  /// an approach from rest saturates while it catches up with the plan, and
  /// the unit fixtures measured runs of 10 ticks (a 14 cm approach) and 76 (the
  /// 60 cm G5-B target) on approaches that go on to converge — 5 cut every
  /// one of them. Provisional until the sim streak distribution (D-S7-4).
  int supervisor_sat_ticks{100};
  /// Joint-space homing / return (Q3): speed cap [rad/s] > 0, fraction of
  /// the derived q̈_max (0, 1], and the arrival velocity bound [rad/s] > 0.
  double supervisor_homing_v_max{0.5};
  double supervisor_homing_eta_a{0.5};
  double supervisor_homing_qd_tol{0.02};
  /// "At the wait pose": max |q − wait_pose| [rad] > 0 (Q13's skip test too).
  double supervisor_ready_pose_tol{0.02};
  /// Contact judgement (L7 §4.4, S7.3; #537 S7 Q1 / D-S7-3). A fingertip is
  /// in contact when |F − b| > max(f_min, k_sigma·σ̂) for n_debounce samples
  /// in a row; the attempt holds the ball when m_min fingertips agree. b and
  /// σ̂ are an EMA (baseline_alpha) learned at q_pre in ARMED/TRACKING, and
  /// fewer than n_baseline_min samples makes the verdict Undetermined.
  double supervisor_contact_f_min{0.2};            // N, >= 0 (Q1: the user's value)
  double supervisor_contact_k_sigma{3.0};          // –, >= 0
  int supervisor_contact_n_debounce{3};            // samples, >= 1
  int supervisor_contact_m_min{2};                 // fingertips, >= 1
  double supervisor_contact_t_confirm{0.2};        // s, [0, 1] — window [t_cmd, t_c + T_confirm]
  double supervisor_contact_t_stale{0.02};         // s, (0, 0.5] — TIP_STALE age limit
  double supervisor_contact_baseline_alpha{0.02};  // –, (0, 1]
  int supervisor_contact_n_baseline_min{20};       // samples, >= 1

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
/// array; arrays of different length; empty arrays; a `caging_mask` or a
/// `q_open` of the wrong length); a hand array longer than `kMaxHandDof`. A present-but-malformed
/// key is refused rather than defaulted — defaulting would read a typo as "still TBD".
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
  kWeightOrdering,           // CLIK weights violate w_task/w_a >> w_arm >> damping_sq (L5 §4.3)
  kCloseTimeoutNotAboveE2e,  // robot.hand.T_close_timeout <= T_close_e2e (L6 §6)
  kFreezeShorterThanClose,   // T_freeze < T_close_e2e + T_arm + h (L3 §4.11, #537 S7)
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

/// Which `future_tol` applies to a configuration (A-S5-2).
///
/// The rule lives here rather than at the call site because getting it
/// backwards is silent in both directions: the sim value on hardware accepts a
/// clock offset no real link should have, and the hardware value in sim
/// rejects most of the ball lane as future-stamped and the controller simply
/// never sees a trajectory. Returns the sim key when `real_arm_config` is
/// false AND that key is resolved; otherwise the shared key, so a sim
/// configuration that does not override it inherits the strict value rather
/// than silently getting no limit.
[[nodiscard]] constexpr TbdDouble EffectiveFutureTol(const CatchingParams& params,
                                                     bool real_arm_config) noexcept {
  if (!real_arm_config && !params.sim_io_future_tol.tbd) {
    return params.sim_io_future_tol;
  }
  return params.io_future_tol;
}

/// Key reported when the freeze window cannot contain the hand's closure.
inline constexpr const char* kFreezeWindowKey = "planner.freeze.T_freeze";

/// L3 §4.11's lower bound on the freeze window: T_freeze ≥ T_close_e2e + T_arm
/// + h. COMMITTED starts at t_c − T_freeze and the close command goes out at
/// t_cmd = t_c − T_close_e2e, so a window shorter than the closure has already
/// missed its own close command on the tick it commits; T_arm is the servo
/// lead the reference runs ahead by, and one tick h is the rounding of both
/// instants onto the tick grid.
///
/// `t_freeze_s` is `planner.freeze.T_freeze` (the planner's key, parsed by
/// planner_params.hpp — this header does not depend on that one, so the
/// caller passes the number, the same arrangement as
/// CheckCatchFrameProvisional). A NaN `t_freeze_s` or an unresolved
/// T_close_e2e / T_arm is not judged here: an unset value is reported by the
/// check that owns it. Allocation-free, noexcept.
void CheckFreezeCoversClose(CatchingValidationReport& report, const CatchingParams& params,
                            double t_freeze_s, double control_rate_hz) noexcept;

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
