// ── Catching parameter validator ─────────────────────────────────────────────
// The YAML half of the arming validator: a POD that carries the subset of the
// `catching:` schema the cross-layer checks need, a parser, and a
// fixed-capacity, allocation-free ValidateCatchingParams(). Non-RT
// (`on_configure`), same repo idiom as params/reach_gate_params.hpp.
//
// SCOPE. This is NOT the full controller config. It carries only the keys the
// cross-layer checks reference:
//   - reference.omega/zeta/v_max/a_max      (the closed_form reference)
//   - planner.search.grid.gamma.eta_v       (γ-window TCP speed margin)
//   - planner.search.grid.catchability.manipulability_min.arm_5row
//   - planner.search.grid.reference.*, planner.search.grid.stop.a_dec — the
//     search's own copies of the five values above and below
//   - planner.search.mode, planner.segment.mode,
//     planner.segment.mpc.{switch_margin, eta_v}
//   - supervisor.decel.a_dec
//   - core.ball.diameter/mass/restitution
//   - sim.ball.drag_k                       (sim-fixture-only, see below)
//   - robot.hand.q_open/q_pre/q_close/caging_mask/rho_eps
//   - robot.hand.eta_close/T_close_e2e
//   - robot.hand.T_close_lead                (what the sequencer subtracts from t_c)
// WHICH KEYS A CONFIGURATION IS HELD TO follows what it selects:
// `reference.*` and `supervisor.decel.a_dec` are the closed_form law's and are
// judged only under `planner.segment.mode: closed_form`; the
// `planner.search.grid.*` keys only under `planner.search.mode: grid`. A key of
// a function the configuration did not select is neither required nor judged —
// by the parser too: such a function's keys are read as one block, taken when
// all of it reads (the values stay visible to whoever compares two functions)
// and left at their defaults when any of it does not, where under the
// selection that runs the function the same key refuses the parse.
// `planner.search.grid.ik.*` (catch_pose_ik_params.hpp), `planner.search.grid.hand.d_eff/r_cap`,
// the catch frame and the joint-accel box are out of scope. See the .cpp for the YAML keys this
// file had to invent (a physical relationship with no schema field) and why.
//
// The hand-timing keys — `q_open` (the pose homing opens the hand to),
// `eta_close` (the closure threshold η, JointClosureProgress) and
// `T_close_e2e` (the identified end-to-end closure time) — and the hand
// sequencer's own keys (`T_hold`, `T_close_timeout`, `hold.mode` /
// `hold.delta_rad`, the arrival test `q_tol` / `qd_tol`) are parsed and
// validated here. `T_pre` is NOT: the hand waits at `q_pre` from the moment
// the arm reaches its wait pose, so there is no time-based preshape, and a
// leftover `T_pre` key is refused by name rather than silently ignored.
//
// ACTIVE CONFIGURATION. A TBD in an active-config key blocks arming; a TBD in
// an inactive-config key passes. Every field above is active in BOTH the sim
// and the real-arm configuration except `sim.ball.drag_k`, which is
// fixture-only (the drag-identification tool) — it is active only when
// `real_arm_config` is false, and skipped entirely (TBD or not) otherwise.
// This is the concrete active/inactive pair the tests exercise; there is no
// second one in scope (no `lead_enable`-gated key lives in this subset).
//
// PROVISIONAL. A value flagged provisional warns in the sim configuration and
// blocks the real-arm configuration (armable = false).
#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>

namespace rtc::catching {

// ── TBD-capable scalar ────────────────────────────────────────────────────
/// `joint_cmd.accel_constraint`. Names the forms of
/// rtc::tsid::ClikReferenceGenerator::AccelConstraint a catching profile can
/// select, so this parameter header does not pull in the CLIK header
/// (pinocchio + proxsuite); the binding maps one onto the other in a switch.
/// The CLIK's `box` form (a per-joint acceleration window) is not among them.
/// Two values are not forms. `kUnset` is a profile that selects none (the key
/// is absent or the literal `TBD`): the key has no default, so the validator
/// reports it instead of a form being assumed. `kRemovedBox` is a profile that
/// still writes `box`, the form this key used to offer and to default to; it
/// is kept apart from kUnset so that the caller can say which of the two the
/// operator has to fix.
enum class CatchingAccelConstraint : std::uint8_t { kUnset, kRemovedBox, kKinematic, kDynamic };

/// The key's YAML spelling of `form` — the one place it is written, for the
/// parser and for whoever prints the loaded form. `unset` is not a spelling a
/// profile can write; it names the absence.
[[nodiscard]] constexpr const char* AccelConstraintName(CatchingAccelConstraint form) noexcept {
  switch (form) {
    case CatchingAccelConstraint::kKinematic:
      return "kinematic";
    case CatchingAccelConstraint::kDynamic:
      return "dynamic";
    case CatchingAccelConstraint::kRemovedBox:
      return "box";
    case CatchingAccelConstraint::kUnset:
      break;
  }
  return "unset";
}

/// Whether `form` is one the CLIK can be given (not kUnset, not kRemovedBox).
[[nodiscard]] constexpr bool IsSelectedForm(CatchingAccelConstraint form) noexcept {
  return form == CatchingAccelConstraint::kKinematic || form == CatchingAccelConstraint::kDynamic;
}

/// `planner.segment.mode` — what the arm follows from APPROACH to the end of
/// the stop, chosen once per configure and never mixed within an activation.
/// kClosedForm has the RT tick make the soft-catch reference and the virtual
/// decel target itself and is the default (the key absent); kMpc and
/// kMpcDocking follow the joint segments a segment planner publishes — the mpc
/// segment planner's, or the docking planner's.
enum class CatchingSegmentMode : std::uint8_t { kClosedForm, kMpc, kMpcDocking };

/// `planner.search.mode` — which search turns a predicted ball into a catch
/// plan, chosen once per configure. kGrid is the default (the key
/// absent): the grid over the vision samples. kNlp solves the docking problem
/// per candidate.
enum class CatchingSearchMode : std::uint8_t { kGrid, kNlp };

/// The YAML spelling of each mode — what the parser accepts and what a log or
/// a mirrored parameter writes back.
[[nodiscard]] constexpr const char* SegmentModeName(CatchingSegmentMode mode) noexcept {
  switch (mode) {
    case CatchingSegmentMode::kClosedForm:
      return "closed_form";
    case CatchingSegmentMode::kMpc:
      return "mpc";
    case CatchingSegmentMode::kMpcDocking:
      return "mpc_docking";
  }
  return "unknown";
}

[[nodiscard]] constexpr const char* SearchModeName(CatchingSearchMode mode) noexcept {
  switch (mode) {
    case CatchingSearchMode::kGrid:
      return "grid";
    case CatchingSearchMode::kNlp:
      return "nlp";
  }
  return "unknown";
}

/// Whether the arm follows published joint segments under `mode` (every mode
/// but closed_form). The RT's segment lane, the pair admission and the
/// planner's first-segment solve are on exactly when this holds.
[[nodiscard]] constexpr bool FollowsSegments(CatchingSegmentMode mode) noexcept {
  return mode != CatchingSegmentMode::kClosedForm;
}

/// Whether a search and a segment mode can run together. The nlp search's plan
/// is the catch node of a joint trajectory it solved; the closed_form law
/// follows a Cartesian reference to a catch point and has no use for it — its
/// plan fields the law reads (the γ ramp) are not what that search produces.
[[nodiscard]] constexpr bool SearchSegmentCombinationAllowed(CatchingSearchMode search,
                                                             CatchingSegmentMode segment) noexcept {
  return !(search == CatchingSearchMode::kNlp && segment == CatchingSegmentMode::kClosedForm);
}

/// A key of the `catching:` tree that moved: `old_path` is no longer
/// read, `new_path` is where its value belongs. Paths are dotted and relative
/// to `catching:`.
struct RenamedCatchingKey {
  const char* old_path;
  const char* new_path;
};

/// Every key that moved. A tree that still carries one would run on the new
/// key's default under the old key's name, so the binding parks on it
/// (FindRenamedCatchingKeys) instead of ignoring it. `supervisor.decel` is
/// listed leaf by leaf: `supervisor.decel.a_dec` stays where it is.
inline constexpr std::array<RenamedCatchingKey, 18> kRenamedCatchingKeys{{
    {"supervisor.decel.mode", "planner.segment.mode"},
    {"supervisor.decel.switch_margin", "planner.segment.mpc.switch_margin"},
    {"planner.decel_mpc", "planner.segment.mpc"},
    {"planner.budget_s", "planner.search.grid.budget_s"},
    {"planner.max_ik", "planner.search.grid.max_ik"},
    {"planner.n_settle", "planner.search.grid.n_settle"},
    {"planner.slice", "planner.search.grid.slice"},
    {"planner.time", "planner.search.grid.time"},
    {"planner.unc", "planner.search.grid.unc"},
    {"planner.gamma", "planner.search.grid.gamma"},
    {"planner.rollout", "planner.search.grid.rollout"},
    {"planner.budget", "planner.search.grid.budget"},
    {"planner.score", "planner.search.grid.score"},
    {"planner.workspace", "planner.search.grid.workspace"},
    {"planner.hand", "planner.search.grid.hand"},
    {"planner.ik", "planner.search.grid.ik"},
    {"planner.catchability", "planner.search.grid.catchability"},
    {"planner.switch", "planner.search.grid.switch"},
}};

/// What else to say about a moved key, or nullptr: two of the old maps held a
/// value that TWO functions read and that now has a key per function. A
/// profile that only moves the map to its new path would leave the mpc segment
/// planner on its default for that value, so the park names the second key.
[[nodiscard]] constexpr const char* RenamedCatchingKeyNote(std::string_view old_path) noexcept {
  if (old_path == "planner.gamma") {
    return "its eta_v was also the mpc segment planner's speed margin, which is "
           "'catching.planner.segment.mpc.eta_v' now";
  }
  if (old_path == "planner.ik") {
    return "its v_eps was also the mpc segment planner's ball-speed floor, which is "
           "'catching.planner.segment.mpc.v_eps' now";
  }
  if (old_path == "planner.workspace") {
    return "and that key has since been removed, so delete it instead: the search does not "
           "judge where the catch point is";
  }
  return nullptr;
}

/// The entries of kRenamedCatchingKeys whose old path is present in
/// `catching` — with any value, a map or a null included: a key that is
/// written is a key somebody meant. Never throws; a node on the way that is
/// not a map has no children.
[[nodiscard]] std::vector<RenamedCatchingKey> FindRenamedCatchingKeys(const YAML::Node& catching);

/// Keys of the `catching:` tree that were removed with nothing in their place:
/// the catch box. The searches used to remove every candidate
/// whose catch point (or stopping point) lay outside it; they no longer judge
/// where a catch point is, and the parsers do not read the keys. A tree that
/// still writes one was tuned for a gate that is gone, so the binding parks on
/// it (FindRemovedCatchingKeys) instead of running as if the box were in force.
/// Paths are dotted and relative to `catching:`.
inline constexpr std::array<const char*, 2> kRemovedCatchingKeys{
    "planner.search.grid.workspace",
    "planner.search.nlp.catch_box",
};

/// The entries of kRemovedCatchingKeys present in `catching` — with any value,
/// under the map of a search that runs or of one that does not. Never throws.
[[nodiscard]] std::vector<const char*> FindRemovedCatchingKeys(const YAML::Node& catching);

// A YAML scalar the schema may leave open as the literal string "TBD" (or an
// unparseable/non-finite number, which is treated the same way) until a
// decision fills it in. `value` is only meaningful when `tbd` is false.
struct TbdDouble {
  double value{std::numeric_limits<double>::quiet_NaN()};
  bool tbd{true};

  [[nodiscard]] static constexpr TbdDouble Resolved(double v) noexcept {
    return TbdDouble{v, false};
  }
};

// Hand joint-array capacity: the widest supported hand is 16 DoF.
// This header is the one definition — there is no shared common-types header
// for it, and other code takes the value from here.
inline constexpr std::size_t kMaxHandDof = 16;

/// `robot.hand.T_close_timeout` when the profile omits it: this many
/// T_close_e2e (provisional). Twice the p99 closure time
/// is late enough that a nominal close never trips it and early enough that a
/// hand that stalled short of η is noticed within one more closure.
inline constexpr double kCloseTimeoutPerE2e = 2.0;

/// `robot.hand.T_release_timeout` when the profile omits it: a multiple of
/// T_close_e2e DERIVED from the profile, because one
/// constant cannot serve a torque-saturated hand and a stiffness-dominated one
/// alike (3 × T_close_e2e would time out every release of the latter). The
/// release travels from the hold posture back to q_pre and must SETTLE there
/// (q_tol), where T_close_e2e only timed ρ reaching η. Of the two limit plants
///   torque-saturated: time ∝ travel          → 1/η
///   first-order:      time ∝ ln(travel/q_tol) → ln(S_max/q_tol) / ln(1/(1−η))
/// the slower is taken and doubled, the margin kCloseTimeoutPerE2e gives the
/// close. S_max is the widest |q_close − q_pre| over EVERY joint (the arrival
/// test reads every joint). Returns NaN for an input it cannot use.
[[nodiscard]] inline double ReleaseTimeoutPerE2e(double eta, double s_max, double q_tol) noexcept {
  if (!std::isfinite(eta) || !(eta > 0.0) || eta > 1.0 || !std::isfinite(s_max) ||
      !std::isfinite(q_tol) || !(q_tol > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double ratio = 1.0 / eta;
  // η = 1 makes ln(1/(1−η)) infinite: the first-order term then vanishes.
  if (eta < 1.0 && s_max > q_tol) {
    ratio = std::max(ratio, std::log(s_max / q_tol) / -std::log1p(-eta));
  }
  return kCloseTimeoutPerE2e * ratio;
}

/// Per-joint closure progress
///   ρ_i = (q_i − q_pre,i)·s_i / |q_close,i − q_pre,i|,   s_i = sign(q_close,i − q_pre,i)
/// 0 at q_pre, 1 at q_close, signed by the closing DIRECTION (a joint whose
/// q_close is below its q_pre closes by decreasing). The one definition the
/// sequencer's ρ = min over the caging joints C (closure reached when ρ ≥ η)
/// and the capture witness (per joint) both use. Non-finite for a zero span.
[[nodiscard]] inline double JointClosureProgress(double q, double q_pre, double q_close) noexcept {
  const double span = q_close - q_pre;
  const double s = span > 0.0 ? 1.0 : -1.0;
  return (q - q_pre) * s / std::abs(span);
}

/// `robot.hand.capture`: the hand-joint evidence the
/// attempt verdict adds to the fingertip agreement. A caging joint is STALLED
/// when it stopped part-way — rho_min ≤ ρ_i ≤ rho_max, |q̇_i| ≤ q̇_tol — while
/// still pushing toward q_close (s_i·τ_i / max_torque_i ≥ effort_frac_min).
/// `enabled` is the section's presence: without it the verdict is the
/// fingertips' alone. `provisional` blocks a real-arm configuration, since the
/// effort lane a real hand driver publishes need not be a joint torque.
struct HandCaptureParams {
  bool enabled{false};
  TbdDouble rho_min;          // –, [0, rho_max)
  TbdDouble rho_max;          // –, (rho_min, 1)
  TbdDouble effort_frac_min;  // –, (0, 1]
  TbdDouble t_persist;        // s, [0, T_hold)
  int min_joints{1};          // [1, caging joints]
  bool provisional{true};
};

/// The hold rule (`robot.hand.hold.mode`): what the hand is commanded to
/// once closure is reached. `kCloseTarget` keeps commanding `q_close` (the
/// posture pair already includes "a little squeeze" past the stall); the
/// `kMeasuredOffset` form commands the measured pose at Hold entry plus
/// `hold.delta_rad` toward `q_close` on every caging joint, for a hand whose
/// `q_close` would squeeze too hard once it is a real one.
enum class HandHoldMode : std::uint8_t { kCloseTarget, kMeasuredOffset };

/// One hand profile (`robot.hand.*`, scope limited to what the caging check
/// and the hand-timing measurement need). `dof` is the number of
/// valid entries in the arrays; `tbd` is true while the `q_pre`/`q_close` pair
/// is still the `TBD` placeholder. `q_open_tbd` tracks the third pose array
/// separately: it is not part of the caging pair, so a profile that gives
/// the pair but omits `q_open` parses (and the validator reports the one key)
/// rather than being refused wholesale. `provisional` reads an invented
/// `robot.hand.provisional` key (see .cpp): every hand-profile value is
/// provisional, with no schema field to say so.
struct HandProfile {
  std::array<double, kMaxHandDof> q_open{};  // homing pose
  std::array<double, kMaxHandDof> q_pre{};
  std::array<double, kMaxHandDof> q_close{};
  std::array<bool, kMaxHandDof> caging_mask{};  // caging joints C (the gap check, ρ)
  int dof{0};
  // Closure, over the caging joints C (caging_mask), each of which needs
  // |q_close,i − q_pre,i| > rho_eps:
  //   ρ(t) = min_{i∈C} (q_i(t) − q_pre,i)·s_i / |q_close,i − q_pre,i|,
  //   s_i = sign(q_close,i − q_pre,i)                    (JointClosureProgress)
  //   T_close,e2e(η) = inf{ t − t_cmd : ρ(t) ≥ η }      (steady clock)
  // t_cmd the tick the close command was recorded; q measured (encoders).
  double rho_eps{0.02};   // [rad] min |q_close − q_pre| on a caging joint
  TbdDouble eta_close;    // –, [0.5, 1]   — closure threshold η on ρ
  TbdDouble T_close_e2e;  // s, >= 0       — T_close,e2e(η_close) (MEASURED)
  /// How long before the ball reaches the catch point (the catch-frame
  /// origin) the hand is told to close [s], >= 0. A DESIGN value of
  /// the hand, apart from the measured T_close_e2e — a hand that holds only
  /// when it closes after the ball has arrived wants a lead well below its
  /// closure time. What reads which: the lead times the command (the
  /// sequencer, the plan's t_cmd, the freeze-window bound, the search's commit
  /// gate); T_close_e2e bounds how long a closure may take (the timeouts) and
  /// is what the closing hand absorbs (the search's γ window).
  /// The sequencer subtracts a lead from a plan's CATCH INSTANT, which is that
  /// arrival only for a planner that puts t_c there; the binding hands each
  /// reader the lead from the t_c its configuration plans with.
  /// Absent from the YAML the lead IS T_close_e2e — the two were one key
  /// before, and a profile written then keeps the timing it had. Read it
  /// through CloseLead(), which applies that rule; this field alone is only
  /// what the YAML wrote.
  TbdDouble T_close_lead;
  bool T_close_lead_given{false};  // true when the YAML has the key
  // ── Sequencer ──────────────────────────────────────────────────────────────
  /// How long HOLD lasts before RETREAT [s], [0, 5]. 0.5 s, not 1.0: a trial
  /// cycle is dominated by this wait and the ball is caged well before it
  /// (provisional).
  TbdDouble T_hold{TbdDouble::Resolved(0.5)};
  /// Close → Hold without reaching η [s], > T_close_e2e. Absent from the YAML
  /// it is DERIVED at parse, kCloseTimeoutPerE2e × T_close_e2e — a stored
  /// default would outlive the measurement it was derived from.
  TbdDouble T_close_timeout;
  /// RETREAT's wait for the hand at q_pre [s], > T_close_e2e. Absent
  /// from the YAML it is DERIVED at parse, ReleaseTimeoutPerE2e × T_close_e2e.
  TbdDouble T_release_timeout;
  bool T_release_timeout_derived{false};  // true when the key was absent
  HandCaptureParams capture;
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

  /// The lead the sequencer runs on: `T_close_lead` when the profile gives it
  /// (resolved or the `TBD` literal), else `T_close_e2e`.
  [[nodiscard]] constexpr TbdDouble CloseLead() const noexcept {
    return T_close_lead_given ? T_close_lead : T_close_e2e;
  }
};

/// Ball specification (`core.ball.*`), user-supplied. `provisional` reads an
/// invented `core.ball.provisional` key (see .cpp): diameter/mass/restitution
/// are provisional with no schema field to say so.
struct BallSpec {
  TbdDouble diameter;     // m,  [0.02, 0.3]
  TbdDouble mass;         // kg, [0.005, 1.0]
  TbdDouble restitution;  // –,  [0, 1]
  bool provisional{true};
};

/// Parsed subset of the `catching:` YAML tree the validator needs. See
/// the file header for scope and the .cpp for every invented key.
struct CatchingParams {
  // reference: the closed_form soft-catch reference (SoftCatchTranslation)
  TbdDouble reference_omega{TbdDouble::Resolved(10.0)};  // rad/s, [1, 25]
  TbdDouble reference_zeta{TbdDouble::Resolved(1.0)};    // –, v1 requires exactly 1
  TbdDouble reference_v_max;                             // m/s, > 0
  TbdDouble reference_a_max;                             // m/s², > 0
  /// Provisional flag for the whole reference block, read from an invented
  /// `reference.provisional` key (see the .cpp header for why these keys are
  /// invented and why they default to true).
  ///
  /// It exists because `reference.a_max` is a DERIVED bound whose derivation
  /// is still open, while `v_max` is decided — so the two cannot share a TBD.
  /// A profile that left `a_max` at TBD instead would be refused outright in
  /// sim, and because CM latches `bring_up_failed` on any controller's
  /// configure failure that takes EVERY controller on the robot down with it
  /// (observed on a shipped sim profile). The provisional rule says the right
  /// thing instead: sim warns, a real arm is blocked.
  bool reference_provisional{true};

  // planner.search.grid: the grid search's own design values
  TbdDouble planner_search_grid_gamma_eta_v{TbdDouble::Resolved(0.9)};         // –, (0, 1]
  TbdDouble planner_catchability_manip_min_arm5row{TbdDouble::Resolved(0.1)};  // –, >= 0
  bool planner_catchability_manip_min_provisional{true};  // invented key, see .cpp
  /// `planner.search.grid.reference.*` and `planner.search.grid.stop.a_dec` —
  /// the reference and the stop the search rolls a candidate out on
  /// (ChooseGamma, StoppingPoint). The search runs under both segment modes, so it has its own keys
  /// for these five rather than reading the closed_form law's (`reference.*`,
  /// `supervisor.decel.a_dec`). Same defaults and ranges as the keys they
  /// copy; what a difference between the two means is SearchCopiesThatDiffer's.
  TbdDouble planner_search_grid_reference_omega{TbdDouble::Resolved(10.0)};  // rad/s, [1, 25]
  TbdDouble planner_search_grid_reference_zeta{TbdDouble::Resolved(1.0)};    // –, exactly 1
  TbdDouble planner_search_grid_reference_v_max;                             // m/s, > 0
  TbdDouble planner_search_grid_reference_a_max;                             // m/s², > 0
  TbdDouble planner_search_grid_stop_a_dec;  // m/s², > 0 and <= the a_max above

  // planner.search.mode: which search plans
  CatchingSearchMode planner_search_mode{CatchingSearchMode::kGrid};

  // planner.segment: what the arm follows
  CatchingSegmentMode planner_segment_mode{CatchingSegmentMode::kClosedForm};
  /// `planner.segment.mpc.switch_margin` — ρ_max of the segment-switch
  /// continuity gate, > 0. Refused at parse time when not a positive
  /// finite number; read only under kMpc.
  double planner_segment_mpc_switch_margin{1.0};
  /// `planner.segment.mpc.eta_v` — the mpc segment planner's speed margin on
  /// the joint ratings and the headroom (1 − η_v) of the RT's segment-switch
  /// gate, (0, 1]. Its own key, beside the search's
  /// `planner.search.grid.gamma.eta_v`; judged only under kMpc.
  TbdDouble planner_segment_mpc_eta_v{TbdDouble::Resolved(0.9)};

  // supervisor:
  TbdDouble supervisor_decel_a_dec;  // m/s², > 0 and <= reference_a_max (EvaluateDecelTarget)

  // io: vision ingress.
  //
  // `io_n_min` is a COUNT, so it is an int with 0 meaning "absent" rather than
  // a TbdDouble: the schema's TBD placeholder exists for values a decision has
  // not produced yet, and a point count that arrived as 10.5 is a malformed
  // key, not an open one. 0 is reported as an active TBD.
  int io_n_min{0};           // points, [2, kCap]
  TbdDouble io_t_stale;      // s, [0.02, 0.2]  — steady RECEIVE age limit
  TbdDouble io_future_tol;   // s, [1e-4, 1e-2] — real-arm clock-sync budget
  TbdDouble io_horizon_min;  // s, > 0        — usable-window requirement
  TbdDouble io_track_eval_offset{TbdDouble::Resolved(0.05)};  // s, [0, 0.3]
  TbdDouble io_track_j_warn;  // m, > 0 — DIAGNOSTIC threshold, never a gate
  /// The sim configuration's own `future_tol`. Active only when
  /// `real_arm_config` is false, exactly like `sim.ball.drag_k`.
  ///
  /// It exists because the two numbers are two orders of magnitude apart and
  /// the controller YAML is shared between sim and hardware: the sim ball
  /// lane's stamps ride the SIM time axis and legitimately lead wall by the
  /// in-flight phase error (`rtc_mujoco_sim` §Projectile Ball stamp),
  /// while a camera on real hardware stamps at capture and may lead wall only
  /// by the clock-sync error. One key with one range could serve only one of
  /// them, and the sim value inside the real-arm range would silently accept a
  /// 100 ms clock offset on hardware.
  TbdDouble sim_io_future_tol;  // s, (0, 0.5]

  // prediction: what the vision profile is expected to produce.
  TbdDouble prediction_dt_expected{TbdDouble::Resolved(0.05)};  // s, (0, 1]

  // joint_cmd: the CLIK step.
  //
  // The weights keep the ordering w_task >> w_arm >> damping_sq: the task
  // must win, the posture must only resolve the redundant
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
  /// `joint_cmd.accel_constraint`: which acceleration
  /// constraint the CLIK QP carries — `kinematic` (task acceleration
  /// J·v̇ + J̇·v of the tracked rows) or `dynamic` (arm torque
  /// M·v̇ + h ≤ η_τ·τ_max, τ_max = the arm device's `joint_limits.max_torque`).
  /// Each form reads only its own keys below; a key of another form is refused
  /// at parse. No default: an absent key (or `TBD`) stays kUnset and the
  /// validator reports `joint_cmd.accel_constraint` as kActiveConfigTbd. `box`
  /// reads as kRemovedBox and is reported as kRemovedValue — refusing it at
  /// parse would fail the configure, where the caller is meant to park.
  CatchingAccelConstraint joint_cmd_accel_constraint{CatchingAccelConstraint::kUnset};
  TbdDouble joint_cmd_task_accel_max_linear{};   // m/s², > 0 — kinematic
  TbdDouble joint_cmd_task_accel_max_angular{};  // rad/s², > 0 — kinematic
  /// η_τ of the torque rows. The default 0.8 is the margin the joint-accel
  /// box is derived with: the same torque budget, now spent per tick.
  TbdDouble joint_cmd_eta_tau{TbdDouble::Resolved(0.8)};  // –, (0, 1] — dynamic
  /// T_arm [s], the arm servo lag the lead axis runs ahead by
  /// (now_lead = now + T_arm). The parser default is 0 — no lag, and the lead
  /// axis then coincides with the real one. A non-zero value is the lag of the
  /// plant the configuration runs on: a sim servo's, the hardware
  /// identification, or the axis-confusion fixture.
  TbdDouble joint_cmd_lag_t_arm{TbdDouble::Resolved(0.0)};  // s, >= 0
  bool joint_cmd_lag_lead_enable{false};
  /// `lag.provisional` blocks a real-arm configuration until T_arm has been
  /// identified on the arm it runs on — whatever the
  /// value and whether or not the lead compensation reads it, since T_arm also
  /// sits in the `T_freeze` floor. Absent = true (fail-closed).
  bool joint_cmd_lag_provisional{true};

  // robot.arm: the boxes CLIK is given.
  /// How far INSIDE the device's own position limits the CLIK box sits. The
  /// backend clamps commands to the device limits; if CLIK were given the same
  /// box, its solution and the command actually written would differ whenever
  /// it touched a bound, and the difference would surface as a tracking error
  /// nobody can attribute.
  TbdDouble robot_arm_limit_margin{TbdDouble::Resolved(0.05)};  // rad, [0, 0.3]

  // supervisor: the two keys the joint command layer reports to.
  TbdDouble supervisor_track_err_abort;  // rad, > 0 — the supervisor owns it, CLIK only reads it
  int supervisor_n_qp{0};  // consecutive trials ended by a CLIK failure before FAULT, >= 1

  // supervisor: the supervisor driver's keys. Defaults are provisional.
  /// How long COMMITTED/CLOSING may run on a stale prediction past io.t_stale
  /// before it is BALL_STALE_LONG [s], [0, 1].
  TbdDouble supervisor_stale_committed_max_s{TbdDouble::Resolved(0.10)};
  /// REF_SATURATED: this many consecutive saturated reference ticks.
  /// 60 (0.12 s at 500 Hz): a reference that starts an approach from rest
  /// saturates while it catches up with the plan. Measured: unit fixtures ran
  /// 10 ticks (a 14 cm approach) and 76 (a 60 cm approach, run with the check
  /// off); sim throws (25 trials) had a longest normal streak of 44 (p99
  /// 42.5), and 50 fired once, on a ball that had already missed. Every
  /// shipped robot profile sets it explicitly (value and measurement in that
  /// profile's controller YAML), so this default only covers a config without
  /// the key.
  int supervisor_sat_ticks{60};
  /// Joint-space homing / return: speed cap [rad/s] > 0, fraction of
  /// the derived q̈_max (0, 1], and the arrival velocity bound [rad/s] > 0.
  double supervisor_homing_v_max{0.5};
  double supervisor_homing_eta_a{0.5};
  double supervisor_homing_qd_tol{0.02};
  /// Motion deadlines [s], > 0. `stop_s` bounds a stop —
  /// ABORT_SAFE's ramp, and RETREAT's stop stage until the measured arm has
  /// caught up with the stopped command; `return_s` bounds RETREAT's return to
  /// the wait pose. Past either, the supervisor latches a fault
  /// (ABORT_ESCALATED → FAULT). Two keys because a return scales with the
  /// distance to the wait pose and a stop does not: one key would have to be
  /// as loose as the longest return. Defaults are twice the longest such stage
  /// in the sim sessions, floored at 0.5 s; each shipped profile sets its
  /// own robot's, and these parser defaults are the larger robot's (2 × 2.138
  /// s stop, 2 × 7.380 s return), so a config without the keys is not the
  /// tighter one. `deadline.provisional` blocks a real-arm configuration
  /// until the real arm is measured.
  TbdDouble supervisor_deadline_stop_s{TbdDouble::Resolved(4.28)};
  TbdDouble supervisor_deadline_return_s{TbdDouble::Resolved(14.76)};
  bool supervisor_deadline_provisional{true};
  /// "At the wait pose": max |q − wait_pose| [rad] > 0 (also the test that
  /// skips homing for an arm already there).
  double supervisor_ready_pose_tol{0.02};
  /// Contact judgement (ContactDebouncer). A fingertip is
  /// in contact when |F − b| > max(f_min, k_sigma·σ̂) for n_debounce samples
  /// in a row; the attempt holds the ball when m_min fingertips agree. b and
  /// σ̂ are an EMA (baseline_alpha) learned at q_pre in ARMED/TRACKING, and
  /// fewer than n_baseline_min samples makes the verdict Undetermined.
  double supervisor_contact_f_min{0.2};            // N, >= 0 (the user's value)
  double supervisor_contact_k_sigma{3.0};          // –, >= 0
  int supervisor_contact_n_debounce{3};            // samples, >= 1
  int supervisor_contact_m_min{2};                 // fingertips, >= 1
  double supervisor_contact_t_confirm{0.2};        // s, [0, 1] — window [t_cmd, t_c + T_confirm]
  double supervisor_contact_t_stale{0.02};         // s, (0, 0.5] — TIP_STALE age limit
  double supervisor_contact_baseline_alpha{0.02};  // –, (0, 1]
  int supervisor_contact_n_baseline_min{20};       // samples, >= 1

  // core / sim:
  BallSpec ball;
  TbdDouble sim_ball_drag_k;  // 1/m, [0, 0.2] — sim-fixture-only (active iff !real_arm_config)

  // robot:
  HandProfile hand;
};

/// Parse the `catching:` map. An absent
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
  kActiveConfigTbd,            // an active-config key is still the TBD placeholder
  kControlRateOutOfRange,      // control_rate_hz outside [kMinControlRateHz, kMaxControlRateHz]
  kRangeViolation,             // a resolved value is outside its range (field comments)
  kZetaNotCriticallyDamped,    // reference.zeta != 1 (v1 requires the closed-form solution)
  kEtaVOutOfRange,             // an eta_v key not in (0, 1]
  kDecelExceedsAMax,           // an a_dec key above its a_max
  kUnstableDiscretization,     // omega*h >= 2*sqrt(2)-2 (discrete stability, SoftCatchTranslation)
  kDiscretizationAccuracy,     // omega*h > 0.05 (accuracy recommendation) — WARNING only
  kHandCagingGapTooSmall,      // |q_close[i] - q_pre[i]| <= rho_eps on a caging joint
  kProvisionalOnRealArm,       // a provisional value blocks the real-arm configuration
  kProvisionalWarning,         // same value, but the sim configuration only warns — WARNING only
  kWeightOrdering,             // CLIK weights violate w_task/w_a >> w_arm >> damping_sq
  kCloseTimeoutNotAboveE2e,    // robot.hand.T_close_timeout <= T_close_e2e
  kFreezeShorterThanClose,     // T_freeze < T_close_lead + T_arm + h (CheckFreezeCoversClose)
  kReleaseTimeoutNotAboveE2e,  // robot.hand.T_release_timeout <= T_close_e2e
  kRemovedValue,  // the key holds a value that no longer exists (accel_constraint: box)
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

/// Fixed-capacity validation result: `armable`, the failed keys and the
/// warnings. No heap: `failures`/`warnings` are
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

/// The arming check. `control_rate_hz` is the base controller's already-resolved
/// `control_rate` (rtc_base owns that key; this validator only consumes the
/// number). `real_arm_config` selects the active-configuration set (see file
/// header) and gates every `provisional` flag (sim warns, real-arm
/// blocks). Allocation-free and noexcept: every input is already a POD, and a
/// full report is not something we would refuse to construct.
[[nodiscard]] CatchingValidationReport ValidateCatchingParams(const CatchingParams& params,
                                                              double control_rate_hz,
                                                              bool real_arm_config) noexcept;

/// Which `future_tol` applies to a configuration.
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

/// Lower bound on the freeze window: T_freeze ≥ T_close_lead + T_arm + h
/// [s], h the control period. COMMITTED starts at t_c − T_freeze and the close command goes
/// out at t_cmd = t_c − T_close_lead, so a window shorter than the lead has
/// already missed its own close command on the tick it commits; T_arm is the
/// servo lead the reference runs ahead by, and one tick h is the rounding of
/// both instants onto the tick grid.
///
/// `t_freeze_s` is `planner.freeze.T_freeze` (the planner's key, parsed by
/// planner_params.hpp — this header does not depend on that one, so the
/// caller passes the number, the same arrangement as
/// CheckCatchFrameProvisional). A NaN `t_freeze_s` or an unresolved
/// T_close_lead / T_arm is not judged here: an unset value is reported by the
/// check that owns it. Allocation-free, noexcept.
void CheckFreezeCoversClose(CatchingValidationReport& report, const CatchingParams& params,
                            double t_freeze_s, double control_rate_hz) noexcept;

/// One of the grid search's copies and the closed_form law's key it copies
/// (both dotted, below `catching:`).
struct CatchingKeyCopy {
  const char* copy;
  const char* source;
};

/// The search's copies (`planner.search.grid.reference.*`,
/// `planner.search.grid.stop.a_dec`) whose value differs from the closed_form
/// law's key (`reference.*`, `supervisor.decel.a_dec`), written to the front
/// of `out`; returns how many. A pair with a TBD on either side is skipped —
/// the validator reports the TBD. What a difference means is the caller's:
/// under kClosedForm the search would rank candidates by a motion the arm does
/// not make, under kMpc the two are different functions' values.
[[nodiscard]] std::size_t SearchCopiesThatDiffer(const CatchingParams& params,
                                                 std::array<CatchingKeyCopy, 5>& out) noexcept;

/// Key reported for the catch frame's provisional flag.
inline constexpr const char* kCatchFrameProvisionalKey =
    "urdf.extra_frames.<catch_frame>.provisional";

/// Applies the provisional rule to the catch frame: a
/// provisional frame warns in sim and blocks a real-arm configuration. The
/// flag lives in the robot config (`urdf.extra_frames.<name>.provisional`,
/// carried by rtc_urdf_bridge::ExtraFrameConfig), not in the `catching:`
/// section, so the caller reads it from the model config and adds it to the
/// report ValidateCatchingParams produced. Allocation-free, noexcept.
void CheckCatchFrameProvisional(CatchingValidationReport& report, bool catch_frame_provisional,
                                bool real_arm_config) noexcept;

}  // namespace rtc::catching
