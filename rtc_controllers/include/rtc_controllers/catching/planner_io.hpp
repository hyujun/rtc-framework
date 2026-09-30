// ── Planner ↔ RT data contract (dynamic_catching S6-A, L3 §5.2–5.3, D-7) ────
//
// The two POD payloads that cross the planner-thread boundary, and the RT
// side's admission rule for a published plan. ROS-free and allocation-free: the
// planner thread may run SCHED_FIFO (D-7a), so everything it touches follows
// RT-1~10, and the RT tick evaluates the admission rule every tick.
//
// DIRECTION AND WRITERS. Each payload rides its own `rtc::SeqLock` and each
// has exactly ONE writer:
//   - `PlannerRtState`  RT tick → planner. Stored EVERY tick.
//   - `PlanSnapshot`    planner → RT (trajectory.hpp). Stored by the planner,
//     or — in a test profile only — by the RT tick's oracle stand-in
//     (A-S5-8). The two are never enabled together; the binding parks a
//     configuration that asks for both, because a SeqLock with two writers is
//     a torn read waiting to happen.
//   - `DecelPlanSnapshot` planner → RT (trajectory.hpp, MPC E1-F02). Stored
//     by the planner only; judged by `JudgeDecelPlan` below. The RT tick reads
//     it under `supervisor.decel.mode: mpc` only (E1-F04, MD-44).
//
// WHY `plan_id` DECIDES NEWNESS (L3 §5.2, S6 implementation note). The
// provenance token's `snapshot_sequence` belongs to the TRAJECTORY the plan
// was computed from, so two plans computed from one trajectory — the planner
// re-planning on a timeout wake, or after the RT state moved — carry the same
// token. `plan_id` is the writer's own monotone counter and is the only field
// that separates them.
//
// WHY THE RESET FLOOR. An E-STOP resets the trial WITHOUT bumping the
// activation generation (that is the lifecycle's counter, not the supervisor's),
// so a plan published a moment before the stop would still pass the generation
// check afterwards and be adopted by the next trial. The RT records when it
// last reset and refuses any plan published before that instant. The RT does
// NOT invalidate the box itself: it is not the box's writer.
#pragma once

#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/catching/transition_table.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>

namespace rtc::catching {

/// RT tick → planner (L3 §5.3 "데이터"). What the planner needs to know about
/// the controller it is planning for, as of one tick.
///
/// Filled from scratch every tick (a default-constructed value, then the
/// fields this tick knows), so a field the tick did not compute is zero or
/// false rather than a previous tick's value — the same PROC-7 discipline the
/// controller's own tick record follows.
struct PlannerRtState {
  /// `false` until the first tick of a configured controller has stored it.
  bool valid{false};
  /// The controller's ActivationGeneration() on this tick (D-23). A planner
  /// output stamped with anything else is refused by the RT.
  std::uint64_t activation_generation{0};
  /// ControllerState::iteration and the tick's steady instant (D-22: the
  /// plan's `rt_iteration` / `rt_state_ns` are copied from here).
  std::uint64_t rt_iteration{0};
  std::int64_t rt_state_ns{0};
  /// Bumped by the RT on every trial reset (L7 §4.8). The planner compares it
  /// against the value it last saw and, on a change, drops whatever it was
  /// holding and drains its wake signal — the RT cannot do either itself.
  std::uint32_t reset_epoch{0};
  /// The supervisor mode (Mode) after this tick's decision.
  std::uint8_t mode{0};
  /// The operator arm latch as the tick saw it.
  bool armed{false};

  /// Arm command state, DEVICE order, `nv` entries. `cmd_seeded` false means
  /// the command is not tied to a measurement yet and the arrays are zero.
  std::int32_t nv{0};
  bool cmd_seeded{false};
  std::array<double, kMaxPlanNv> q_cmd{};
  std::array<double, kMaxPlanNv> qd_cmd{};

  /// The wait pose the RT homes to and the planner seeds its IK from, DEVICE
  /// order (S8-I, `planner.wait_pose_source: current`). `wait_pose_adopted`
  /// false means the RT runs the configure-time YAML pose, which the planner
  /// already holds as its seed; true means the arrays carry the pose adopted
  /// on this activation's first readable tick and the seed must follow it.
  bool wait_pose_adopted{false};
  std::array<double, kMaxPlanNv> wait_pose{};

  /// The L4 reference state (x, ẋ, γ, γ̇, γ̈) the tracking law produced on
  /// this tick — the input to the §4.7 switching rule (γ̇ and γ̈ size the step
  /// a replacement's restarted γ ramp puts into u_des). `ref_valid` false on
  /// every tick that did not run the law.
  bool ref_valid{false};
  std::array<double, 3> ref_x{};
  std::array<double, 3> ref_xd{};
  double gamma{0.0};
  double gamma_d{0.0};
  double gamma_dd{0.0};
  /// The γ ramp the law is running (the followed plan's, as the RT adopted
  /// it — g0 and t0 are the RT's, not the planner's), lead axis. The switch
  /// rule evaluates it over the instants the RT may adopt a new plan: γ̇ and
  /// γ̈ above are this tick's and may be 0 just before the ramp starts
  /// (2026-09-23 /code-review). `ramp_valid` false with no plan followed.
  bool ramp_valid{false};
  double ramp_g0{0.0};
  double ramp_gf{0.0};
  std::int64_t ramp_t0_ns{0};
  std::int64_t ramp_t1_ns{0};

  /// The plan the RT is following, if any.
  bool plan_active{false};
  std::uint32_t plan_id{0};
  /// That plan's catch instant t_c (BallTime / lead axis), 0 with no plan.
  /// The decel planner's grid anchor t_c + k·Δ_s (MD-10): it must be the
  /// plan the RT FOLLOWS, not the one the planner last published — after
  /// COMMITTED the RT takes no new plan, so the two can differ.
  std::int64_t plan_t_c_ns{0};
  /// Whether the RT is following a decel segment this tick, and which one
  /// (its decel_seq). The planner predicts the next segment's initial state
  /// from that segment when it is its own latest (MD-28 path (i)). Always
  /// false / 0 under `supervisor.decel.mode: closed_form`, and outside
  /// DECEL / HOLD (E1-F04).
  bool decel_active{false};
  std::uint32_t decel_seq{0};

  /// The vision track epoch of the last trajectory the RT consumed (L1 §4.4),
  /// and whether it has consumed one at all in this trial.
  bool track_seen{false};
  std::uint64_t track_generation{0};
};

static_assert(std::is_trivially_copyable_v<PlannerRtState>);

/// What the planner does in a given supervisor mode (L3 §5.3). The decel MPC
/// (MPC E1-F03) runs alongside kMonitor and in kDecel when it is configured.
///
/// A PREDICATE over the mode, not an ordering test: `mode >= kCommitted` would
/// silently change meaning the day a mode is inserted into the enum (L3 §5.3
/// "isFrozen").
enum class PlannerActivity : std::uint8_t {
  kIdle,     ///< nothing to plan for — wake, publish nothing
  kSearch,   ///< TRACKING / APPROACH: search candidates, publish a plan
  kMonitor,  ///< COMMITTED / CLOSING: the plan is frozen; monitorOnly (§4.6)
  /// DECEL: nothing to search or monitor, but the decel MPC's post-catch
  /// replans (MD-31) happen here — a no-op without a configured decel planner.
  kDecel,
};

[[nodiscard]] constexpr PlannerActivity ActivityFor(Mode mode) noexcept {
  switch (mode) {
    case Mode::kTracking:
    case Mode::kApproach:
      return PlannerActivity::kSearch;
    case Mode::kCommitted:
    case Mode::kClosing:
      return PlannerActivity::kMonitor;
    case Mode::kDecel:
      return PlannerActivity::kDecel;
    case Mode::kIdle:
    case Mode::kArmed:
    case Mode::kHold:
    case Mode::kRetreat:
    case Mode::kAbortSafe:
    case Mode::kFault:
      return PlannerActivity::kIdle;
  }
  return PlannerActivity::kIdle;
}

// ── RT-side admission (L3 §5.2 (a)–(f)) ─────────────────────────────────────

/// Why the RT did not take the plan in the box. `kNone` = admitted.
///
/// These are NOT supervisor reasons: a refused plan is, to the supervisor,
/// simply "no plan" (kNoCatchablePlan in TRACKING). They exist for the tick
/// record and the tests, which need to tell a planner that published nothing
/// from one whose output the RT threw away.
enum class PlanRefusal : std::uint8_t {
  kNone = 0,
  kInvalid,      ///< (a) `valid` false — the writer says there is no plan
  kActivation,   ///< (b) stamped with another activation generation (D-23)
  kTrack,        ///< (c) computed from a different vision track than the RT holds
  kRepeat,       ///< (d) the plan the RT already took
  kAged,         ///< (e) published longer ago than the admission age bound
  kBeforeReset,  ///< (f) published before the RT's last trial reset
  kTooLate,      ///< (g) its catch instant is already inside the freeze window (S7)
};

/// What the RT knows when it judges a plan. All of it is the RT's own state.
struct PlanAdmissionContext {
  std::uint64_t activation_generation{0};
  /// The vision epoch of the last trajectory the RT consumed. A plan cannot be
  /// admitted before the RT has consumed a trajectory at all (`track_seen`).
  bool track_seen{false};
  std::uint64_t track_generation{0};
  NowReal now{0};
  /// Upper bound on `now − publish_ns` [ns]. The binding uses `io.t_stale`:
  /// a plan older than the ingress staleness bound was computed against a
  /// prediction the RT would itself refuse as stale.
  std::int64_t max_age_ns{0};
  /// Steady instant of the RT's last trial reset; plans published before it
  /// belong to the trial the reset ended. 0 = no reset yet.
  std::int64_t reset_floor_ns{0};
  /// (g) T_freeze [ns] (L7 §4.8's third admission condition, #537 S7): a plan
  /// whose t_c − now is not ABOVE this would commit on the tick it is taken —
  /// and with t_c ≤ now, decelerate the tick after — so the controller would
  /// commit to a catch it never approached. 0 disables the check (a profile
  /// with no freeze window has nothing to be inside of).
  std::int64_t t_freeze_ns{0};
};

/// The RT's memory of the last plan it admitted (payload-side, D-21 — never
/// the SeqLock's own sequence).
struct AdmittedPlan {
  bool seen{false};
  std::uint32_t plan_id{0};
};

/// Judge a plan the caller has already loaded (D-21: `Load()` once per tick,
/// unconditionally, then call this). Checks run in the order the enum lists
/// them, so the refusal names the FIRST reason.
[[nodiscard]] constexpr PlanRefusal JudgePlan(const PlanSnapshot& plan,
                                              const PlanAdmissionContext& ctx,
                                              const AdmittedPlan& admitted) noexcept {
  if (!plan.valid) {
    return PlanRefusal::kInvalid;
  }
  if (plan.token.activation_generation != ctx.activation_generation) {
    return PlanRefusal::kActivation;
  }
  if (!ctx.track_seen || plan.token.generation != ctx.track_generation) {
    return PlanRefusal::kTrack;
  }
  if (admitted.seen && plan.plan_id == admitted.plan_id) {
    return PlanRefusal::kRepeat;
  }
  // A publish instant in the FUTURE is as untrustworthy as an old one — it can
  // only come from a writer on another clock — and it would otherwise pass an
  // age test by being negative.
  const std::int64_t age = ctx.now.ns - plan.publish_ns;
  if (plan.publish_ns <= 0 || age < 0 || age > ctx.max_age_ns) {
    return PlanRefusal::kAged;
  }
  if (plan.publish_ns < ctx.reset_floor_ns) {
    return PlanRefusal::kBeforeReset;
  }
  if (ctx.t_freeze_ns > 0 && detail::SatSub(plan.t_c_ns, ctx.now.ns) <= ctx.t_freeze_ns) {
    return PlanRefusal::kTooLate;
  }
  return PlanRefusal::kNone;
}

// ── RT-side admission of a decel segment (MPC E1-F03, MD-27 · MD-32) ────────

/// Why the RT did not take the decel segment in the box. `kNone` = admitted.
enum class DecelRefusal : std::uint8_t {
  kNone = 0,
  kInvalid,      ///< `valid` false — the planner withdrew it (e.g. on a reset)
  kActivation,   ///< another activation generation (D-23)
  kPlan,         ///< not a stop for the plan the RT follows (id or t_c differ)
  kRepeat,       ///< decel_seq not newer than the one the RT already took
  kAged,         ///< published outside [now − max_age, now]
  kBeforeReset,  ///< published before the RT's last trial reset
  kMalformed,    ///< ValidateDecelNodes refused the shape or a node value
};

/// What the RT knows when it judges a decel segment.
struct DecelAdmissionContext {
  std::uint64_t activation_generation{0};
  /// The plan the RT follows: a stop belongs to exactly one catch plan.
  bool plan_active{false};
  std::uint32_t plan_id{0};
  std::int64_t plan_t_c_ns{0};
  NowReal now{0};
  /// Upper bound on `now − publish_ns` [ns]. NOT the trajectory's staleness
  /// bound: a segment solved t_pre before t_c is adopted around t_c and
  /// followed for N_s·Δ_s after, so it is read once, at admission (MD-37 —
  /// the binding's kDecelAdmissionMaxAgeNs). 0 disables.
  std::int64_t max_age_ns{0};
  std::int64_t reset_floor_ns{0};
  /// Floor on the RT state the segment was PREDICTED from (`rt_state_ns`),
  /// refused as kBeforeReset below it; 0 disables (MD-37). A planner that
  /// stamped its publish after the RT's reset can still have read the RT
  /// state of the tick before it: the publish floor alone lets that through.
  std::int64_t state_floor_ns{0};
};

/// The RT's memory of the last decel segment it admitted.
struct AdmittedDecel {
  bool seen{false};
  std::uint32_t decel_seq{0};
};

/// Judge a decel segment the caller has already loaded (D-21: Load() every
/// tick, unconditionally). Checks run in the enum's order; the node scan
/// (kMalformed) is last because it is the only one that costs anything, and
/// a caller that admits a segment runs it once per decel_seq.
[[nodiscard]] inline DecelRefusal JudgeDecelPlan(const DecelPlanSnapshot& p,
                                                 const DecelAdmissionContext& ctx,
                                                 const AdmittedDecel& admitted) noexcept {
  if (!p.valid) {
    return DecelRefusal::kInvalid;
  }
  if (p.token.activation_generation != ctx.activation_generation) {
    return DecelRefusal::kActivation;
  }
  if (!ctx.plan_active || p.plan_id != ctx.plan_id || p.t_c_ns != ctx.plan_t_c_ns) {
    return DecelRefusal::kPlan;
  }
  // `>` on the planner's own counter (it starts at 1 and never wraps within a
  // controller lifetime at one store per wake).
  if (admitted.seen && !(p.decel_seq > admitted.decel_seq)) {
    return DecelRefusal::kRepeat;
  }
  const std::int64_t age = ctx.now.ns - p.publish_ns;
  if (p.publish_ns <= 0 || age < 0 || (ctx.max_age_ns > 0 && age > ctx.max_age_ns)) {
    return DecelRefusal::kAged;
  }
  if (p.publish_ns < ctx.reset_floor_ns) {
    return DecelRefusal::kBeforeReset;
  }
  if (ctx.state_floor_ns > 0 && p.rt_state_ns < ctx.state_floor_ns) {
    return DecelRefusal::kBeforeReset;
  }
  if (!ValidateDecelNodes(p)) {
    return DecelRefusal::kMalformed;
  }
  return DecelRefusal::kNone;
}

[[nodiscard]] constexpr const char* DecelRefusalName(DecelRefusal r) noexcept {
  switch (r) {
    case DecelRefusal::kNone:
      return "none";
    case DecelRefusal::kInvalid:
      return "invalid";
    case DecelRefusal::kActivation:
      return "activation";
    case DecelRefusal::kPlan:
      return "plan";
    case DecelRefusal::kRepeat:
      return "repeat";
    case DecelRefusal::kAged:
      return "aged";
    case DecelRefusal::kBeforeReset:
      return "before_reset";
    case DecelRefusal::kMalformed:
      return "malformed";
  }
  return "unknown";
}

/// Which segment the RT samples this tick (the effective-instant switch
/// rule, MD-10 · MD-32). An admitted segment is PENDING until now_lead
/// reaches its node 0 (t0 = t_eff): before that the RT keeps sampling the
/// segment it follows — the sampler refuses t < t0 anyway (jerk_segment.hpp).
/// At t0 the pending one takes over; the planner built its node 0 as the
/// state the current one reaches there, so the switch is continuous to the
/// accuracy of that prediction.
enum class DecelSegmentChoice : std::uint8_t {
  kNone = 0,  ///< nothing to sample yet (closed form / pre-DECEL continues)
  kCurrent,   ///< keep sampling the followed segment
  kPending,   ///< switch: the pending segment becomes the followed one
};

[[nodiscard]] constexpr DecelSegmentChoice ChooseDecelSegment(bool current_valid,
                                                              bool pending_valid,
                                                              std::int64_t pending_t0_ns,
                                                              std::int64_t now_lead_ns) noexcept {
  if (pending_valid && now_lead_ns >= pending_t0_ns) {
    return DecelSegmentChoice::kPending;
  }
  return current_valid ? DecelSegmentChoice::kCurrent : DecelSegmentChoice::kNone;
}

/// The continuity gate at a segment switch (MD-39). Per joint i, with
/// d_i = (1 − η_v)·q̇_max,i — the velocity headroom the MPC leaves the CLIK's
/// feedback (formulation §2.3) — the switch passes only when
///   |q̇_c,i − q̇_ref,i| + K_p·|q_c,i − q_ref,i| ≤ ρ_max · d_i.
/// Written multiplied and negated so a NaN on either side refuses, and a
/// d_i that is not a positive finite number refuses rather than divides.
/// K_p is an eigenvalue bound of the task gain, not a per-joint one: ρ is a
/// heuristic measure, recorded to be tightened on measurement.
struct DecelSwitchVerdict {
  bool pass{false};
  /// max_i lhs_i / d_i over the joints that have a d_i (+inf without one).
  double rho{0.0};
  /// The first joint that refused, −1 when it passed.
  int joint{-1};
  double dq_max{0.0};   ///< max_i |q_c − q_ref| [rad]
  double dqd_max{0.0};  ///< max_i |q̇_c − q̇_ref| [rad/s]
};

[[nodiscard]] inline DecelSwitchVerdict JudgeDecelSwitch(
    std::span<const double> q_c, std::span<const double> qd_c, std::span<const double> q_ref,
    std::span<const double> qd_ref, std::span<const double> qdot_max, int nv, double k_p,
    double eta_v, double rho_max) noexcept {
  DecelSwitchVerdict v;
  const auto n = static_cast<std::size_t>(nv < 0 ? 0 : nv);
  if (nv < 1 || q_c.size() < n || qd_c.size() < n || q_ref.size() < n || qd_ref.size() < n ||
      qdot_max.size() < n || !(std::isfinite(k_p) && k_p >= 0.0) ||
      !(std::isfinite(rho_max) && rho_max > 0.0) || !std::isfinite(eta_v)) {
    v.rho = std::numeric_limits<double>::infinity();
    return v;
  }
  v.pass = true;
  for (std::size_t i = 0; i < n; ++i) {
    const double dq = std::fabs(q_c[i] - q_ref[i]);
    const double dqd = std::fabs(qd_c[i] - qd_ref[i]);
    const double lhs = dqd + k_p * dq;
    const double d = (1.0 - eta_v) * qdot_max[i];
    v.dq_max = std::max(v.dq_max, dq);
    v.dqd_max = std::max(v.dqd_max, dqd);
    const bool headroom = std::isfinite(d) && d > 0.0;
    // std::max drops a NaN, so a non-finite ratio is recorded as +inf.
    const double r =
        headroom && std::isfinite(lhs) ? lhs / d : std::numeric_limits<double>::infinity();
    v.rho = std::max(v.rho, r);
    if (v.pass && (!headroom || !(lhs <= rho_max * d))) {
      v.pass = false;
      v.joint = static_cast<int>(i);
    }
  }
  return v;
}

[[nodiscard]] constexpr const char* PlanRefusalName(PlanRefusal r) noexcept {
  switch (r) {
    case PlanRefusal::kNone:
      return "none";
    case PlanRefusal::kInvalid:
      return "invalid";
    case PlanRefusal::kActivation:
      return "activation";
    case PlanRefusal::kTrack:
      return "track";
    case PlanRefusal::kRepeat:
      return "repeat";
    case PlanRefusal::kAged:
      return "aged";
    case PlanRefusal::kBeforeReset:
      return "before_reset";
    case PlanRefusal::kTooLate:
      return "too_late";
  }
  return "unknown";
}

}  // namespace rtc::catching
