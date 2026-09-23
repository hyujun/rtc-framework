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

#include <array>
#include <cstdint>
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

  /// The vision track epoch of the last trajectory the RT consumed (L1 §4.4),
  /// and whether it has consumed one at all in this trial.
  bool track_seen{false};
  std::uint64_t track_generation{0};
};

static_assert(std::is_trivially_copyable_v<PlannerRtState>);

/// What the planner does in a given supervisor mode (L3 §5.3).
///
/// A PREDICATE over the mode, not an ordering test: `mode >= kCommitted` would
/// silently change meaning the day a mode is inserted into the enum (L3 §5.3
/// "isFrozen").
enum class PlannerActivity : std::uint8_t {
  kIdle,     ///< nothing to plan for — wake, publish nothing
  kSearch,   ///< TRACKING / APPROACH: search candidates, publish a plan
  kMonitor,  ///< COMMITTED / CLOSING: the plan is frozen; monitorOnly (§4.6)
};

[[nodiscard]] constexpr PlannerActivity ActivityFor(Mode mode) noexcept {
  switch (mode) {
    case Mode::kTracking:
    case Mode::kApproach:
      return PlannerActivity::kSearch;
    case Mode::kCommitted:
    case Mode::kClosing:
      return PlannerActivity::kMonitor;
    case Mode::kIdle:
    case Mode::kArmed:
    case Mode::kDecel:
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
  return PlanRefusal::kNone;
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
  }
  return "unknown";
}

}  // namespace rtc::catching
