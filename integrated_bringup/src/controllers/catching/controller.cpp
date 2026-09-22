#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup {

namespace {

// Fallback position bounds for a device whose YAML declares no `joint_limits`
// (LoadDeviceLimitsFromConfig fills the slot with these). ±2π is the same
// envelope the other demo bindings pass — stated here rather than hidden in a
// default argument so this controller's clamp is visible at its call site.
inline constexpr double kFallbackPositionLower = -6.2832;
inline constexpr double kFallbackPositionUpper = 6.2832;
inline constexpr double kFallbackMaxVelocity = 2.0;

/// The tick's own reading of the steady clock.
///
/// Read per tick rather than accumulated as `iteration * dt` (plan §3): the
/// two diverge by exactly the jitter and overrun this controller's deadlines
/// are about, and the accumulated version cannot see a missed tick at all.
/// RT-safe — a vDSO read, no allocation and no lock, which is the same thing
/// the device backends do on their own RT-priority callback lane.
[[nodiscard]] std::int64_t SteadyNowNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

DemoCatchingController::DemoCatchingController(std::string_view urdf_path) : urdf_path_(urdf_path) {
  // Nothing to build: this skeleton runs no model. urdf_path_ is stored so the
  // signature matches the registry factory and S5.1 can build one here without
  // changing the registration.
}

// ── Configuration ───────────────────────────────────────────────────────────

void DemoCatchingController::LoadConfig(const YAML::Node& cfg) {
  // Base first: `topics:` (the group list this controller's device indices are
  // defined against) and `command_type`.
  RTControllerInterface::LoadConfig(cfg);

  if (!cfg || !cfg.IsMap()) {
    throw std::runtime_error(
        "DemoCatchingController: config node is absent or not a map — this controller is "
        "registered as REQUIRING_CONFIG and has no defensible defaults (no hand profile)");
  }

  // ── diagnostic.hand_step ────────────────────────────────────────────────
  // Default false: from S7.1 the hand sequencer owns the hand, and an
  // operator-issued step would then race it. S4a's own YAML turns it on.
  hand_step_enabled_ = false;
  if (const YAML::Node diag = cfg["diagnostic"]; diag) {
    if (!diag.IsMap()) {
      throw std::runtime_error("DemoCatchingController: 'diagnostic' must be a map");
    }
    if (const YAML::Node step = diag["hand_step"]; step) {
      hand_step_enabled_ = step.as<bool>();
    }
  }

  // ── catching: (L0 §5.3 / §9 G0-C subset) ────────────────────────────────
  // Parsed here so a malformed tree refuses the configure through the same
  // try/catch every other controller's LoadConfig uses. The VERDICT is applied
  // in on_configure, which is the hook that can refuse.
  const YAML::Node catching = cfg["catching"];
  catching_section_present_ = static_cast<bool>(catching);
  if (catching_section_present_) {
    params_ = rtc::catching::ParseCatchingParams(catching);
  }

  // ── io: the two keys the numeric core does not carry ────────────────────
  // `CatchingParams` is deliberately numbers-only (it is the schema the
  // validator judges), so the topic and frame names are read here — the same
  // split `diagnostic.hand_step` already uses. They live under `catching.io`
  // with the rest of the lane so the operator reads one block, not two.
  traj_topic_.clear();
  expected_frame_ = "world";
  if (catching_section_present_) {
    if (const YAML::Node io = catching["io"]; io) {
      if (!io.IsMap()) {
        throw std::runtime_error("DemoCatchingController: 'catching.io' must be a map");
      }
      if (const YAML::Node topic = io["traj_topic"]; topic) {
        traj_topic_ = topic.as<std::string>();
      }
      if (const YAML::Node frame = io["expected_frame"]; frame) {
        expected_frame_ = frame.as<std::string>();
      }
    }
  }

  // ── logs: (Phase C) ─────────────────────────────────────────────────────
  parsed_log_entries_.clear();
  if (const YAML::Node logs = cfg["logs"]; logs) {
    if (!logs.IsSequence()) {
      throw std::runtime_error("DemoCatchingController: 'logs' must be a sequence");
    }
    for (const auto& entry : logs) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoCatchingController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      // Closed set: this controller owns exactly one channel type. A typo is a
      // hard fail at parse time rather than a CSV that never appears.
      if (e.msg_type != "rtc_msgs/DeviceStateLog") {
        throw std::runtime_error("DemoCatchingController: unknown msg_type in `logs`: " +
                                 e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

void DemoCatchingController::OnDeviceConfigsSet() {
  ResolveDevices();
}

void DemoCatchingController::ResolveDevices() {
  const auto arm = GetPrimaryDeviceName();
  const auto hand = GetSecondaryDeviceName();

  arm_dof_ = 0;
  hand_dof_ = 0;
  arm_joint_names_.clear();
  hand_joint_names_.clear();
  hand_motor_names_.clear();
  non_sim_groups_.clear();
  device_configs_seen_ = false;

  if (const auto* cfg = GetDeviceNameConfig(arm); cfg != nullptr) {
    arm_dof_ = static_cast<int>(cfg->joint_state_names.size());
    arm_joint_names_ = cfg->joint_state_names;
  }
  if (!hand.empty()) {
    if (const auto* cfg = GetDeviceNameConfig(hand); cfg != nullptr) {
      hand_dof_ = static_cast<int>(cfg->joint_state_names.size());
      hand_joint_names_ = cfg->joint_state_names;
      hand_motor_names_ = cfg->motor_state_names;
    }
  }
  arm_dof_ = std::min(arm_dof_, kDemoCatchingMaxArmDof);
  hand_dof_ = std::min(hand_dof_, kDemoCatchingMaxHandDof);

  // ── Which validator axis this configure is judged on (A-S5-1) ───────────
  // Fail-closed on BOTH "wrong backend" and "no backend block": the question
  // is not "is this a real arm" but "has this device PROVEN it is the
  // simulator", and silence does not prove it. Since E-8 was approved this no
  // longer gates activation by itself — it selects `real_arm_config`, and the
  // provisional/TBD rule (L0 §5.3) is what can then park the instance.
  for (const auto& group : topic_config_.groups) {
    const auto* cfg = GetDeviceNameConfig(group.first);
    if (cfg == nullptr) {
      continue;  // no config for this group yet — nothing claimed, nothing to prove
    }
    device_configs_seen_ = true;
    if (!cfg->backend.has_value() || cfg->backend->type != kCatchingSimBackendType) {
      non_sim_groups_.push_back(group.first);
    }
  }
  // No device config resolved yet ⇒ nothing has proven anything, so the strict
  // axis applies. ResolveDevices runs twice (hook, then on_configure), and the
  // first run can legitimately see no groups.
  real_arm_config_ = !device_configs_seen_ || !non_sim_groups_.empty();

  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             kFallbackPositionLower, kFallbackPositionUpper, kFallbackMaxVelocity);
}

// ── Targets ─────────────────────────────────────────────────────────────────

void DemoCatchingController::SetDeviceTarget(int device_idx,
                                             std::span<const double> target) noexcept {
  // Off-RT (the controller's non-RT callback group). Two refusals happen here
  // rather than on the RT side so the operator's goal is answered where it was
  // issued, and so a refused goal never occupies a mailbox slot.
  if (device_idx != kCatchingHandDeviceIdx) {
    // The arm slot has exactly one writer: the activation latch. S5.1 opens
    // this path together with the E-8 approval.
    arm_target_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "target for device %d refused: this controller holds the arm and accepts "
                         "goals on the hand device (%d) only",
                         device_idx, kCatchingHandDeviceIdx);
    return;
  }
  if (!hand_step_enabled_) {
    hand_step_disabled_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "hand target refused: 'diagnostic.hand_step' is false, so the hand holds "
                         "its activation pose");
    return;
  }
  // A partial goal is refused rather than half-applied. This controller has no
  // persistent target row — the overlay in WriteDeviceCommand covers
  // [0, hand_target_width_) and the REST comes from the activation latch — so a
  // short goal does not leave the untouched joints where the previous step put
  // them, it snaps them back to the pose held at activation. The base already
  // refuses a goal LONGER than the device and refuses partial NAMED goals; this
  // closes the unnamed-short case for the one binding whose goals are unshaped
  // steps. (It is also what stops a 6-value task goal, see SetDeviceTaskTarget.)
  if (hand_dof_ > 0 && target.size() != static_cast<std::size_t>(hand_dof_)) {
    hand_target_width_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "hand target refused: %zu values for a %d-joint hand. A step commands "
                         "every joint or none — a partial one would snap the rest back to the "
                         "activation pose.",
                         target.size(), hand_dof_);
    return;
  }
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

void DemoCatchingController::SetDeviceTaskTarget(int device_idx,
                                                 std::span<const double> task6) noexcept {
  // There is no task lane here — no CLIK, no IK, nothing that turns a Cartesian
  // pose into joint values (see the header). The base's default forwards a task
  // goal to SetDeviceTarget, which on this controller would write the six pose
  // numbers into the first six HAND JOINTS as an unshaped step, clamped to the
  // joint limits and with the base's limit warning deliberately skipped because
  // "joint limits do not apply to a Cartesian goal". Refuse instead.
  static_cast<void>(task6);
  task_target_reject_count_.fetch_add(1, std::memory_order_relaxed);
  RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                       "task goal for device %d refused: this controller has no task lane. Send "
                       "joint values (goal_type 'joint') to step the hand.",
                       device_idx);
}

void DemoCatchingController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                                bool /*is_task*/) noexcept {
  // RT tick, once per surviving mailbox entry. Entries invalidated by a later
  // activation never reach here (the base's generation gate), which is what
  // keeps a goal queued while Inactive off the first tick after re-activation.
  if (device_idx != kCatchingHandDeviceIdx) {
    return;
  }
  const auto n = std::min(values.size(), hand_target_raw_.size());
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_raw_[i] = values[i];
  }
  hand_target_width_ = static_cast<int>(n);
  hand_step_applied_count_.fetch_add(1, std::memory_order_relaxed);
}

void DemoCatchingController::ResetTargetInitialization() noexcept {
  // Lifecycle thread, from the base's on_activate. Under P-1 (a) this hook
  // may REQUEST a reset and must not perform one: the single writer of the
  // values is the RT tick.
  //
  // It needs no request of its own. The base bumped the activation generation
  // immediately before calling this, and ServiceResetRequests keys off exactly
  // that (D-23) — so the first tick of the new activation already reseeds the
  // hold latches from the pose the robot is actually in and drops a step the
  // operator issued before this activation.
  //
  // WHY NOT JUST CLEAR THEM HERE, as S4.0 did. It worked, because CM calls
  // on_activate with no tick running. It stops working the moment anything
  // else can request a reset — and E-STOP can, from a service callback, with
  // a tick mid-flight. Two writers of the same state under two different
  // synchronisation stories is the arrangement P-1 (a) exists to forbid, so
  // the one path that could have kept its shortcut gives it up too.
  reset_requested_.fetch_add(1, std::memory_order_release);
}

// ── Vision ingress (S5.2) ───────────────────────────────────────────────────

void DemoCatchingController::OnTrajectoryCloud(const sensor_msgs::msg::PointCloud2& msg) noexcept {
  // Non-RT: the controller LifecycleNode's default callback group, which CM
  // runs on `nrt_callback_executor` — shared with the lifecycle services, so
  // this must stay small. It does exactly three things: stamp, decode, store.
  //
  // The two instants are sampled TOGETHER and FIRST. They are the pair the
  // D-2 conversion stands on, and any work between them lands in the origin
  // delay as if the publisher had been slow.
  const std::int64_t recv_steady = SteadyNowNs();
  const std::int64_t recv_wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count();

  rtc::catching::TrajectorySnapshot snap{};
  rtc::catching::CovarianceSnapshot cov{};
  // The snapshot is stamped with the CURRENT activation generation (D-23).
  // The subscription outlives deactivation, so a message that arrives while
  // the controller is inactive is decoded and stored — and then refused by the
  // RT read, which is where the judgement belongs: the callback cannot know
  // whether an activation will happen before the next tick.
  const CloudReject reject =
      traj_input_.OnCloud(msg, recv_steady, recv_wall, ActivationGeneration(), snap, cov);
  if (reject != CloudReject::kNone) {
    // Counted inside the ingress. Throttled here so a publisher that has
    // started producing garbage is visible without a per-message log on the
    // executor the lifecycle services share.
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "vision message refused (%s); check the reject counters",
                         CloudRejectName(reject));
    return;
  }

  // Covariance first. The two locks are written in the order the planner will
  // read them in, so the window in which they disagree is the one where the
  // covariance is NEWER than the trajectory — and the token check rejects
  // that pairing, while the reverse would let an old covariance pass as the
  // current one (D-22).
  cov_box_.Store(cov);
  traj_box_.Store(snap);

  if (traj_input_.LastDiagnostics().horizon_short) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "vision horizon %ld ms is shorter than the %ld ms this controller needs "
                         "(D-15): late catch points are not reaching the planner",
                         static_cast<long>(traj_input_.LastDiagnostics().horizon_ns / 1000000),
                         static_cast<long>(traj_horizon_min_ns_ / 1000000));
  }
}

// ── E-STOP and fault hooks (P-1 (a): request only) ──────────────────────────
//
// Every body here is a store. None of them touches the hold latches, the step
// target, the supervisor mode or any counter the tick owns — that is the whole
// of P-1 (a), and it is also what makes these safe to call from a service
// callback while a tick is mid-flight.
//
// The base's own comment warns that an override must not branch on the global
// latch, because CM propagates both directions while the latch still reads
// true. None of these reads it.

void DemoCatchingController::TriggerEstop() noexcept {
  estop_requested_.store(true, std::memory_order_release);
  // The epoch moves on the TRIGGER as well as on the clear. A trigger→clear
  // pair that lands entirely between two ticks would otherwise be invisible:
  // the flag is back to false and the tick would have nothing to tell it that
  // a stop happened at all, so q_c would carry on from before the stop.
  estop_epoch_.fetch_add(1, std::memory_order_release);
}

void DemoCatchingController::ClearEstop() noexcept {
  estop_requested_.store(false, std::memory_order_release);
  estop_epoch_.fetch_add(1, std::memory_order_release);
  // NOT cleared here: fault_latched_ (P-1 (d) — the fault path is separate and
  // a global clear must not launder it) and arm_requested_, which the tick
  // lowered on the stop. Leaving the arm latch down is what makes "no
  // automatic resume" (P-1 (c)) a property of the mechanism rather than of
  // whoever is holding the clear button.
}

bool DemoCatchingController::IsEstopped() const noexcept {
  return estop_requested_.load(std::memory_order_acquire);
}

void DemoCatchingController::ResetFault() noexcept {
  // A request, not a clear: the tick decides whether the cause is gone. The
  // base's contract says exactly this ("the clear happens on the RT thread,
  // one tick later, and can be refused there"), and HasLatchedFault is how the
  // caller finds out.
  fault_reset_epoch_.fetch_add(1, std::memory_order_release);
}

bool DemoCatchingController::HasLatchedFault() const noexcept {
  return fault_latched_.load(std::memory_order_acquire);
}

// ── Supervisor (S1.8 transition table, S5.1 subset) ─────────────────────────

void DemoCatchingController::AdvanceMode(rtc::catching::Reason reason) noexcept {
  rtc::catching::Mode next = mode_;
  // A reason with no row in this mode is INAPPLICABLE here, which the table
  // expresses by having no row — not an error and not a fallback. Leaving the
  // mode alone is the documented contract of LookupTransition, and it is why
  // this driver can be this small: every "may I go there from here" question
  // is already answered by the data, and S7.2 grows the driver rather than
  // the table.
  if (rtc::catching::LookupTransition(rtc::catching::kTransitionTable, mode_, reason, next)) {
    mode_ = next;
  }
  last_reason_ = reason;
}

DemoCatchingController::ReasonDecision DemoCatchingController::EvaluateReason(
    const ControllerState& state) noexcept {
  using rtc::catching::Mode;
  using rtc::catching::Reason;
  static_cast<void>(state);

  // Ordered by authority, not by likelihood. E-STOP outranks everything
  // because it is the one condition whose handling must not depend on what
  // else is true this tick.
  if (estop_active_) {
    return {Reason::kEstop, true};
  }
  if (fault_reset_serviced_) {
    // Consumed here rather than in ServiceResetRequests so the FSM edge and
    // the state reset happen on the same tick and in the table's order.
    fault_reset_serviced_ = false;
    return {Reason::kFaultReset, true};
  }
  if (fault_latched_.load(std::memory_order_relaxed)) {
    // No reason at all: in Mode::kFault the table has rows only for
    // kFaultReset and kEstop, so anything else leaves the latch standing.
    // Holding rather than inventing a "still faulted" reason keeps the
    // latch's persistence a property of the driver, not of a reason code.
    return {Reason::kNone, false};
  }
  if (!armable_ || !arm_requested_.load(std::memory_order_relaxed)) {
    // §4.5's readiness conditions are not met — either the profile is not
    // armable (G0-C) or the operator has not armed this controller.
    //
    // kParamsTbd is the DOCUMENTED reuse for "an ARMED precondition stopped
    // holding" (L7 §4.5 has no dedicated reason; transition_table.hpp's header
    // records the reuse). It is a self-loop in IDLE and sends ARMED back to
    // IDLE, which is exactly the two behaviours wanted here.
    return {Reason::kParamsTbd, true};
  }
  // ── The vision lane (S5.2) ───────────────────────────────────────────────
  // From here the controller is armed and the profile is clean, so what is
  // left to decide is what the prediction says. The ORDER matters: a track
  // change outranks staleness because it says the old plan was about a
  // different ball, and a stale snapshot of the right ball is a reason to
  // stop planning, not to plan against something else.
  const bool tracking = (mode_ == Mode::kTracking || mode_ == Mode::kApproach);

  if (tracking && traj_new_track_) {
    traj_new_track_ = false;
    return {Reason::kTrackChanged, true};
  }
  traj_new_track_ = false;

  // Whether there is a prediction worth acting on. `expired` is grouped with
  // `stale` for the ARMING question and kept apart for the ABORT question:
  // both mean "do not start tracking on this", but they call for different
  // reasons once tracking has started, and the fixes differ (a publisher that
  // stopped against a horizon that is too short, D-15).
  const bool usable = !traj_view_.stale && !traj_view_.expired;

  if (!usable) {
    if (tracking) {
      return traj_view_.stale ? ReasonDecision{Reason::kBallStale, true}
                              : ReasonDecision{Reason::kHorizonExtrap, true};
    }
    // Not tracking yet, and nothing to report: waiting for a ball IS the
    // ARMED state. IDLE still advances — readiness is about the ROBOT, and
    // gating it on vision would leave the arm un-homed until a ball appeared,
    // which is backwards.
    return {Reason::kNone, mode_ == Mode::kIdle};
  }

  // A usable prediction. From IDLE this advances to ARMED and from ARMED to
  // TRACKING — the edge the ball lane exists to drive.
  //
  // From TRACKING, `kNone` would advance to APPROACH, and APPROACH means "a
  // valid plan is being followed". There is no planner until S6, so the honest
  // answer is `kNoCatchablePlan` — which the table self-loops in TRACKING and
  // which is exactly what it means here: the ball is seen, and no plan exists
  // for it. S6 replaces this with a real plan check rather than adding one.
  if (mode_ == Mode::kTracking) {
    return {Reason::kNoCatchablePlan, true};
  }
  return {Reason::kNone, true};
}

// ── Reset service (P-1 (a)/(b): the tick is the only writer) ────────────────

void DemoCatchingController::ResetTrialState() noexcept {
  // L7 §4.8's re-arm list, restricted to the state that exists at S5.1. The
  // list grows with the controller, and the rule that comes with it is that a
  // new stateful member is added HERE in the same change that adds it — a
  // member missing from this function is a trial that starts with the previous
  // trial's state, which is the failure mode §4.8 was written for.
  arm_hold_ = HoldLatch{};
  hand_hold_ = HoldLatch{};
  hand_target_raw_.fill(0.0);
  hand_target_width_ = 0;
  // Queued goals are dropped rather than carried over: a target issued before
  // this reset was issued against the state the reset just discarded. The
  // base's generation gate already stops goals from a previous ACTIVATION, but
  // an E-STOP does not bump that generation, so this is the half of the
  // problem the gate does not cover.
  DiscardPendingTargets();
  mode_ = rtc::catching::Mode::kIdle;
  last_reason_ = rtc::catching::Reason::kNone;
  // L7 §4.8's list, vision half: the consumed-sequence memory and the track
  // identity. Without the first, the snapshot in the box reads as "already
  // seen" after the reset and the trial starts by ignoring the only
  // prediction it has; without the second, the first snapshot of the new
  // trial looks like a track CHANGE and aborts it.
  last_consumed_sequence_ = 0;
  last_track_generation_ = 0;
  track_seen_ = false;
  traj_new_track_ = false;
  traj_view_ = rtc::catching::TrajView{};
}

bool DemoCatchingController::ServiceResetRequests(const ControllerState& state) noexcept {
  static_cast<void>(state);
  bool reset = false;

  // D-23: the activation boundary is decided from the generation, not from a
  // quiescence wait. PeriodicRtThread::Pause does not stop an iteration
  // already in flight, so "no tick is running" is not something on_activate
  // can establish — but "the generation changed" is something this tick can
  // observe, and it is true exactly once per activation.
  const std::uint32_t generation = ActivationGeneration();
  if (!activation_seen_ || generation != serviced_activation_generation_) {
    serviced_activation_generation_ = generation;
    activation_seen_ = true;
    reset = true;
  }

  const std::uint32_t reset_epoch = reset_requested_.load(std::memory_order_acquire);
  if (reset_epoch != serviced_reset_epoch_) {
    serviced_reset_epoch_ = reset_epoch;
    reset = true;
  }

  const std::uint32_t estop_epoch = estop_epoch_.load(std::memory_order_acquire);
  if (estop_epoch != serviced_estop_epoch_) {
    serviced_estop_epoch_ = estop_epoch;
    reset = true;
    // Both edges disarm. On the trigger it is the stop itself; on the clear it
    // is P-1 (c), and doing it on BOTH is what covers the trigger→clear pair
    // that lands between two ticks — the tick then sees one epoch move, and
    // that single reset must still leave the controller disarmed.
    arm_requested_.store(false, std::memory_order_relaxed);
  }

  const std::uint32_t fault_epoch = fault_reset_epoch_.load(std::memory_order_acquire);
  if (fault_epoch != serviced_fault_reset_epoch_) {
    serviced_fault_reset_epoch_ = fault_epoch;
    // The cause is gone as soon as it is asked about, at S5.1: the only thing
    // that can raise this latch is the supervisor reaching Mode::kFault, and
    // no S5.1 path reaches it (the QP failure streak that does is S5.3). When
    // that path lands, this is where "refuse the clear while the cause is
    // still present" belongs.
    if (fault_latched_.load(std::memory_order_relaxed)) {
      fault_latched_.store(false, std::memory_order_release);
      fault_reset_serviced_ = true;
      reset = true;
    }
  }

  // A latched fault disarms too, and it does so on EVERY tick the latch is up
  // rather than on the edge that raised it: the latch outlives the reset that
  // noticed it, so an operator setting `catching.enable` while faulted must not
  // find the controller armed the moment the fault is cleared. (Nothing raises
  // this latch before S5.3's QP-failure streak — this is the rule being in
  // place before the cause exists, not dead code for its own sake.)
  if (fault_latched_.load(std::memory_order_relaxed)) {
    arm_requested_.store(false, std::memory_order_relaxed);
  }

  if (reset) {
    ResetTrialState();
    rt_reset_count_.fetch_add(1, std::memory_order_relaxed);
  }
  return reset;
}

// ── RT tick ─────────────────────────────────────────────────────────────────

void DemoCatchingController::ReportGateClosure(const ControllerState& state, int device_idx,
                                               int dof, const char* axis) noexcept {
  if (device_idx >= state.num_devices) {
    return;
  }
  const auto& dev = state.devices[static_cast<std::size_t>(device_idx)];
  // RT-3's throttle exception: primitive-only format, no assembly. `axis` is a
  // string literal selected by the caller, which is on the allowed list.
  //
  // Not an else-if pair: IsGateClosedByHoles requires the width test to have
  // PASSED, so the two are mutually exclusive by construction and chaining them
  // would only hide that.
  if (rtc::IsGateClosedByWidth(dev, dof)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "F5 gate closed: %s device reports num_channels=%d < dof=%d — that axis "
                         "is silenced every tick; fix the group's 'joint_state_names' or the "
                         "backend's reported channel count",
                         axis, dev.num_channels, dof);
  }
  if (rtc::IsGateClosedByHoles(dev, dof)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "F5 gate closed: %s device slot %d was never written although "
                         "num_channels=%d >= dof=%d — that axis is silenced every tick; the state "
                         "message carries no joint named at that position of the group's "
                         "'joint_command_names' (defaults to 'joint_state_names')",
                         axis, rtc::FirstHoleSlot(dev, dof), dev.num_channels, dof);
  }
}

void DemoCatchingController::WriteDeviceCommand(const ControllerState& state,
                                                ControllerOutput& output, int device_idx,
                                                bool readable) noexcept {
  const auto idx = static_cast<std::size_t>(device_idx);
  const auto& dev = state.devices[idx];
  auto& out = output.devices[idx];
  out.goal_type = rtc::GoalType::kJoint;

  const bool is_hand = (device_idx == kCatchingHandDeviceIdx);
  HoldLatch& latch = is_hand ? hand_hold_ : arm_hold_;

  // Latch on the first READABLE tick, and never again until the next
  // activation. Seeding from an unreadable device would freeze a hold target
  // whose unreported joints are 0 — i.e. "go to the origin" — and the latch
  // would make that permanent.
  if (!latch.IsLatched() && readable) {
    // Trimmed at the first HOLE, not at num_channels. `readable` only vouches
    // for [0, dof) — slots past it can be wire channels no state message ever
    // wrote, which sit at 0.0 in the persistent cache. Latching those would
    // freeze "go to the origin" on them for the whole activation, and unlike
    // the joint controller's per-tick tail this latch never re-seeds. This is
    // what SelfReportedChannelBound exists for, and it is a no-op on a
    // hole-free device, so a legitimately wide device still gets full width.
    const int width = rtc::SelfReportedChannelBound(dev, static_cast<int>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
      latch.commands[i] = dev.positions[i];
    }
    rtc::utils::ClampRange(latch.commands, width,
                           std::span<const double>(device_position_lower_[idx]),
                           std::span<const double>(device_position_upper_[idx]),
                           kFallbackPositionLower, kFallbackPositionUpper);
    latch.width = width;
  }

  if (!latch.IsLatched()) {
    // Nothing honest to command yet: zero-length is "no update" and the drive
    // holds its own setpoint. nc zeros would be a real command to the origin.
    rtc::SilenceDeviceOutput(out);
    rtc::HoldTelemetryAtMeasured(out, dev.num_channels, std::span<const double>(dev.positions));
    return;
  }

  out.num_channels = latch.width;
  for (std::size_t i = 0; i < static_cast<std::size_t>(latch.width); ++i) {
    out.commands[i] = latch.commands[i];
    // The reference lanes carry the same value: this controller has no
    // trajectory, so the reference IS the command.
    out.target_positions[i] = latch.commands[i];
    out.trajectory_positions[i] = latch.commands[i];
    out.goal_positions[i] = latch.commands[i];
  }

  if (!is_hand || hand_target_width_ == 0 || estop_active_) {
    // `estop_active_`: a stop holds the pose it was stopped in. Without this
    // the overlay below would keep re-applying the last accepted step on every
    // tick of the stop — the value is latched in `hand_target_raw_`, so it
    // does not need a NEW target to reach the output.
    return;
  }

  // ── The step ────────────────────────────────────────────────────────────
  // G6-B's controller half: what this writes into `commands` is what the CM
  // copies to the backend verbatim (it neither clamps nor filters), so the
  // clamp against YAML ∩ URDF limits has to happen HERE. `goal_positions`
  // keeps the raw target so a clamp shows up in the CSV as a divergence
  // instead of silently rewriting what was asked for.
  const int width = std::min(hand_target_width_, latch.width);
  for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
    out.commands[i] = hand_target_raw_[i];
    out.goal_positions[i] = hand_target_raw_[i];
    out.target_positions[i] = hand_target_raw_[i];
    out.trajectory_positions[i] = hand_target_raw_[i];
  }
  rtc::utils::ClampRange(out.commands, width, std::span<const double>(device_position_lower_[idx]),
                         std::span<const double>(device_position_upper_[idx]),
                         kFallbackPositionLower, kFallbackPositionUpper);
}

ControllerOutput DemoCatchingController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoCatchingController::Compute");
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = CommandType::kPosition;
  output.valid = true;

  // Read ONCE per tick, into a tick-local bool. Re-reading the atomic further
  // down would let one tick act on two different answers — the command written
  // for a stop the supervisor did not see, or the reverse.
  estop_active_ = estop_requested_.load(std::memory_order_acquire);
  if (estop_active_) {
    estop_tick_count_.fetch_add(1, std::memory_order_relaxed);
  }

  arm_readable_ = state.num_devices > kCatchingArmDeviceIdx &&
                  rtc::IsDeviceReadable(state.devices[kCatchingArmDeviceIdx], arm_dof_);
  hand_readable_ = state.num_devices > kCatchingHandDeviceIdx &&
                   rtc::IsDeviceReadable(state.devices[kCatchingHandDeviceIdx], hand_dof_);
  ReportGateClosure(state, kCatchingArmDeviceIdx, arm_dof_, "arm");
  ReportGateClosure(state, kCatchingHandDeviceIdx, hand_dof_, "hand");

  // BEFORE the target drain and before the command write (P-1 (b)): a reset
  // discards queued goals and drops the hold latches, so anything that
  // consumed a goal first would apply state the reset is about to throw away,
  // and anything that wrote a command first would command the pose the reset
  // has just decided not to trust.
  (void)ServiceResetRequests(state);

  if (estop_active_) {
    // Drained AND DISCARDED while stopped. Not skipped: the mailbox is a
    // fixed-capacity SPSC, so leaving entries in it would hand the hand a step
    // issued mid-stop as soon as the stop cleared — later, and with no
    // operator expecting it. Discarding says what the stop means.
    //
    // The CM substitutes its own hold for this controller's entire output
    // while the global latch is up, so nothing here reaches an actuator either
    // way. That is exactly why this is worth doing rather than relying on it:
    // a second layer costs one branch, and the layer that is doing the work
    // today belongs to a different component.
    DiscardPendingTargets();
  } else {
    // Drained before the write, not after: WriteDeviceCommand seeds the hold
    // latch and then overlays the step, so a step arriving on the same tick as
    // the first readable state is honoured on that tick instead of a tick later.
    (void)DrainPendingTargets();
  }

  // D-21: Load() unconditionally, once per tick, BEFORE anything decides
  // anything. Not gated on SeqLock::sequence() — reading the counter and the
  // payload as two steps lets a writer land between them and pair a new
  // counter with an old payload, which hides the newest snapshot for as long
  // as the pattern repeats.
  const rtc::catching::TrajectorySnapshot snapshot = traj_box_.Load();
  const rtc::catching::NowReal now{SteadyNowNs()};
  traj_view_ =
      rtc::catching::ReadTraj(snapshot, now, rtc::catching::MakeNowLead(now, t_arm_ns_),
                              t_stale_ns_, ActivationGeneration(), last_consumed_sequence_);
  if (traj_view_.is_new) {
    // A track change is latched here rather than compared in EvaluateReason:
    // the comparison is only meaningful on the tick the snapshot arrives, and
    // the supervisor may need a tick or two to act on it.
    traj_new_track_ = track_seen_ && snapshot.token.generation != last_track_generation_;
    last_track_generation_ = snapshot.token.generation;
    track_seen_ = true;
  }

  const ReasonDecision decision = EvaluateReason(state);
  if (decision.advance) {
    AdvanceMode(decision.reason);
  } else {
    last_reason_ = decision.reason;
  }
  // Published every tick, including the early ones and the estopped ones. The
  // state message (S5.4) inherits this discipline as a PROC-7 requirement; the
  // two atomics here are its S5.1 stand-in.
  mode_observed_.store(static_cast<std::uint8_t>(mode_), std::memory_order_relaxed);
  reason_observed_.store(static_cast<std::uint8_t>(last_reason_), std::memory_order_relaxed);

  if (state.num_devices > kCatchingArmDeviceIdx) {
    WriteDeviceCommand(state, output, kCatchingArmDeviceIdx, arm_readable_);
  }
  if (state.num_devices > kCatchingHandDeviceIdx) {
    WriteDeviceCommand(state, output, kCatchingHandDeviceIdx, hand_readable_);
  }

  if (arm_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kCatchingArmDeviceIdx, pod);
    arm_state_log_handle_.Push(pod);
  }
  if (hand_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kCatchingHandDeviceIdx, pod);
    hand_state_log_handle_.Push(pod);
  }

  return output;
}

}  // namespace integrated_bringup
