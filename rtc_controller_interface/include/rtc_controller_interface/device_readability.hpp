#ifndef RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_
#define RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_

#include "rtc_base/types/types.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

// ── F5 device-readability gate (issue #236 S7b) ─────────────────────────────
//
// CONTRACT SSoT: §3.7 of rtc_controllers/docs/compliance-conventions.md.
// That section owns the decision table, the rationale, the counterexamples and
// the known limits; the comments here state what each function does and why
// THIS code is shaped the way it is, and point there for the rest. Do not
// restate the contract in this file — one sentence in two places drifts, and
// keeping the copies in sync was mistaken for evidence of correctness once
// already (#297).
//
// Every binding that reads ControllerState::devices[0] answers the same
// question — "is this device usable as the joint state of THIS tick?" — and
// before this header the repository held seven different answers to it. The
// three-tier rule (agent_docs/design-principles.md §3계층 배치) assigns the
// gate to base for exactly that reason: what a target MEANS is per-binding,
// but whether the arm reported enough channels to run FK is not.
//
// TWO PREDICATES, DELIBERATELY NAMED APART (issue #265 decision B). They
// answer different questions and neither substitutes for the other:
//
//   ModelChannelBound  — how many channels may I INDEX? Always applied.
//                        Pure out-of-bounds defence, no policy.
//   IsDeviceReadable   — may I USE this device at all this tick? The gate.
//                        Policy: a false answer means emit nothing.
//
// Collapsing them is the trap this header exists to close: a bound removes the
// crash and keeps the hazard, because skipped slots in a PERSISTENT buffer hold
// their previous value. Worked example and the pinning tests: §3.7.
//
// IsGateClosedByWidth (issue #307), IsGateClosedByHoles and FirstHoleSlot
// (issue #284) are further predicates but not further members of that pair —
// they decide nothing about this tick's output. They answer WHY a closed gate
// is closed, for the bindings' diagnostic log only, and they partition the
// closed state so that no cause is silent: not reported (!valid, diagnosed by
// the controller manager instead), too narrow, or holed.
//
// The gate itself stayed at ONE predicate through both of those issues, and
// that is a load-bearing choice rather than an accident. Freshness could have
// shipped as a second gate for the bindings to && together, and the audit that
// prepared #284 shows what that costs: it enumerated "the three controllers"
// and missed two support-layer readers of this same gate. A term folded into
// IsDeviceReadable reaches every consumer by construction; a second predicate
// reaches the ones somebody remembered.

namespace rtc {

// Largest channel count that may be indexed on both sides of a device↔model
// copy: `num_channels` is what the device reported, `model_dim` is what the
// consumer has room for (nv, arm_dof_, a fixed-width safe_position_ array).
//
// ALWAYS APPLIED — over-reporting is a normal input, not a fault. A backend
// publishes whatever its state message carried (`num_channels` is the WIRE
// length, capped at kMaxDeviceChannels), and BuildJointStateReorder is built
// to skip names the reference list does not have. So `num_channels > model_dim`
// happens on correct hardware with a broad state topic, and any loop that
// indexes a model-sized buffer by `num_channels` reads out of range there.
//
// NOT A GATE — a bound alone leaves the skipped slots at their previous value.
// Pair it with IsDeviceReadable, never replace it (§3.7).
//
// Negative inputs clamp to 0 rather than propagating: the result is used as a
// loop bound and as a std::span length, where a negative int would convert to
// an enormous std::size_t.
[[nodiscard]] constexpr int ModelChannelBound(int num_channels, int model_dim) noexcept {
  return std::max(0, std::min(num_channels, model_dim));
}

// THE GATE. True when `dev` can serve as this tick's joint state for a
// consumer that needs `model_dim` channels.
//
// A false answer has ONE correct response — emit nothing for this device
// (SilenceDeviceOutput). Emitting `num_channels` zeros instead is a REAL 0
// command, which on a torque-mode arm is a drop rather than a stop; §3.7 is
// where that decision and its alternatives are recorded.
//
// THREE TERMS, and the third one closes the coverage gap the first two left
// (issue #265 decision D1-a → issue #284). Width alone did NOT prove
// that slots [0, model_dim) were written: with an active reorder map the
// written slots are the MATCHED reference indices, so holes survived a passing
// gate. `hole_mask` is the per-slot record the producer keeps for exactly that
// (rtc_base/types/types.hpp), and folding it in HERE rather than exposing a
// second predicate for the bindings to call is deliberate — every consumer of
// this gate gets the stronger answer without being edited, so the failure mode
// this repository actually hit (auditing "the three controllers" and missing
// the two support-layer readers) cannot recur. The counterexample, the
// rejected alternatives and the polarity argument are in §3.7.
//
// THE THIRD TERM IS BOUNDED TWO WAYS, and neither bound is closed here — do not
// read it as "the state this gate passes is fully fresh". It covers the
// `positions` lane only (velocity and effort are copied under their own message
// lengths, so a slot can be fresh in q and stale in q_dot), and the polarity leaves
// a producer that never fills the mask indistinguishable from a hole-free one.
// Both limits are stated at the field itself (rtc_base/types/types.hpp), which
// is the SSoT for what the mask does and does not claim.
//
// `model_dim <= 0` degrades to a plain validity check, which is what a
// controller whose runtime DOF is not resolved yet (unit fixtures that bypass
// YAML) should see: nothing is known to be missing. SlotMaskBelow(<=0) == 0
// keeps the new term out of that degrade rather than re-deriving the rule.
[[nodiscard]] inline bool IsDeviceReadable(const DeviceState& dev, int model_dim) noexcept {
  return dev.valid && dev.num_channels >= model_dim &&
         (dev.hole_mask & SlotMaskBelow(model_dim)) == 0;
}

// WHY the gate is closed, for the one cause an operator can act on (issue
// #307). Decision table and the "no timer" rationale it rests on: §3.7.
//
// A false IsDeviceReadable has THREE causes and they are not equally
// reportable. This predicate owns the second; IsGateClosedByHoles below owns
// the third:
//
//   !dev.valid                      the backend has not reported. §3.7 records
//                                   that CM's startup gate refuses to run
//                                   Compute() until every configured device
//                                   has reported at least once, so a binding
//                                   only sees this for a group with no backend
//                                   at all — and CM's init timeout already
//                                   names that group and shuts down. Reporting
//                                   it here would duplicate that diagnosis and
//                                   fire on every fixture that bypasses YAML.
//
//   num_channels < model_dim        the width mismatch. §3.7 calls this the
//                                   gate's one real trigger (기동시 설정
//                                   불일치) — it is closed from the first tick
//                                   and never opens, and nothing else in the
//                                   process says so.
//
//   holes inside [0, model_dim)     wide enough, but slots the model reads were
//                                   never written this message. Same startup
//                                   profile as the width cause; owned by
//                                   IsGateClosedByHoles (#284).
//
// THIS IS WHY THERE IS NO COUNTER. The obvious shape for "the gate has been
// closed a while" is a duration or a tick threshold, and both are wrong here:
// §3.7 deliberately puts NO time bound on the closed state (a timer would make
// every misconfigured startup drop the arm N seconds in), and the causes above
// are already separable by term — startup does not need to be waited out
// because in production it never reaches Compute() at all. Splitting the terms
// is what makes the first reportable tick honest.
//
// Returns a plain bool rather than the (declared, reported) pair the message
// needs: both integers are already in the caller's hand, and the KEY TO FIX
// differs per axis (`arm_dof` is a required YAML key since #340; hand width
// comes from the device group's `joint_state_names` length), so the sentence
// that names it belongs at the binding, not here.
//
// Deliberately NOT folded into IsDeviceReadable's return. The gate answers
// "may I use this device"; a binding that conflated the two would start
// treating an unreported device as usable.
[[nodiscard]] inline bool IsGateClosedByWidth(const DeviceState& dev, int model_dim) noexcept {
  return dev.valid && dev.num_channels < model_dim;
}

// WHY the gate is closed, third cause: wide enough, but with holes (#284).
//
// This exists because the alternative is a SILENT closure. Before hole_mask a
// closed gate had exactly two causes and IsGateClosedByWidth separated the one
// an operator can act on; adding a third term to IsDeviceReadable without a
// matching diagnostic would produce ticks that are !valid == false and
// width-closed == false and still emit nothing, which is the failure shape
// #307 was raised to end. The three causes now partition a closed gate: not
// reported, too narrow, holed.
//
// The DOMINANT trigger is the same as the width cause, and the same reasoning
// about not having a timer applies (see above): the reorder map is built once
// from the first named message and never rebuilt, so a device whose declared
// names do not cover the model is holed from the first tick and stays holed.
//
// It is NOT only a startup fault, though — an earlier draft of this comment
// said so and was wrong. The mask is ASSIGNED per message, so a later message
// can hole a slot an earlier one filled. Usually that narrows `num_channels` in
// lockstep and the WIDTH cause fires instead (which is why the narrowing test
// in integrated_bringup pins the mask rather than this predicate) — but not
// always: with a map built from a broad first message whose model slots sit at
// high message indices, a shorter follow-up leaves [0, model_dim) unwritten
// while `num_channels >= model_dim` still holds, and this cause fires
// mid-session with no width shortfall behind it.
//
// That case still wants NO hysteresis, for a different reason than startup:
// those slots really are stale on that tick, so silencing is the correct answer
// and a debounce would spend the gap feeding the control law values the device
// did not send. A publisher that flaps makes the gate flap, and that is the
// gate reporting the publisher.
//
// The bindings pair this with FirstHoleSlot to name a slot. Unlike the width
// cause the KEY TO FIX does NOT differ per axis: a hole means the incoming
// state message never carried the name the device group declared for that
// slot, so the prescription on both axes is to reconcile the group's declared
// names with the publisher's. The key named there is `joint_command_names` —
// that is the reference list BuildJointStateReorder is actually built against
// in all three backends, and it is what defines the SLOT INDEX this reports.
// CM defaults it to `joint_state_names` when a group does not declare it
// (rt_controller_node_device_config.cpp), which is why the message has to name
// both rather than the more familiar one alone.
[[nodiscard]] inline bool IsGateClosedByHoles(const DeviceState& dev, int model_dim) noexcept {
  return dev.valid && dev.num_channels >= model_dim &&
         (dev.hole_mask & SlotMaskBelow(model_dim)) != 0;
}

// Lowest model slot the device did not write, or -1 when there is none.
//
// The one number that turns "the hand is silenced" into something an operator
// can act on: with a reorder map the holed slot is an INDEX INTO THE DEVICE's
// declared order, so naming it points straight at the entry of
// joint_state_names whose name never arrived. Returns the lowest rather than a
// count because the fix is per-name and the operator repairs them one at a
// time; a count would say how bad it is and not where.
//
// Bounded to the model's slots — a hole at slot 40 of a 6-DOF arm is not this
// gate's business and must not be reported as if it were.
[[nodiscard]] inline int FirstHoleSlot(const DeviceState& dev, int model_dim) noexcept {
  const uint64_t holes = dev.hole_mask & SlotMaskBelow(model_dim);
  return (holes == 0) ? -1 : std::countr_zero(holes);
}

// The width a binding may adopt when it has NO declared model_dim and has to
// take one from the device's own report — the deferred self-init in the shipped
// bindings (a fixture that bypassed YAML, or a hand group that declared no
// `joint_state_names`).
//
// `min(num_channels, cap)` was that answer until #284 and is no longer. Those
// call sites justify the wire length with "bounded by nc0, so the resolved DOF
// can never re-open the gate this branch just passed", and that held while the
// gate was width-only — it does NOT hold against a term the wire length does not
// bound. A reorder map leaves holes INSIDE [0, num_channels), so a width taken
// from the wire can be one the gate then refuses.
//
// It matters more here than at an ordinary read site because these callers
// LATCH: they seed a hold target once and never re-seed. Adopting a width that
// closes the gate therefore silences the device permanently AND freezes the seed
// on slots that were never written — the poisoned-seed failure those blocks
// already exist to prevent, arriving through the new term instead of the old.
//
// Trimming to the hole-free prefix restores the property those comments claim:
// IsDeviceReadable(dev, SelfReportedChannelBound(dev, cap)) is true for every
// `valid` device. A device with no holes — every producer that predates this
// field, every fixture that leaves the mask zero — gets exactly the old answer,
// which is why adopting this changed no existing test.
[[nodiscard]] inline int SelfReportedChannelBound(const DeviceState& dev, int cap) noexcept {
  const int wire = ModelChannelBound(dev.num_channels, cap);
  const int hole = FirstHoleSlot(dev, wire);
  return (hole < 0) ? wire : hole;
}

// The F5 answer: this device contributes no command this tick.
//
// Zero-length is "no update" to every shipped backend — each one loops to
// `min(num_channels, values.size())` and therefore early-returns, leaving the
// drive on its previous setpoint. It is NOT a safe stop and must not be read as
// one (rtc_base/devices/device_backend.hpp spells out the same for
// WriteSafeCommand).
//
// Silences the device THIS gate judged, and only that one — a statement about
// which GATE, not about which DEVICE may have one (#291). These predicates are
// device-agnostic on purpose: a secondary judged unreadable against ITS OWN
// width is silenced here exactly like the arm. Both directions of that
// asymmetry, and the secondary-axis traps, are in §3.7.
//
// Commands already staged in the array are left in place — `num_channels == 0`
// makes them unreachable, and clearing kMaxDeviceChannels doubles would spend
// RT budget to change nothing observable.
//
// This silences the WIRE only. The log lane is bounded by the DEVICE's channel
// count, not this one, so pair every call with HoldTelemetryAtMeasured below or
// the silenced tick is recorded as a row of zeros.
inline void SilenceDeviceOutput(DeviceOutput& out) noexcept {
  out.num_channels = 0;
}

// The telemetry half of the F5 answer: on a silenced tick the reference lanes
// say WHERE THE DRIVE IS PARKED, not 0.
//
// SilenceDeviceOutput stops the wire, but it does not stop the log: the device
// state POD is bounded by the DEVICE's num_channels, not the output's
// (integrated_bringup/logging/pod_fill.hpp), so a silenced tick left at fresh
// zero is recorded as a row of zeros — which reads as "commanded every joint to
// the origin". Why that misreading is the one worth spending a loop on, and the
// matching precedent in the controller manager's BuildHoldOutput, are in §3.7.
//
// `measured` is THIS tick's device positions, never a stale computed reference,
// so this does not re-introduce the staleness the gate withholds — the withheld
// thing is last readable tick's trajectory/goal, and what replaces it is a
// measurement taken this tick. Velocities stay at their fresh zero: a parked
// joint is not moving.
//
// Touches the REFERENCE lanes only. `goal_positions` is left to the binding:
// what a goal MEANS is per-binding (a joint hold target, a TCP pose split
// across the first three slots) and a goal the operator set does not stop
// existing because the arm went unreadable — that is the one column on a
// silenced row that should still say what was asked for.
//
// RT-safe: noexcept, no allocation, bounded by the shorter of the two spans.
inline void HoldTelemetryAtMeasured(DeviceOutput& out, int num_channels,
                                    std::span<const double> measured) noexcept {
  const auto end = std::min(measured.size(), static_cast<std::size_t>(std::max(0, num_channels)));
  for (std::size_t i = 0; i < end; ++i) {
    out.target_positions[i] = measured[i];
    out.trajectory_positions[i] = measured[i];
  }
}

// Fills the channels a readable device reported but the model does not cover —
// `[model_bound, num_channels)` — with the neutral value for `cmd`.
//
// This tail is part of the contract, not a leftover (#265 comment 2 §5-2). An
// unwritten command array is a FRESH ZERO, which is passive in a torque lane
// but means "go to the origin" in a position lane, so the two domains need
// different fills:
//
//   kTorque                    → 0.0            (no torque on a joint the law
//                                                never modelled)
//   kPosition / kPdFeedforward → measured[i]    (hold where it already is)
//
// `measured` is the device's own position array; a short or empty span falls
// back to 0.0 on the channels it does not cover, which is the best available
// answer once even the measurement is missing. The reference implementation
// this converges is rtc_controllers/include/rtc_controllers/joint/
// joint_pd_law.hpp (ComputeJointPdCommand's trailing loop).
//
// RT-safe: noexcept, no allocation, bounded by the shortest span involved.
inline void FillCommandTail(std::span<double> commands, int model_bound, int num_channels,
                            CommandType cmd, std::span<const double> measured) noexcept {
  const auto begin = static_cast<std::size_t>(std::max(0, model_bound));
  const auto end = std::min(commands.size(), static_cast<std::size_t>(std::max(0, num_channels)));
  for (std::size_t i = begin; i < end; ++i) {
    commands[i] = (cmd == CommandType::kTorque) ? 0.0 : ((i < measured.size()) ? measured[i] : 0.0);
  }
}

}  // namespace rtc

#endif  // RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_
