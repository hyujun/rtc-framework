#ifndef RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_
#define RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_

#include "rtc_base/types/types.hpp"

#include <algorithm>
#include <cstddef>
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
// NECESSARY, NOT SUFFICIENT (issue #265 decision D1-a → issue #284). A true
// answer does NOT prove that slots [0, num_channels) were written this tick —
// with an active reorder map the written slots are the MATCHED reference
// indices, so holes survive a passing gate. Closing that gap needs a new field
// or a new predicate and is owned by issue #284; the counterexample and the
// rejected alternatives are in §3.7.
//
// `model_dim <= 0` degrades to a plain validity check, which is what a
// controller whose runtime DOF is not resolved yet (unit fixtures that bypass
// YAML) should see: nothing is known to be missing.
[[nodiscard]] inline bool IsDeviceReadable(const DeviceState& dev, int model_dim) noexcept {
  return dev.valid && dev.num_channels >= model_dim;
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
