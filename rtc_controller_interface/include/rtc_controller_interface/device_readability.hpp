#ifndef RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_
#define RTC_CONTROLLER_INTERFACE_DEVICE_READABILITY_HPP_

#include "rtc_base/types/types.hpp"

#include <algorithm>
#include <cstddef>
#include <span>

// ── F5 device-readability gate (issue #236 S7b, contract SSoT: §3.7 of
//    rtc_controllers/docs/compliance-conventions.md) ─────────────────────────
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
// Collapsing them is the trap this header exists to close. `min(nc0, nv)`
// looks like it makes a narrow device safe, and it does not: a loop that
// scatters into a PERSISTENT buffer (a Pinocchio q vector, a cached full
// state) leaves every skipped slot holding its previous value — zero on the
// first tick — so the model runs at a partially-ZERO configuration that is
// numerically identical to reading the unreported channels directly. The
// bound removed the crash and kept the hazard. #172's `min(nc0, nv)` is
// therefore correct AS AN OOB DEFENCE and must not be reverted, but it was
// never the F5 answer (#265 comment 2 §2).

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
// NOT A GATE. See the header comment: a bound alone leaves the skipped slots
// at their previous value. Pair it with IsDeviceReadable, never replace it.
//
// Negative inputs clamp to 0 rather than propagating: the result is used as a
// loop bound and as a std::span length, where a negative int would convert to
// an enormous std::size_t.
[[nodiscard]] constexpr int ModelChannelBound(int num_channels, int model_dim) noexcept {
  return std::max(0, std::min(num_channels, model_dim));
}

// THE GATE. True when `dev` can serve as this tick's joint state for a
// consumer that needs `model_dim` channels. §3.7's predicate verbatim:
// `!dev.valid || dev.num_channels < model_dim` is the unreadable case.
//
// A false answer has ONE correct response — emit nothing for this device
// (SilenceDeviceOutput) — because there is no honest substitute for a joint
// position nobody reported. Do NOT emit `num_channels` zeros instead: that is
// a REAL 0 command, which on a torque-mode arm is a drop rather than a stop,
// and on a position lane is "servo to the origin".
//
// NECESSARY, NOT SUFFICIENT (issue #265 decision D1-a → issue #284). A true
// answer does NOT prove that slots [0, num_channels) were written this tick.
// `num_channels` is the wire length, and with an active reorder map the slots
// that get written are the MATCHED reference indices — so a message whose
// names only partly overlap the reference list yields `num_channels >=
// model_dim` with holes still in it. That counterexample is pinned as
// intended behaviour by integrated_bringup/test/test_joint_state_reorder.cpp,
// and narrowing `num_channels` to mean freshness was rejected (#265 decision
// B) because ValidateControllerOutput already uses the field as an egress
// bound. Closing the gap needs a new field or a new predicate — issue #284.
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
// drive on its previous setpoint. It is NOT a safe stop and must not be read
// as one (rtc_base/devices/device_backend.hpp spells out the same for
// WriteSafeCommand). It is the honest statement that this controller has
// nothing to say about this device right now.
//
// Only the primary device is silenced by a gate on the primary device: a hand
// does not stop being commandable because the arm's state went missing (§3.7,
// "secondary passthrough 유지"). Leave the other DeviceOutput entries alone.
//
// Commands already staged in the array are left in place — `num_channels == 0`
// makes them unreachable, and clearing kMaxDeviceChannels doubles would spend
// RT budget to change nothing observable.
inline void SilenceDeviceOutput(DeviceOutput& out) noexcept {
  out.num_channels = 0;
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
