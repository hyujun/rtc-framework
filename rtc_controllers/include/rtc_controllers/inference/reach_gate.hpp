// ── Reach gate: proximity ∨ tactile hold, as a grasp policy observes it ──────
//
// Some grasp policies are trained with a scalar that says "the hand is where
// the recorded grasp wants it, or it is already holding something" and gate
// their own hand closure on it. The scalar is computed OUTSIDE the network, so
// a deployment has to reproduce the training environment's formula exactly —
// a 0/1 approximation skips the ramp the policy learned to close along:
//
//   gate = max( exp(-(d_tip / σ)²),  hold ? 1 : 0 )        (no ratchet)
//   d_tip = mean_i ‖ tip_i − (p_obj + R(q_obj)·c_i) ‖
//   hold  = Schmitt trigger on "the first digit and ≥ min_fingers of the
//           others are pressing harder than the threshold":
//           ON after `hold_on_steps` consecutive grasped steps,
//           OFF after `hold_off_steps` consecutive non-grasped steps
//
// `c_i` are recorded contact points in the OBJECT frame, so the targets ride
// along with the object. The first tip is the opposing digit (a thumb): the
// grasped test is asymmetric on purpose — a thumb alone, or three fingers with
// no thumb, is not a grasp.
//
// The step counts are counted in POLICY steps, not RT ticks, so the caller
// updates once per evaluation. Hold is updated BEFORE the proximity term is
// read, matching the event order the formula was trained with.
//
// Core, not binding (the boundary rule of policy_io.hpp): spans of double in, a
// double out. No robot facts — which links are tips and which force lanes they
// read is YAML (params/reach_gate_params.hpp). RT: noexcept, no allocation.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>

namespace rtc::inference {

/// Schmitt-trigger state. Value-initialised is "not holding, nothing counted",
/// which is also what a reset must restore.
struct ReachHoldState {
  bool hold{false};
  int on_count{0};   ///< consecutive grasped steps
  int off_count{0};  ///< consecutive non-grasped steps
};

/// The asymmetric grasp test. `tip_force_norms[0]` is the opposing digit; the
/// comparison is STRICT (`>`), as trained. Fewer than two tips cannot grasp.
/// A non-finite norm is treated as "not pressing" — it cannot vouch for a grasp.
[[nodiscard]] inline bool IsGraspedByForce(std::span<const double> tip_force_norms,
                                           double threshold, int min_fingers) noexcept {
  if (tip_force_norms.size() < 2) {
    return false;
  }
  const auto pressing = [threshold](double f) { return std::isfinite(f) && f > threshold; };
  if (!pressing(tip_force_norms[0])) {
    return false;
  }
  int others = 0;
  for (std::size_t i = 1; i < tip_force_norms.size(); ++i) {
    others += pressing(tip_force_norms[i]) ? 1 : 0;
  }
  return others >= min_fingers;
}

/// Advance the trigger by one POLICY step and return the new hold.
///
/// Both counters run every step and each resets when the other condition
/// holds, so a single contrary step restarts the count — that is the whole
/// debounce. Counters saturate rather than wrap on a run that never ends.
inline bool UpdateReachHold(ReachHoldState& state, bool grasped, int hold_on_steps,
                            int hold_off_steps) noexcept {
  constexpr int kCountCap = std::numeric_limits<int>::max() - 1;
  state.on_count = grasped ? std::min(state.on_count + 1, kCountCap) : 0;
  state.off_count = grasped ? 0 : std::min(state.off_count + 1, kCountCap);
  if (state.on_count >= hold_on_steps) {
    state.hold = true;
  }
  if (state.off_count >= hold_off_steps) {
    state.hold = false;
  }
  return state.hold;
}

/// `v' = R(q)·v` for a Hamilton quaternion serialised x, y, z, w.
///
/// Normalises first: a published pose is unit to rounding, but the formula is
/// only a rotation for a unit quaternion and a scaled one would stretch every
/// target. A (near-)zero quaternion has no rotation to offer and yields NaN, so
/// the caller's finiteness screen turns it into a refusal (NUM-2).
inline void RotateByQuatXyzw(std::span<const double, 4> q_xyzw, std::span<const double, 3> v,
                             std::span<double, 3> out) noexcept {
  constexpr double kMinNormSquared = 1e-12;
  const double n2 = (q_xyzw[0] * q_xyzw[0]) + (q_xyzw[1] * q_xyzw[1]) + (q_xyzw[2] * q_xyzw[2]) +
                    (q_xyzw[3] * q_xyzw[3]);
  if (!(n2 > kMinNormSquared) || !std::isfinite(n2)) {
    out[0] = out[1] = out[2] = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  const double inv = 1.0 / std::sqrt(n2);
  const double x = q_xyzw[0] * inv;
  const double y = q_xyzw[1] * inv;
  const double z = q_xyzw[2] * inv;
  const double w = q_xyzw[3] * inv;
  // v + 2 u × (u × v + w v), u = (x, y, z)
  const double cx = (y * v[2]) - (z * v[1]) + (w * v[0]);
  const double cy = (z * v[0]) - (x * v[2]) + (w * v[1]);
  const double cz = (x * v[1]) - (y * v[0]) + (w * v[2]);
  out[0] = v[0] + (2.0 * ((y * cz) - (z * cy)));
  out[1] = v[1] + (2.0 * ((z * cx) - (x * cz)));
  out[2] = v[2] + (2.0 * ((x * cy) - (y * cx)));
}

/// Mean distance between each tip and its recorded contact point carried by the
/// object's current pose. `tip_positions` and `contact_points_obj` are packed
/// xyz triples, one per tip, in the SAME order; everything is in one frame
/// (the one the object pose is quoted in).
///
/// Returns NaN when the spans disagree, when there are no tips, or when any
/// input is non-finite — a distance that looks valid but was computed from half
/// the tips would open the gate on a guess.
[[nodiscard]] inline double MeanTipDistance(std::span<const double> tip_positions,
                                            std::span<const double, 3> object_position,
                                            std::span<const double, 4> object_orientation_xyzw,
                                            std::span<const double> contact_points_obj) noexcept {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  if (tip_positions.empty() || tip_positions.size() % 3 != 0 ||
      contact_points_obj.size() != tip_positions.size()) {
    return kNaN;
  }
  const std::size_t n = tip_positions.size() / 3;
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    std::array<double, 3> target{};
    RotateByQuatXyzw(object_orientation_xyzw,
                     std::span<const double, 3>(contact_points_obj.subspan(3 * i, 3).data(), 3),
                     std::span<double, 3>(target));
    const double dx = tip_positions[3 * i] - (object_position[0] + target[0]);
    const double dy = tip_positions[(3 * i) + 1] - (object_position[1] + target[1]);
    const double dz = tip_positions[(3 * i) + 2] - (object_position[2] + target[2]);
    sum += std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
  }
  const double mean = sum / static_cast<double>(n);
  return std::isfinite(mean) ? mean : kNaN;
}

/// `exp(-(d/σ)²)`. NaN in, NaN out (σ ≤ 0 is a configure-time refusal, so it is
/// not re-checked here).
[[nodiscard]] inline double ReachProximity(double distance, double tip_std) noexcept {
  const double r = distance / tip_std;
  return std::exp(-(r * r));
}

/// `max(proximity, hold ? 1 : 0)`, recomputed every step — no ratchet, so a
/// released hold lets the gate fall back to proximity and the hand reopen.
/// A NaN proximity stays NaN even under a hold: the caller screens it and holds
/// the robot, rather than letting the tactile half launder a broken distance.
[[nodiscard]] inline double ReachGate(double proximity, bool hold) noexcept {
  if (!std::isfinite(proximity)) {
    return proximity;
  }
  return std::max(proximity, hold ? 1.0 : 0.0);
}

}  // namespace rtc::inference
