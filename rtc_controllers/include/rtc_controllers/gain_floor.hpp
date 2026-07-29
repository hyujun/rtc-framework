// ── The non-negative gain floor, in ONE place ───────────────────────────────
// Two invariants name the same bound on different gains — NUM-6 on the
// null-space posture gains (K_pⁿ, K_dⁿ) and, since #280, the §5.3 joint-limit
// safety gains (k_lim, d_lim) the output safety layer adds inside the margin
// band. Both say "≥ 0, and a non-finite value must reach the finite check
// downstream rather than be laundered into a plausible one". A bound named by
// an invariant must be a symbol a future binding can GREP, not a `std::max`
// open-coded at each site plus a row in a doc table — the argument
// `compliance::FloorMaxDamping` makes for NUM-1's λ_max, which stays a separate
// symbol only because its floor is a different constant (1e-4) and it belongs
// next to the law it protects.
//
// The name is deliberately NOT `FloorPostureGain` (which is what this was
// through #277/#279): #280 gave the same bound to gains that are not posture
// gains at all, and a symbol whose name lies at half its call sites is how a
// reader concludes the rule does not apply to theirs.
#pragma once

#include <algorithm>
#include <cmath>

namespace rtc {

/// Floor a gain at 0, passing non-finite values through UNCHANGED.
///
/// Why not `std::max(0.0, g)` written out at each site: `std::max` is
/// `a < b ? b : a`, and `0.0 < NaN` is FALSE, so it returns **0.0** for a NaN
/// gain. That is not a floor, it is laundering — a non-finite gain used to reach
/// the law, make the torque non-finite and latch SAFE_STOP through the
/// controller's `nan_inf` fault, and a plain `std::max` instead discards the
/// operator's corrupt gain in silence (for a posture gain it also closes the
/// activation gate, so the symptom disappears entirely). So a non-finite gain is
/// passed through UNCHANGED and left for the finite-output check that already
/// exists downstream — `compliance::AllFinite`, which §10.5 evaluates on the
/// post-repulsive command before saturation can mask an ∞. (+inf is unchanged
/// either way; −inf reaches the same fault as NaN.)
///
/// Applied by the CONSUMER, never by the law. Every law here takes its gains as
/// plain arguments and floors nothing (`ComputePostureTorque`,
/// `AdaptiveDampingSquared`, `compliance::AddJointLimitRepulsive`): a law that
/// silently rewrites its argument falsifies its own literal oracles, and every
/// oracle in this repo drives positive gains, so the change would be invisible
/// to them. The consumer is also where the activation gate and the fault live.
///
/// Consumers apply it TWICE — in the loader unconditionally (not inside
/// `if (cfg[key])`: an absent key still leaves a value from the constructor or a
/// previous `set_gains()`, which are exactly the two paths the floor exists
/// for), and again at the point of use, because `set_gains()` writes the Gains
/// POD straight into the SeqLock and bypasses configure entirely. The
/// point-of-use half of both NUM-1 and NUM-6 lived in the adapters deleted by
/// #298 S7c-2; for the safety-layer gains it is a BINDING REQUIREMENT with no
/// in-tree caller (nothing shipped reads `joint_limit_stiffness` yet), which is
/// precisely why the rule needs a symbol rather than a convention.
///
/// Not every bound in this family is a floor: `joint_limit_margin` is REJECTED
/// at configure instead, because clamping it to 0 is indistinguishable from the
/// misconfiguration — see the parsers in `params/`.
[[nodiscard]] inline double FloorNonNegativeGain(double gain) noexcept {
  return std::isfinite(gain) ? std::max(0.0, gain) : gain;
}

/// The same family's REJECT half: true iff @p v is a usable non-negative bound.
///
/// A floor works when the floored value still does the job the gain was there to
/// do — 0 is a legitimate posture gain (no centering) and a legitimate k_lim (no
/// spring). It does NOT work for a value that PARAMETERISES a guard's geometry,
/// where clamping produces a configuration indistinguishable from the
/// misconfiguration: `joint_limit_margin` sets the band edges
/// (`lo = q_min + δ`), so a negative δ puts the band OUTSIDE the joint limits
/// and a non-finite one makes both `q < lo` and `q > hi` false — either way the
/// §5.3 repulsive term never fires again, with no fault and no diagnostic,
/// because nothing downstream can tell "the guard is disarmed" from "the joint
/// is comfortably inside its band". Clamping δ to 0 hides the operator's
/// mistake behind a value the arm will happily run.
///
/// `!(v >= 0.0)` rather than `v < 0.0` for the same reason `!(v > 0.0)` guards
/// the F8 thresholds: NaN makes every comparison false, so the negated form is
/// the one that catches it. Zero is ACCEPTED — a zero margin is the degenerate
/// but meaningful "band edges are the joint limits" (and is the shipped default
/// for task admittance).
[[nodiscard]] inline bool IsFiniteNonNegative(double v) noexcept {
  return std::isfinite(v) && v >= 0.0;
}

}  // namespace rtc
