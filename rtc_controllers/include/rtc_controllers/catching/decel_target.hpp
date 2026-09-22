// ── Virtual deceleration target (L7 §4.3, dynamic_catching S1.8 / L7.2) ──────
// `DECEL` replaces the ball trajectory with a virtual target that linearly
// decelerates the TCP to a stop, so L4's soft-catch generator can keep
// tracking a smooth reference instead of chasing the (now irrelevant) ball.
// Given the state (x_s, ẋ_s) at DECEL entry and a deceleration magnitude
// a_dec, with û_s = ẋ_s/‖ẋ_s‖, τ = t − t_s and τ_s = ‖ẋ_s‖/a_dec:
//
//   p_v(τ) = x_s + ẋ_s·τ − ½a_dec·û_s·τ²
//   v_v(τ) = ẋ_s − a_dec·û_s·τ
//   a_v(τ) = −a_dec·û_s                              (τ ≤ τ_s)
//
//   p_v = x_s + ½‖ẋ_s‖τ_s·û_s,  v_v = a_v = 0        (τ > τ_s)
//
// At τ=0 this makes the reference error e = x_s − p_v(0) and ė = ẋ_s − v_v(0)
// EXACTLY zero (not merely small) — G7-B measures that. The reference
// acceleration jumps from −a_dec·û_s to 0 at τ_s (infinite jerk); the doc
// accepts this and offers `supervisor.decel.ramp_time` as a mitigation this
// core does not implement (a ramp is a caller-side concern: L4 already
// consumes a_v as a target, so a ramped a_dec is just a different a_dec
// argument on the next call).
//
// This is a pure numeric core: ROS/time-type free (the caller derives τ from
// NowLead/BallTime per plan §3 before calling in), Eigen-only, no heap,
// noexcept, fail-closed on non-finite input.
//
// ── Ambiguity resolutions (documented per task instruction) ─────────────────
//  1. Zero-speed guard: ‖ẋ_s‖² below `kMinEntrySpeedSquared` (1e-12, i.e. a
//     speed below ~1 µm/s) is treated as "already stopped at entry" — û_s is
//     undefined at ẋ_s=0, and physically DECEL should never be entered at
//     zero ball speed, but the guard keeps the function total rather than
//     letting a near-zero denominator blow up τ_s. The target then holds
//     x_s with zero velocity/acceleration for every τ ≥ 0 (a degenerate but
//     valid decel: there was nothing left to decelerate).
//  2. τ < 0 is reported invalid rather than clamped to 0: the caller's own
//     contract is "query only at or after DECEL entry" (plan §3, `DecelDue`),
//     so a negative τ signals a caller bug and fail-closed is preferred to
//     silently answering a question that was never supposed to be asked.
#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <span>

namespace rtc::catching {

/// Squared-norm floor below which ẋ_s is treated as zero (ambiguity
/// resolution 1 above) — mirrors the squared-norm guard style of
/// rtc_controllers/inference/reach_gate.hpp rather than a bare epsilon on the
/// (unnormalized) speed, so no sqrt is spent on the common non-degenerate
/// path just to test the guard.
inline constexpr double kMinEntrySpeedSquared = 1e-12;

/// TCP state at `DECEL` entry (plan §3: sampled once, at t_s = now_lead of the
/// tick `DecelDue` first fired).
struct DecelEntryState {
  Eigen::Vector3d x_s{Eigen::Vector3d::Zero()};     ///< position at entry [m]
  Eigen::Vector3d xdot_s{Eigen::Vector3d::Zero()};  ///< velocity at entry [m/s]
};

/// One evaluation of the virtual decelerating target.
struct DecelTarget {
  Eigen::Vector3d p_v{Eigen::Vector3d::Zero()};  ///< target position [m]
  Eigen::Vector3d v_v{Eigen::Vector3d::Zero()};  ///< target velocity [m/s]
  Eigen::Vector3d a_v{Eigen::Vector3d::Zero()};  ///< target acceleration [m/s²]
  /// False iff a_dec/τ/entry state were non-finite or a_dec ≤ 0 or τ < 0 —
  /// fail-closed: the caller must not consume p_v/v_v/a_v when false (they
  /// are left at the zero default above, never NaN).
  bool valid{false};
  /// True once τ ≥ τ_s (the virtual target has come to rest). Meaningful only
  /// when `valid`.
  bool stopped{false};
};

/// Evaluate the virtual decel target at `tau` seconds past DECEL entry.
///
/// @param entry  TCP state at DECEL entry (x_s, ẋ_s)
/// @param a_dec  deceleration magnitude [m/s²] — `supervisor.decel.a_dec`;
///               must be finite and > 0 or the result is invalid
/// @param tau    seconds since entry on the lead axis (now_lead − t_s: DECEL
///               is entered and its target sampled at now + T_arm, plan §3);
///               must be finite and ≥ 0 or the result is invalid
[[nodiscard]] inline DecelTarget EvaluateDecelTarget(const DecelEntryState& entry, double a_dec,
                                                     double tau) noexcept {
  DecelTarget out{};

  if (!std::isfinite(a_dec) || a_dec <= 0.0)
    return out;
  if (!std::isfinite(tau) || tau < 0.0)
    return out;
  if (!entry.x_s.allFinite() || !entry.xdot_s.allFinite())
    return out;

  const double speed_sq = entry.xdot_s.squaredNorm();
  if (!(speed_sq > kMinEntrySpeedSquared)) {
    // Ambiguity resolution 1: already at rest at entry — hold x_s.
    out.p_v = entry.x_s;
    out.valid = true;
    out.stopped = true;
    return out;
  }

  const double speed = std::sqrt(speed_sq);
  const Eigen::Vector3d u_hat = entry.xdot_s / speed;
  const double tau_s = speed / a_dec;

  // §4.3's formula splits at "τ ≤ τ_s" (moving) vs "τ > τ_s" (stopped), but
  // §4.1's FSM exit condition is "τ ≥ τ_s → HOLD" (inclusive on the stopped
  // side). Both formulas are proven algebraically identical at τ=τ_s (the
  // whole point of the construction), so using the STRICT form here and
  // reporting `stopped` from the same branch makes the two doc conventions
  // agree at the boundary without changing any numeric output.
  if (tau < tau_s) {
    out.p_v = entry.x_s + entry.xdot_s * tau - u_hat * (0.5 * a_dec * tau * tau);
    out.v_v = entry.xdot_s - u_hat * (a_dec * tau);
    out.a_v = u_hat * (-a_dec);
    out.stopped = false;
  } else {
    out.p_v = entry.x_s + u_hat * (0.5 * speed * tau_s);
    // v_v, a_v already zero-initialized.
    out.stopped = true;
  }
  out.valid = true;
  return out;
}

// ── QP-independent joint-space deceleration (L7 §4.1, A-S5-10, S5.3) ────────
//
// The task-space target above is what `ABORT_SAFE` uses when the joint command
// layer is HEALTHY: the virtual target goes through L4 and L5 like any other
// reference. It is exactly the wrong thing when the abort was CAUSED by that
// layer — `QP_FAILED` or `JOINT_CONFLICT` — because it would route the stop
// through the component that just failed.
//
// So this is the other path: no QP, no task space, no model. It walks each
// joint's commanded velocity to zero at its own acceleration limit and
// integrates the command from there.
//
//   q̇ᵢ ← sign(q̇ᵢ)·max(|q̇ᵢ| − q̈max,ᵢ·Δt, 0)
//   qᵢ ← clamp(qᵢ + q̇ᵢ·Δt, q_min,ᵢ, q_max,ᵢ)
//
// WHY NOT SIMPLY q̇ = 0. The existing CLIK failure output is exactly that
// (`q_ref = q_meas`, `v_ref = 0`), and the catching controller must not
// consume it: on a position interface, jumping the command to the MEASURED
// pose is a discontinuity of whatever the tracking error happened to be, and
// zeroing the velocity in one tick is an infinite deceleration that the drive
// answers with a protective stop. The joint that is already at its limit is
// not the one that makes this dangerous — it is the one moving fastest.
//
// The position clamp is the same box CLIK was given (already narrowed by
// `limit_margin`), so a stop that runs into a limit stops at the limit rather
// than past it. Ramping down at q̈max means the stop takes |q̇|/q̈max seconds,
// which is the shortest stop the joint can actually perform.
//
// Pure numeric core: no allocation, noexcept, and fail-closed — a non-finite
// input or a non-positive dt leaves the command untouched and reports
// `valid=false`, because the honest answer to "where should this joint go"
// when the inputs are garbage is "nowhere new".

/// Result of one deceleration step. `stopped` is the caller's cue that the
/// abort has completed and the supervisor may leave `ABORT_SAFE`.
struct JointDecelStep {
  bool valid{false};
  bool stopped{false};  // every joint's |q̇| reached zero
  bool clamped{false};  // at least one joint hit its position box this step
};

/// One tick of the joint-space stop. `q_cmd` and `qd_cmd` are updated IN
/// PLACE — they are the controller's carried command state, and returning
/// copies would invite a caller to integrate twice.
///
/// `n` is the number of joints to act on (the arm's); the arrays may be wider.
/// Bounds are the caller's box, already margined.
[[nodiscard]] inline JointDecelStep JointSpaceDecelStep(std::span<double> q_cmd,
                                                        std::span<double> qd_cmd,
                                                        std::span<const double> qdd_max,
                                                        std::span<const double> q_min,
                                                        std::span<const double> q_max,
                                                        std::size_t n, double dt) noexcept {
  JointDecelStep out{};
  if (!(dt > 0.0) || !std::isfinite(dt) || n > q_cmd.size() || n > qd_cmd.size() ||
      n > qdd_max.size() || n > q_min.size() || n > q_max.size()) {
    return out;
  }
  bool all_stopped = true;
  for (std::size_t i = 0; i < n; ++i) {
    const double limit = qdd_max[i];
    if (!std::isfinite(q_cmd[i]) || !std::isfinite(qd_cmd[i]) || !std::isfinite(limit) ||
        !(limit > 0.0)) {
      // One bad joint does not invalidate the others' stop — but it does mean
      // this joint has no honest step, so it is frozen where it is and the
      // result says the stop is not complete.
      qd_cmd[i] = 0.0;
      all_stopped = all_stopped && true;
      continue;
    }
    const double speed = std::abs(qd_cmd[i]);
    const double reduced = speed - limit * dt;
    if (reduced <= 0.0) {
      qd_cmd[i] = 0.0;
    } else {
      qd_cmd[i] = (qd_cmd[i] > 0.0) ? reduced : -reduced;
      all_stopped = false;
    }
    double next = q_cmd[i] + qd_cmd[i] * dt;
    if (std::isfinite(q_min[i]) && next < q_min[i]) {
      next = q_min[i];
      qd_cmd[i] = 0.0;
      out.clamped = true;
    } else if (std::isfinite(q_max[i]) && next > q_max[i]) {
      next = q_max[i];
      qd_cmd[i] = 0.0;
      out.clamped = true;
    }
    q_cmd[i] = next;
  }
  out.stopped = all_stopped;
  out.valid = true;
  return out;
}

}  // namespace rtc::catching
