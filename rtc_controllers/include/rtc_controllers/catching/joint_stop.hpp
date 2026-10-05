// ── Joint-space stop ramp: QP-independent (L7 §4.1, A-S5-10, S5.3) ───────────
//
// The task-space target (decel_target.hpp) is DECEL's reference: it goes
// through L4 and L5 like any other reference. `ABORT_SAFE` never uses it — its
// stop is always in joint space, whatever the cause (L7 §4.1). A task-space
// stop would be exactly the wrong thing when the abort was CAUSED by that layer
// (`QP_FAILED` or `JOINT_CONFLICT`), because it would route the stop through
// the component that just failed, and one path for every cause is one path to
// verify.
//
// So this is that path: no QP, no task space, no model. It walks each
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
#pragma once

#include <cmath>
#include <cstddef>
#include <span>

namespace rtc::catching {

/// Result of one deceleration step. `stopped` is the caller's cue that the
/// abort has completed and the supervisor may leave `ABORT_SAFE`.
struct JointStopStep {
  bool valid{false};
  bool stopped{false};  // every joint's |q̇| reached zero
  bool clamped{false};  // at least one joint hit its position box this step
  /// At least one joint's command, velocity or limit was not a finite number.
  /// That joint was left untouched and the stop is reported INCOMPLETE — the
  /// caller decides what to do about a command it can no longer integrate
  /// (the supervisor's answer is to stay in the abort, which is also what
  /// keeps the value from being treated as a finished stop).
  bool non_finite{false};
};

/// One tick of the joint-space stop. `q_cmd` and `qd_cmd` are updated IN
/// PLACE — they are the controller's carried command state, and returning
/// copies would invite a caller to integrate twice.
///
/// `n` is the number of joints to act on (the arm's); the arrays may be wider.
/// Bounds are the caller's box, already margined.
[[nodiscard]] inline JointStopStep JointSpaceStopStep(std::span<double> q_cmd,
                                                      std::span<double> qd_cmd,
                                                      std::span<const double> qdd_max,
                                                      std::span<const double> q_min,
                                                      std::span<const double> q_max, std::size_t n,
                                                      double dt) noexcept {
  JointStopStep out{};
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
      // NOT counted as stopped, and the joint is left ALONE. `stopped` is the
      // caller's cue to LEAVE the abort, and a joint whose command is not a
      // number has not been brought anywhere — ending the stop would carry
      // that value into the next trial. Zeroing the velocity would also be the
      // one-tick stop this function exists to avoid, on a joint whose state is
      // unknown.
      out.non_finite = true;
      all_stopped = false;
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
