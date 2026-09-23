// ── Joint-space homing / return law (L7 §4.1, dynamic_catching S7.2) ────────
// IDLE's homing and RETREAT's return to the wait pose move the arm in JOINT
// space, without the QP (#537 S7 decision Q3): the wait pose is a joint vector
// and "arrived" is judged per joint (`supervisor.ready.pose_tol`, rad), and a
// RETREAT that follows QP_FAILED / JOINT_CONFLICT cannot route its motion back
// through the solver that just failed. L4 §5.3's task-space retreat reference
// was never built; this replaces it.
//
// THE LAW, per joint, on the controller's CARRIED command (q, q̇) — the same
// state the abort ramp (JointSpaceDecelStep) integrates, so the two hand over
// without a step:
//
//   d      = q* − q                                (q* clamped into the box)
//   v_brk  = √((a·Δt/2)² + 2·a·|d|) − a·Δt/2
//   v_des  = sign(d)·min(v_max, v_brk, |d|/Δt)
//   q̇     ← q̇ + clamp(v_des − q̇, −a·Δt, a·Δt)
//   q      ← clamp(q + q̇·Δt, q_min, q_max)
//
// with a = η_a·q̈_max,i. v_brk is the speed from which the DISCRETE law —
// velocity first, then position, shedding a·Δt per tick — still stops at q*:
// the distance it covers is v²/(2a) + v·Δt/2, and solving that for v gives
// the root above. The continuous-time √(2a|d|) brakes one step late on every
// step and overshoots by tens of a·Δt² on a short move; this form overshoots
// by at most a·Δt²/8 from rest. |d|/Δt keeps a single step from jumping past
// q*; the acceleration clamp means a start from a nonzero carried velocity —
// even one pointing AWAY from q* — turns around at a, never in one tick. A
// joint that is within a·Δt² of q* and slower than a·Δt is snapped onto q* at
// rest: below that the discrete law can only chatter.
//
// Joints move independently (no synchronised arrival): the wait pose is a
// posture, and the path to it only has to stay inside the box — the time to
// the slowest joint is what the trial cadence pays either way.
//
// Pure numeric core: no allocation, noexcept, fail-closed. A non-finite input
// on a joint leaves that joint untouched and reports the step incomplete, like
// JointSpaceDecelStep — "arrived" must never be the answer for a joint whose
// command is not a number.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

namespace rtc::catching {

struct JointHomeStep {
  bool valid{false};
  /// Every joint's command sits ON q* at rest (the law's own arrival — the
  /// supervisor separately checks that the MEASURED arm followed).
  bool arrived{false};
  bool clamped{false};     // a joint's command hit the position box this step
  bool non_finite{false};  // a joint's input was not finite; it was left alone
};

/// One tick of the homing law. `q_cmd` / `qd_cmd` are updated IN PLACE (the
/// carried command). `n` joints are acted on; the spans may be wider. The box
/// is the caller's, already margined (the same one CLIK and the abort ramp
/// get). `eta_a` ∈ (0, 1] scales each joint's q̈_max; `v_max` > 0 [rad/s] caps
/// every joint.
[[nodiscard]] inline JointHomeStep JointSpaceHomeStep(
    std::span<double> q_cmd, std::span<double> qd_cmd, std::span<const double> q_target,
    std::span<const double> qdd_max, double eta_a, double v_max, std::span<const double> q_min,
    std::span<const double> q_max, std::size_t n, double dt) noexcept {
  JointHomeStep out{};
  if (!(dt > 0.0) || !std::isfinite(dt) || !(eta_a > 0.0) || !(eta_a <= 1.0) || !(v_max > 0.0) ||
      !std::isfinite(v_max) || n > q_cmd.size() || n > qd_cmd.size() || n > q_target.size() ||
      n > qdd_max.size() || n > q_min.size() || n > q_max.size()) {
    return out;
  }
  bool all_arrived = true;
  for (std::size_t i = 0; i < n; ++i) {
    const double a = eta_a * qdd_max[i];
    if (!std::isfinite(q_cmd[i]) || !std::isfinite(qd_cmd[i]) || !std::isfinite(q_target[i]) ||
        !std::isfinite(a) || !(a > 0.0)) {
      out.non_finite = true;
      all_arrived = false;
      continue;
    }
    double target = q_target[i];
    if (std::isfinite(q_min[i])) {
      target = std::max(target, q_min[i]);
    }
    if (std::isfinite(q_max[i])) {
      target = std::min(target, q_max[i]);
    }

    const double d = target - q_cmd[i];
    const double dv = a * dt;
    if (std::abs(d) <= dv * dt && std::abs(qd_cmd[i]) <= dv) {
      q_cmd[i] = target;
      qd_cmd[i] = 0.0;
      continue;  // arrived
    }
    all_arrived = false;

    const double half_dv = 0.5 * dv;
    const double v_brake = std::sqrt(half_dv * half_dv + 2.0 * a * std::abs(d)) - half_dv;
    const double speed = std::min({v_max, v_brake, std::abs(d) / dt});
    const double v_des = d > 0.0 ? speed : -speed;
    qd_cmd[i] += std::clamp(v_des - qd_cmd[i], -dv, dv);
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
  out.arrived = all_arrived;
  out.valid = true;
  return out;
}

}  // namespace rtc::catching
