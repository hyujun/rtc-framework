// ── §7.3 joint command tail ──────────────────────────────────────────────────
// The last three things that happen to a position-output controller's joint
// command after the law has produced a joint velocity:
//
//     q_cmd = q_base + q̇·Δt  →  clamp to [q_min+δ, q_max−δ]
//       →  rate rebound to q_base ± |v_max|·Δt  →  round-trip q̇
//
// §7.3 marks the ORDER as a MUST ("순서 반대 금지") and that is the whole reason
// this is a shared unit rather than a dozen lines in each binding. The clamp can
// WIDEN the step it was handed: a command sitting outside the band is pulled to
// the boundary in one tick no matter how small q̇ was, so `joint_limit_margin:
// 0.08` is a 0.08 rad jump unless the rebound runs AFTER it. Reverse the two and
// the rebound bounds a step the clamp then discards — the arm still jumps, and
// every value in the row still looks reasonable.
//
// The round-trip is not bookkeeping either: `target_velocities` is what the log
// and the publish lane report, and after two bounds the solve output is no
// longer the velocity the arm was actually asked for.
//
// Deliberately model-free and Eigen-free — spans in, spans out. `q` carries
// q_base IN and q_cmd OUT because that is how both callers already store it
// (`desired_q_` persists across ticks); `dq` is likewise in/out.
//
// The bindings that need this are the position-output ones whose command is
// self-integrated. It is NOT compliance-specific despite the directory: the
// task-space binding borrows it the same way it borrows differential_ik.hpp.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

namespace rtc::compliance {

/// Per-joint limits for the tail. A span shorter than `n` (or empty) falls back
/// to the matching default for the joints it does not cover — the same idiom
/// `rtc::utils::ClampRange` uses, so a config that describes only some joints
/// behaves identically in both places.
struct JointCommandBounds {
  std::span<const double> lower;         ///< q_min per joint
  std::span<const double> upper;         ///< q_max per joint
  std::span<const double> max_velocity;  ///< |q̇|_max per joint
  double margin{0.0};                    ///< δ [rad], shrinks the band on BOTH sides
  double default_lower{-6.2832};
  double default_upper{6.2832};
  double default_max_velocity{2.0};
};

/// What the tail actually did, for diagnostics and for tests that need to see a
/// bound fire rather than infer it from a value that happens to be in range.
struct JointCommandTailReport {
  int position_clamped{0};  ///< joints the [q_min+δ, q_max−δ] clamp moved
  int rate_rebounded{0};    ///< joints the q_base ± |v|Δt rebound then moved
};

/// RT, noexcept, no allocation. Integrates, bounds in §7.3 order, and rewrites
/// `dq` to the velocity the bounded command actually represents.
///
/// `dt <= 0` is a no-op on both spans: no time passed, so there is no step to
/// bound and `(q − q_base)/dt` has no value to report. Callers that floor their
/// dt (the RT loop does) never reach it; a test harness handing a zero dt gets
/// "nothing moved" instead of a NaN command.
///
/// A margin wide enough to invert the band (`lo >= hi`) skips the clamp for that
/// joint rather than collapsing the command onto a point. That is the RT half of
/// a defect whose other half belongs at configure time — see #473 for the
/// sibling binding that rejects an inverting margin outright, which is what
/// keeps this branch from being a silent way to disable the clamp.
///
/// NaN is deliberately NOT scrubbed: every comparison against it is false, so a
/// non-finite command passes through unchanged and reaches the fault
/// classification that owns it (`ComplianceFaults::nan_inf`). Clamping it to a
/// bound here would launder a fault into a plausible command.
inline JointCommandTailReport IntegrateAndBoundJointCommand(std::span<double> q,
                                                            std::span<double> dq, std::size_t n,
                                                            double dt,
                                                            const JointCommandBounds& b) noexcept {
  JointCommandTailReport report;
  if (!(dt > 0.0)) {
    return report;
  }
  n = std::min({n, q.size(), dq.size()});
  for (std::size_t i = 0; i < n; ++i) {
    const double q_base = q[i];
    double q_cmd = q_base + dq[i] * dt;

    const double lo = ((i < b.lower.size()) ? b.lower[i] : b.default_lower) + b.margin;
    const double hi = ((i < b.upper.size()) ? b.upper[i] : b.default_upper) - b.margin;
    if (lo < hi) {
      const double clamped = std::clamp(q_cmd, lo, hi);
      if (clamped != q_cmd) {
        q_cmd = clamped;
        ++report.position_clamped;
      }
    }

    const double v = (i < b.max_velocity.size()) ? b.max_velocity[i] : b.default_max_velocity;
    const double step = std::abs(v) * dt;
    const double bounded = std::clamp(q_cmd, q_base - step, q_base + step);
    if (bounded != q_cmd) {
      q_cmd = bounded;
      ++report.rate_rebounded;
    }

    q[i] = q_cmd;
    dq[i] = (q_cmd - q_base) / dt;
  }
  return report;
}

}  // namespace rtc::compliance
