// ── Output safety layer for compliance controllers (spec §10.5) ─────────────
// Applied to the raw joint-torque command, in this MANDATED order (order is
// load-bearing — see §10.5):
//   1. raw command (caller)
//   2. joint-limit repulsive term added        (§5.3)
//   3. absolute per-joint saturation           (±max_joint_torque)
//   4. rate limit using the ACTUAL tick dt      (never assume 500 Hz)
//   5. non-finite check → caller raises a fault (§10.6)
//
// Rate limiting comes AFTER saturation on purpose: saturating a rate-limited
// command would re-introduce a step. Each stage is a free function so it can be
// unit-tested in isolation. Eigen-domain analog of rtc_base/utils/clamp_commands
// (which clamps the device-channel array at the backend boundary); kept in Eigen
// here to stay allocation-free on the joint-torque vector.
#pragma once

#include <Eigen/Core>

#include <cmath>

namespace rtc::compliance {

// Result of one ApplySafetyLayer() pass — feeds the state machine (§10.6) and
// the nullspace-priority diagnostic (§6.4).
struct SafetyStatus {
  bool saturated{false};  ///< a joint hit ±max_joint_torque (nullspace priority may be violated)
  bool rate_limited{false};  ///< a joint's slew was clamped this tick
  bool finite{true};         ///< false ⇒ NaN/Inf in the command ⇒ caller must fault
};

// Stage 2 — §5.3 joint-limit repulsive potential: a spring pushing away from the
// soft bound (q_min+δ, q_max−δ) PLUS velocity damping inside the margin band.
// Added into `tau` (does not overwrite the control law). The damping term is
// mandatory — a spring alone chatters at the boundary. Per joint, RT-safe.
inline void AddJointLimitRepulsive(Eigen::Ref<Eigen::VectorXd> tau,
                                   const Eigen::Ref<const Eigen::VectorXd>& q,
                                   const Eigen::Ref<const Eigen::VectorXd>& qdot,
                                   const Eigen::Ref<const Eigen::VectorXd>& q_min,
                                   const Eigen::Ref<const Eigen::VectorXd>& q_max, double margin,
                                   double k_lim, double d_lim) noexcept {
  for (Eigen::Index i = 0; i < tau.size(); ++i) {
    const double lo = q_min(i) + margin;
    const double hi = q_max(i) - margin;
    if (q(i) < lo)
      tau(i) += -k_lim * (q(i) - lo) - d_lim * qdot(i);
    else if (q(i) > hi)
      tau(i) += -k_lim * (q(i) - hi) - d_lim * qdot(i);
  }
}

// Stage 3 — absolute per-joint saturation to ±tau_max(i). Returns true if any
// joint was clamped. RT-safe.
[[nodiscard]] inline bool SaturateAbsolute(
    Eigen::Ref<Eigen::VectorXd> tau, const Eigen::Ref<const Eigen::VectorXd>& tau_max) noexcept {
  bool hit = false;
  for (Eigen::Index i = 0; i < tau.size(); ++i) {
    const double lim = std::abs(tau_max(i));
    if (tau(i) > lim) {
      tau(i) = lim;
      hit = true;
    } else if (tau(i) < -lim) {
      tau(i) = -lim;
      hit = true;
    }
  }
  return hit;
}

// Stage 4 — per-joint rate limit: |tau − tau_prev| ≤ max_rate·dt, using the
// REAL tick dt (state.dt), never a hard-coded 500 Hz. tau_prev is advanced to
// the limited command so the next tick continues from it. dt ≤ 0 disables the
// stage (a degenerate tick must not freeze the command at tau_prev). Returns
// true if any joint's slew was clamped. RT-safe.
[[nodiscard]] inline bool RateLimit(Eigen::Ref<Eigen::VectorXd> tau,
                                    Eigen::Ref<Eigen::VectorXd> tau_prev, double max_rate,
                                    double dt) noexcept {
  if (dt <= 0.0 || max_rate <= 0.0) {
    tau_prev = tau;
    return false;
  }
  const double max_step = max_rate * dt;
  bool clamped = false;
  for (Eigen::Index i = 0; i < tau.size(); ++i) {
    const double delta = tau(i) - tau_prev(i);
    if (delta > max_step) {
      tau(i) = tau_prev(i) + max_step;
      clamped = true;
    } else if (delta < -max_step) {
      tau(i) = tau_prev(i) - max_step;
      clamped = true;
    }
    tau_prev(i) = tau(i);
  }
  return clamped;
}

// Stage 5 — non-finite check (NaN/Inf). true iff every entry is finite.
[[nodiscard]] inline bool AllFinite(const Eigen::Ref<const Eigen::VectorXd>& tau) noexcept {
  return tau.allFinite();
}

// Orchestrator applying stages 2→4 in the mandated order, with the non-finite
// check (§10.5 step 5) captured on the post-repulsive command BEFORE saturation.
// Reason: SaturateAbsolute clamps ±∞ to the limit (∞ > lim → lim), which would
// MASK an infinite command and let it reach the backend as max torque with no
// fault. NaN survives any clamp (its comparisons are all false), but ∞ does not
// — so finiteness must be read before the clamps to catch both. The clamp
// ordering itself is unchanged (saturation before rate limit). `tau` is
// modified in place; `tau_prev` carries the rate-limit history across ticks.
// RT-safe.
[[nodiscard]] inline SafetyStatus ApplySafetyLayer(
    Eigen::Ref<Eigen::VectorXd> tau, Eigen::Ref<Eigen::VectorXd> tau_prev,
    const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& qdot,
    const Eigen::Ref<const Eigen::VectorXd>& q_min, const Eigen::Ref<const Eigen::VectorXd>& q_max,
    const Eigen::Ref<const Eigen::VectorXd>& tau_max, double margin, double k_lim, double d_lim,
    double max_rate, double dt) noexcept {
  SafetyStatus s;
  AddJointLimitRepulsive(tau, q, qdot, q_min, q_max, margin, k_lim, d_lim);
  s.finite = AllFinite(tau);  // before saturation can mask an ∞ (see note above)
  s.saturated = SaturateAbsolute(tau, tau_max);
  s.rate_limited = RateLimit(tau, tau_prev, max_rate, dt);
  return s;
}

}  // namespace rtc::compliance
