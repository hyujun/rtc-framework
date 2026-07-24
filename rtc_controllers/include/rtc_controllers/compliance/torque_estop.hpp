// ── Torque-mode E-STOP hold (E-8, approved 2026-07-24) ──────────────────────
// A torque controller cannot E-STOP by slewing a position setpoint (the #184
// position-mode pattern) — it commands torque. Instead hold the arm against
// gravity while bleeding kinetic energy:
//
//   τ_estop = ĝ(q) − D·q̇,   clamped per joint to ±τ_max
//
// ĝ(q) keeps the arm from collapsing; −D·q̇ damps residual motion to rest. This
// is the SAME helper that #184's OSC E-STOP should migrate onto (deferred), so
// the two do not diverge. Not merged with the CM global E-STOP latch — see
// compliance_state_machine.hpp. RT-safe: noexcept, fixed size, no heap/throw.
#pragma once

#include <Eigen/Core>

#include <cmath>

namespace rtc::compliance {

// Write the gravity-compensated damped-hold torque into `tau_out` (nv). `gravity`
// is ĝ(q) (nv), `qdot` the measured joint velocity (nv), `damping` the scalar D
// (≥ 0), `tau_max` the per-joint torque limit (nv, used as ±|tau_max(i)|).
//
// Finite-guard: this is the LAST-RESORT safety fallback (it runs precisely when
// the main path already produced non-finite output), so a non-finite input must
// not propagate to the actuator. A NaN in ĝ(q) or q̇ (e.g. a frozen/NaN encoder
// channel) would survive the per-joint clamp untouched — NaN fails every
// comparison, and ±∞ would clamp to ±lim and silently mask the fault — so any
// non-finite result is forced to 0 N·m (no energy injection) rather than reaching
// the backend. (The same helper is what #184's OSC E-STOP migrates onto.)
inline void GravityCompDampedHold(Eigen::Ref<Eigen::VectorXd> tau_out,
                                  const Eigen::Ref<const Eigen::VectorXd>& gravity,
                                  const Eigen::Ref<const Eigen::VectorXd>& qdot, double damping,
                                  const Eigen::Ref<const Eigen::VectorXd>& tau_max) noexcept {
  for (Eigen::Index i = 0; i < tau_out.size(); ++i) {
    const double lim = std::abs(tau_max(i));
    const double t = gravity(i) - damping * qdot(i);
    if (!std::isfinite(t))
      tau_out(i) = 0.0;  // sensor fault ⇒ cannot form a valid hold; command zero
    else
      tau_out(i) = (t > lim) ? lim : (t < -lim ? -lim : t);
  }
}

}  // namespace rtc::compliance
