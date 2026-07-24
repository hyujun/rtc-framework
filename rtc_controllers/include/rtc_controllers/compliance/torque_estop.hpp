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
inline void GravityCompDampedHold(Eigen::Ref<Eigen::VectorXd> tau_out,
                                  const Eigen::Ref<const Eigen::VectorXd>& gravity,
                                  const Eigen::Ref<const Eigen::VectorXd>& qdot, double damping,
                                  const Eigen::Ref<const Eigen::VectorXd>& tau_max) noexcept {
  for (Eigen::Index i = 0; i < tau_out.size(); ++i) {
    const double lim = std::abs(tau_max(i));
    const double t = gravity(i) - damping * qdot(i);
    tau_out(i) = (t > lim) ? lim : (t < -lim ? -lim : t);
  }
}

}  // namespace rtc::compliance
