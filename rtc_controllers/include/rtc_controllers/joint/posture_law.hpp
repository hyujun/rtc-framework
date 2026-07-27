// ── Null-space posture regulation law (#236 S6) ─────────────────────────────
// The per-channel posture command of a redundancy-resolution secondary task,
// in the two forms the callers actually need:
//
//     torque domain (PD):   τ₀[i] = Kp·(q_ref[i] − q[i]) − Kd·q̇[i]
//     velocity domain (P):  q̇₀[i] = Kp·(q_ref[i] − q[i])
//
// and nothing else. The caller owns the reference (where q_ref comes from — a
// measured seed, a configured safe posture, a mailbox slot — is a BINDING
// decision, see below), the channel-order gather, the null-space projector
// Nᵀ (or N), the activation ramp and the torque assembly.
//
// ── Why q_ref is an argument and not a policy ───────────────────────────────
// The three shipping consumers disagree only on where the reference comes from:
// TaskImpedanceController and CascadedComplianceController seed it from the
// measured q on the activation tick, OperationalSpaceController takes it from
// the configured `safe_position`. That is a difference in BINDING, not in law —
// so the law takes the reference as an argument and stays the single definition
// for all of them. (OSC still runs its own inline copy: migrating it would move
// the Λ/Nᵀ block that #236 S2b has yet to decide a convergence point for, so it
// is deliberately left for that slice.)
//
// ── Why the P form is a separate function and not Kd = 0 ────────────────────
// It is tempting to serve TaskAdmittanceController's P form by calling the PD
// form with Kd = 0. That is NOT bitwise inert. `x − 0.0·q̇` equals `x` for every
// finite non-zero x, but for x = −0.0 and q̇ < 0 it evaluates to +0.0: IEEE 754
// gives 0.0·q̇ = −0.0 there, and (−0.0) − (−0.0) = +0.0. The sign of a zero is
// not academic here — x = Kp·(q_ref − q) is EXACTLY −0.0 on a reachable and in
// fact ordinary state: q_ref is seeded from the measurement, so q_ref − q is
// exactly +0.0 while the arm holds still, and a negative Kp (which
// TaskAdmittanceController::LoadConfig and TaskImpedanceController::LoadConfig
// both accept — neither floors nullspace_kp, and set_gains() bypasses every
// floor) turns that into −0.0 on every channel. A pre-extraction probe measured
// the flip on 50% of draws in that state at -O0/-O2/-O3. Two shapes, two
// functions, two literal oracles — the same call this repo made for
// task/task_vel_law.hpp's six-axis and translation-only forms.
//
// ── Extraction note (#236 S6) ───────────────────────────────────────────────
// These expressions lived inline in TaskImpedanceController::Compute()
// (8526a4c9, task_impedance_controller.cpp:472-474),
// CascadedComplianceController::Compute() (:540-542 — character-for-character
// the same loop, which is the ARCH-3 trigger this slice exists to resolve) and
// TaskAdmittanceController::Compute() (:386-388, the P form). They were lifted
// here verbatim — same operation order, same association, same sign placement —
// so the migration is bit-for-bit inert; PostureLaw.MatchesThePreExtraction*
// pin that against literal copies of the pre-extraction loops, and the
// CoreDrivenShim* suites pin it against the live adapters while they exist.
// Preserve the association if you touch it: `kp·(q_ref − q) − kd·q̇` is not
// bitwise `kp·q_ref − kp·q − kd·q̇`.
#pragma once

#include <algorithm>
#include <cstddef>
#include <span>

namespace rtc::joint {

/// The per-tick posture inputs, all in ONE channel order — whichever order the
/// caller intends to project in. Grouped in a struct rather than passed
/// positionally because three same-typed spans in a row is a swap waiting to
/// happen (the same call joint/joint_pd_law.hpp made).
///
/// Channel order is the caller's business and the law never reorders: both
/// shipping torque callers form the error in DEVICE order and gather to
/// Pinocchio order afterwards, because the projector is Pinocchio-ordered
/// (#172 A2). Mixing orders inside these three spans is a caller bug the law
/// cannot see.
struct PostureInputs {
  std::span<const double> q_ref;  ///< posture reference [rad]
  std::span<const double> q;      ///< measured joint positions [rad]
  std::span<const double> qdot;   ///< measured joint velocities [rad/s]; UNREAD by the P form
};

namespace detail {

/// Channels the law may write: every span it will READ participates, so a short
/// or empty input degrades to the tail policy instead of reading out of range.
/// On a correctly sized caller (all spans nv long) this is a no-op.
[[nodiscard]] inline std::size_t PostureBound(const PostureInputs& in, std::size_t n,
                                              std::size_t out_size, bool reads_qdot) noexcept {
  std::size_t bound = std::min({n, out_size, in.q_ref.size(), in.q.size()});
  if (reads_qdot)
    bound = std::min(bound, in.qdot.size());
  return bound;
}

}  // namespace detail

/// τ₀ = Kp·(q_ref − q) − Kd·q̇ on channels [0, n), TORQUE domain.
///
/// @param kp   posture centering stiffness [N·m/rad]
/// @param kd   posture damping [N·m·s/rad]
/// @param in   reference, measured position, measured velocity
/// @param n    channels to write — the caller's joint count (nv)
/// @param tau  OUT, written on [0, n); see the tail policy below
///
/// Channels past the bound are ZEROED rather than left alone. The buffer is
/// about to be gathered and projected, so a stale value there is a torque the
/// projector will happily inject; a fresh zero is the passive answer. On a
/// correctly sized caller nothing past `n` is touched because the bound is `n`.
///
/// The caller must still gate on redundancy: for nv ≤ 6 task DOF the projector
/// is not a null-space projector at all (Nᵀ ≈ 0 at nv = 6, and rank-deficient
/// below it, where the posture torque leaks into the primary task — issue #172).
/// That gate is a binding decision about the arm, not part of the law.
///
/// RT-safe: noexcept, no heap, fixed loop, no divide — nothing here for NUM-2
/// to guard.
inline void ComputePostureTorque(double kp, double kd, const PostureInputs& in, std::size_t n,
                                 std::span<double> tau) noexcept {
  const std::size_t bound = detail::PostureBound(in, n, tau.size(), /*reads_qdot=*/true);
  for (std::size_t i = 0; i < bound; ++i)
    tau[i] = kp * (in.q_ref[i] - in.q[i]) - kd * in.qdot[i];
  for (std::size_t i = bound; i < tau.size(); ++i)
    tau[i] = 0.0;
}

/// q̇₀ = Kp·(q_ref − q) on channels [0, n), VELOCITY domain.
///
/// The proportional form of the law above, NOT the Kd = 0 special case of it —
/// see the header note; the reduction is not bitwise inert. @p in.qdot is not
/// read.
///
/// @param kp    posture centering gain [1/s]
/// @param in    reference and measured position (`qdot` unused)
/// @param n     channels to write — the caller's joint count (nv)
/// @param qdot_cmd  OUT, written on [0, n); same tail policy as above, and here
///                  a stale value would be a commanded VELOCITY
///
/// RT-safe: noexcept, no heap, fixed loop, no divide.
inline void ComputePostureVelocity(double kp, const PostureInputs& in, std::size_t n,
                                   std::span<double> qdot_cmd) noexcept {
  const std::size_t bound = detail::PostureBound(in, n, qdot_cmd.size(), /*reads_qdot=*/false);
  for (std::size_t i = 0; i < bound; ++i)
    qdot_cmd[i] = kp * (in.q_ref[i] - in.q[i]);
  for (std::size_t i = bound; i < qdot_cmd.size(); ++i)
    qdot_cmd[i] = 0.0;
}

}  // namespace rtc::joint
