#pragma once

// ────────────────────────────────────────────────
// Shared SE(3) task-error convention — single source of truth for the two WBC
// command paths so they measure pose/velocity error on an IDENTICAL scale:
//   - Dynamics WBC : SE3Task / ObjectSE3Task   (acceleration-level TSID QP)
//   - Kinematics WBC: ClikReferenceGenerator    (velocity-level CLIK)
//
// Unification level U1:
//   pose error  → LWA-aligned BodyLog6  (both paths, this file)
//   velocity err→ exactPoseErrorRate (Jlog6 forward), dynamics path only
//   (kinematics keeps its first-order r_task = Kx⊙e_x velocity law)
//
// Frame contract — every error returned here is expressed in the
// LOCAL_WORLD_ALIGNED (LWA) frame, matching the registered-frame Jacobian
// `rf.J` (pinocchio::getFrameJacobian(..., LOCAL_WORLD_ALIGNED, ...)). rtc_math
// returns BodyLog6 errors in the LOCAL (body) frame, so we transport them with
// twistLocalToWorld (blockdiag(R,R) — rotation only, NOT the full adjoint).
//
// All functions are header-only inline, RT-safe (fixed-size Eigen, noexcept,
// zero heap), and delegate the numeric core to rtc_math (rtc::math::se3).
// ────────────────────────────────────────────────

#include "rtc_math/se3/pinocchio_adapter.hpp"

namespace rtc::tsid {

/// 6D task pose error in the LWA frame: twistLocalToWorld(log6(T⁻¹ T_d)).
/// `T` = current frame pose, `T_d` = desired frame pose (same reference frame).
/// [linear(3); angular(3)] ordering (rtc_math / pinocchio::Motion). For zero
/// rotation error the screw coupling J_l⁻¹ vanishes and the linear part reduces
/// to the plain position difference p_d − p (in the LWA frame).
[[nodiscard]] inline rtc::math::se3::Vec6 ComputeTaskPoseError(const pinocchio::SE3& T,
                                                               const pinocchio::SE3& T_d) noexcept {
  namespace se3 = rtc::math::se3;
  // Convert T once (the SE3 overload of computePoseError would re-convert it).
  const se3::Iso3 T_iso = se3::toIso3(T);
  return se3::twistLocalToWorld(
      T_iso, se3::computePoseError(T_iso, se3::toIso3(T_d), se3::ErrorType::BodyLog6));
}

/// 6D task velocity error in the LWA frame = exact time-derivative of
/// ComputeTaskPoseError (the BodyLog6 error rate), transported to LWA.
///
/// `nu_lwa`   : current frame spatial twist (J·v) in LWA.
/// `nu_d_lwa` : desired frame spatial twist (feedforward v_des) in LWA.
///
/// CAUTION: BOTH inputs MUST be LOCAL_WORLD_ALIGNED twists (world-axis-aligned at
/// the body origin), NOT LOCAL/body twists — they are rotated into the body
/// frame internally (nu via T, nu_d via T_d). Passing an already-LOCAL twist
/// double-rotates the linear part and corrupts the damping term.
///
/// rtc_math's exactPoseErrorRate expects LOCAL (body) twists, so each input is
/// rotated into its own body frame first (nu via T, nu_d via T_d), and the
/// resulting LOCAL ė is transported back to LWA. At small error this reduces to
/// the first-order twist error v_des − J·v (the quantity it replaces); the
/// Jlog6 factor only matters at large pose error.
///
/// NOTE (U1 / standard approximation): the LWA transport uses the instantaneous
/// rotation R, dropping the blockdiag(Ṙ,Ṙ)·log6 frame-rotation coupling term
/// (the SE(3) analogue of the missing J̇log6 acceleration term). This is the
/// conventional TSID approximation and is exact at the velocity level for the
/// LOCAL error; see rtc_math se3/README "Limits".
[[nodiscard]] inline rtc::math::se3::Vec6 ComputeTaskVelocityError(
    const pinocchio::SE3& T, const pinocchio::SE3& T_d, const rtc::math::se3::Vec6& nu_lwa,
    const rtc::math::se3::Vec6& nu_d_lwa) noexcept {
  namespace se3 = rtc::math::se3;
  const se3::Iso3 T_iso = se3::toIso3(T);
  const se3::Iso3 Td_iso = se3::toIso3(T_d);
  const se3::Vec6 nu_local = se3::twistWorldToLocal(T_iso, nu_lwa);
  const se3::Vec6 nu_d_local = se3::twistWorldToLocal(Td_iso, nu_d_lwa);
  const se3::Vec6 edot_local =
      se3::exactPoseErrorRate(T_iso, Td_iso, nu_local, nu_d_local, se3::ErrorType::BodyLog6);
  return se3::twistLocalToWorld(T_iso, edot_local);
}

}  // namespace rtc::tsid
