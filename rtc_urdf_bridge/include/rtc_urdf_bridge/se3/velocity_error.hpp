// ── SE(3) velocity (twist) error (Eigen-only, RT-safe) ───────────────────────
// The twist ν lives in the tangent space at T and ν_d at T_d; the naive
// difference ν_d − ν is only valid near zero error. For large error the
// adjoint transport map (Bullo & Murray 1999) carries one twist into the
// other's frame. Each ErrorType has exactly one consistent velocity error.
//
// KEY IDENTITY (test 8/9): ad_ξ ξ = 0 ⇒ every J(ξ) acts as identity along ξ, so
// a SCALAR gain k with correct Ad transport already gives ė = −k·e exactly at
// large error (Modern Robotics §11.3). Jlog corrections matter only for
// ANISOTROPIC gains.
//
// Twist ordering [linear; angular] (pinocchio::Motion). All noexcept / fixed
// size / no heap — RT-safe.
#pragma once

#include "rtc_urdf_bridge/se3/pose_error.hpp"
#include "rtc_urdf_bridge/se3/se3.hpp"

namespace rtc_urdf_bridge::se3 {

/// Frame convention of an input twist (mirrors pinocchio::ReferenceFrame).
enum class TwistFrame {
  Local,             ///< body frame (pinocchio LOCAL)
  LocalWorldAligned  ///< world-axis-aligned at the body origin (pinocchio LOCAL_WORLD_ALIGNED)
};

namespace detail {
// blockdiag(R, R): rotates a [linear; angular] twist's axes only (no lever-arm
// coupling). This is the LWA↔LOCAL relation — NOT the full adjoint.
[[nodiscard]] inline Mat6 blockDiagR(const Mat3& R) noexcept {
  Mat6 M;
  M.setZero();
  M.block<3, 3>(0, 0) = R;
  M.block<3, 3>(3, 3) = R;
  return M;
}
}  // namespace detail

// ── Frame transport helpers ──────────────────────────────────────────────────
//
// LOCAL_WORLD_ALIGNED ↔ LOCAL: pure axis rotation blockdiag(R, R). Both frames
// share the body origin, so the lever-arm (translation) coupling is absent —
// using the full adjoint here is a common bug.
[[nodiscard]] inline Vec6 twistWorldToLocal(const Iso3& T, const Vec6& nu_lwa) noexcept {
  return detail::blockDiagR(T.linear().transpose()) * nu_lwa;
}

[[nodiscard]] inline Vec6 twistLocalToWorld(const Iso3& T, const Vec6& nu_local) noexcept {
  return detail::blockDiagR(T.linear()) * nu_local;
}

// LOCAL ↔ WORLD (spatial): the FULL adjoint (includes the lever arm). Provided
// separately so callers don't confuse LWA (above) with true spatial twists.
[[nodiscard]] inline Vec6 twistLocalToSpatial(const Iso3& T, const Vec6& nu_local) noexcept {
  return adjoint(T) * nu_local;
}

[[nodiscard]] inline Vec6 twistSpatialToLocal(const Iso3& T, const Vec6& nu_spatial) noexcept {
  return adjoint(T.inverse()) * nu_spatial;
}

// ── computeVelocityError: type-consistent velocity error (control use) ───────
//
// Frame premises (apply a transport helper first if your input differs):
//   SplitWorld / SplitLee / SplitQuat / SplitBodyRot → ν, ν_d in LOCAL_WORLD_ALIGNED
//   BodyLog6                                          → ν, ν_d in LOCAL (each own frame)
//   SpatialLog6                                       → ν, ν_d spatial (WORLD)
//
// Split returns [v_d−v ; ω_d−ω]: linear part EXACT (ė_p = v_d−v); rotation part
// ω_d−ω is FIRST ORDER (exact rate via exactPoseErrorRate). The angle-axis
// stiffness + ω_d−ω damping pairing is the Caccavale 1999 combination, proven
// stable. BodyLog6/SpatialLog6 use exact Ad transport.
[[nodiscard]] inline Vec6 computeVelocityError(const Iso3& T, const Iso3& T_d, const Vec6& nu,
                                               const Vec6& nu_d, ErrorType type) noexcept {
  switch (type) {
    case ErrorType::BodyLog6:
      // e_ν^cur = Ad_{T⁻¹T_d}·ν_d − ν  (current body frame).
      // Pairs with the Modern Robotics law V* = Ad_{T⁻¹T_d}ν_d + K_p·e.
      return adjoint(T.inverse() * T_d) * nu_d - nu;
    case ErrorType::SpatialLog6:
      // e_ν^s = ν_d^s − Ad_{T_d T⁻¹}·ν^s.
      return nu_d - adjoint(T_d * T.inverse()) * nu;
    case ErrorType::SplitWorld:
    case ErrorType::SplitLee:
    case ErrorType::SplitQuat:
    case ErrorType::SplitBodyRot:
      // World/LWA component-wise; rotation part first order (see above).
      return nu_d - nu;
  }
  return Vec6::Zero();  // unreachable
}

// ── exactPoseErrorRate: exact ė (validation / anisotropic-gain compensation) ─
//
// Returns the EXACT time derivative of computePoseError along the motion, with
// ν, ν_d given as LOCAL (body) twists for ALL types — i.e. T evolves as
// Ṫ = T·[ν]∧ (right multiplication). This matches the finite-difference test
// (test 7), which integrates T ← T·exp6(ν dt). Uses the right (local)
// trivialization paired with Jlog6 (BodyLog6) and the left trivialization with
// J_l⁻¹ (Split rotation parts).
[[nodiscard]] inline Vec6 exactPoseErrorRate(const Iso3& T, const Iso3& T_d, const Vec6& nu,
                                             const Vec6& nu_d, ErrorType type) noexcept {
  const Mat3 R = T.linear();
  const Mat3 R_d = T_d.linear();
  const Vec3 v = nu.head<3>();
  const Vec3 w = nu.tail<3>();
  const Vec3 v_d = nu_d.head<3>();
  const Vec3 w_d = nu_d.tail<3>();

  switch (type) {
    case ErrorType::BodyLog6: {
      // ė = Jlog6(F)·e_ν^des,  e_ν^des = ν_d − Ad_{F⁻¹}·ν,  F = T⁻¹T_d.
      const Iso3 F = T.inverse() * T_d;
      return Jlog6(F) * (nu_d - adjoint(F.inverse()) * nu);
    }
    case ErrorType::SpatialLog6: {
      // G = T_d T⁻¹.  (G⁻¹Ġ)∨ = Ad_T(ν_d − ν).  ė = Jlog6(G)·Ad_T·(ν_d − ν).
      const Iso3 G = T_d * T.inverse();
      return Jlog6(G) * (adjoint(T) * (nu_d - nu));
    }
    case ErrorType::SplitWorld: {
      // ė_p = R_d v_d − R v; ė_R = J_l⁻¹(φ)·η, η = R_d ω_d − R_d ω, φ=log3(R_dRᵀ).
      Vec6 e;
      e.head<3>() = R_d * v_d - R * v;
      const Vec3 phi = log3(R_d * R.transpose());
      const Vec3 eta = R_d * w_d - R_d * w;
      e.tail<3>() = leftJacobianInverse(phi) * eta;
      return e;
    }
    case ErrorType::SplitLee: {
      // e_R = vee(E), E = R_d Rᵀ ⇒ ė_R = vee(Ė), Ė = R_d([ω_d]×−[ω]×)Rᵀ.
      Vec6 e;
      e.head<3>() = R_d * v_d - R * v;
      const Mat3 Edot = R_d * (hat(w_d) - hat(w)) * R.transpose();
      e.tail<3>() = vee(Edot);
      return e;
    }
    case ErrorType::SplitQuat: {
      // e_R = 2v_q (q_err=(w_q,v_q) of E). ė_R = w_q·η + η×v_q, η = R_d ω_d − R_d ω.
      Vec6 e;
      e.head<3>() = R_d * v_d - R * v;
      Eigen::Quaterniond q_err(R_d * R.transpose());
      q_err.normalize();
      const double s = q_err.w() < 0.0 ? -1.0 : 1.0;
      const double wq = s * q_err.w();
      const Vec3 vq = s * q_err.vec();
      const Vec3 eta = R_d * w_d - R_d * w;
      e.tail<3>() = wq * eta + eta.cross(vq);
      return e;
    }
    case ErrorType::SplitBodyRot: {
      // E_b = Rᵀ R_d. ė_R = Jlog3(E_b)·(ω_d − E_bᵀ ω). ė_p = R_d v_d − R v.
      Vec6 e;
      e.head<3>() = R_d * v_d - R * v;
      const Mat3 Eb = R.transpose() * R_d;
      e.tail<3>() = Jlog3(Eb) * (w_d - Eb.transpose() * w);
      return e;
    }
  }
  return Vec6::Zero();  // unreachable
}

}  // namespace rtc_urdf_bridge::se3
