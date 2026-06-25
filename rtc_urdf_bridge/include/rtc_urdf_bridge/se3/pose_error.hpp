// ── SE(3) pose-error definitions (Eigen-only, RT-safe) ───────────────────────
// Five literature pose-error definitions + the workspace's legacy 6th
// (SplitBodyRot). Pick a definition with ErrorType; the matching velocity error
// lives in velocity_error.hpp (each pose error has exactly one consistent twist
// error via the transport map — Bullo & Murray 1999).
//
// Convention: T, T_d are world←frame poses (Eigen::Isometry3d). 6D output is
// [linear(3); angular(3)] (pinocchio::Motion ordering). Input R assumed valid
// SO(3). All noexcept, fixed-size, no heap — RT-safe.
#pragma once

#include "rtc_urdf_bridge/se3/se3.hpp"

namespace rtc_urdf_bridge::se3 {

/// Pose-error definition. Each row lists frame, rotation scale (pure rotation
/// of angle θ), and behaviour at θ=π. See computePoseError() doxygen per case.
enum class ErrorType {
  /// [p_d−p ; log3(R_d Rᵀ)] — world-frame separated error. Rotation scale θ
  /// (linear in angle). Source: Caccavale et al. 1999, Siciliano et al. 2009.
  /// θ=π: finite (quat+atan2 log3), axis direction defined up to sign.
  SplitWorld,
  /// log6(T⁻¹ T_d) — body screw (single Lie-group error). Source: MLS 1994,
  /// Modern Robotics 2017 §11.3. Scale θ. Couples translation+rotation (screw).
  BodyLog6,
  /// log6(T_d T⁻¹) — spatial screw. e_spatial = Ad_T · e_body (test 5).
  SpatialLog6,
  /// [p_d−p ; ½(R_dRᵀ − RR_dᵀ)∨] — world frame, rotation scale sinθ.
  /// Source: Lee et al. 2010 (SE(3) geometric tracking). θ→π: sinθ→0 (extra
  /// critical point — early-convergence stall, see comparison experiment S2).
  SplitLee,
  /// [p_d−p ; 2·sign(w)·vec(q_d ⊗ q⁻¹)] — world frame, rotation scale 2sin(θ/2).
  /// Source: Nakanishi et al. 2008. sign(w) ⇒ shortest path (no unwinding).
  SplitQuat,
  /// [p_d−p ; log3(Rᵀ R_d)] — workspace LEGACY default: base-frame position +
  /// CURRENT-BODY-frame rotation (mixed). Numerically equals the historical
  /// rtc::tsid::ComputeSe3Error. Scale θ. Documented 6th type for callsite
  /// migration; prefer SplitWorld/BodyLog6 for new code.
  SplitBodyRot,
};

/// True iff the twist error for `t` is expressed fully in the body (LOCAL)
/// frame — only BodyLog6. SplitBodyRot is mixed (world position + body
/// rotation) and is intentionally NOT body-frame here.
[[nodiscard]] constexpr bool isBodyFrame(ErrorType t) noexcept {
  return t == ErrorType::BodyLog6;
}

/// 6D pose error per `type`. [linear; angular] ordering.
[[nodiscard]] inline Vec6 computePoseError(const Iso3& T, const Iso3& T_d,
                                           ErrorType type) noexcept {
  const Mat3 R = T.linear();
  const Mat3 R_d = T_d.linear();
  Vec6 e;
  switch (type) {
    case ErrorType::BodyLog6:
      return log6(T.inverse() * T_d);
    case ErrorType::SpatialLog6:
      return log6(T_d * T.inverse());
    case ErrorType::SplitWorld:
      e.head<3>() = T_d.translation() - T.translation();
      e.tail<3>() = log3(R_d * R.transpose());  // world-frame, scale θ
      return e;
    case ErrorType::SplitLee:
      e.head<3>() = T_d.translation() - T.translation();
      // axial part of E = R_d Rᵀ equals ½(E−Eᵀ)∨ = sinθ·n.
      e.tail<3>() = vee(R_d * R.transpose());
      return e;
    case ErrorType::SplitQuat: {
      e.head<3>() = T_d.translation() - T.translation();
      Eigen::Quaterniond q_err(R_d * R.transpose());  // q_d ⊗ q⁻¹ (world frame)
      q_err.normalize();
      const double s = q_err.w() < 0.0 ? -1.0 : 1.0;  // sign(w): shortest path
      e.tail<3>() = 2.0 * s * q_err.vec();            // scale 2 sin(θ/2)
      return e;
    }
    case ErrorType::SplitBodyRot:
      e.head<3>() = T_d.translation() - T.translation();
      e.tail<3>() = log3(R.transpose() * R_d);  // CURRENT-body frame, scale θ
      return e;
  }
  return Vec6::Zero();  // unreachable (all enum cases return)
}

}  // namespace rtc_urdf_bridge::se3
