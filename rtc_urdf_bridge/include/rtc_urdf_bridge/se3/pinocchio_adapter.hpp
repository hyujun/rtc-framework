// ── Pinocchio adapter for the SE(3) error module ─────────────────────────────
// Thin overloads that accept the workspace-native pinocchio::SE3 / Motion types
// and forward to the Eigen-only core. The numeric core (so3/se3/pose_error/
// velocity_error) stays Pinocchio-free; this header is the ONLY one that pulls
// in Pinocchio, so consumers that already depend on it (all rtc controllers)
// get ergonomic overloads while the core remains independently testable.
//
// Ordering note: pinocchio::Motion::toVector() is [linear; angular] — identical
// to the core's Vec6 convention, so no reordering is needed.
#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include "rtc_urdf_bridge/se3/pose_error.hpp"
#include "rtc_urdf_bridge/se3/velocity_error.hpp"

namespace rtc_urdf_bridge::se3 {

// ── Conversions (RT-safe, fixed size) ────────────────────────────────────────
[[nodiscard]] inline Iso3 toIso3(const pinocchio::SE3& M) noexcept {
  Iso3 T = Iso3::Identity();
  T.linear() = M.rotation();
  T.translation() = M.translation();
  return T;
}

[[nodiscard]] inline pinocchio::SE3 toSE3(const Iso3& T) noexcept {
  return pinocchio::SE3(T.linear(), T.translation());
}

// pinocchio::Motion ↔ Vec6 ([linear; angular]).
[[nodiscard]] inline Vec6 toVec6(const pinocchio::Motion& m) noexcept {
  return m.toVector();
}

[[nodiscard]] inline pinocchio::Motion toMotion(const Vec6& v) noexcept {
  return pinocchio::Motion(v.template head<3>(), v.template tail<3>());
}

// ── Pinocchio-typed overloads (return Vec6 for direct gain multiplication) ───
[[nodiscard]] inline Vec6 computePoseError(const pinocchio::SE3& T, const pinocchio::SE3& T_d,
                                           ErrorType type) noexcept {
  return computePoseError(toIso3(T), toIso3(T_d), type);
}

[[nodiscard]] inline Vec6 computeVelocityError(const pinocchio::SE3& T, const pinocchio::SE3& T_d,
                                               const pinocchio::Motion& nu,
                                               const pinocchio::Motion& nu_d,
                                               ErrorType type) noexcept {
  return computeVelocityError(toIso3(T), toIso3(T_d), toVec6(nu), toVec6(nu_d), type);
}

[[nodiscard]] inline Vec6 exactPoseErrorRate(const pinocchio::SE3& T, const pinocchio::SE3& T_d,
                                             const pinocchio::Motion& nu,
                                             const pinocchio::Motion& nu_d,
                                             ErrorType type) noexcept {
  return exactPoseErrorRate(toIso3(T), toIso3(T_d), toVec6(nu), toVec6(nu_d), type);
}

}  // namespace rtc_urdf_bridge::se3
