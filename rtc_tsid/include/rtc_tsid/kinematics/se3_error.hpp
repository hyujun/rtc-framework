#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage C-1: shared SE(3) error convention — single source for SE3Task
// (acceleration-level residual) and ClikReferenceGenerator (velocity-level
// reference), so the A/B command paths measure pose error on an identical
// scale:
//   position error : p_des - p_cur                  (base frame)
//   rotation error : log3(R_curᵀ · R_des)           (separated, base frame)
// with the θ → π log3 singularity clamped to keep the axis direction and a
// finite magnitude. All functions are RT-safe (fixed-size, no heap).
// ────────────────────────────────────────────────

// Margin below π at which log3 is clamped (axis kept, magnitude π - margin).
inline constexpr double kLog3PiMargin = 1e-4;

// log3(R_err) with θ → π singularity protection.
[[nodiscard]] inline Eigen::Vector3d Log3Clamped(const Eigen::Matrix3d& R_err) noexcept {
  const Eigen::AngleAxisd aa(R_err);
  if (aa.angle() < M_PI - kLog3PiMargin) {
    return pinocchio::log3(R_err);
  }
  // π 근처: 방향 유지, 크기 clamp
  return aa.axis() * (M_PI - kLog3PiMargin);
}

// 6D pose error [position; rotation], both poses expressed in the same
// (base) frame. See convention block above.
[[nodiscard]] inline Eigen::Matrix<double, 6, 1> ComputeSe3Error(
    const pinocchio::SE3& pose_cur, const pinocchio::SE3& pose_des) noexcept {
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = pose_des.translation() - pose_cur.translation();
  error.tail<3>() = Log3Clamped(pose_cur.rotation().transpose() * pose_des.rotation());
  return error;
}

}  // namespace rtc::tsid
