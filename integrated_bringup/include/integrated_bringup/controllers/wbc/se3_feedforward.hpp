#pragma once

/// @file se3_feedforward.hpp
/// @brief Body-frame trajectory feed-forward → the axes SE3Task reads.
///
/// rtc_controllers' TaskSpaceTrajectory reports its velocity and acceleration
/// as LOCAL spatial motions of the trajectory pose (body twist [v_b; ω_b] and
/// its time derivative). SE3Task::SetSe3Reference takes them aligned with its
/// base frame at the frame origin (LOCAL_WORLD_ALIGNED of the pose in the
/// base), and its acceleration is the CLASSICAL one — it is compared against
/// J·a + J̇·v with pinocchio's classical drift. For a pose (R, p) in the base:
///
///   ṗ  = R·v_b                     ω  = R·ω_b
///   p̈  = R·(v̇_b + ω_b × v_b)       ω̇  = R·ω̇_b
///
/// (d/dt(R·ω_b) = R·ω̇_b because ω_b × ω_b = 0.) Header-only, noexcept,
/// fixed-size — RT-safe.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup::wbc {

/// Origin velocity / classical acceleration, [linear; angular], in the axes
/// of the frame the pose is expressed in.
struct AlignedFeedForward {
  Eigen::Matrix<double, 6, 1> velocity{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> acceleration{Eigen::Matrix<double, 6, 1>::Zero()};
};

/// `pose` is the trajectory pose in the base; `v_local` / `a_local` are its
/// LOCAL twist and LOCAL spatial acceleration (TaskSpaceTrajectory::State).
[[nodiscard]] inline AlignedFeedForward LocalToAlignedFeedForward(
    const pinocchio::SE3& pose, const pinocchio::Motion& v_local,
    const pinocchio::Motion& a_local) noexcept {
  const Eigen::Matrix3d& R = pose.rotation();
  const Eigen::Vector3d v_b = v_local.linear();
  const Eigen::Vector3d w_b = v_local.angular();
  const Eigen::Vector3d coriolis = w_b.cross(v_b);
  AlignedFeedForward out;
  out.velocity.head<3>().noalias() = R * v_b;
  out.velocity.tail<3>().noalias() = R * w_b;
  out.acceleration.head<3>().noalias() = R * (a_local.linear() + coriolis);
  out.acceleration.tail<3>().noalias() = R * a_local.angular();
  return out;
}

}  // namespace integrated_bringup::wbc
