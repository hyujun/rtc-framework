#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-1: Object frame for grasp formulation.
//
// World-frame placement of the object reference point used to express the
// grasp matrix G (6 × n_λ). Convention:
//   - p_w : object origin position in world frame [m]
//   - R_w : object orientation (world ← object) — currently unused by the
//           grasp matrix (G is built with world-aligned contact wrenches
//           translated to the object origin), but reserved for Stage B-2+
//           where object-body wrenches may be expressed in the object's
//           own frame.
//
// POD: trivially copyable, no allocator state, safe to memcpy into a
// SeqLock POD wrapper (cf. feedback_eigen_seqlock_pod_wrapper).
// ────────────────────────────────────────────────
struct ObjectFrame {
  Eigen::Vector3d p_w{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_w{Eigen::Matrix3d::Identity()};
};

// ────────────────────────────────────────────────
// Stage B-1: Adjoint dual map for translating a surface wrench expressed at
// contact point p_ci (world frame) to the object origin p_o (world frame).
//
// Given λ_ci = [f; m] (6×1) at the contact point, the equivalent wrench
// at the object origin is:
//   w_obj = Ad*(p_ci - p_o) · λ_ci
//   where Ad*(d) = [ I_3         0_3 ;
//                   skew(d)      I_3 ]   with d = p_ci - p_o (world frame).
//
// Force passes through unchanged; the moment at p_o picks up the lever-arm
// torque (p_ci - p_o) × f in addition to the original moment m.
//
// `world_aligned`: both source and target wrenches are expressed in a
// world-aligned basis (matches Pinocchio LOCAL_WORLD_ALIGNED contact
// Jacobians used elsewhere in rtc_tsid). The object orientation R_w is
// not consumed here — reserved for future object-body-frame variants.
//
// RT-safe: pure stack work on a fixed 6×6 matrix.
// ────────────────────────────────────────────────
[[nodiscard]] inline Eigen::Matrix<double, 6, 6> AdjointDualWorldAligned(
    const Eigen::Vector3d& p_ci, const Eigen::Vector3d& p_o) noexcept {
  Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Vector3d d = p_ci - p_o;
  // skew(d) in the lower-left 3×3 block.
  Ad(3, 1) = -d.z();
  Ad(3, 2) = d.y();
  Ad(4, 0) = d.z();
  Ad(4, 2) = -d.x();
  Ad(5, 0) = -d.y();
  Ad(5, 1) = d.x();
  return Ad;
}

// ────────────────────────────────────────────────
// Stage B-1: Point-contact contribution to G — force-only mapping. Returns
// the 6×3 block that maps a world-frame contact force at p_ci to the
// object-origin wrench at p_o:
//   [ I_3       ]
//   [ skew(d)   ]   with d = p_ci - p_o.
//
// RT-safe: stack 6×3.
// ────────────────────────────────────────────────
[[nodiscard]] inline Eigen::Matrix<double, 6, 3> PointGraspBlock(
    const Eigen::Vector3d& p_ci, const Eigen::Vector3d& p_o) noexcept {
  Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
  B.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d d = p_ci - p_o;
  // skew(d)
  B(3, 1) = -d.z();
  B(3, 2) = d.y();
  B(4, 0) = d.z();
  B(4, 2) = -d.x();
  B(5, 0) = -d.y();
  B(5, 1) = d.x();
  return B;
}

}  // namespace rtc::tsid
