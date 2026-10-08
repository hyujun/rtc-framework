#pragma once

// ────────────────────────────────────────────────
// Relative frame Jacobian — the motion of a registered frame `t` as seen from
// another registered frame `b`, in b's axes:
//
//   ω_rel = R_bᵀ·(ω_t − ω_b)                      J_rel,ang = R_bᵀ·(J_t,ang − J_b,ang)
//   v_rel = R_bᵀ·(v_t − v_b − ω_b × (p_t − p_b))  J_rel,lin = R_bᵀ·(J_t,lin − J_b,lin
//                                                              + [p_t − p_b]ₓ·J_b,ang)
//
// with J_t / J_b the frames' LOCAL_WORLD_ALIGNED Jacobians (rf.J), R_b / p_b
// the base frame's world pose and p_t the frame's world origin. v_rel is
// d/dt of the frame origin's coordinates in b (d/dt(R_bᵀ·d) = R_bᵀ·(ḋ − ω_b ×
// d)), so [linear; angular] is the LOCAL_WORLD_ALIGNED twist of t for an
// observer fixed in b — the axes ComputeTaskPoseError uses when it is handed
// the pose of t IN b (se3_error.hpp). A joint upstream of both frames moves
// them rigidly together and its column is zero (to rounding); that is what
// lets a task on (t in b) leave such joints alone.
//
// Header-only inline, RT-safe: fixed-size temporaries only. The columns are
// written one at a time because rf.J is dynamic — a block expression over it
// would allocate.
// ────────────────────────────────────────────────

#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Core>

#include <span>

namespace rtc::tsid {

/// Column `vi` of the relative Jacobian of `rf` with respect to `rb`: `lin` and
/// `ang` in rb's axes. Both frames must be Update()d at the same state.
inline void RelativeFrameJacobianColumn(const PinocchioCache::RegisteredFrame& rf,
                                        const PinocchioCache::RegisteredFrame& rb, Eigen::Index vi,
                                        Eigen::Vector3d& lin, Eigen::Vector3d& ang) noexcept {
  const Eigen::Matrix3d& R_b = rb.oMf.rotation();
  const Eigen::Vector3d d = rf.oMf.translation() - rb.oMf.translation();
  const Eigen::Vector3d jb_ang = rb.J.col(vi).tail<3>();
  const Eigen::Vector3d dlin = rf.J.col(vi).head<3>() - rb.J.col(vi).head<3>() + d.cross(jb_ang);
  const Eigen::Vector3d dang = rf.J.col(vi).tail<3>() - jb_ang;
  lin.noalias() = R_b.transpose() * dlin;
  ang.noalias() = R_b.transpose() * dang;
}

/// The relative Jacobian [6 × nv] on the columns `v_idx`; every other column
/// is zero. `j_rel` must already be 6 × nv (no resize here).
inline void FillRelativeFrameJacobian(const PinocchioCache::RegisteredFrame& rf,
                                      const PinocchioCache::RegisteredFrame& rb,
                                      std::span<const int> v_idx, Eigen::MatrixXd& j_rel) noexcept {
  j_rel.setZero();
  Eigen::Vector3d lin;
  Eigen::Vector3d ang;
  for (const int v : v_idx) {
    const auto vi = static_cast<Eigen::Index>(v);
    RelativeFrameJacobianColumn(rf, rb, vi, lin, ang);
    j_rel.col(vi).head<3>() = lin;
    j_rel.col(vi).tail<3>() = ang;
  }
}

}  // namespace rtc::tsid
