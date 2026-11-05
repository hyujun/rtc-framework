#ifndef INTEGRATED_BRINGUP_BACKENDS_FLOATING_BASE_CONVENTIONS_H_
#define INTEGRATED_BRINGUP_BACKENDS_FLOATING_BASE_CONVENTIONS_H_

#include "rtc_base/types/types.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cmath>

namespace rtc {

// ── MuJoCo ↔ pinocchio floating-base conventions ─────────────────────────────
//
// The ONE place the two conventions meet (W0-F02, #976). Everything the
// floating-base lane of DeviceState carries is in pinocchio's free-flyer
// convention; everything a MuJoCo ground-truth sensor publishes is in
// MuJoCo's. The two differ in exactly two places, and both differences are
// silent when wrong — a swapped quaternion order is still a unit quaternion,
// a world-frame twist is still a finite vector:
//
//   quaternion : MuJoCo `framequat` (and `qpos` of a free joint) is (w, x, y, z);
//                pinocchio's configuration tail is (x, y, z, w).
//   twist      : MuJoCo `framelinvel` / `frameangvel` are BOTH expressed in the
//                world frame (MuJoCo docs, "Sensor": "in global coordinates").
//                DeviceState::base_twist is (v, ω) in the BASE frame, the
//                pinocchio LOCAL convention, so v_b = Rᵀ v_w and ω_b = Rᵀ ω_w
//                with R the base orientation.
//
// Note that MuJoCo's free-joint *qvel* is a third convention (linear in world,
// angular in body); it is not what the frame sensors publish and is not
// handled here — a producer reading qvel directly would need its own path.
//
// The four frame sensors must be attached to the base BODY FRAME:
// `objtype="xbody" objname="<base>"`. In MuJoCo `objtype="body"` is the
// body's INERTIAL frame (xipos / ximat, i.e. the CoM), not the body frame
// pinocchio's free-flyer joint sits at — with it, framepos carries the CoM
// offset and framelinvel the ω × r term of that offset (checked on MuJoCo
// 3.7.0: inertial pos 0.3 m, ω_z = 1 rad/s → framelinvel (0, 0.3, 0) under
// "body", 0 under "xbody"). `xbody` gives xpos / xquat, whose origin velocity
// is pinocchio's LOCAL linear velocity. This header applies no ω × r
// correction, so a sensor on a site or on the inertial frame is wrong here.
//
// Allocation-free, fixed-size Eigen only: the producer calls this on the
// rt_callback lane (RT-1). `Eigen::Quaterniond` is Hamilton, the repo's
// internal rotation type (AGENTS.md §10).

/// One base sample in DeviceState's convention: `pose` = (p_xyz, q_xyzw),
/// `twist` = (v, ω) in the base frame.
struct FloatingBaseSample {
  std::array<double, kBasePoseDim> pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, kBaseTwistDim> twist{};
};

/// @brief Unit-quaternion guard (NUM-2): a zero or non-finite quaternion is not
///        a rotation, and normalising one would invent a pose.
/// @param q_wxyz MuJoCo-order quaternion.
/// @return true when every component is finite and the norm is within 1e-3 of 1.
[[nodiscard]] inline bool IsUnitQuaternionWxyz(const std::array<double, 4>& q_wxyz) noexcept {
  double n2 = 0.0;
  for (double c : q_wxyz) {
    if (!std::isfinite(c)) {
      return false;
    }
    n2 += c * c;
  }
  return std::abs(n2 - 1.0) < 1e-3;
}

/// @brief MuJoCo frame sensors → pinocchio free-flyer sample.
/// @param pos_w     `framepos`, [m], world.
/// @param quat_wxyz `framequat`, (w, x, y, z), unit — caller guards with
///                  IsUnitQuaternionWxyz.
/// @param linvel_w  `framelinvel`, [m/s], world.
/// @param angvel_w  `frameangvel`, [rad/s], world.
/// @return pose = (pos_w, q_xyzw); twist = (Rᵀ linvel_w, Rᵀ angvel_w).
[[nodiscard]] inline FloatingBaseSample MujocoFrameToPinocchio(
    const std::array<double, 3>& pos_w, const std::array<double, 4>& quat_wxyz,
    const std::array<double, 3>& linvel_w, const std::array<double, 3>& angvel_w) noexcept {
  FloatingBaseSample s;
  s.pose[0] = pos_w[0];
  s.pose[1] = pos_w[1];
  s.pose[2] = pos_w[2];

  // Eigen's constructor takes (w, x, y, z) — the MuJoCo order — so no
  // reshuffle is needed here; the reshuffle into the pose array is below.
  // Renormalised once: the guard admits |‖q‖² − 1| < 1e-3, and both the pose
  // handed to pinocchio and the Rᵀ below assume exactly unit length. Cheap
  // (one sqrt) and a no-op to round-off on a sensor that is already unit.
  Eigen::Quaterniond q(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
  q.normalize();
  s.pose[3] = q.x();
  s.pose[4] = q.y();
  s.pose[5] = q.z();
  s.pose[6] = q.w();  // w last in pinocchio

  const Eigen::Vector3d v_w(linvel_w[0], linvel_w[1], linvel_w[2]);
  const Eigen::Vector3d w_w(angvel_w[0], angvel_w[1], angvel_w[2]);
  // Rᵀ x = q⁻¹ x q; for a unit quaternion the conjugate is the inverse and
  // costs no division.
  const Eigen::Vector3d v_b = q.conjugate() * v_w;
  const Eigen::Vector3d w_b = q.conjugate() * w_w;
  s.twist[0] = v_b.x();
  s.twist[1] = v_b.y();
  s.twist[2] = v_b.z();
  s.twist[3] = w_b.x();
  s.twist[4] = w_b.y();
  s.twist[5] = w_b.z();
  return s;
}

/// @brief Inverse of MujocoFrameToPinocchio — pinocchio sample → MuJoCo frame
///        quantities. Exists for the round-trip test and for a consumer that
///        has to hand a pose back to MuJoCo (a keyframe writer, a reset).
/// @param sample     pose (p_xyz, q_xyzw), twist (v, ω) in the base frame.
/// @param pos_w      out: `framepos`.
/// @param quat_wxyz  out: `framequat` order.
/// @param linvel_w   out: world linear velocity, R v_b.
/// @param angvel_w   out: world angular velocity, R ω_b.
inline void PinocchioToMujocoFrame(const FloatingBaseSample& sample, std::array<double, 3>& pos_w,
                                   std::array<double, 4>& quat_wxyz,
                                   std::array<double, 3>& linvel_w,
                                   std::array<double, 3>& angvel_w) noexcept {
  pos_w[0] = sample.pose[0];
  pos_w[1] = sample.pose[1];
  pos_w[2] = sample.pose[2];
  quat_wxyz[0] = sample.pose[6];  // w
  quat_wxyz[1] = sample.pose[3];  // x
  quat_wxyz[2] = sample.pose[4];  // y
  quat_wxyz[3] = sample.pose[5];  // z

  const Eigen::Quaterniond q(sample.pose[6], sample.pose[3], sample.pose[4], sample.pose[5]);
  const Eigen::Vector3d v_b(sample.twist[0], sample.twist[1], sample.twist[2]);
  const Eigen::Vector3d w_b(sample.twist[3], sample.twist[4], sample.twist[5]);
  const Eigen::Vector3d v_w = q * v_b;
  const Eigen::Vector3d w_w = q * w_b;
  linvel_w[0] = v_w.x();
  linvel_w[1] = v_w.y();
  linvel_w[2] = v_w.z();
  angvel_w[0] = w_w.x();
  angvel_w[1] = w_w.y();
  angvel_w[2] = w_w.z();
}

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_FLOATING_BASE_CONVENTIONS_H_
