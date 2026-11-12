// Implementation choices only — the contract (layouts, formulas, the weight
// approximation) is the header's.
//
// - Every base quantity is computed into a fixed-size local before the first
//   output write, so a rejected call writes nothing and `v_o` / `v` (or `w_o` /
//   `w`) may share one buffer: the joint tails sit at the same offset in both
//   and copy onto themselves.
// - The Euler-rate matrix and the R → rpy map are pinocchio's (design
//   principle P5), the rpy → R map is rtc_math's; none is re-derived here.
//   `computeRpyJacobian[Inverse]` throws only for an unknown ReferenceFrame,
//   unreachable with the LOCAL literal passed here.

#include "integrated_bringup/controllers/locomotion/original_layout_adapter.hpp"

#include "integrated_bringup/backends/floating_base_conventions.hpp"
#include "rtc_math/se3/so3.hpp"
#include "rtc_urdf_bridge/free_flyer_dims.hpp"

#include <Eigen/Geometry>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <cmath>
#include <numbers>

namespace integrated_bringup::locomotion {

namespace {

constexpr int kNq = rtc_urdf_bridge::kFreeFlyerNq;
constexpr int kNv = rtc_urdf_bridge::kFreeFlyerNv;

// Offsets inside one original half: position / linear velocity at 0..2, the
// Euler triple (ψ, θ, φ) or its rates at 3..5.
constexpr int kEulerOffset = 3;

// Original (ψ, θ, φ) at `offset` → pinocchio / rtc_math (r, p, y) = (φ, θ, ψ).
Eigen::Vector3d ReversedTriple(const Eigen::Ref<const Eigen::VectorXd>& x,
                               Eigen::Index offset) noexcept {
  return Eigen::Vector3d(x[offset + 2], x[offset + 1], x[offset]);
}

}  // namespace

bool OriginalToFreeFlyer(const Eigen::Ref<const Eigen::VectorXd>& q_o,
                         const Eigen::Ref<const Eigen::VectorXd>& v_o,
                         Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> v) noexcept {
  const Eigen::Index nj = q_o.size() - kOriginalBaseDim;
  if (nj < 0 || v_o.size() != q_o.size() || q.size() != nj + kNq || v.size() != nj + kNv) {
    return false;
  }
  if (!q_o.allFinite() || !v_o.allFinite()) {
    return false;
  }

  const Eigen::Vector3d rpy = ReversedTriple(q_o, kEulerOffset);
  const Eigen::Vector3d rpy_dot = ReversedTriple(v_o, kEulerOffset);
  const Eigen::Matrix3d R = rtc::math::se3::RpyToRotationZyx(rpy);
  Eigen::Quaterniond quat(R);
  if (quat.w() < 0.0) {
    quat.coeffs() = -quat.coeffs();  // one sign for one rotation: w >= 0
  }
  const Eigen::Vector3d p_w = q_o.head<3>();
  const Eigen::Vector3d v_b = R.transpose() * v_o.head<3>();
  const Eigen::Matrix3d E_b = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL);
  const Eigen::Vector3d w_b = E_b * rpy_dot;

  q.head<3>() = p_w;
  q[3] = quat.x();
  q[4] = quat.y();
  q[5] = quat.z();
  q[6] = quat.w();  // w last in pinocchio
  q.tail(nj) = q_o.tail(nj);
  v.head<3>() = v_b;
  v.segment<3>(3) = w_b;
  v.tail(nj) = v_o.tail(nj);
  return true;
}

bool FreeFlyerToOriginal(const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& v,
                         Eigen::Ref<Eigen::VectorXd> q_o,
                         Eigen::Ref<Eigen::VectorXd> v_o) noexcept {
  const Eigen::Index nj = q.size() - kNq;
  if (nj < 0 || v.size() != nj + kNv || q_o.size() != nj + kOriginalBaseDim ||
      v_o.size() != nj + kOriginalBaseDim) {
    return false;
  }
  if (!q.allFinite() || !v.allFinite()) {
    return false;
  }
  const std::array<double, 4> quat_wxyz{q[6], q[3], q[4], q[5]};
  if (!rtc::IsUnitQuaternionWxyz(quat_wxyz)) {
    return false;
  }

  Eigen::Quaterniond quat(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
  quat.normalize();
  const Eigen::Matrix3d R = quat.toRotationMatrix();
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(R);
  if (!(std::abs(rpy[1]) < std::numbers::pi / 2.0 - kGimbalMargin)) {
    return false;
  }
  const Eigen::Vector3d p_w = q.head<3>();
  const Eigen::Vector3d v_w = R * v.head<3>();
  const Eigen::Matrix3d E_b_inv = pinocchio::rpy::computeRpyJacobianInverse(rpy, pinocchio::LOCAL);
  const Eigen::Vector3d rpy_dot = E_b_inv * v.segment<3>(3);

  q_o.head<3>() = p_w;
  q_o[3] = rpy[2];  // ψ
  q_o[4] = rpy[1];  // θ
  q_o[5] = rpy[0];  // φ
  q_o.tail(nj) = q.tail(nj);
  v_o.head<3>() = v_w;
  v_o[3] = rpy_dot[2];  // ψ̇
  v_o[4] = rpy_dot[1];  // θ̇
  v_o[5] = rpy_dot[0];  // φ̇
  v_o.tail(nj) = v.tail(nj);
  return true;
}

bool OriginalWeightsToTangent(const Eigen::Ref<const Eigen::VectorXd>& w_o,
                              Eigen::Ref<Eigen::VectorXd> w) noexcept {
  const Eigen::Index n = w_o.size();
  if (w.size() != n || n % 2 != 0 || n < 2 * kOriginalBaseDim) {
    return false;
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    if (!std::isfinite(w_o[i]) || w_o[i] < 0.0) {
      return false;
    }
  }

  const Eigen::Index half = n / 2;  // = 6 + nj = nv
  for (const Eigen::Index off : std::array<Eigen::Index, 2>{0, half}) {
    // Read before the identity copy so w_o and w may be one buffer.
    const double w_yaw = w_o[off + kEulerOffset];       // ψ / ψ̇
    const double w_roll = w_o[off + kEulerOffset + 2];  // φ / φ̇
    w.segment(off, half) = w_o.segment(off, half);
    w[off + kEulerOffset] = w_roll;     // tangent x ← φ
    w[off + kEulerOffset + 2] = w_yaw;  // tangent z ← ψ (y ← θ by the copy)
  }
  return true;
}

pinocchio::SE3 WorldFromBase(const pinocchio::SE3& base_pose,
                             const pinocchio::SE3& target_in_base) noexcept {
  return base_pose * target_in_base;
}

}  // namespace integrated_bringup::locomotion
