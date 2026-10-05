// ── Decel MPC catch-node linearisation seam (E1-F07, formulation §1.2) ───────
// The first-order models of the three nonlinear outputs the catch terms use,
// at one reference point (q, v), exposed so a test can check each Jacobian
// against a finite difference of the nonlinear output instead of only through
// a QP answer (the same reason mpc_segment_core_torque.hpp exists):
//
//   p_C(q + δq)          ≈ p + J_v δq                      catch-frame position
//   e_a(q + δq)          ≈ e_a + L_a δq,  L_a = J_a J_ω    approach-axis error
//   J_v(q + δq) q̇        ≈ J_v q̇ + H_v δq                 catch-frame velocity
//
// all in LOCAL_WORLD_ALIGNED (world axes, origin at the frame).
//  • e_a, J_a are rtc_math's axis-alignment error and Jacobian of the frame's
//    +z against a_d (ė_a = J_a ω, ω in world). Inside the aligned deadband
//    rtc_math returns J_a = 0, because e_a is held at 0 there; a least-squares
//    term would then lose the axis's curvature for that cycle, so this seam
//    substitutes the Jacobian's limit [a_d]×[z]× (θ → 0: f → 1, m mᵀ → 0).
//  • H_v = ∂_q[J_v(q) v] is pinocchio's POINT velocity derivative
//    (getPointVelocityDerivatives). The frame variant differentiates the
//    spatial velocity and lacks the ω × v term a LOCAL_WORLD_ALIGNED point
//    velocity carries.
//
// RT-safe: no heap, noexcept. The model must have nq == nv; every matrix
// argument must already be sized (j6_work 6 × n, the others 3 × n).
#pragma once

#include "rtc_controllers/catching/mpc_segment_core.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace rtc::catching {

/// @brief The catch frame's pose quantities at the linearisation point —
///        the constant parts of the three first-order models above.
struct CatchLinearization {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};    ///< p_C(q) [m]
  Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};   ///< approach axis R_WC e_z
  Eigen::Vector3d e_a{Eigen::Vector3d::Zero()};  ///< axis error [rad]; zero unless with_axis
};

/// @brief Linearise the catch frame's position, approach-axis error and point
///        velocity at (q, v), in LOCAL_WORLD_ALIGNED.
/// @param model          the arm (nq == nv)
/// @param data           pinocchio workspace of `model`; its kinematics are
///                       overwritten (oMf holds the pose at q on return)
/// @param frame          catch frame index in `model`
/// @param q              joint positions of the linearisation point [rad], size n
/// @param v              joint velocities of the linearisation point [rad/s],
///                       size n; read when with_velocity
/// @param a_d            desired approach axis (unit, world); read when with_axis
/// @param axis_theta_max largest ‖e_a‖ the axis model is trusted at [rad]
/// @param with_axis      compute e_a and l_a (else both are left untouched)
/// @param with_velocity  compute h_v (else left untouched)
/// @param j6_work        6 × n scratch (the frame Jacobian)
/// @param[out] j_v       linear rows of the frame Jacobian, ∂p_C/∂q (3 × n)
/// @param[out] j_w       angular rows of the frame Jacobian (3 × n)
/// @param[out] l_a       J_a J_ω (3 × n)
/// @param[out] h_v       ∂_q[J_v(q) v] (3 × n)
/// @param dv_work        3 × n scratch (pinocchio's ∂/∂v output)
/// @param[out] out       p_C, the approach axis and e_a at q
/// @return kNone; kDimMismatch on a size mismatch; kCatchAxisOutOfRange when
///         a_d is not unit, ‖e_a‖ > axis_theta_max, or the axes are
///         antiparallel (the rotation axis is undefined there).
/// @note RT-safe: no heap, noexcept. Every matrix argument must already be
///       sized; the outputs are unspecified when the return is not kNone.
[[nodiscard]] DecelMpcReason LinearizeCatchAt(
    const pinocchio::Model& model, pinocchio::Data& data, pinocchio::FrameIndex frame,
    const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Vector3d& a_d, double axis_theta_max, bool with_axis, bool with_velocity,
    Eigen::MatrixXd& j6_work, Eigen::Matrix<double, 3, Eigen::Dynamic>& j_v,
    Eigen::Matrix<double, 3, Eigen::Dynamic>& j_w, Eigen::Matrix<double, 3, Eigen::Dynamic>& l_a,
    Eigen::Matrix<double, 3, Eigen::Dynamic>& h_v,
    Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_work, CatchLinearization& out) noexcept;

}  // namespace rtc::catching
