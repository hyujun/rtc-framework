#ifndef RTC_MPC_OCP_CONTACT_KINEMATICS_HPP_
#define RTC_MPC_OCP_CONTACT_KINEMATICS_HPP_

/// @file contact_kinematics.hpp
/// @brief The contact-frame kinematics the kinodynamic OCP's residuals share:
///        each contact frame's placement, velocity and classical acceleration
///        on world axes, with the acceleration taken through the stage's own
///        kinodynamic ODE, and their analytic Jacobians.
///
/// ## Quantities
///
/// For a frame `f` with world placement `(R, p)`:
///
///   `[p; θ; v; ω; a; α]` (18 rows, world axes, at the frame origin)
///
/// - `v, ω` the frame velocity and `a, α` its **classical** acceleration
///   (`LOCAL_WORLD_ALIGNED`; `a = a_spatial + ω × v`);
/// - `θ` the world rotation increment of the frame (`δR = [δθ]× R`): it has no
///   value, only a Jacobian (`J_ω`, the world angular Jacobian), which a
///   residual composes with the derivative of whatever function of `R` it
///   evaluates (`Tilt` below).
///
/// The joint-space acceleration `v̇ = [v̇_b; q̈_j]` comes out of the ODE
/// (`forward`), so `a` and `α` are those the dynamics integrates; their
/// Jacobians chain the frame derivatives with the ODE's `∂v̇/∂(x, u)`:
///
///   ∂a/∂x = ∂a/∂(q, v)|_{v̇} + ∂a/∂v̇ · ∂v̇/∂x,    ∂a/∂u = ∂a/∂v̇ · ∂v̇/∂u.
///
/// The frame derivatives are taken in `LOCAL` and rotated (`x_w = R·x_L`,
/// `δx_w = R·δx_L − [x_w]× δθ`): pinocchio's `LOCAL_WORLD_ALIGNED`
/// derivatives disagree with central differences of the
/// `LOCAL_WORLD_ALIGNED` quantities in the q columns.
///
/// ## Order and real time
///
/// `Forward`, then `FrameMotionOf` for any frame; `Derivatives` (after the
/// `Forward` of the same `(x, u)` on the same buffers — the ODE's data
/// contract), then `FrameJacobian` for any frame. None of them allocates
/// itself; the ODE's `dForward` allocates inside the library. The buffers
/// are built at configure time (`ContactKinematics(ode)` allocates).

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/dynamics.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <memory>

namespace rtc::mpc {

/// @brief Rows of the frame quantities `[p; θ; v; ω; a; α]`.
inline constexpr int kFrameKinematicsRows = 18;

/// @brief The world-axis value of one frame after `ContactKinematics::Forward`.
struct FrameMotion {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};  ///< classical
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
};

/// @brief Evaluation buffers of the frame kinematics through one ODE.
struct ContactKinematics {
  using Ode = KinodynamicDynamics::Ode;
  using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using Matrix3x = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using MatrixRef = Eigen::Ref<Eigen::MatrixXd>;
  using ConstVectorRef = Eigen::Ref<const Eigen::VectorXd>;

  /// @brief Buffers for @p ode's model. Allocates (configure time).
  explicit ContactKinematics(const Ode& ode);

  /// @brief `ode.forward(x, u)` on these buffers, then the forward kinematics
  ///        of `(q, v, v̇)`.
  void Forward(const Ode& ode, const ConstVectorRef& x, const ConstVectorRef& u);

  /// @brief Frame @p fid's world-axis value. After `Forward`.
  void FrameMotionOf(const pinocchio::Model& model, pinocchio::FrameIndex fid,
                     FrameMotion& out) noexcept;

  /// @brief `ode.dForward(x, u)` and the forward-kinematics derivatives of
  ///        `(q, v, v̇)`. After the `Forward` of the same `(x, u)`.
  void Derivatives(const Ode& ode, const ConstVectorRef& x, const ConstVectorRef& u);

  /// @brief The Jacobian of frame @p fid's `[p; θ; v; ω; a; α]` with respect
  ///        to the state tangent (@p jx, `18 × 2·nv`) and the input (@p ju,
  ///        `18 × nu`). Writes every entry of both. After `Derivatives`.
  void FrameJacobian(const pinocchio::Model& model, pinocchio::FrameIndex fid, MatrixRef jx,
                     MatrixRef ju) noexcept;

  pinocchio::Data pin_data;
  std::shared_ptr<aligator::dynamics::ContinuousDynamicsDataTpl<double>> ode_data;
  Eigen::VectorXd vdot;  ///< `[v̇_b; q̈_j]` of the last `Forward`
  Matrix6x v_dq;         ///< frame velocity derivatives (LOCAL)
  Matrix6x v_dv;         ///< also the frame Jacobian (LOCAL)
  Matrix6x a_dq;         ///< frame spatial acceleration derivatives (LOCAL)
  Matrix6x a_dv;
  Matrix6x a_da;
  Matrix3x jw;       ///< world angular Jacobian `J_ω` (rotation of the frame)
  Matrix3x lin;      ///< scratch: a LOCAL classical-acceleration derivative
  Matrix6x acc_jac;  ///< `∂(a, α)/∂v̇` on world axes
};

/// @brief The tilt `e = −c / N` of a frame whose z axis is @p a against the
///        plane normal @p n (`c = a × n`, `w = 1 + a·n`, `N = ‖(c, w)‖`
///        floored at @p min_norm) and, when @p de_da is given, `∂e/∂a` (zero
///        change of `N` once floored). `e = −vec(q_arc)`, `q_arc` the
///        shortest-arc rotation taking `a` onto `n`: zero when they agree,
///        norm `sin(θ/2)` for a tilt `θ`, unchanged by a rotation about `n`.
void FrameTilt(const Eigen::Vector3d& a, const Eigen::Vector3d& n, double min_norm,
               Eigen::Vector3d& e, Eigen::Matrix3d* de_da) noexcept;

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_CONTACT_KINEMATICS_HPP_
