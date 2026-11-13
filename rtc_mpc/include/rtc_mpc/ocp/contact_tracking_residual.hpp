#ifndef RTC_MPC_OCP_CONTACT_TRACKING_RESIDUAL_HPP_
#define RTC_MPC_OCP_CONTACT_TRACKING_RESIDUAL_HPP_

/// @file contact_tracking_residual.hpp
/// @brief The contact (foot) tracking residual of the kinodynamic OCP: for
///        every contact frame, its position, its tilt against the support
///        plane, and its velocity and acceleration, against per-node targets.
///
/// ## Rows
///
/// `nr = 18·n_contacts`. Contact k owns rows `18k … 18k + 17`:
///
///   r_k = s_k · [ p − p*;  e_R;  v − v*;  ω − ω*;  a − a*;  α − α* ]
///
/// - `p` the frame origin in world; `v, ω` its velocity and `a, α` its
///   **classical** acceleration, all on world axes at the frame origin
///   (`LOCAL_WORLD_ALIGNED`; `a = a_spatial + ω × v`).
/// - `e_R = −vec(q_arc)`, where `q_arc` is the shortest-arc rotation that
///   takes the frame's z axis `R·e_z` onto the plane normal `n` (world). It is
///   zero when the frame's z axis is along `n`, its norm is `sin(θ/2)` for a
///   tilt `θ`, and a rotation about `n` (yaw) does not change it.
/// - `s_k ∈ [0, 1]` the contact's scale. With `s_k = 0` the block's value
///   and Jacobian rows are exactly zero (a stance contact, or one not
///   tracked at this node), so the row count never depends on the mode.
///
/// The residual carries no weight: a `QuadraticResidualCost` with diagonal
/// `W` over it gives `1/2 rᵀ W r = 1/2 s_k² Σ w_i e_i²`.
///
/// ## Acceleration and the dynamics
///
/// `a` and `α` follow from the joint-space acceleration `v̇ = [v̇_b; q̈_j]`,
/// and the base part `v̇_b` comes out of the kinodynamic law — so the
/// residual evaluates the stage's **own** ODE (`Bind`) on data of its own:
/// the acceleration the cost sees is the one the dynamics integrates, and a
/// mode switched on the stage's ODE reaches the residual too. The analytic
/// Jacobian chains the frame kinematics derivatives with the ODE's
/// `∂v̇/∂(x, u)`:
///
///   ∂a/∂x = ∂a/∂(q, v)|_{v̇} + ∂a/∂v̇ · ∂v̇/∂x,    ∂a/∂u = ∂a/∂v̇ · ∂v̇/∂u.
///
/// The position, tilt and velocity rows do not depend on `u`.
///
/// ## Lifetime and binding
///
/// Aligator copies a function into every cost, stage and problem built from
/// it, so `Bind` must reach the copy that the final problem evaluates, after
/// that problem is built, and before its data is created (`createData`
/// throws `std::logic_error` while unbound — configure time). A copy keeps
/// the binding of its source. The bound ODE must outlive the residual's use.
///
/// ## Real-time
///
/// `SetTarget`, `SetScale`, `SetPlaneNormal`, `SetFromStage`, `AcceptsStage`,
/// `Bind` and the accessors are noexcept and do not allocate. `evaluate` and
/// `computeJacobians` allocate nothing of their own; the ODE's `dForward`
/// allocates inside the library (counted by the allocation gate). The
/// targets live on the function, the evaluation on the data: write targets
/// between solves, on the thread that runs them.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/function-abstract.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <vector>

namespace rtc::mpc {

/// @brief What contact k is driven toward at one node (world axes, SI). The
///        velocity and acceleration are at the frame origin, the acceleration
///        classical — the `StageReference` convention.
struct ContactTrackingTarget {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
};

/// @brief Evaluation buffers of one @ref ContactTrackingResidual.
struct ContactTrackingData : aligator::StageFunctionDataTpl<double> {
  using Base = aligator::StageFunctionDataTpl<double>;
  using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using Matrix3x = Eigen::Matrix<double, 3, Eigen::Dynamic>;

  ContactTrackingData(int ndx, int nu, int nr, const KinodynamicDynamics::Ode& ode);

  pinocchio::Data pin_data;
  std::shared_ptr<aligator::dynamics::ContinuousDynamicsDataTpl<double>> ode_data;
  Eigen::VectorXd vdot;  ///< `[v̇_b; q̈_j]` of the last `evaluate`
  /// The scale each contact was evaluated with (0 = block skipped): the
  /// Jacobian uses the same, so value and Jacobian always belong together.
  std::vector<double> scale_used;
  bool evaluated{false};  ///< an `evaluate` with a non-zero scale ran on this data
  Matrix6x v_dq;          ///< frame velocity derivatives (LOCAL)
  Matrix6x v_dv;          ///< also the frame Jacobian (LOCAL)
  Matrix6x a_dq;          ///< frame spatial acceleration derivatives (LOCAL)
  Matrix6x a_dv;
  Matrix6x a_da;
  Matrix3x jw;       ///< world angular Jacobian `J_ω` (rotation of the frame)
  Matrix3x lin;      ///< scratch: a LOCAL classical-acceleration derivative
  Matrix6x acc_jac;  ///< `∂(a, α)/∂v̇` on world axes
};

/// @brief The `18·n_contacts` contact tracking residual over the kinodynamic
///        state `x = [q; v]` and input `u = [W_0 … W_{nc−1}; q̈_j]`.
class ContactTrackingResidual final : public aligator::StageFunctionTpl<double> {
 public:
  using Base = aligator::StageFunctionTpl<double>;
  using Ode = KinodynamicDynamics::Ode;
  using BaseData = aligator::StageFunctionDataTpl<double>;

  /// @brief Rows per contact.
  static constexpr int kRowsPerContact = 18;

  /// @brief Smallest `‖(R·e_z × n, 1 + R·e_z·n)‖ = 2·cos(θ/2)` the tilt
  ///        divides by (θ the tilt): reached only by a frame within ~0.06° of
  ///        upside down against the plane, outside any stance or swing. Below
  ///        it the divisor is held, so the tilt rows stay of order
  ///        `1/kMinArcNorm` (value and Jacobian of the same held function).
  static constexpr double kMinArcNorm = 1e-3;

  /// @brief A residual over @p ode's state, input and contact frames, with
  ///        every scale 0, zero targets and the plane normal `+z`. Unbound —
  ///        `Bind` before `createData`. Allocates (configure time).
  explicit ContactTrackingResidual(const Ode& ode);

  /// @brief Evaluate @p ode (the stage's own copy) for the acceleration; null
  ///        unbinds. Does not allocate.
  /// @return false, leaving the binding unchanged, when @p ode has another
  ///         state, input, contact frame list or model structure (`nq`,
  ///         joint and frame counts) than the ODE this residual was made from.
  [[nodiscard]] bool Bind(const Ode* ode) noexcept;

  [[nodiscard]] const Ode* bound_ode() const noexcept { return ode_; }

  /// @return false, writing nothing, for a contact out of range or a
  ///         non-finite target.
  [[nodiscard]] bool SetTarget(int contact, const ContactTrackingTarget& target) noexcept;

  /// @return false, writing nothing, for a contact out of range or a scale
  ///         outside `[0, 1]` (NaN included).
  [[nodiscard]] bool SetScale(int contact, double scale) noexcept;

  /// @return false, writing nothing, for a normal that is not finite and unit
  ///         (`|‖n‖² − 1| ≤ kSurfaceNormalTolerance`). Stored normalised.
  [[nodiscard]] bool SetPlaneNormal(const Eigen::Vector3d& normal) noexcept;

  /// @brief All of one node at once: each contact's target from @p stage
  ///        (`contact_pose` position, `contact_twist`, `contact_acceleration`),
  ///        its scale `0` when its bit of `stage.mode` is set (stance) and
  ///        `impact_proximity[k]` otherwise, and the plane normal from
  ///        @p terrain.
  /// @return false, writing nothing, when any of those would be refused by
  ///         the setters above.
  [[nodiscard]] bool SetFromStage(const StageReference& stage,
                                  const SurfaceFrame& terrain) noexcept;

  /// @return whether `SetFromStage` would accept @p stage and @p terrain on a
  ///         residual of @p n_contacts contacts — the check it runs before
  ///         writing, for a caller that must validate many nodes first.
  [[nodiscard]] static bool AcceptsStage(const StageReference& stage, const SurfaceFrame& terrain,
                                         int n_contacts) noexcept;

  [[nodiscard]] int n_contacts() const noexcept { return static_cast<int>(contact_ids_.size()); }

  [[nodiscard]] double scale(int contact) const noexcept;

  [[nodiscard]] const ContactTrackingTarget& target(int contact) const noexcept;

  [[nodiscard]] const Eigen::Vector3d& plane_normal() const noexcept { return normal_; }

  /// @brief Unbound: every value entry NaN.
  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, BaseData& data) const override;

  /// @brief Needs the `evaluate` of the same `(x, u)` on the same data first
  ///        (Aligator's order). Unbound: every Jacobian entry NaN.
  void computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u,
                        BaseData& data) const override;

  /// @throws std::logic_error while unbound. Allocates.
  [[nodiscard]] std::shared_ptr<BaseData> createData() const override;

 private:
  std::vector<pinocchio::FrameIndex> contact_ids_;
  int nq_{0};
  int njoints_{0};
  int nframes_{0};
  std::vector<ContactTrackingTarget> targets_;
  std::vector<double> scales_;
  Eigen::Vector3d normal_{0.0, 0.0, 1.0};
  const Ode* ode_{nullptr};  // non-owning; the stage's ODE
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_CONTACT_TRACKING_RESIDUAL_HPP_
