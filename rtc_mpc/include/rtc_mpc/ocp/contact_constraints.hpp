#ifndef RTC_MPC_OCP_CONTACT_CONSTRAINTS_HPP_
#define RTC_MPC_OCP_CONTACT_CONSTRAINTS_HPP_

/// @file contact_constraints.hpp
/// @brief The hard contact constraints of the kinodynamic OCP, at a row count
///        that never depends on the contact mode: the per-contact equality
///        (stance foot held, swing foot on its height plan) and the wrench
///        cone inequality (friction and centre of pressure).
///
/// ## Fixed rows, active by mode
///
/// Every stage carries every contact's rows of both phases. A contact's mode
/// is read from the stage's own ODE (`Bind`) at `evaluate` and kept on the
/// data, so the value, the Jacobian and the dynamics always see the same
/// mode. Rows of the phase a contact is not in are inactive: value `+0.0`
/// (equality) or `−1` (inequality, `c ≤ 0` holds strictly) and Jacobian `0`
/// — they change neither the solution nor the iterates.
///
/// ## Equality (`ContactConstraintResidual`, `c = 0`)
///
/// Contact k owns `EqualityRows(force_size)` rows: a stance block (6 rows for
/// a 6-D contact, 3 for a 3-D one) and one swing row. With `p, e_R, v, ω, a,
/// α` the frame quantities of `ContactKinematics` (world axes; `e_R` the tilt
/// against the plane normal, `FrameTilt`) and `g` the gains:
///
///   stance  [ g_vxy·v_x + g_axy·a_x;
///             g_vxy·v_y + g_axy·a_y;
///             g_pz·(z − z*) + g_vz·v_z + g_az·a_z;
///             g_R·e_R + g_ω·ω + g_α·α ]          (6-D; a 3-D contact keeps
///                                                  the first three)
///   swing   g_pz·(z − z*) + g_vz·(ż − ż*) + g_az·(z̈ − z̈*)
///
/// — the stance foot's acceleration driven to a stabilised zero (no slip, no
/// tilt, held at the reference height), and the swing foot's vertical motion
/// to its plan. The targets `z*, ż*, z̈*` are per node (`SetFromStage`).
///
/// ## Inequality (`ContactWrenchConeResidual`, `c ≤ 0`)
///
/// Contact k owns `InequalityRows(force_size)` rows, active in stance. With
/// the wrench `W_k = [f; τ]` (world axes, moment about the frame origin — the
/// ODE's convention):
///
///   friction   c = √(f_x² + f_y² + ε) − μ·(f_z + f_grip)        (world axes)
///   CoP        c = −[ τ_x − y_min·F_z;  −τ_x + y_max·F_z;
///                     −τ_y − x_min·F_z;   τ_y + x_max·F_z ]     (6-D only)
///
/// with `[F; τ]_L = Rᵀ·[f; τ]` on the contact frame's own axes (`R` its
/// world rotation, `F_z = (Rᵀf)_z`): the centre of pressure inside the
/// rectangle `[x_min, x_max] × [y_min, y_max]`. `ε > 0` keeps the cone
/// differentiable at its apex (a normal force of at least `√ε/μ − f_grip`).
/// No `τ_z` bound.
///
/// ## Lifetime, binding and real time
///
/// As `ContactTrackingResidual`: bind each stage's copy to that stage's ODE
/// after the problem is built and before its data is created (`createData`
/// throws `std::logic_error` while unbound); the bound ODE must outlive the
/// use. `Bind`, `SetFromStage`, `AcceptsStage` and the accessors are
/// noexcept and do not allocate. `evaluate` and `computeJacobians` allocate
/// nothing of their own; the equality's ODE evaluation (`dForward`) allocates
/// inside the library. Unbound, every value and Jacobian entry is NaN.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/ocp/contact_kinematics.hpp"
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

/// @brief The gains of the contact equality (stance and swing share them;
///        swing uses the three vertical ones). Position x, y carry no gain.
struct FootConstraintGains {
  double position_z{0.0};
  double orientation{0.0};
  double linear_velocity_xy{0.0};
  double linear_velocity_z{0.0};
  double angular_velocity{0.0};
  double linear_acceleration_xy{1.0};
  double linear_acceleration_z{1.0};
  double angular_acceleration{1.0};
};

/// @brief The friction cone `√(f_x² + f_y² + ε) ≤ μ·(f_z + f_grip)`.
struct FrictionCone {
  double mu{0.0};              ///< friction coefficient, > 0
  double grip_force{0.0};      ///< [N] ≥ 0
  double regularization{0.0};  ///< ε [N²] > 0
};

/// @brief The centre-of-pressure rectangle on the contact frame's x-y plane
///        [m]; it contains the frame origin (`x_min < 0 < x_max`, same in y).
struct ContactRectangle {
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
};

/// @brief What the contact equality drives contact k toward at one node
///        (world z of the frame origin, SI).
struct ContactHeightTarget {
  double z{0.0};
  double vz{0.0};
  double az{0.0};
};

/// @brief Evaluation buffers of one @ref ContactConstraintResidual.
struct ContactConstraintData : aligator::StageFunctionDataTpl<double> {
  using Base = aligator::StageFunctionDataTpl<double>;

  ContactConstraintData(int ndx, int nu, int nr, const KinodynamicDynamics::Ode& ode);

  ContactKinematics kin;
  std::vector<char> stance;  ///< each contact's mode at the last `evaluate`
  bool evaluated{false};     ///< `evaluate` ran on this data while bound
  Eigen::MatrixXd jx;        ///< `18 × 2·nv` — one frame's kinematic Jacobian
  Eigen::MatrixXd ju;        ///< `18 × nu`
  Eigen::Matrix<double, 3, Eigen::Dynamic> tilt_jac;  ///< `3 × nv` scratch
};

/// @brief The per-contact stance / swing equality over the kinodynamic state
///        and input.
class ContactConstraintResidual final : public aligator::StageFunctionTpl<double> {
 public:
  using Base = aligator::StageFunctionTpl<double>;
  using Ode = KinodynamicDynamics::Ode;
  using BaseData = aligator::StageFunctionDataTpl<double>;

  /// @brief The tilt divisor floor (as `ContactTrackingResidual::kMinArcNorm`).
  static constexpr double kMinArcNorm = 1e-3;

  /// @return rows of one contact: stance 6 (3 for a 3-D contact) + swing 1.
  [[nodiscard]] static constexpr int EqualityRows(int force_size) noexcept {
    return (force_size == 6 ? 6 : 3) + 1;
  }

  /// @brief Unbound, every target at zero height, the plane normal `+z`.
  ///        Allocates (configure time).
  ContactConstraintResidual(const Ode& ode, const FootConstraintGains& gains);

  /// @return false, leaving the binding unchanged, for an ODE of another
  ///         state, input, contact list or model structure. Null unbinds.
  [[nodiscard]] bool Bind(const Ode* ode) noexcept;

  [[nodiscard]] const Ode* bound_ode() const noexcept { return ode_; }

  /// @brief Each contact's `contact_pose` z, `contact_twist` z and
  ///        `contact_acceleration` z, and the plane normal of @p terrain.
  /// @return false, writing nothing, when `AcceptsStage` would refuse.
  [[nodiscard]] bool SetFromStage(const StageReference& stage,
                                  const SurfaceFrame& terrain) noexcept;

  /// @return whether `SetFromStage` would accept @p stage and @p terrain on
  ///         a residual of @p n_contacts contacts (finite targets, a unit
  ///         normal, a contact count in range).
  [[nodiscard]] static bool AcceptsStage(const StageReference& stage, const SurfaceFrame& terrain,
                                         int n_contacts) noexcept;

  [[nodiscard]] int n_contacts() const noexcept { return static_cast<int>(contact_ids_.size()); }

  [[nodiscard]] int force_size() const noexcept { return force_size_; }

  [[nodiscard]] int rows_per_contact() const noexcept { return EqualityRows(force_size_); }

  [[nodiscard]] const FootConstraintGains& gains() const noexcept { return gains_; }

  /// @return contact @p k's target; a zero target for one out of range.
  [[nodiscard]] const ContactHeightTarget& target(int k) const noexcept;

  [[nodiscard]] const Eigen::Vector3d& plane_normal() const noexcept { return normal_; }

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, BaseData& data) const override;

  /// @brief Needs the `evaluate` of the same `(x, u)` on the same data first.
  ///        A bound ODE whose mode changed since that `evaluate` gives a NaN
  ///        Jacobian (the rows and the ODE derivatives would disagree).
  void computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u,
                        BaseData& data) const override;

  /// @throws std::logic_error while unbound. Allocates.
  [[nodiscard]] std::shared_ptr<BaseData> createData() const override;

 private:
  std::vector<pinocchio::FrameIndex> contact_ids_;
  int force_size_{0};
  int nq_{0};
  int njoints_{0};
  int nframes_{0};
  FootConstraintGains gains_;
  std::vector<ContactHeightTarget> targets_;
  Eigen::Vector3d normal_{0.0, 0.0, 1.0};
  const Ode* ode_{nullptr};  // non-owning; the stage's ODE
};

/// @brief Evaluation buffers of one @ref ContactWrenchConeResidual.
struct ContactWrenchConeData : aligator::StageFunctionDataTpl<double> {
  using Base = aligator::StageFunctionDataTpl<double>;

  ContactWrenchConeData(int ndx, int nu, int nr, const KinodynamicDynamics::Ode& ode);

  pinocchio::Data pin_data;
  std::vector<char> stance;  ///< each contact's mode at the last `evaluate`
  bool evaluated{false};     ///< `evaluate` ran on this data while bound
  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jac;  ///< LOCAL frame Jacobian
  Eigen::Matrix<double, 3, Eigen::Dynamic> jw;         ///< world angular Jacobian
};

/// @brief The per-contact friction and centre-of-pressure inequality over
///        the kinodynamic input (and, through the contact frame's rotation,
///        the configuration).
class ContactWrenchConeResidual final : public aligator::StageFunctionTpl<double> {
 public:
  using Base = aligator::StageFunctionTpl<double>;
  using Ode = KinodynamicDynamics::Ode;
  using BaseData = aligator::StageFunctionDataTpl<double>;

  /// @return rows of one contact: friction 1 + CoP 4 (a 3-D contact: 1).
  [[nodiscard]] static constexpr int InequalityRows(int force_size) noexcept {
    return force_size == 6 ? 5 : 1;
  }

  /// @brief The value an inactive row holds.
  static constexpr double kInactiveValue = -1.0;

  /// @brief Unbound. @p friction and @p rectangle are taken as given (the
  ///        config loader checks them). Allocates (configure time).
  ContactWrenchConeResidual(const Ode& ode, const FrictionCone& friction,
                            const ContactRectangle& rectangle);

  /// @return false, leaving the binding unchanged, for an ODE of another
  ///         state, input, contact list or model structure. Null unbinds.
  [[nodiscard]] bool Bind(const Ode* ode) noexcept;

  [[nodiscard]] const Ode* bound_ode() const noexcept { return ode_; }

  [[nodiscard]] int n_contacts() const noexcept { return static_cast<int>(contact_ids_.size()); }

  [[nodiscard]] int force_size() const noexcept { return force_size_; }

  [[nodiscard]] int rows_per_contact() const noexcept { return InequalityRows(force_size_); }

  [[nodiscard]] const FrictionCone& friction() const noexcept { return friction_; }

  [[nodiscard]] const ContactRectangle& rectangle() const noexcept { return rectangle_; }

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, BaseData& data) const override;

  /// @brief Needs the `evaluate` of the same `(x, u)` on the same data first.
  void computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u,
                        BaseData& data) const override;

  /// @throws std::logic_error while unbound. Allocates.
  [[nodiscard]] std::shared_ptr<BaseData> createData() const override;

 private:
  std::vector<pinocchio::FrameIndex> contact_ids_;
  int force_size_{0};
  int nq_{0};
  int njoints_{0};
  int nframes_{0};
  FrictionCone friction_;
  ContactRectangle rectangle_;
  const Ode* ode_{nullptr};  // non-owning; the stage's ODE
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_CONTACT_CONSTRAINTS_HPP_
