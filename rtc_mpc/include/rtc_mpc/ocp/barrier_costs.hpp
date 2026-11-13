#ifndef RTC_MPC_OCP_BARRIER_COSTS_HPP_
#define RTC_MPC_OCP_BARRIER_COSTS_HPP_

/// @file barrier_costs.hpp
/// @brief The soft limits of the kinodynamic OCP: a piecewise-polynomial
///        barrier on a margin `h` (`h ≥ 0` inside), applied to the joint
///        position limits and to sphere pairs (self-collision), and a relaxed
///        log barrier on the contact wrench cone (friction and centre of
///        pressure).
///
/// ## Polynomial barrier
///
///   B(h) = μ·(½h² − δh/2 + δ²/6)                   h ≤ 0
///          μ·(−h³/(6δ) + ½h² − δh/2 + δ²/6)        0 < h < δ
///          0                                       h ≥ δ
///
/// — `C²`, zero (with its derivatives) from `h = δ` on, a quadratic penalty
/// of curvature `μ` below 0. Each cost is a running cost: the OCP weights it
/// by `dt` and puts none on the terminal node.
///
/// ## Joint limits (`JointLimitBarrierCost`)
///
/// `Σ_j B(q_j − q_j,min) + B(q_j,max − q_j)` over the model joints with one
/// configuration and one velocity entry (`nq = nv = 1`: revolute, prismatic)
/// and finite limits — the free-flyer root and any other multi-entry joint
/// are skipped. Value from `q[idx_q]`, gradient and (diagonal) Hessian at
/// `[idx_v]` of the state tangent.
///
/// ## Sphere pairs (`CollisionBarrierCost`)
///
/// Pair i: a sphere of radius `r_a` at offset `o_a` in frame `a` and one of
/// `r_b` at `o_b` in frame `b`; `h_i = ‖p_a − p_b‖ − r_a − r_b` (world
/// points). Active only while the node is not in full stance (read from the
/// bound ODE at `evaluate`). The Hessian is Gauss–Newton, `Σ B''(h_i)·∇h_i
/// ∇h_iᵀ` (PSD; the exact one is not). Below `kMinDistance` the direction is
/// undefined and the pair's gradient is zero.
///
/// ## Relaxed log barrier (`RelaxedLogBarrier`)
///
///   B(h) = −μ·ln h                                  h > δ
///          μ·(−ln δ + ½((h − 2δ)/δ)² − ½)           h ≤ δ
///
/// — the log barrier, continued below `δ` by the quadratic that matches its
/// value, slope and curvature at `h = δ` (`C²`): finite everywhere, so an
/// iterate outside the set (`h ≤ 0`) is penalised, not refused.
///
/// ## Contact wrench cone (`ContactConeBarrierCost`)
///
/// `Σ_rows B(h_i)` over the rows of `ContactWrenchConeResidual` of the
/// contacts in stance at the node (`h = −c`: the friction margin
/// `μ(f_z + f_grip) − √(f_x² + f_y² + ε)` [N] with the friction barrier, and
/// the four centre-of-pressure margins `h = a·τ_L + b·F_z` [N·m] with the CoP
/// barrier); a swing contact's rows add nothing. The mode is the bound ODE's
/// at `evaluate` (the residual reads it). Gradient `Σ B'(h_i)·∇h_i`. Hessian
/// Gauss–Newton `Σ B''(h_i)·∇h_i ∇h_iᵀ` plus, for each friction row, the
/// curvature of its square root, `B'(h)·∇²h` with `∇²_{f_xy} h = −(I −
/// f_xy f_xyᵀ/s²)/s`, `s = √(f_x² + f_y² + ε)` (PSD: `B' < 0`). The other
/// second-order terms of the CoP margins — through the frame rotation, in
/// `q` and across `q` and `[f; τ]` — are dropped: the gradient is exact,
/// the Hessian a PSD approximation (the friction row's is exact).
///
/// ## Real time
///
/// Construction and `createData` allocate (configure time). `evaluate`,
/// `computeGradients`, `computeHessians` and the setters allocate nothing of
/// their own.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/cost-abstract.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <vector>

namespace rtc::mpc {

/// @brief The piecewise-polynomial barrier `B(h)` (file comment).
struct PolynomialBarrier {
  double mu{0.0};     ///< curvature below 0, ≥ 0
  double delta{0.0};  ///< width of the polynomial band, > 0

  [[nodiscard]] double Value(double h) const noexcept;
  [[nodiscard]] double Derivative(double h) const noexcept;
  [[nodiscard]] double SecondDerivative(double h) const noexcept;
};

/// @brief The relaxed log barrier `B(h)` (file comment).
struct RelaxedLogBarrier {
  double mu{0.0};     ///< scale, > 0
  double delta{0.0};  ///< where the log turns quadratic, > 0

  [[nodiscard]] double Value(double h) const noexcept;
  [[nodiscard]] double Derivative(double h) const noexcept;
  [[nodiscard]] double SecondDerivative(double h) const noexcept;
};

/// @brief The joint position limit barrier (file comment).
class JointLimitBarrierCost final : public aligator::CostAbstractTpl<double> {
 public:
  using Base = aligator::CostAbstractTpl<double>;
  using Space = KinodynamicDynamics::Space;
  using CostData = aligator::CostDataAbstractTpl<double>;

  /// @brief Over @p model's limited one-entry joints, on @p space (the
  ///        model's phase space) with @p nu inputs. Allocates.
  JointLimitBarrierCost(const Space& space, int nu, const pinocchio::Model& model,
                        const PolynomialBarrier& barrier);

  /// @return the joints this cost limits: one entry each, finite limits.
  [[nodiscard]] static int CountLimitedJoints(const pinocchio::Model& model) noexcept;

  /// @return whether this cost limits joint @p j of @p model (one
  ///         configuration and one velocity entry, both limits finite).
  [[nodiscard]] static bool IsLimitedJoint(const pinocchio::Model& model,
                                           pinocchio::JointIndex j) noexcept;

  [[nodiscard]] int n_joints() const noexcept { return static_cast<int>(idx_q_.size()); }

  [[nodiscard]] const PolynomialBarrier& barrier() const noexcept { return barrier_; }

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override;
  void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                        CostData& data) const override;
  void computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                       CostData& data) const override;

 private:
  PolynomialBarrier barrier_;
  std::vector<Eigen::Index> idx_q_;
  std::vector<Eigen::Index> idx_v_;
  std::vector<double> lower_;
  std::vector<double> upper_;
};

/// @brief One sphere pair of @ref CollisionBarrierCost.
struct SpherePair {
  pinocchio::FrameIndex frame_a{0};
  Eigen::Vector3d offset_a{Eigen::Vector3d::Zero()};  ///< [m] in frame a
  double radius_a{0.0};                               ///< [m]
  pinocchio::FrameIndex frame_b{0};
  Eigen::Vector3d offset_b{Eigen::Vector3d::Zero()};
  double radius_b{0.0};
};

/// @brief Evaluation buffers of one @ref CollisionBarrierCost.
struct CollisionBarrierData : aligator::CostDataAbstractTpl<double> {
  using Base = aligator::CostDataAbstractTpl<double>;

  CollisionBarrierData(int ndx, int nu, const pinocchio::Model& model, int n_pairs);

  pinocchio::Data pin_data;
  bool active{false};      ///< the node was not in full stance at `evaluate`
  bool evaluated{false};   ///< `evaluate` ran on this data while bound
  std::vector<double> h;   ///< each pair's margin at `evaluate`
  Eigen::MatrixXd grad_h;  ///< `n_pairs × nv`: each pair's `∇h` (q tangent)
  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jac;  ///< LOCAL frame Jacobian
  Eigen::Matrix<double, 3, Eigen::Dynamic> point_a;    ///< world point Jacobians
  Eigen::Matrix<double, 3, Eigen::Dynamic> point_b;
  Eigen::Matrix<double, 3, Eigen::Dynamic> scratch;
};

/// @brief The sphere-pair self-collision barrier (file comment).
class CollisionBarrierCost final : public aligator::CostAbstractTpl<double> {
 public:
  using Base = aligator::CostAbstractTpl<double>;
  using Space = KinodynamicDynamics::Space;
  using Ode = KinodynamicDynamics::Ode;
  using CostData = aligator::CostDataAbstractTpl<double>;

  /// @brief Distance below which a pair's direction is undefined [m].
  static constexpr double kMinDistance = 1e-9;

  /// @brief Over @p ode's model (frames of @p pairs must exist in it).
  ///        Unbound — `Bind` before `createData`. Allocates.
  CollisionBarrierCost(const Space& space, const Ode& ode, std::vector<SpherePair> pairs,
                       const PolynomialBarrier& barrier);

  /// @return false, leaving the binding unchanged, for an ODE of another
  ///         contact list or model structure. Null unbinds.
  [[nodiscard]] bool Bind(const Ode* ode) noexcept;

  [[nodiscard]] const Ode* bound_ode() const noexcept { return ode_; }

  [[nodiscard]] int n_pairs() const noexcept { return static_cast<int>(pairs_.size()); }

  [[nodiscard]] const PolynomialBarrier& barrier() const noexcept { return barrier_; }

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override;
  /// @brief After the `evaluate` of the same `(x, u)` on the same data.
  void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                        CostData& data) const override;
  /// @brief After `computeGradients` (it reuses the `∇h` that left behind).
  void computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                       CostData& data) const override;

  /// @throws std::logic_error while unbound. Allocates.
  [[nodiscard]] std::shared_ptr<CostData> createData() const override;

 private:
  std::vector<SpherePair> pairs_;
  PolynomialBarrier barrier_;
  std::vector<pinocchio::FrameIndex> contact_ids_;
  int nq_{0};
  int njoints_{0};
  int nframes_{0};
  const Ode* ode_{nullptr};  // non-owning; the stage's ODE
};

/// @brief Evaluation buffers of one @ref ContactConeBarrierCost.
struct ContactConeBarrierData : aligator::CostDataAbstractTpl<double> {
  using Base = aligator::CostDataAbstractTpl<double>;

  ContactConeBarrierData(int ndx, int nu, int nv,
                         std::shared_ptr<aligator::StageFunctionDataTpl<double>> cone);

  /// The wrench cone residual's own data (values, Jacobians, modes).
  std::shared_ptr<aligator::StageFunctionDataTpl<double>> cone;
  Eigen::VectorXd jx;           ///< `nv` — one row's configuration Jacobian
  Eigen::VectorXd jx_scaled;    ///< `nv` — the same times `B''`
  bool jacobians_fresh{false};  ///< `computeGradients` ran after the last `evaluate`
};

/// @brief The soft contact wrench cone (file comment).
class ContactConeBarrierCost final : public aligator::CostAbstractTpl<double> {
 public:
  using Base = aligator::CostAbstractTpl<double>;
  using Space = KinodynamicDynamics::Space;
  using Ode = KinodynamicDynamics::Ode;
  using CostData = aligator::CostDataAbstractTpl<double>;

  /// @brief Over a copy of @p residual (its binding included). Allocates.
  /// @throws std::invalid_argument for a barrier `μ` or `δ` not finite and > 0.
  ContactConeBarrierCost(const Space& space, const ContactWrenchConeResidual& residual,
                         const RelaxedLogBarrier& friction, const RelaxedLogBarrier& cop);

  /// @return as `ContactWrenchConeResidual::Bind` (this cost's copy).
  [[nodiscard]] bool Bind(const Ode* ode) noexcept { return residual_.Bind(ode); }

  [[nodiscard]] const Ode* bound_ode() const noexcept { return residual_.bound_ode(); }

  /// @return the residual whose margins this cost penalises.
  [[nodiscard]] const ContactWrenchConeResidual& residual() const noexcept { return residual_; }

  [[nodiscard]] const RelaxedLogBarrier& friction_barrier() const noexcept { return friction_; }

  [[nodiscard]] const RelaxedLogBarrier& cop_barrier() const noexcept { return cop_; }

  /// @brief Unbound: NaN.
  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override;
  /// @brief After the `evaluate` of the same `(x, u)` on the same data.
  void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                        CostData& data) const override;
  /// @brief After `computeGradients` (it reuses the Jacobians that left);
  ///        without it since the last `evaluate`, NaN.
  void computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                       CostData& data) const override;

  /// @throws std::logic_error while unbound. Allocates.
  [[nodiscard]] std::shared_ptr<CostData> createData() const override;

 private:
  ContactWrenchConeResidual residual_;
  RelaxedLogBarrier friction_;
  RelaxedLogBarrier cop_;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_BARRIER_COSTS_HPP_
