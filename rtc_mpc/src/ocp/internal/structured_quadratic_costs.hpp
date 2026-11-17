#ifndef RTC_MPC_OCP_INTERNAL_STRUCTURED_QUADRATIC_COSTS_HPP_
#define RTC_MPC_OCP_INTERNAL_STRUCTURED_QUADRATIC_COSTS_HPP_

/// @file structured_quadratic_costs.hpp
/// @brief Internal: Aligator's quadratic state and control costs with their
///        gradient and Gauss–Newton Hessian assembled from the sparsity of
///        the residual's Jacobian instead of the dense `Jᵀ W J`
///        (`KinodynamicOCP::QuadraticCostDerivatives::kStructured`).
///
/// Not installed. Not part of the public `rtc_mpc` API.
///
/// ## The structure
///
/// Aligator's `QuadraticResidualCostTpl` computes `grad = Jᵀ (W r)` and
/// `hess = (Jᵀ W) J` with `J` the residual's dense `nr × (ndx + nu)`
/// Jacobian. For the two costs here most of `J` is known:
///
/// - state error `r = x ⊖ x_ref` on a multibody phase space whose root joint
///   is a free flyer (configuration rows 0..5 of the tangent) and whose other
///   joints all have a vector-space configuration (revolute, prismatic,
///   translation — their difference is `q₁ − q₀`): `J_x` is
///   `blkdiag(J_b, I)` with `J_b` the free flyer's 6 × 6 block, and `J_u = 0`.
///   So `grad_x = blkdiag(J_bᵀ, I) (W r)`, `H_xx = blkdiag(J_bᵀ W_b J_b,
///   W_rest)` for a diagonal `W`, and every `u` entry is 0;
/// - control error `r = u − u_ref` on a Euclidean control space: `J = [0, I]`,
///   so `grad_u = W r`, `H_uu = W`, and every `x` entry is 0.
///
/// `J_b` is read from the residual's own `computeJacobians` (the same call
/// and the same values Aligator's path uses); only the products change.
/// The Hessian is the Gauss–Newton one, as Aligator's with `gauss_newton`.
///
/// ## When the structure is used
///
/// Decided once at construction: the residual is Aligator's state (control)
/// error on a `MultibodyPhaseSpace` with the root above (a
/// `VectorSpaceTpl`), the weight matrix is square, of the residual's size and
/// exactly diagonal, and `gauss_newton` is set. Anything else — another root,
/// a joint with `nq ≠ nv` or a non-Euclidean configuration (spherical,
/// planar, unbounded revolute, composite, mimic …), a fixed base, an
/// off-diagonal weight — and every call is Aligator's own (`Base::`), with
/// the same results bit for bit. `gauss_newton` is also read at every call: cleared
/// later, the call is Aligator's. `weights_` stays public as in Aligator: the
/// structured path reads its diagonal at every call, so a new diagonal is
/// used, but an off-diagonal entry written after construction is ignored.
///
/// ## Non-finite inputs
///
/// The dense products spread a NaN / Inf further (`0 · NaN`): one in `r`
/// reaches every gradient entry (through `W r` and `Jᵀ`), one in `J_b` the
/// root's whole Hessian rows and columns. The structured path keeps it
/// local: a non-finite `r_k` stays in gradient entry `k` (in the root's six
/// gradient rows for a root row), a non-finite `J_b` in the root's gradient
/// rows and 6 × 6 Hessian block. The value (`evaluate`, Aligator's) is
/// unchanged, so a finite check downstream still sees a non-finite cost.
///
/// ## The zero entries of `grad_` and `hess_`
///
/// `CostStackTpl` adds every component's `grad_` and `hess_` densely, and
/// only the component writes them. A structured call writes only the
/// structural entries (`grad_x`'s rows, the free flyer's 6 × 6 block and the
/// diagonal for the state cost; `grad_u` and `H_uu`'s diagonal for the
/// control cost); the rest must be 0. `CostDataAbstractTpl` zeroes both
/// buffers at construction, and this cost's data (`createData`) records that
/// they hold the pattern. A call that falls back to Aligator's path writes
/// the whole buffer (with other entries, without `gauss_newton`), so it
/// clears the record and the next structured call zeroes the buffer once
/// before writing. No structured call sets the whole `(ndx + nu)²` buffer.
///
/// ## Threads and real time
///
/// Construction and `createData` allocate (configure time). `evaluate`,
/// `computeGradients` and `computeHessians` allocate nothing of their own
/// (the state cost's residual `computeJacobians` — pinocchio's `dDifference`
/// — is the call Aligator's path makes; the control cost makes none). A
/// fallback is decided per call: clearing `gauss_newton` between a
/// `computeGradients` and its `computeHessians` is not supported. Const; all
/// state is in the data.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#pragma GCC diagnostic pop

#include "rtc_urdf_bridge/free_flyer_dims.hpp"

#include <memory>

namespace rtc::mpc::internal {

/// @brief Aligator's composite cost data plus whether the entries a
///        structured call does not write are still 0.
struct StructuredQuadCostData final : aligator::CompositeCostDataTpl<double> {
  using Base = aligator::CompositeCostDataTpl<double>;
  using Base::Base;

  bool grad_pattern_clean{true};  ///< every non-structural `grad_` entry is 0
  bool hess_pattern_clean{true};  ///< every non-structural `hess_` entry is 0
};

/// @return true when @p model's tangent-space difference Jacobian is
///         `blkdiag(J_root, I)`: the root is a free flyer
///         (`rtc_urdf_bridge::IsFreeFlyerRoot`), and every other joint is a
///         revolute or prismatic joint (any axis) or a translation joint — by
///         joint type, i.e. a vector-space configuration.
[[nodiscard]] bool HasFreeFlyerRootAndEuclideanJoints(const pinocchio::Model& model);

/// @brief `½‖x ⊖ x_ref‖²_W` with the structured derivatives above.
class StructuredQuadStateCost final : public aligator::QuadraticStateCostTpl<double> {
 public:
  using Base = aligator::QuadraticStateCostTpl<double>;
  using CostData = aligator::CostDataAbstractTpl<double>;

  /// The free flyer's tangent block.
  static constexpr int kRootDim = rtc_urdf_bridge::kFreeFlyerNv;

  /// @param state_space  the state manifold (a manifold, or a polymorphic one)
  /// @param n_u          control size
  /// @param target       `x_ref`, a point of @p state_space
  /// @param weights      `W`, `ndx × ndx`
  template <typename StateSpace>
  StructuredQuadStateCost(const StateSpace& state_space, int n_u,
                          const Eigen::Ref<const Eigen::VectorXd>& target,
                          const Eigen::MatrixXd& weights)
      : Base(state_space, n_u, target, weights) {
    DetectStructure();
  }

  void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                        CostData& data) const override;
  void computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                       CostData& data) const override;
  [[nodiscard]] std::shared_ptr<CostData> createData() const override;

  /// @return whether the structure was found at construction (false: every
  ///         call is Aligator's).
  [[nodiscard]] bool structured() const noexcept { return structured_; }

 private:
  /// Sets `structured_` (construction).
  void DetectStructure();

  [[nodiscard]] bool UseStructure() const noexcept { return structured_ && this->gauss_newton; }

  bool structured_{false};
};

/// @brief `½‖u − u_ref‖²_W` with the structured derivatives above.
class StructuredQuadControlCost final : public aligator::QuadraticControlCostTpl<double> {
 public:
  using Base = aligator::QuadraticControlCostTpl<double>;
  using CostData = aligator::CostDataAbstractTpl<double>;

  /// @param state_space  the state manifold (a manifold, or a polymorphic one)
  /// @param target       `u_ref` (its size is `nu`)
  /// @param weights      `W`, `nu × nu`
  template <typename StateSpace>
  StructuredQuadControlCost(const StateSpace& state_space,
                            const Eigen::Ref<const Eigen::VectorXd>& target,
                            const Eigen::Ref<const Eigen::MatrixXd>& weights)
      : Base(state_space, target, weights) {
    DetectStructure();
  }

  void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                        CostData& data) const override;
  void computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                       CostData& data) const override;
  [[nodiscard]] std::shared_ptr<CostData> createData() const override;

  /// @return whether the structure was found at construction (false: every
  ///         call is Aligator's).
  [[nodiscard]] bool structured() const noexcept { return structured_; }

 private:
  /// Sets `structured_` (construction).
  void DetectStructure();

  [[nodiscard]] bool UseStructure() const noexcept { return structured_ && this->gauss_newton; }

  bool structured_{false};
};

}  // namespace rtc::mpc::internal

#endif  // RTC_MPC_OCP_INTERNAL_STRUCTURED_QUADRATIC_COSTS_HPP_
