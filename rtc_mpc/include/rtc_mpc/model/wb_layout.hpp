#ifndef RTC_MPC_MODEL_WB_LAYOUT_HPP_
#define RTC_MPC_MODEL_WB_LAYOUT_HPP_

/// @file wb_layout.hpp
/// @brief Whole-body state / input layout: where the base, the joints, the
///        contact wrenches and the joint accelerations live inside the
///        vectors `rtc_mpc` exchanges, for fixed-base and floating-base
///        models alike. The one place where these conventions are written
///        down; everything else composes and splits through it.
///
/// Conventions (pinocchio):
/// - Configuration `q = [base_pose(n_base_q); q_j]`. A floating base is the
///   free-flyer `[p_xyz, quaternion_xyzw]`; a fixed base contributes nothing.
/// - Tangent `v = [base_twist(n_base_v); v_j]`. The base twist is expressed
///   in the base **LOCAL** frame (linear, then angular) — pinocchio's joint
///   velocity convention for a free-flyer, so `v.head(6)` is exactly the
///   root joint's spatial velocity in LOCAL.
/// - State tangent `dx = [δq(nv); δv(nv)]`, `ndx = 2·nv`, with
///   `δq = pinocchio::difference(q_ref, q_cur)` (the tangent step that moves
///   the reference onto the current configuration) and `δv = v_cur − v_ref`.
///   For the free-flyer block this is the SE(3) log of `M_ref⁻¹·M_cur`, a
///   vector in the **reference** body's LOCAL frame (its translation part is
///   `R_refᵀ(p_cur − p_ref)` plus a rotation coupling, not a world
///   displacement), and `δv` subtracts the two LOCAL twists as they are —
///   the same `[difference; v_cur − v_ref]` the solver's multibody phase
///   space uses, so a gain linearised there applies as `u = u_ff + K·dx`
///   with this sign. On a fixed-base model `δq = q_cur − q_ref` exactly.
///   Near a relative rotation of π the log is discontinuous (the axis sign
///   flips); a consumer that can see such errors must cap or fade the
///   feedback itself.
/// - Kinodynamic input `u = [W_1 … W_nc; q̈_j]`: contact wrenches first (each
///   contact its own dimension, model-handler frame order, world axes acting
///   at the contact frame origin), then the actuated joint accelerations.
///   `acc_u_offset = Σ contact_dim`, `n_acc = nv − n_base_v`. A fixed-base
///   torque model has `nu = nv` and no wrench block.
///
/// All member functions are allocation-free, `noexcept`, and return `false`
/// without writing when a size does not match or the layout is not
/// @ref valid (no exception can escape). Inputs and outputs of one call must
/// not partially overlap — identical buffers are fine, shifted windows of one
/// buffer are not (plain Eigen assignment, no temporaries).

#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/fwd.hpp>
#pragma GCC diagnostic pop

#include <array>

namespace rtc::mpc {

class RobotModelHandler;

/// @return true when the model's root (joint 1; joint 0 is the universe) is
///         a `JointModelFreeFlyer`. A model-level fact, independent of any
///         YAML. The whole-body QP side decides the same question from joint
///         dimensions and a YAML override; aligning the two is the floating
///         base work of that package.
[[nodiscard]] bool IsFloatingBaseModel(const pinocchio::Model& model) noexcept;

/// Most contact frames a layout can describe (each contact has at least a
/// point-contact's dimension, so this bounds the per-contact tables).
inline constexpr int kMaxContacts = kMaxContactVars / 3;

struct WbLayout {
  // ── Dimensions ──────────────────────────────────────────────────────
  int nq{0};
  int nv{0};
  int ndx{0};  ///< `2·nv`
  int nu{0};   ///< `Σ contact_dim + nv − n_base_v` (kinodynamic) or `nv` (fixed-base torque)
  int n_base_q{0};  ///< free-flyer configuration entries, or 0
  int n_base_v{0};  ///< free-flyer tangent entries, or 0

  // ── Contact / input blocks ──────────────────────────────────────────
  int n_contacts{0};  ///< 0 when only the total is known (see FromModel)
  std::array<int, kMaxContacts> contact_dim{};
  std::array<int, kMaxContacts> contact_u_offset{};
  int n_contact_vars{0};  ///< `Σ contact_dim` — width of the contact-force (λ) reference
  int n_wrench_u{0};      ///< wrench rows at the front of `u`: `n_contact_vars` on a
                          ///< floating base, 0 on a fixed torque base (no wrench block)
  int acc_u_offset{0};    ///< first joint-acceleration row of `u` (`== n_wrench_u`)
  int n_acc{0};           ///< `nv − n_base_v`

  [[nodiscard]] bool floating_base() const noexcept { return n_base_v > 0; }

  /// @return false for the layout a failed factory returns (every dim 0).
  [[nodiscard]] bool valid() const noexcept { return nq > 0 && nv > 0; }

  // ── Factories ───────────────────────────────────────────────────────

  /// @brief Fixed-base torque layout (`nu = nv`, no base, no wrench block);
  ///        the contact total is carried for the interpolator's λ reference.
  [[nodiscard]] static WbLayout FixedBase(int nq, int nv, int n_contact_vars) noexcept;

  /// @brief The layout @p handler computed at its Init (per-contact dims
  ///        from its YAML; floating when the model's root is a free-flyer).
  ///        Empty for an uninitialised handler.
  [[nodiscard]] static WbLayout FromHandler(const RobotModelHandler& handler) noexcept;

  /// @brief Layout of @p model with only the contact **total** known (no
  ///        per-contact split, `n_contacts == 0`). Enough for a consumer
  ///        that sizes its buffers and asks whether the base floats.
  [[nodiscard]] static WbLayout FromModel(const pinocchio::Model& model,
                                          int n_contact_vars) noexcept;

  // ── Compose / split (sizes must match exactly) ──────────────────────
  [[nodiscard]] bool ComposeQ(const Eigen::Ref<const Eigen::VectorXd>& base_pose,
                              const Eigen::Ref<const Eigen::VectorXd>& q_j,
                              Eigen::Ref<Eigen::VectorXd> q) const noexcept;
  [[nodiscard]] bool SplitQ(const Eigen::Ref<const Eigen::VectorXd>& q,
                            Eigen::Ref<Eigen::VectorXd> base_pose,
                            Eigen::Ref<Eigen::VectorXd> q_j) const noexcept;
  [[nodiscard]] bool ComposeV(const Eigen::Ref<const Eigen::VectorXd>& base_twist,
                              const Eigen::Ref<const Eigen::VectorXd>& v_j,
                              Eigen::Ref<Eigen::VectorXd> v) const noexcept;
  [[nodiscard]] bool SplitV(const Eigen::Ref<const Eigen::VectorXd>& v,
                            Eigen::Ref<Eigen::VectorXd> base_twist,
                            Eigen::Ref<Eigen::VectorXd> v_j) const noexcept;
  /// `wrench` has `n_wrench_u` entries (none on a fixed base), `qdd_j` has `n_acc`.
  [[nodiscard]] bool ComposeU(const Eigen::Ref<const Eigen::VectorXd>& wrench,
                              const Eigen::Ref<const Eigen::VectorXd>& qdd_j,
                              Eigen::Ref<Eigen::VectorXd> u) const noexcept;
  [[nodiscard]] bool SplitU(const Eigen::Ref<const Eigen::VectorXd>& u,
                            Eigen::Ref<Eigen::VectorXd> wrench,
                            Eigen::Ref<Eigen::VectorXd> qdd_j) const noexcept;

  /// @brief State-tangent difference `dx = [difference(q_ref, q_cur); v_cur − v_ref]`.
  /// @param model  the model this layout was built from (`model.nv == nv`)
  /// @param dx     output, size `ndx`
  /// @return false when any size disagrees with the model or this layout
  ///         (nothing written — checked before pinocchio is called, so no
  ///         exception can reach this `noexcept` boundary), when a free-flyer
  ///         quaternion is not unit length, or when the result is not finite
  ///         (then `dx` is zeroed).
  [[nodiscard]] bool StateDifference(const pinocchio::Model& model,
                                     const Eigen::Ref<const Eigen::VectorXd>& q_ref,
                                     const Eigen::Ref<const Eigen::VectorXd>& v_ref,
                                     const Eigen::Ref<const Eigen::VectorXd>& q_cur,
                                     const Eigen::Ref<const Eigen::VectorXd>& v_cur,
                                     Eigen::Ref<Eigen::VectorXd> dx) const noexcept;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_MODEL_WB_LAYOUT_HPP_
