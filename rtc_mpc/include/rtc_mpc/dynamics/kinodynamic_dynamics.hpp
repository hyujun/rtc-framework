#ifndef RTC_MPC_DYNAMICS_KINODYNAMIC_DYNAMICS_HPP_
#define RTC_MPC_DYNAMICS_KINODYNAMIC_DYNAMICS_HPP_

/// @file kinodynamic_dynamics.hpp
/// @brief Floating-base kinodynamic dynamics for the whole-body MPC: builds
///        Aligator's `KinodynamicsFwdDynamics` ODE from a `RobotModelHandler`
///        and switches its contact mode in place.
///
/// ## State, input and output
///
/// - state `x = [q; v]` — `q` the configuration (free-flyer position · unit
///   quaternion `(x, y, z, w)` · joints, size `nq`), `v` the tangent velocity
///   (base twist in the base frame · joint rates, size `nv`)
/// - input `u = [W_0; …; W_{nc−1}; q̈_j]` — one wrench block of `force_size`
///   entries per contact, in the handler's contact order, then the joint
///   accelerations (`nv − 6`). `nu = nc·force_size + nv − 6`, the same for
///   every mode — the same layout as `WbLayout` (`contact_u_offset[k] =
///   k·force_size`, `acc_u_offset = nc·force_size`)
/// - output `ẋ = [v; v̇_b; q̈_j]` (tangent, size `2·nv`)
///
/// ## Wrench convention
///
/// `W_k = [f_k; τ_k]`: force (N) and moment (N·m) in **world axes**, the
/// moment taken about the **origin of contact frame k** (the point the force
/// acts at). A 3-D contact carries `f_k` only. The derivatives use the
/// contact Jacobian in `LOCAL_WORLD_ALIGNED`, which is the same convention.
///
/// ## Base acceleration
///
/// With the centroidal momentum matrix `A = [A_b A_j]` (`A_b` its 6×6 base
/// block), CoM `c`, total mass `m` and gravity `g`,
///
///   v̇_b = A_b⁻¹ ( Σ_{k active} [f_k; (p_k − c)×f_k + τ_k] + [m·g; 0]
///                 − Ȧ v − A_j q̈_j ),
///
/// which is the base block of the rigid-body equation
///
///   M_bb v̇_b = −h_b − M_bj q̈_j + Σ_{k active} J_{k,b}ᵀ W_k
///
/// (`h` the nonlinear effects with gravity, `J_{k,b}` the base columns of
/// contact k's `LOCAL_WORLD_ALIGNED` Jacobian). `A_b` is invertible for any
/// joint configuration — a straight knee is not a singularity of this map.
///
/// ## Contact mode
///
/// A mode is a bitmask: bit k set = contact k active (the `ContactSchedule`
/// convention). An inactive contact's wrench block is ignored: it reaches
/// neither `ẋ` nor the Jacobians (its `Ju` columns are exactly zero). The
/// mode is a plain flag vector on the ODE, so it switches **in place** — no
/// allocation, no size change. Aligator copies a dynamics object into every
/// integrator, stage and problem built from it, so the mode must be written to
/// the copy the stage owns (`OdeOf`), not to the ODE the stage was made from.
///
/// ## Data contract
///
/// The ODE's `dForward` reads what the **same data**'s last `forward` left
/// behind (the inverse of `A_b`, `ẋ`, the active contact placements). After
/// `SetMode`, or for a new `(x, u)`, call `forward` before `dForward`.
///
/// Gravity is taken once, from the model (`model.gravity`), at `MakeOde`.
///
/// Lifetime: `MakeOde` reads the handler's model, so that model must outlive
/// this object; the ODEs it returns own copies of it.
///
/// Threads: `SetMode` writes what `forward` reads — switch modes between
/// solves, on the thread that runs them.

#include "rtc_mpc/model/robot_model_handler.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <cstdint>
#include <optional>
#include <vector>

namespace rtc::mpc {

/// @brief Why `KinodynamicDynamics::Init` refused a handler.
enum class KinodynamicError {
  kNoError = 0,
  kModelNotInitialised,    ///< the handler has not been initialised
  kFixedBase,              ///< the model's root is not a free-flyer
  kNoContacts,             ///< the handler lists no contact frame
  kMixedContactDim,        ///< contacts of different dims (one force size per ODE)
  kUnsupportedContactDim,  ///< a contact dim other than 3 (point) or 6 (wrench);
                           ///< the handler refuses these today
  kLayoutMismatch,         ///< the handler's input layout is not the ODE's; a
                           ///< handler builds its layout so, today, never
  kAligatorFailure,        ///< an Aligator constructor threw (caught)
};

/// @brief Builds and drives the kinodynamic ODE of one floating-base model.
///
/// `Init` and `MakeOde` allocate (configure time). `OdeOf`, `SetMode` and
/// the accessors do not.
class KinodynamicDynamics {
 public:
  using Ode = aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>;
  using OdeData = aligator::dynamics::KinodynamicsFwdDataTpl<double>;
  using Space = aligator::MultibodyPhaseSpace<double>;

  /// @brief Check @p handler and cache what every ODE of it shares. Allocates.
  [[nodiscard]] KinodynamicError Init(const RobotModelHandler& handler) noexcept;

  [[nodiscard]] bool Initialised() const noexcept { return model_ != nullptr; }

  /// @return the ODE for @p mode, or `std::nullopt` before `Init`, for a mode
  ///         naming a contact that does not exist, or if Aligator threw.
  ///         Allocates (copies the model).
  [[nodiscard]] std::optional<Ode> MakeOde(std::uint32_t mode) const noexcept;

  /// @return the kinodynamic ODE inside @p stage's integrator — the copy the
  ///         stage evaluates — or null when the stage's dynamics is not an
  ///         explicit integrator over that ODE. Does not allocate. The
  ///         pointer stays valid as long as the stage does, so resolve it
  ///         once after the problem is built rather than per tick.
  [[nodiscard]] static Ode* OdeOf(aligator::StageModelTpl<double>& stage) noexcept;

  /// @brief Switch @p ode to @p mode in place. Does not allocate.
  /// @return false, leaving @p ode untouched, when @p mode names a contact
  ///         that does not exist or @p ode was not built for this object's
  ///         contacts (other frames, count or force size).
  [[nodiscard]] bool SetMode(Ode& ode, std::uint32_t mode) const noexcept;

  /// @return the phase space `[q; v]` of the model, or null before a
  ///         successful `Init`.
  [[nodiscard]] const Space* space() const noexcept {
    return space_.has_value() ? &*space_ : nullptr;
  }

  [[nodiscard]] int n_contacts() const noexcept { return static_cast<int>(contact_ids_.size()); }

  /// @return entries per contact wrench block: 3 or 6.
  [[nodiscard]] int force_size() const noexcept { return force_size_; }

  [[nodiscard]] int nu() const noexcept { return nu_; }

  /// @return `2^n_contacts`: modes are `0 … n_modes() − 1`.
  [[nodiscard]] std::uint32_t n_modes() const noexcept { return n_modes_; }

 private:
  /// @brief Write @p mode's bits into @p states (sized `n_contacts`).
  void FillMask(std::uint32_t mode, std::vector<bool>& states) const noexcept;

  const pinocchio::Model* model_{nullptr};  // non-owning; the handler's
  std::optional<Space> space_{};
  std::vector<pinocchio::FrameIndex> contact_ids_{};
  int force_size_{0};
  int nu_{0};
  std::uint32_t n_modes_{0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_DYNAMICS_KINODYNAMIC_DYNAMICS_HPP_
