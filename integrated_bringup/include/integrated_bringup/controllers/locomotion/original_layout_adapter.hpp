#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_ORIGINAL_LAYOUT_ADAPTER_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_ORIGINAL_LAYOUT_ADAPTER_HPP_

/// @file original_layout_adapter.hpp
/// @brief Adapter between the original `wb_humanoid_mpc` (OCS2) state / weight
///        layout and this repo's pinocchio free-flyer layout (W1-F03, #1026).
///
/// ## Layouts
///
/// Original (Euler-angle base, `nj` actuated joints):
/// - `q_o = [p_W(3); θ_ZYX(3); q_j(nj)]`, size `6 + nj`, with
///   `θ_ZYX = (ψ yaw, θ pitch, φ roll)` — yaw FIRST.
/// - `v_o = [v_W(3); θ̇_ZYX(3); q̇_j(nj)]`, size `6 + nj`: world-frame linear
///   velocity of the base origin, then the Euler-angle RATES `(ψ̇, θ̇, φ̇)` (not
///   an angular velocity).
/// - A weight vector over the original state `[q_o; v_o]` has size
///   `2·(6 + nj)`; its names in the original task.info Q diagonal are
///   `p_base_{x,y,z}, theta_base_{z,y,x}, <joints>, v_base_{x,y,z},
///   omega_base_{z,y,x}, <joints>`.
/// - Source: `humanoid_wb_mpc/common/WBAccelMpcRobotModel.h` at the pinned commit
///   `d6a0be77` — "Base linear velocity … in world frame", "Base euler angle
///   derivatives", and "Contains Euler angle derivatives, not angular velocity!"
///   (the `omega_base_*` names in task.info notwithstanding).
///
/// Ours (pinocchio free-flyer, rtc_mpc/model/wb_layout.hpp):
/// - `q = [p_W(3); quaternion xyzw(4); q_j]`, `nq = 7 + nj`.
/// - `v = [v_B(3); ω_B(3); q̇_j]`, `nv = 6 + nj`, linear and angular both in
///   the base LOCAL frame.
/// - State tangent `dx = [δp_B, δθ_B, δq_j, δv_B, δω_B, δq̇_j]`, `ndx = 2·nv`.
///
/// ## Formulas
///
/// Rotation: \f$ R = R_z(\psi) R_y(\theta) R_x(\phi) \f$ (world ← base). It is
/// built by `rtc::math::se3::RpyToRotationZyx(rpy)` with
/// `rpy = (φ, θ, ψ)` — the original triple REVERSED — and read back by
/// `pinocchio::rpy::matrixToRpy`, which returns the same `(r, p, y)` order.
///
/// - \f$ v_B = R^\top v_W \f$, \f$ v_W = R\,v_B \f$.
/// - \f$ \omega_B = E_B(\phi,\theta)\,(\dot\phi, \dot\theta, \dot\psi)^\top \f$
///   with \f$ E_B \f$ = `pinocchio::rpy::computeRpyJacobian(rpy, LOCAL)`
///   \f$ = \begin{bmatrix}1&0&-s_\theta \\ 0&c_\phi&s_\phi c_\theta
///   \\ 0&-s_\phi&c_\phi c_\theta\end{bmatrix} \f$; the inverse direction uses
///   `computeRpyJacobianInverse(rpy, LOCAL)`, singular at
///   \f$ \cos\theta = 0 \f$ (gimbal lock).
/// - Joint positions and velocities are copied unchanged.
///
/// ## CompatibilityDeviation — weight mapping
///
/// @ref OriginalWeightsToTangent maps a diagonal weight by index PERMUTATION
/// only: position and linear velocity keep (x, y, z); the Euler triple
/// `(ψ, θ, φ)` at original offsets (3, 4, 5) lands on tangent (z, y, x) =
/// (5, 4, 3); joints keep their place. The exact transform of a quadratic cost
/// \f$ \tfrac12 \delta x_o^\top W_o\, \delta x_o \f$ is
/// \f$ W = J^\top W_o J \f$ with
/// \f$ J = \mathrm{blkdiag}(R,\ P E_B^{-1},\ I,\ R,\ P E_B^{-1},\ I) \f$
/// (\f$ P \f$ the (r, p, y) → (y, p, r) reversal; linearised at zero base
/// velocity — at non-zero velocity \f$ \partial v_o / \partial \delta q \f$
/// adds off-diagonal blocks). That matrix equals the permuted diagonal only
/// when
/// - roll = pitch = 0, which makes \f$ E_B = I \f$ (angular blocks exact for
///   any yaw and any weights), and
/// - for the two linear blocks additionally \f$ w_x = w_y \f$ or
///   \f$ \sin\psi = 0 \f$, since \f$ R_z^\top \mathrm{diag}(w) R_z \f$ is
///   diagonal and unchanged only then.
///
/// Elsewhere a diagonal weight rotates with the frame and the permutation is
/// an approximation. The original task's values (x / y isotropic, yaw weight
/// 0) make no difference near upright, which is where a walking base lives.
/// Anisotropic x / y weights are accepted, not rejected — the deviation is the
/// caller's to accept.
///
/// ## Contract
///
/// Every function is `noexcept` and allocation-free for contiguous inputs (an
/// `Eigen::Ref<const VectorXd>` bound to a strided expression makes Eigen copy it). The state /
/// weight conversions return `false` WITHOUT WRITING any output on a size mismatch, a non-finite
/// input, a non-unit quaternion, gimbal lock, or a negative weight. Outputs are sized by the
/// caller. `v_o` and `v` (and `w_o` and `w`) may be the same buffer; no other overlap is allowed.
/// Not RT-tick code, but safe to call from one.

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup::locomotion {

/// Base entries of one original-layout half (`q_o` or `v_o`): position /
/// linear velocity (3) + Euler angles / Euler rates (3).
inline constexpr int kOriginalBaseDim = 6;

/// Gimbal margin [rad]: @ref FreeFlyerToOriginal rejects
/// `|pitch| ≥ π/2 − kGimbalMargin`. At the margin the entries of
/// \f$ E_B^{-1} \f$ grow as \f$ 1/\cos\theta \approx 10^3 \f$, so a double
/// round-off in ω_B (~1e-16) stays ~1e-13 in the Euler rates; past it the map
/// loses its digits and the original model is itself singular. A walking base
/// never comes near it — this is a guard, not an operating limit.
inline constexpr double kGimbalMargin = 1e-3;

/// @brief Original `(q_o, v_o)` → pinocchio free-flyer `(q, v)`.
///
/// `p_W` copied; quaternion (xyzw, sign fixed to w ≥ 0) of \f$ R = R_z(\psi)R_y(\theta)R_x(\phi)
/// \f$; \f$ v_B = R^\top v_W \f$; \f$ \omega_B = E_B(\phi,\theta)(\dot\phi,\dot\theta,\dot\psi)
/// \f$; joints copied. Total in pitch (no gimbal rejection — the forward map is regular
/// everywhere).
///
/// @param q_o Original configuration `[p_W [m]; (ψ, θ, φ) [rad]; q_j]`,
///            size `6 + nj`.
/// @param v_o Original velocity `[v_W [m/s]; (ψ̇, θ̇, φ̇) [rad/s]; q̇_j]`,
///            size `6 + nj`.
/// @param q   out: `[p_W; quat_xyzw; q_j]`, size `7 + nj`.
/// @param v   out: `[v_B; ω_B; q̇_j]` (base LOCAL), size `6 + nj`.
/// @return false (outputs untouched) when the four sizes do not share one
///         `nj ≥ 0`, or any input entry is non-finite.
[[nodiscard]] bool OriginalToFreeFlyer(const Eigen::Ref<const Eigen::VectorXd>& q_o,
                                       const Eigen::Ref<const Eigen::VectorXd>& v_o,
                                       Eigen::Ref<Eigen::VectorXd> q,
                                       Eigen::Ref<Eigen::VectorXd> v) noexcept;

/// @brief Pinocchio free-flyer `(q, v)` → original `(q_o, v_o)`.
///
/// `(φ, θ, ψ) = matrixToRpy(R)` — principal values, φ, ψ ∈ [−π, π],
/// θ ∈ (−π/2, π/2); a consumer that needs a continuous (unwrapped) yaw, as
/// the original MPC state does, unwraps against its previous sample.
/// \f$ v_W = R v_B \f$;
/// \f$ (\dot\phi,\dot\theta,\dot\psi) = E_B^{-1}(\phi,\theta)\,\omega_B \f$;
/// joints copied.
///
/// @param q   `[p_W; quat_xyzw; q_j]`, size `7 + nj`. The quaternion must pass
///            `rtc::IsUnitQuaternionWxyz` (backends/floating_base_conventions.hpp,
///            `|‖quat‖² − 1| < 1e-3` — that function is the tolerance's single
///            owner, so both floating-base ingress paths agree on what counts
///            as a rotation) and is then renormalised once.
/// @param v   `[v_B; ω_B; q̇_j]` (base LOCAL), size `6 + nj`.
/// @param q_o out: `[p_W; (ψ, θ, φ); q_j]`, size `6 + nj`.
/// @param v_o out: `[v_W; (ψ̇, θ̇, φ̇); q̇_j]`, size `6 + nj`.
/// @return false (outputs untouched) on a size mismatch, a non-finite entry, a
///         non-unit quaternion, or `|θ| ≥ π/2 − kGimbalMargin`.
[[nodiscard]] bool FreeFlyerToOriginal(const Eigen::Ref<const Eigen::VectorXd>& q,
                                       const Eigen::Ref<const Eigen::VectorXd>& v,
                                       Eigen::Ref<Eigen::VectorXd> q_o,
                                       Eigen::Ref<Eigen::VectorXd> v_o) noexcept;

/// @brief Original state-weight diagonal → tangent-weight diagonal, by index
///        permutation (see the CompatibilityDeviation in the file header).
///
/// In each half (offset 0, and `6 + nj` = `nv` for the velocity half):
/// entries 0..2 → 0..2; original (3, 4, 5) = (ψ, θ, φ) → tangent (5, 4, 3);
/// joint entries keep their index.
///
/// @param w_o Original diagonal over `[q_o; v_o]`, size `2·(6 + nj)`,
///            every entry finite and ≥ 0.
/// @param w   out: diagonal over `dx`, size `2·nv` (= `w_o.size()`).
/// @return false (output untouched) when the sizes differ, the size is odd or
///         below `2·kOriginalBaseDim`, or any entry is non-finite or negative.
[[nodiscard]] bool OriginalWeightsToTangent(const Eigen::Ref<const Eigen::VectorXd>& w_o,
                                            Eigen::Ref<Eigen::VectorXd> w) noexcept;

/// @brief Compose a base-frame pose into the world frame.
///
/// \f$ {}^W\!X_T = {}^W\!X_B \cdot {}^B\!X_T \f$, i.e.
/// \f$ p_W = R_{WB}\, p_B + p_{WB} \f$, \f$ R_{WT} = R_{WB} R_{BT} \f$.
///
/// @param base_pose      \f$ {}^W\!X_B \f$: base frame expressed in world [m].
/// @param target_in_base \f$ {}^B\!X_T \f$: target frame expressed in the base
///                       frame [m].
/// @return \f$ {}^W\!X_T \f$: target frame expressed in world.
[[nodiscard]] pinocchio::SE3 WorldFromBase(const pinocchio::SE3& base_pose,
                                           const pinocchio::SE3& target_in_base) noexcept;

}  // namespace integrated_bringup::locomotion

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_ORIGINAL_LAYOUT_ADAPTER_HPP_
