// ── Task-space velocity law (#236 S3a) ──────────────────────────────────────
// The desired task-space velocity of a closed-loop inverse-kinematics
// controller:
//
//     task_vel = K_p ⊙ e + ν_ff        (per axis)
//
// and nothing else. The caller owns the model (J, J⁺), the damped-least-squares
// inversion that turns this twist into q̇ = J⁺·task_vel, the null-space secondary
// task, the pose-error DEFINITION, the frame the feedforward is expressed in,
// the trajectory, the mailbox, the E-STOP policy, the integrator and the output
// clamp.
//
// ── Why this is not task_accel_law.hpp ──────────────────────────────────────
// Next door in this same namespace, and NOT the same law. This one is
// VELOCITY-form: K_p is [1/s] and the result is a twist that a pseudoinverse
// maps to joint velocity. `ComputeTaskAcceleration` is ACCELERATION-form (K_p in
// [1/s²], K_d in [1/s]) and its result is fed through Λ to become a force. This
// law has no derivative term at all — CLIK is a pure proportional law plus a
// feedforward twist, because the plant it closes around is an integrator, not a
// second-order system. Merging the two would mean one helper with two gain
// conventions and a dead K_d; they stay separate, for the same reason
// task_accel_law.hpp stays separate from compliance/impedance_law.hpp.
//
// ── Why the pose error is an argument ───────────────────────────────────────
// e arrives already computed (BodyLog6 transported LOCAL → LOCAL_WORLD_ALIGNED,
// via rtc::math::se3) rather than being derived here from two SE3s. That keeps
// this law on Eigen alone — no rtc_math, no pinocchio — and leaves the CHOICE of
// error definition visible at the layer that also owns the frames it is
// expressed in. Same boundary ComputeTaskAcceleration and ComputeImpedanceForce
// draw; the rule is agent_docs/design-principles.md §코어의 형태.
//
// ── Why the feedforward arrives already rotated ─────────────────────────────
// The trajectory sample's twist is expressed in the TRAJECTORY pose's frame, so
// the caller rotates it into the world-aligned frame the Jacobian and the error
// live in — with R_trajectory, deliberately NOT R_current. That transport is a
// FRAME decision, the same category as the error definition, so it stays with
// the trajectory sample it belongs to instead of moving in here. This law never
// sees a rotation matrix and therefore cannot silently apply the wrong one; the
// binding is where that choice is legible, next to the trajectory it samples.
// Hoisting the product out of the accumulation is bitwise inert, and the literal
// oracle in test_task_vel_core.cpp pins that rather than assuming it — it keeps
// the un-hoisted `+= R·ν` form the adapter had before this extraction.
//
// ── Why there is no trajectory here (D-T) ───────────────────────────────────
// ν_ff is a trajectory SAMPLE, not a generator. Which trajectory feeds which law
// — and how many — is a structural decision owned by the integration layer; the
// duration heuristic and *_trajectory_speed are trajectory parameterisation, not
// law gains. See joint_pd_law.hpp for the same boundary.
//
// ── Extraction note (#236 S3a) ──────────────────────────────────────────────
// These two expressions lived inline in ClikController::Compute()
// (59284d14, clik_controller.cpp:418-428 for the six-axis form and :439-443 for
// the translation-only form). They were lifted here with the same operation
// order and the same association, so the migration is bit-for-bit inert;
// TaskVelLaw.MatchesThePreExtractionInlineFormBitwise and
// .MatchesThePreExtractionInlineFormBitwiseTranslationOnly pin that against
// literal copies of the pre-extraction forms, and the CoreDrivenShim* suite pins
// it against the live adapter while the adapter still exists.
//
// One shape DID change: the inline six-axis form accumulated the feedforward
// into the two halves separately (`head<3>() += …; tail<3>() += …`) because each
// half needed its own rotation product. With the transport hoisted to the caller
// (above) there is no longer anything to split, and `task_vel += ν_ff` is the
// same six element-wise adds — no reassociation is possible in an element-wise
// sum, so the collapse is bitwise inert. That is MEASURED rather than asserted:
// the literal oracle keeps the original half-by-half form and the comparison is
// still bit-for-bit.
//
// The two FUNCTIONS are not collapsed into one another, though. Their expression
// shapes genuinely differ — the six-axis form materialises `K_p ⊙ e` and then
// accumulates, the translation-only form is a single sum expression — and the
// bitwise claim is per-shape. Routing a position-only CLIK through the six-axis
// form would also cost it an angular rotation product per tick that it then
// discards, on the RT path.
//
// NOTE (D-S3): the damped pseudoinverse that consumes this twist is deliberately
// NOT here. Migrating it to compliance/differential_ik.hpp is S3b (#258), and it
// is not bit-identical in the current form of that helper — CLIK damps with a
// constant λ while differential_ik uses the σ_min-adaptive law. See the plan's
// §S3 R3.
#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>

namespace rtc::task {

/// @brief Per-axis velocity-form task gains for the six-axis law.
///
/// K_p is [1/s] — NOT the [1/s²] of task::TaskAccelParams and not the [N/m] of
/// compliance::ImpedanceParams. Trivially copyable so a controller can hold it
/// inside a SeqLock'd Gains POD.
struct TaskVelParams {
  std::array<double, 3> kp_pos{{1.0, 1.0, 1.0}};  ///< translation gain [1/s]
  std::array<double, 3> kp_rot{{1.0, 1.0, 1.0}};  ///< rotation gain    [1/s]
};

/// @brief task_vel = K_p ⊙ e + ν_ff, on all six task axes.
///
/// @param k      velocity-form gains, task-axis order [x,y,z, rx,ry,rz]
/// @param e      pose error CURRENT → DESIRED (6) — [linear; angular], the
///               ordering rtc::math::se3::computePoseError and pinocchio::Motion
///               both use
/// @param nu_ff  feedforward task twist from the trajectory sample (6), ALREADY
///               rotated into the same frame as @p e (see the header note)
/// @return desired task twist (6), [linear; angular], in the frame @p e and
///         @p nu_ff share — the twist a pseudoinverse maps to joint velocity.
///
/// @note Sign convention: e runs CURRENT → DESIRED, so +K_p·e drives the tip
///       TOWARD the setpoint (compliance-conventions.md §3.3).
/// @note RT-safe: fixed-size return (stack), no heap, no branch on data, no
///       divide — nothing here for NUM-2 to guard. Six fixed axes, so unlike the
///       joint-space law there is no channel bound to apply: a task twist is
///       always 6D.
[[nodiscard]] inline Eigen::Matrix<double, 6, 1> ComputeTaskVelocity(
    const TaskVelParams& k, const Eigen::Matrix<double, 6, 1>& e,
    const Eigen::Matrix<double, 6, 1>& nu_ff) noexcept {
  Eigen::Matrix<double, 6, 1> kp_vec;
  for (std::size_t i = 0; i < 3; ++i) {
    kp_vec[static_cast<Eigen::Index>(i)] = k.kp_pos[i];
    kp_vec[static_cast<Eigen::Index>(i + 3)] = k.kp_rot[i];
  }

  Eigen::Matrix<double, 6, 1> task_vel = kp_vec.cwiseProduct(e);
  task_vel += nu_ff;
  return task_vel;
}

/// @brief task_vel = K_p ⊙ e + ν_ff on the three TRANSLATION axes only.
///
/// The law a position-only CLIK runs, where the orientation is left to the null
/// space of the translational Jacobian rather than commanded. A separate
/// function rather than an overload or a dimension template: the two forms
/// differ in expression shape, not just in size, and the bitwise claim is
/// per-shape.
///
/// Takes the translation gains DIRECTLY rather than a TaskVelParams, so the
/// signature states what the law reads. Handing it the rotation gains would
/// invite the reader to conclude that tuning `kp_rotation` changes a
/// position-only command; it does not.
///
/// @param kp_pos     translation gains [1/s], axis order [x,y,z]
/// @param e_pos      translation error CURRENT → DESIRED (3)
/// @param nu_ff_lin  feedforward linear velocity (3), ALREADY rotated into the
///                   same frame as @p e_pos
/// @return desired linear velocity (3), in the frame @p e_pos and @p nu_ff_lin
///         share.
///
/// @note RT-safe on the same terms as ComputeTaskVelocity.
[[nodiscard]] inline Eigen::Vector3d ComputeTranslationVelocity(
    const std::array<double, 3>& kp_pos, const Eigen::Vector3d& e_pos,
    const Eigen::Vector3d& nu_ff_lin) noexcept {
  Eigen::Vector3d kp_vec(kp_pos[0], kp_pos[1], kp_pos[2]);
  return kp_vec.cwiseProduct(e_pos) + nu_ff_lin;
}

}  // namespace rtc::task
