// ── Task-space acceleration law (#236 S2a) ──────────────────────────────────
// The desired task-space acceleration of an operational-space controller:
//
//     a_task = K_p·e + K_d·(ν_d − ν) + a_ff        (per axis, all LWA)
//
// and nothing else. The caller owns the model (M, J, h), the task-inertia
// weighting τ = Jᵀ Λ a_task + h, the pose-error DEFINITION, the trajectory, the
// mailbox, the E-STOP policy, the telemetry lanes and the output clamp.
//
// ── Why this is not impedance_law.hpp ───────────────────────────────────────
// The two look alike and are NOT the same law. This one is ACCELERATION-form:
// K_p is [1/s²] and K_d [1/s], and the result is fed through Λ to become a
// force. `compliance::ComputeImpedanceForce` is FORCE-form (K_p in [N/m]) and
// the two differ by exactly that factor of Λ — impedance_law.hpp:46-49 already
// says so from the other side. It also carries an activation ramp α and a task
// dimension m that the OSC has no notion of, while this law carries a
// feedforward acceleration a_ff that impedance has none of. Unifying them would
// mean one helper with two numerical paths; they stay separate.
//
// ── Why the pose error is an argument ───────────────────────────────────────
// e arrives already computed (SplitWorld, via rtc::math::se3::computePoseError)
// rather than being derived here from two SE3s. That keeps this law on Eigen
// alone — no rtc_math, no pinocchio — and leaves the CHOICE of error definition
// visible at the layer that also owns the frames it is expressed in, next to the
// telemetry lane that echoes it. Same boundary ComputeImpedanceForce draws.
//
// ── Why there is no trajectory here (D-T) ───────────────────────────────────
// ν_d and a_ff are trajectory SAMPLES, not a generator. Which trajectory feeds
// which law — and how many — is a structural decision owned by the integration
// layer; the duration heuristic and *_trajectory_speed are trajectory
// parameterisation, not law gains. See joint_pd_law.hpp for the same boundary
// and agent_docs/design-principles.md §코어의 형태 for the rule.
//
// ── Extraction note (#236 S2a) ──────────────────────────────────────────────
// This expression lived inline in OperationalSpaceController::Compute()
// (bf684040, operational_space_controller.cpp:378-399, including the two
// Vector3d error locals). It was lifted here verbatim — same operation order,
// same association, same Eigen expression shape — so the migration is
// bit-for-bit inert; TaskAccelLaw.MatchesThePreExtractionInlineFormBitwise pins
// that against a literal copy of the pre-extraction form and
// TaskAccelLaw.CoreDrivenShimMatchesTheAdapterBitwise pins it against the live
// adapter while the adapter still exists. Preserve the association if you touch
// it: `kp·e + kd·(ν_d−ν) + a_ff` accumulated left-to-right is NOT bitwise the
// same as a regrouped equivalent, and splitting the cwiseProduct into a scalar
// loop can change FMA contraction.
//
// NOTE (D-S2): the Λ/τ/Nᵀ block that consumes a_task is deliberately NOT here.
// Absorbing it into compliance/task_dynamics.hpp is S2b, and it is not
// bit-identical in the current form of that helper — see the plan's §S2 R3.
#pragma once

#include <Eigen/Core>

#include <array>

namespace rtc::task {

/// Per-axis acceleration-form task gains. K_p is [1/s²] and K_d [1/s] — NOT the
/// stiffness/damping of compliance::ImpedanceParams, which differ from these by
/// a factor of Λ. Trivially copyable so a controller can hold it inside a
/// SeqLock'd Gains POD.
struct TaskAccelParams {
  std::array<double, 3> kp_pos{{100.0, 100.0, 100.0}};  ///< translation gain    [1/s²]
  std::array<double, 3> kd_pos{{20.0, 20.0, 20.0}};     ///< translation damping [1/s]
  std::array<double, 3> kp_rot{{50.0, 50.0, 50.0}};     ///< rotation gain       [1/s²]
  std::array<double, 3> kd_rot{{10.0, 10.0, 10.0}};     ///< rotation damping    [1/s]
};

/// a_task = K_p·e + K_d·(ν_d − ν) + a_ff, on all six task axes.
///
/// @param k     acceleration-form gains, task-axis order [x,y,z, rx,ry,rz]
/// @param e     pose error CURRENT → DESIRED (6) — [linear; angular], the
///              ordering rtc::math::se3::computePoseError and pinocchio::Motion
///              both use
/// @param nu    measured task twist ν = J·q̇ (6), same frame as e
/// @param nu_d  desired task twist from the trajectory sample (6)
/// @param a_ff  feedforward task acceleration from the trajectory sample (6)
///
/// Sign convention: e runs CURRENT → DESIRED, so +K_p·e accelerates the tip
/// TOWARD the setpoint (compliance-conventions.md §3.3).
///
/// RT-safe: fixed-size return (stack), no heap, no branch on data, no divide —
/// nothing here for NUM-2 to guard. Six fixed axes, so unlike the joint-space
/// law there is no channel bound to apply: a task twist is always 6D.
[[nodiscard]] inline Eigen::Matrix<double, 6, 1> ComputeTaskAcceleration(
    const TaskAccelParams& k, const Eigen::Matrix<double, 6, 1>& e,
    const Eigen::Matrix<double, 6, 1>& nu, const Eigen::Matrix<double, 6, 1>& nu_d,
    const Eigen::Matrix<double, 6, 1>& a_ff) noexcept {
  const Eigen::Vector3d kp_p(k.kp_pos[0], k.kp_pos[1], k.kp_pos[2]);
  const Eigen::Vector3d kd_p(k.kd_pos[0], k.kd_pos[1], k.kd_pos[2]);
  const Eigen::Vector3d kp_r(k.kp_rot[0], k.kp_rot[1], k.kp_rot[2]);
  const Eigen::Vector3d kd_r(k.kd_rot[0], k.kd_rot[1], k.kd_rot[2]);

  const Eigen::Vector3d pos_err = e.head<3>();
  const Eigen::Vector3d rot_err = e.tail<3>();

  Eigen::Matrix<double, 6, 1> a_task;
  a_task.head<3>() = kp_p.cwiseProduct(pos_err) + kd_p.cwiseProduct(nu_d.head<3>() - nu.head<3>()) +
                     a_ff.head<3>();
  a_task.tail<3>() = kp_r.cwiseProduct(rot_err) + kd_r.cwiseProduct(nu_d.tail<3>() - nu.tail<3>()) +
                     a_ff.tail<3>();
  return a_task;
}

}  // namespace rtc::task
