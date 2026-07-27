// ── Golden-vector harness for the task-space velocity core (#236 S3a) ────────
// Third slice of the rtc_controllers pure-library refactor. Same contract S1
// established and S2a followed (test_joint_pd_core.cpp, test_task_accel_core.cpp):
// the claim under test is not "the new core is correct" but the stronger "the
// extraction changed NOTHING", so every comparison is bitwise
// (rtc::testing::BitsEqual), never a tolerance.
//
// Two oracles, deliberately:
//
//   1. PreExtractionInlineForm6 / …3 — literal, rename-only copies of
//      clik_controller.cpp:418-428 and :439-443 at 59284d14. DURABLE: they
//      outlive the adapter, so they still pin the core after S7 deletes
//      ClikController. They keep the UN-HOISTED `+= R·ν` form the adapter had —
//      and with it the half-by-half accumulation into head<3>()/tail<3>() —
//      which is what makes them pin the two S3a shape decisions instead of
//      merely arguing them: the caller now rotates the feedforward and hands the
//      core a world-aligned twist, and the core adds that twist as one
//      whole-vector `+= ν_ff`. Both are claimed to be bitwise inert; these
//      oracles are where that is measured.
//
//   2. The live ClikController. TRANSIENT (dies with S7). Its job is the one
//      thing oracle 1 structurally cannot do: catch a CORRELATED transcription
//      error, plus the WIRING — which doubles reach the core as e and ν_ff, and
//      in which of the four (control_6dof × enable_null_space) modes.
//
// Unlike the OSC, CLIK's output IS the core's output pushed through J⁺: the
// adapter publishes target_velocities (= scattered q̇), trajectory_velocities
// (= the feedforward-only lane) and commands (= the integrated q_des), and
// exposes position_error() as a window onto the core's input. ClikShim below
// therefore replicates the model, the trajectory (D-T: the law takes a
// trajectory SAMPLE, so ownership sits in the binding), the π-rotation split,
// the segment transition, the damped pseudoinverse and the null-space task —
// everything S3b and S7 still have to migrate — and compares all three lanes.
// That makes this shim a prototype of the S7 binding, which is what plan §S3 R5
// asked for.
//
// The shim reads J from a SECOND, independently built RtModelHandle. That
// assumption is measured, not assumed — in two places, because the two files
// exercise different halves of the handle:
//
//   • The DYNAMICS half (ComputeJacobians → GetFrameJacobian / GetFramePlacement,
//     plus M and h) is pinned bit for bit by
//     ReferenceDynamics.IndependentHandlesAgreeBitwise in
//     test_task_accel_core.cpp, on this same serial_7dof.urdf fixture. Per plan
//     §S3 R6 this slice REUSES that measurement rather than repeating it.
//
//   • The REORDER half is NOT covered there — that test never calls Reorder* —
//     and this shim does: it gathers the null-space posture error with
//     ReorderInput and scatters both q̇ lanes with ReorderOutput. So it is
//     measured here, in the file that depends on it, by
//     ReferenceHandles.IndependentHandlesAgreeOnTheReorderPathBitwise.
//
// If the fixture or the handle path ever diverges from either measurement, the
// premise has to be re-measured rather than re-tuned.
//
// Every bitwise suite carries a non-vacuity partner
// (BitwiseComparisonRejectsAFusedAccumulation) proving the comparison can
// actually reject an algebraically equivalent regrouping.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/indirect/clik_controller.hpp"
#include "rtc_controllers/joint/posture_law.hpp"
#include "rtc_controllers/task/task_vel_law.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/serial7dof_fixture.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"
#include "rtc_math/se3/velocity_error.hpp"
#include "test_urdf_path.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <vector>

// The allocation gate is TWO complementary sensors, and this core needs both:
//   • rtc::testing::ScopedAllocGate (rtc_controllers/testing/alloc_gate.hpp)
//     counts `operator new`, so it sees a std::vector or a header-inline helper.
//   • rtc::testing::ScopedNoMalloc (rtc_base/testing/no_malloc_scope.hpp) rides
//     eigen_assert, so it sees EIGEN allocation — which the first gate CANNOT:
//     Eigen's internal::aligned_malloc calls std::malloc directly and never
//     routes through operator new. An Eigen-only law's most likely RT-1
//     regression (a parameter or temporary becoming runtime-sized) is therefore
//     invisible to operator-new interposition alone.
// IsAllocationFree arms both.

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;

using rtc::kMaxDeviceChannels;
using rtc::kMaxRobotDOF;
using rtc::task::ComputeTaskVelocity;
using rtc::task::ComputeTranslationVelocity;
using rtc::task::TaskVelParams;
using rtc::testing::BitsEqual;
using rtc::testing::MakeRng;

// The serial_7dof fixture and the measured-state sweep, shared with
// test_task_accel_core.cpp (serial7dof_fixture.hpp) — sharing them is what keeps
// this file's reuse of that file's independent-handle measurement valid, rather
// than dependent on two byte-identical copies staying that way.
using rtc::testing::FillSweep;
using rtc::testing::MakeHandle;
using rtc::testing::MakeState;
using rtc::testing::Serial7dof;

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1 — literal pre-extraction forms
// ═══════════════════════════════════════════════════════════════════════════

// One random tick's worth of core inputs. The trajectory feedforward is kept as
// (R_traj, ν_linear, ν_angular) rather than as a pre-rotated twist, because that
// is what the pre-extraction expression held: the rotation happened INSIDE the
// accumulation. Keeping it is what lets oracle 1 pin the S3a boundary decision.
struct CoreDraw {
  std::array<double, 3> kp_pos{};
  std::array<double, 3> kp_rot{};
  Vec6 task_err{Vec6::Zero()};
  Eigen::Matrix3d R_traj{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d nu_lin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d nu_ang{Eigen::Vector3d::Zero()};
};

// EXACTLY the expression as it stood inline in ClikController::Compute() before
// #236 S3a lifted it into task/task_vel_law.hpp (clik_controller.cpp:418-428 at
// 59284d14). Rename-only transcription:
//   gains.kp_translation → d.kp_pos   gains.kp_rotation → d.kp_rot
//   pos_error_6d_        → d.task_err
//   traj_state_.pose.rotation()     → d.R_traj
//   traj_state_.velocity.linear()   → d.nu_lin
//   traj_state_.velocity.angular()  → d.nu_ang
// Nothing else moved: same operation order, same association, same Eigen
// expression shape — including the rotation product landing directly in the
// += accumulation rather than in a named temporary.
Vec6 PreExtractionInlineForm6(const CoreDraw& d) {
  Eigen::Matrix<double, 6, 1> kp_vec_6d;
  for (std::size_t i = 0; i < 3; ++i) {
    kp_vec_6d[static_cast<Eigen::Index>(i)] = d.kp_pos[i];
    kp_vec_6d[static_cast<Eigen::Index>(i + 3)] = d.kp_rot[i];
  }

  Eigen::Matrix<double, 6, 1> task_vel_6d = kp_vec_6d.cwiseProduct(d.task_err);
  // Feedforward: trajectory local → world-aligned via R_trajectory (not
  // R_current)
  task_vel_6d.head<3>() += d.R_traj * d.nu_lin;
  task_vel_6d.tail<3>() += d.R_traj * d.nu_ang;
  return task_vel_6d;
}

// Likewise for the translation-only branch (clik_controller.cpp:439-443 at
// 59284d14), which is a SINGLE sum expression rather than a materialise-then-
// accumulate — a different shape, hence a separate oracle.
Eigen::Vector3d PreExtractionInlineForm3(const CoreDraw& d) {
  Eigen::Vector3d kp_vec(d.kp_pos[0], d.kp_pos[1], d.kp_pos[2]);
  Eigen::Vector3d task_vel = kp_vec.cwiseProduct(d.task_err.head<3>()) + d.R_traj * d.nu_lin;
  return task_vel;
}

// Drive the core with the same draw — this is the marshalling the binding does,
// including the frame transport that S3a placed on the caller's side.
Vec6 RunCore6(const CoreDraw& d) {
  Eigen::Matrix<double, 6, 1> nu_ff;
  nu_ff.head<3>() = d.R_traj * d.nu_lin;
  nu_ff.tail<3>() = d.R_traj * d.nu_ang;
  return ComputeTaskVelocity(TaskVelParams{d.kp_pos, d.kp_rot}, d.task_err, nu_ff);
}

Eigen::Vector3d RunCore3(const CoreDraw& d) {
  const Eigen::Vector3d nu_ff_lin = d.R_traj * d.nu_lin;
  return ComputeTranslationVelocity(d.kp_pos, Eigen::Vector3d(d.task_err.head<3>()), nu_ff_lin);
}

// Draws sweep the magnitude space the law actually sees: CLIK gains are
// velocity-form and modest, the pose error is small near convergence and O(1)
// after a retarget, and the feedforward can dominate mid-trajectory. Every few
// trials a term is zeroed or made near-cancelling so no axis is permanently
// masked by a larger one — a cancellation that only shows up when the terms are
// comparable is precisely what a regrouping changes. R_traj is a genuine
// rotation, not an arbitrary matrix, so the products stay in the regime the
// controller produces.
CoreDraw RandomDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> gain(0.2, 20.0);
  std::normal_distribution<double> err(0.0, 0.3);
  std::normal_distribution<double> vel(0.0, 0.8);
  std::uniform_real_distribution<double> ang(-M_PI, M_PI);
  std::normal_distribution<double> axis(0.0, 1.0);

  CoreDraw d;
  for (std::size_t i = 0; i < 3; ++i) {
    d.kp_pos[i] = gain(rng);
    d.kp_rot[i] = gain(rng);
  }
  for (int i = 0; i < 6; ++i)
    d.task_err[i] = err(rng);
  for (int i = 0; i < 3; ++i) {
    d.nu_lin[i] = vel(rng);
    d.nu_ang[i] = vel(rng);
  }

  Eigen::Vector3d ax(axis(rng), axis(rng), axis(rng));
  if (ax.norm() < 1e-9)
    ax = Eigen::Vector3d::UnitZ();
  d.R_traj = Eigen::AngleAxisd(ang(rng), ax.normalized()).toRotationMatrix();

  switch (trial % 5) {
    case 1:
      d.task_err.setZero();  // converged — the feedforward alone
      break;
    case 2:
      d.nu_lin.setZero();  // static setpoint on the translation axes
      d.nu_ang.setZero();
      break;
    case 3:
      d.R_traj.setIdentity();  // trajectory frame aligned with the world
      break;
    case 4:
      // Near-cancelling: the feedforward almost exactly opposes the P term, so
      // the sum is a catastrophic subtraction and the ORDER of the surrounding
      // operations matters most. Applied to BOTH halves — ν_ff is R_traj·ν, so
      // pre-rotating by R_trajᵀ makes the transported twist land on −K_p⊙e on
      // all six axes. Doing only the linear half would leave axes 3-5
      // permanently dominated by one term, i.e. exactly the regime where a
      // regrouping of the ANGULAR accumulation is least likely to show up.
      d.nu_lin = -d.R_traj.transpose() * (Eigen::Vector3d(d.kp_pos[0], d.kp_pos[1], d.kp_pos[2])
                                              .cwiseProduct(Eigen::Vector3d(d.task_err.head<3>())));
      d.nu_ang = -d.R_traj.transpose() * (Eigen::Vector3d(d.kp_rot[0], d.kp_rot[1], d.kp_rot[2])
                                              .cwiseProduct(Eigen::Vector3d(d.task_err.tail<3>())));
      break;
    default:
      break;
  }
  return d;
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 — the shim (prototype of the S7 integration-layer binding)
// ═══════════════════════════════════════════════════════════════════════════

// What one tick of the adapter puts on the wire, in device channel order.
struct ShimOutput {
  std::array<double, kMaxDeviceChannels> target_velocities{};
  std::array<double, kMaxDeviceChannels> trajectory_velocities{};
  std::array<double, kMaxDeviceChannels> commands{};
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
};

// Owns the model and the trajectory (STRUCTURE) and calls the core (ALGORITHM).
// Mirrors ClikController::Compute() step for step; any divergence here is a
// divergence the future binding would ship.
class ClikShim {
 public:
  ClikShim() {
    handle_ = MakeHandle(model_);
    tip_ = static_cast<pinocchio::FrameIndex>(model_->nframes - 1);
    nv_ = handle_->nv();

    J_full_ = Eigen::MatrixXd::Zero(6, nv_);
    J_pos_ = Eigen::MatrixXd::Zero(3, nv_);
    dq_ = Eigen::VectorXd::Zero(nv_);
    desired_q_ = Eigen::VectorXd::Zero(nv_);
    traj_dq_ = Eigen::VectorXd::Zero(nv_);
    null_dq_dev_ = Eigen::VectorXd::Zero(nv_);
    qdot_null_ = Eigen::VectorXd::Zero(nv_);
    null_contrib_ = Eigen::VectorXd::Zero(nv_);  // instrumentation only, see peak_null_
    ik_6d_.Resize(nv_, 6);
    ik_3d_.Resize(nv_, 3);
  }

  [[nodiscard]] int nv() const { return nv_; }

  // One tick. `goal` is non-null exactly on the tick the adapter drains a queued
  // target — the caller keeps the two in lock-step, as the adapter's own drain
  // ordering (FK first, trajectory re-init after) requires.
  ShimOutput Step(const rtc::ClikController::Gains& g, const rtc::ControllerState& state,
                  const std::array<double, 6>* goal) {
    const bool use_6dof = g.control_6dof;
    const bool use_null_space = g.enable_null_space;
    const auto& dev0 = state.devices[0];

    // ── Step 1: copy joint state into the q buffer ─────────────────────────
    std::array<double, kMaxDeviceChannels> q_buf{};
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_buf[ui] = dev0.positions[ui];
    }
    std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv_));

    // ── Step 2: FK + Jacobians ─────────────────────────────────────────────
    handle_->ComputeJacobians(q_span);
    handle_->GetFrameJacobian(tip_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
    J_pos_.noalias() = J_full_.topRows(3);

    // ── Step 3: TCP placement ──────────────────────────────────────────────
    const pinocchio::SE3& tcp_pose = handle_->GetFramePlacement(tip_);
    const Eigen::Vector3d tcp = tcp_pose.translation();

    // First-tick self-init (adapter: the !target_initialized_ branch). With no
    // device config the adapter's null_target_init_ is all zeros, so its
    // per-joint reference falls through to the current joint pose.
    if (!initialized_) {
      tcp_target_ = {tcp[0], tcp[1], tcp[2]};
      tcp_target_pose_ = tcp_pose;
      for (int i = 0; i < nv_; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        null_target_[ui] = dev0.positions[ui];
        desired_q_[i] = dev0.positions[ui];
      }
      trajectory_.initialize(tcp_pose, pinocchio::Motion::Zero(), tcp_pose,
                             pinocchio::Motion::Zero(), 0.01);
      time_ = 0.0;
      has_pending_segment_ = false;
      new_target_pending_ = false;
      initialized_ = true;
    }

    // ── SPSC drain (adapter :239-289), reduced to the one entry the test
    //     pushes. The 6-DOF and 3-DOF branches interpret the SAME six doubles
    //     differently — pose+RPY vs TCP position plus null-space references —
    //     and getting that wrong is exactly the wiring error oracle 1 cannot
    //     see. ────────────────────────────────────────────────────────────────
    if (goal != nullptr) {
      if (use_6dof) {
        tcp_target_[0] = (*goal)[0];
        tcp_target_[1] = (*goal)[1];
        tcp_target_[2] = (*goal)[2];
        Eigen::AngleAxisd rollAngle((*goal)[3], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle((*goal)[4], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle((*goal)[5], Eigen::Vector3d::UnitZ());
        const Eigen::Quaternion<double> qrot = yawAngle * pitchAngle * rollAngle;
        tcp_target_pose_.rotation() = qrot.matrix();
        tcp_target_pose_.translation() = Eigen::Vector3d((*goal)[0], (*goal)[1], (*goal)[2]);
      } else {
        const std::size_t pn =
            std::min(std::size_t{6}, static_cast<std::size_t>(nv_));  // num_values = 6
        for (std::size_t i = 0; i < std::min(pn, std::size_t{3}); ++i)
          tcp_target_[i] = (*goal)[i];
        for (std::size_t i = 3; i < pn; ++i)
          null_target_[i] = (*goal)[i];
      }
      new_target_pending_ = true;
    }

    // ── Trajectory re-init on a new target (adapter :295-378) ──────────────
    if (new_target_pending_) {
      pinocchio::SE3 start_pose = tcp_pose;
      pinocchio::SE3 goal_pose;

      if (use_6dof) {
        goal_pose = tcp_target_pose_;
      } else {
        goal_pose = start_pose;  // keep current rotation
        goal_pose.translation() = Eigen::Vector3d(tcp_target_[0], tcp_target_[1], tcp_target_[2]);
      }

      const Eigen::Vector3d start_pos = start_pose.translation();
      const Eigen::Vector3d goal_pos = goal_pose.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      const double T_speed_trans = trans_dist / g.trajectory_speed;
      const double T_vel_trans =
          (g.max_traj_velocity > 0.0) ? (1.875 * trans_dist / g.max_traj_velocity) : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      constexpr double kPiSafetyMargin = 0.15;
      double angular_dist = 0.0;
      Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ();
      bool split_trajectory = false;

      if (use_6dof) {
        const Eigen::AngleAxisd aa(start_pose.rotation().transpose() * goal_pose.rotation());
        angular_dist = aa.angle();
        rot_axis = aa.axis();

        const double T_speed_rot = angular_dist / g.trajectory_angular_speed;
        const double T_vel_rot = (g.max_traj_angular_velocity > 0.0)
                                     ? (1.875 * angular_dist / g.max_traj_angular_velocity)
                                     : 0.0;
        duration = std::max({duration, T_speed_rot, T_vel_rot});

        split_trajectory = (angular_dist > M_PI - kPiSafetyMargin);
      }

      if (split_trajectory) {
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid =
            start_pose.rotation() * Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

        pinocchio::SE3 mid_pose;
        mid_pose.translation() = 0.5 * (start_pos + goal_pos);
        mid_pose.rotation() = R_mid;

        const double half_trans = trans_dist * 0.5;
        const double T1_speed_t = half_trans / g.trajectory_speed;
        const double T1_vel_t =
            (g.max_traj_velocity > 0.0) ? (1.875 * half_trans / g.max_traj_velocity) : 0.0;
        const double T1_speed_r = half_angle / g.trajectory_angular_speed;
        const double T1_vel_r = (g.max_traj_angular_velocity > 0.0)
                                    ? (1.875 * half_angle / g.max_traj_angular_velocity)
                                    : 0.0;
        const double dur1 = std::max({0.01, T1_speed_t, T1_vel_t, T1_speed_r, T1_vel_r});

        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), mid_pose,
                               pinocchio::Motion::Zero(), dur1);
        pending_goal_pose_ = goal_pose;
        pending_duration_ = dur1;
        has_pending_segment_ = true;
        ++splits_;
      } else {
        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), goal_pose,
                               pinocchio::Motion::Zero(), duration);
        has_pending_segment_ = false;
      }

      time_ = 0.0;
      for (int i = 0; i < nv_; ++i)
        desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
      new_target_pending_ = false;
    }

    // Sample, then advance (adapter :380-382) — the order is load-bearing.
    const double dt = state.dt;
    traj_state_ = trajectory_.compute(time_, dt);
    time_ += dt;

    // Segment transition, π-rotation defense (adapter :385-391).
    if (has_pending_segment_ && time_ >= trajectory_.duration()) {
      pinocchio::SE3 mid_pose = trajectory_.compute(trajectory_.duration()).pose;
      trajectory_.initialize(mid_pose, pinocchio::Motion::Zero(), pending_goal_pose_,
                             pinocchio::Motion::Zero(), pending_duration_);
      time_ = 0.0;
      has_pending_segment_ = false;
      ++transitions_;
    }

    // ── Pose error (definition owned by the binding, not the law; adapter
    //     :395-408). BodyLog6 in LOCAL, transported LOCAL → LWA. ─────────────
    const rtc::math::se3::Iso3 tcp_iso = rtc::math::se3::toIso3(tcp_pose);
    const rtc::math::se3::Vec6 e_body = rtc::math::se3::computePoseError(
        tcp_iso, rtc::math::se3::toIso3(traj_state_.pose), rtc::math::se3::ErrorType::BodyLog6);
    pos_error_6d_ = rtc::math::se3::twistLocalToWorld(tcp_iso, e_body);

    const Eigen::Vector3d p_err = pos_error_6d_.head<3>();
    for (int i = 0; i < 3; ++i)
      pos_error_[i] = p_err[i];

    // ── Feedforward frame transport (adapter :411-418): trajectory local →
    //     world-aligned via R_trajectory, computed ONCE and shared by the
    //     primary task and the feedforward-only logging lane. ────────────────
    const Eigen::Matrix3d& R_traj = traj_state_.pose.rotation();
    const Eigen::Vector3d nu_ff_lin = R_traj * traj_state_.velocity.linear();

    // ── Null-space secondary task — 3-DOF mode only. Since #236 S3b the gain
    //     multiplies BEFORE the projection, so the posture term is formed here
    //     and handed to DifferentialIk::Solve rather than scaled after N.
    const auto nvu = static_cast<std::size_t>(nv_);
    const bool nullspace_active = use_null_space && !use_6dof;
    if (nullspace_active) {
      rtc::joint::ComputePostureVelocity(
          g.null_kp,
          rtc::joint::PostureInputs{{null_target_.data(), null_target_.size()},
                                    {dev0.positions.data(), dev0.positions.size()},
                                    {}},
          nvu, {null_dq_dev_.data(), nvu});
      handle_->ReorderInput(std::span<const double>(null_dq_dev_.data(), nvu), qdot_null_);
      ++null_hits_;
    } else {
      qdot_null_.setZero();
    }

    // ── Damped pseudoinverse + THE CORE. Since #236 S3b the DLS is
    //     compliance::DifferentialIk, so this mirrors the adapter's CALLS; the
    //     literal pre-extraction spelling lives in the tier-2 oracle in
    //     test_dls_convergence.cpp. The trailing traj_dq_ line in each branch is
    //     the feedforward-only logging lane — NOT routed through the core: it
    //     has no P term, so it is not this law.
    const double max_damping = std::max(1e-4, g.max_damping);
    if (use_6dof) {
      const rtc::compliance::DifferentialIk::Result r =
          ik_6d_.Compute(J_full_, g.singularity_threshold, max_damping);
      last_ok_ = r.ok;
      last_sigma_min_ = r.sigma_min;
      last_lambda_sq_ = r.lambda_sq;

      Eigen::Matrix<double, 6, 1> nu_ff_6d;
      nu_ff_6d.head<3>() = nu_ff_lin;
      nu_ff_6d.tail<3>() = R_traj * traj_state_.velocity.angular();

      const Eigen::Matrix<double, 6, 1> task_vel_6d = ComputeTaskVelocity(
          TaskVelParams{g.kp_translation, g.kp_rotation}, pos_error_6d_, nu_ff_6d);

      if (r.ok) {
        // Mirrors the adapter: no posture term exists in 6-DOF mode, so the
        // two-argument overload (the pre-migration shape) is what runs.
        ik_6d_.Solve(task_vel_6d, dq_);
        traj_dq_.noalias() = ik_6d_.PseudoInverse() * nu_ff_6d;
      } else {
        dq_.setZero();
        traj_dq_.setZero();
      }
    } else {
      const rtc::compliance::DifferentialIk::Result r =
          ik_3d_.Compute(J_pos_, g.singularity_threshold, max_damping);
      last_ok_ = r.ok;
      last_sigma_min_ = r.sigma_min;
      last_lambda_sq_ = r.lambda_sq;

      const Eigen::Vector3d task_vel =
          ComputeTranslationVelocity(g.kp_translation, pos_error_, nu_ff_lin);

      if (r.ok) {
        ik_3d_.Solve(task_vel, qdot_null_, dq_);
        traj_dq_.noalias() = ik_3d_.PseudoInverse() * nu_ff_lin;
      } else {
        dq_.setZero();
        traj_dq_.setZero();
      }
    }
    last_nullspace_active_ = nullspace_active;
    if (nullspace_active) {
      // Measure the term that actually reaches dq_ — the PROJECTED posture
      // velocity N·(kp·Δq) — not the pre-projection kp·Δq. The vacuity guard
      // this feeds asserts "the branch fired but contributed ~0", and only the
      // projected form can answer that: N annihilates whatever component of Δq
      // lies in the row space of J_pos, so on a fixture where that is all of it
      // the branch fires, contributes exactly nothing, and the pre-projection
      // reading is still large. Before #236 S3b the gain multiplied AFTER the
      // projection, so the adapter's own `null_dq_` WAS the projected term and
      // sampling it was correct; moving the gain in front of N is what quietly
      // turned this line into a different measurement.
      null_contrib_.noalias() = ik_3d_.NullspaceProjector() * qdot_null_;
      peak_null_ = std::max(peak_null_, null_contrib_.cwiseAbs().maxCoeff());
    }

    // ── Scatter, integrate (adapter :476-504). nc0 == nv here, so nq == nc0
    //     and the [nq, nc0) tail policy is not exercised — that tail is binding
    //     territory (G3), out of scope for S3a. The shim does NOT clamp: the
    //     clamp is an integration-layer limit, and the tests assert per channel
    //     that it stayed inert so the bitwise comparison means something. ─────
    ShimOutput out;
    const auto nq = static_cast<std::size_t>(nv_);
    handle_->ReorderOutput(dq_, std::span<double>(out.target_velocities.data(), nq));
    handle_->ReorderOutput(traj_dq_, std::span<double>(out.trajectory_velocities.data(), nq));
    for (std::size_t i = 0; i < nq; ++i) {
      desired_q_[static_cast<Eigen::Index>(i)] += out.target_velocities[i] * dt;
      out.commands[i] = desired_q_[static_cast<Eigen::Index>(i)];
    }
    out.position_error = pos_error_;
    return out;
  }

  // Branch counters — a cross-check that never took the branch it was written
  // for is vacuous with respect to the state machine it exists to pin (S2a: the
  // π-split test was green with splits() == 0 before this instrumentation).
  [[nodiscard]] int splits() const { return splits_; }

  [[nodiscard]] int transitions() const { return transitions_; }

  [[nodiscard]] int null_hits() const { return null_hits_; }

  [[nodiscard]] double peak_null() const { return peak_null_; }

  [[nodiscard]] bool last_ok() const { return last_ok_; }

  [[nodiscard]] bool last_nullspace_active() const { return last_nullspace_active_; }

  [[nodiscard]] double last_sigma_min() const { return last_sigma_min_; }

  [[nodiscard]] double last_lambda_sq() const { return last_lambda_sq_; }

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  int nv_{0};

  Eigen::MatrixXd J_full_, J_pos_;
  Eigen::VectorXd dq_, desired_q_, traj_dq_, null_dq_dev_, qdot_null_;
  Eigen::VectorXd null_contrib_;  ///< N·q̇_null — instrumentation, not part of the mirrored law
  rtc::compliance::DifferentialIk ik_6d_;
  rtc::compliance::DifferentialIk ik_3d_;

  // Gate / diagnostic windows. The CLIK adapter publishes no diagnostics, so
  // these are the only place a gate that is NUMERICALLY INERT when closed can be
  // observed at all (the S5 M4 lesson). Compared bitwise, tick by tick.
  bool last_ok_{false};
  bool last_nullspace_active_{false};
  double last_sigma_min_{0.0};
  double last_lambda_sq_{0.0};

  Eigen::Vector3d pos_error_{Eigen::Vector3d::Zero()};
  Vec6 pos_error_6d_{Vec6::Zero()};

  rtc::trajectory::TaskSpaceTrajectory trajectory_;
  rtc::trajectory::TaskSpaceTrajectory::State traj_state_{};
  double time_{0.0};
  bool initialized_{false};
  bool new_target_pending_{false};
  bool has_pending_segment_{false};
  pinocchio::SE3 tcp_target_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 pending_goal_pose_{pinocchio::SE3::Identity()};
  double pending_duration_{0.0};
  std::array<double, 3> tcp_target_{};
  std::array<double, kMaxRobotDOF> null_target_{};

  int splits_{0};
  int transitions_{0};
  int null_hits_{0};
  double peak_null_{0.0};
};

// A goal pose sitting exactly `angle` radians of rotation away from the TCP
// orientation at `tick`, returned as the [x,y,z, r,p,y] vector SetDeviceTarget
// takes in 6-DOF mode (ZYX Euler, matching the adapter's drain). Deriving the
// target from the fixture instead of hard-coding Euler angles is what makes the
// π-split branch reachable ON PURPOSE — the S2a version of this test used a
// literal yaw and silently took the single-segment path (plan §S3 R6).
std::array<double, 6> GoalRotatedFromTcp(int tick, double dt, double angle,
                                         const Eigen::Vector3d& axis,
                                         const Eigen::Vector3d& translation_offset) {
  std::shared_ptr<const pinocchio::Model> model;
  auto handle = MakeHandle(model);
  const auto tip = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  const int nv = handle->nv();

  auto state = MakeState(nv, dt);
  FillSweep(state, nv, tick, dt);
  std::vector<double> q(static_cast<std::size_t>(nv));
  for (int i = 0; i < nv; ++i)
    q[static_cast<std::size_t>(i)] = state.devices[0].positions[static_cast<std::size_t>(i)];

  handle->ComputeJacobians(q);
  const pinocchio::SE3& tcp = handle->GetFramePlacement(tip);

  const Eigen::Matrix3d R_goal =
      tcp.rotation() * Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(R_goal);
  const Eigen::Vector3d p = tcp.translation() + translation_offset;
  return {p.x(), p.y(), p.z(), rpy[0], rpy[1], rpy[2]};
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Bit-identity against the literal pre-extraction forms
// ═══════════════════════════════════════════════════════════════════════════

TEST(TaskVelLaw, MatchesThePreExtractionInlineFormBitwise) {
  auto rng = MakeRng();
  double peak = 0.0;
  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const Vec6 got = RunCore6(d);
    const Vec6 want = PreExtractionInlineForm6(d);

    for (int i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(got[i], want[i]))
          << "trial " << trial << " axis " << i << ": " << got[i] << " vs " << want[i];
      peak = std::max(peak, std::abs(got[i]));
    }
  }
  EXPECT_GT(peak, 1e-6) << "every compared axis was ~0 — the draw generator degenerated";
}

TEST(TaskVelLaw, MatchesThePreExtractionInlineFormBitwiseTranslationOnly) {
  auto rng = MakeRng();
  double peak = 0.0;
  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const Eigen::Vector3d got = RunCore3(d);
    const Eigen::Vector3d want = PreExtractionInlineForm3(d);

    for (int i = 0; i < 3; ++i) {
      EXPECT_TRUE(BitsEqual(got[i], want[i]))
          << "trial " << trial << " axis " << i << ": " << got[i] << " vs " << want[i];
      peak = std::max(peak, std::abs(got[i]));
    }
  }
  EXPECT_GT(peak, 1e-6) << "every compared axis was ~0 — the draw generator degenerated";
}

// Non-vacuity partner. The obvious "why materialise the rotated feedforward,
// just accumulate it" edit folds the three rotation products into the same
// summation as the P term: ((kp·e + R₀·ν₀) + R₁·ν₁) + R₂·ν₂ instead of
// kp·e + ((R₀·ν₀ + R₁·ν₁) + R₂·ν₂). Algebraically identical, different
// association — and if bit equality could not tell the two apart, the tests
// above would be pinning nothing. This is also what draws the line under the
// S3a boundary decision: hoisting the product into a named temporary is inert
// (the tests above prove it), while folding the accumulation is NOT.
TEST(TaskVelLaw, BitwiseComparisonRejectsAFusedAccumulation) {
  auto rng = MakeRng();
  int differing_axes = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);
    const Vec6 got = RunCore6(d);

    for (int i = 0; i < 6; ++i) {
      const auto row = static_cast<Eigen::Index>(i % 3);
      const Eigen::Vector3d& nu = (i < 3) ? d.nu_lin : d.nu_ang;
      const double kp = (i < 3) ? d.kp_pos[static_cast<std::size_t>(i)]
                                : d.kp_rot[static_cast<std::size_t>(i - 3)];

      double fused = kp * d.task_err[i];
      fused += d.R_traj(row, 0) * nu[0];
      fused += d.R_traj(row, 1) * nu[1];
      fused += d.R_traj(row, 2) * nu[2];

      ++compared;
      if (!BitsEqual(got[i], fused))
        ++differing_axes;
      EXPECT_NEAR(got[i], fused, 1e-6 * (1.0 + std::abs(got[i])))
          << "the mutant must stay algebraically equivalent, else it proves nothing";
    }
  }
  ASSERT_GT(compared, 0) << "no axes compared — the draw generator degenerated";
  EXPECT_GT(differing_axes, 0)
      << "bit comparison cannot distinguish a fused accumulation — the equivalence test is vacuous";
}

// ═══════════════════════════════════════════════════════════════════════════
// Shim premise — the reorder half of the independent-handle assumption
// ═══════════════════════════════════════════════════════════════════════════

// ReferenceDynamics.IndependentHandlesAgreeBitwise (test_task_accel_core.cpp)
// pins J / M / h / placement across two independently built handles on this same
// fixture, and this slice reuses that (plan §S3 R6). It does NOT touch Reorder*,
// though — and the shim does, on three lanes: the null-space posture error is
// gathered device → Pinocchio with ReorderInput, and both q̇ lanes are scattered
// back with ReorderOutput. A non-identity default order or a lazily built
// permutation would change the shim's basis while that test stayed green, so the
// missing half is measured HERE rather than assumed.
TEST(ReferenceHandles, IndependentHandlesAgreeOnTheReorderPathBitwise) {
  std::shared_ptr<const pinocchio::Model> model_a;
  std::shared_ptr<const pinocchio::Model> model_b;
  auto a = MakeHandle(model_a);
  auto b = MakeHandle(model_b);
  ASSERT_EQ(a->nv(), b->nv());
  const int nv = a->nv();

  // MakeHandle never calls SetJointOrder, so both handles must be in the
  // memcpy-fallback (identity) regime the adapter's "Identity order → memcpy"
  // comment and the shim's device-order arithmetic both assume.
  ASSERT_FALSE(a->HasJointReorder()) << "fixture handle has a joint reorder map — the shim's "
                                        "device-order posture error would need re-deriving";
  ASSERT_FALSE(b->HasJointReorder()) << "second handle disagrees with the first on reorder state";

  auto rng = MakeRng();
  std::normal_distribution<double> draw(0.0, 0.7);

  double peak = 0.0;
  for (int trial = 0; trial < 16; ++trial) {
    std::vector<double> ext(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i)
      ext[static_cast<std::size_t>(i)] = draw(rng);

    Eigen::VectorXd pin_a = Eigen::VectorXd::Zero(nv);
    Eigen::VectorXd pin_b = Eigen::VectorXd::Zero(nv);
    a->ReorderInput(std::span<const double>(ext), pin_a);
    b->ReorderInput(std::span<const double>(ext), pin_b);

    std::vector<double> out_a(static_cast<std::size_t>(nv));
    std::vector<double> out_b(static_cast<std::size_t>(nv));
    a->ReorderOutput(pin_a, std::span<double>(out_a));
    b->ReorderOutput(pin_b, std::span<double>(out_b));

    for (int i = 0; i < nv; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      ASSERT_TRUE(BitsEqual(pin_a[i], pin_b[i])) << "trial " << trial << " ReorderInput idx " << i;
      ASSERT_TRUE(BitsEqual(out_a[ui], out_b[ui]))
          << "trial " << trial << " ReorderOutput idx " << i;
      // Round-trip: scatter∘gather is the identity, so the shim's device-order
      // formation and the adapter's agree on more than just "both handles do the
      // same thing".
      ASSERT_TRUE(BitsEqual(out_a[ui], ext[ui])) << "reorder round-trip lost bits at idx " << i;
      peak = std::max(peak, std::abs(ext[ui]));
    }
  }
  EXPECT_GT(peak, 1e-6) << "every reordered value was ~0 — the draw generator degenerated";
}

// ═══════════════════════════════════════════════════════════════════════════
// Cross-check against the live adapter (transient — dies with S7)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

struct CrossCheckStats {
  double peak_vel{0.0};
  double peak_traj_vel{0.0};
  double peak_err{0.0};
  double peak_cmd_delta{0.0};
};

// Drive adapter and shim through the same tick sequence and compare every
// device-0 lane bit for bit.
void RunCrossCheck(rtc::ClikController& ctrl, ClikShim& shim, const rtc::ClikController::Gains& g,
                   const std::array<double, 6>& goal, int retarget_tick, int ticks, double dt,
                   CrossCheckStats* stats) {
  const int nv = shim.nv();
  auto state = MakeState(nv, dt);
  std::array<double, kMaxDeviceChannels> first_cmd{};

  for (int tick = 0; tick < ticks; ++tick) {
    FillSweep(state, nv, tick, dt);

    if (tick == retarget_tick)
      ctrl.SetDeviceTarget(0, std::span<const double>(goal));

    const auto out = ctrl.Compute(state);
    const ShimOutput shim_out = shim.Step(g, state, tick == retarget_tick ? &goal : nullptr);

    for (int j = 0; j < nv; ++j) {
      const auto uj = static_cast<std::size_t>(j);
      const double adapter_vel = out.devices[0].target_velocities[uj];

      // The adapter clamps the joint velocity after the law; the shim does not
      // (limits are an integration-layer concern). Keeping the clamp provably
      // inert is what makes the bitwise comparison meaningful — assert it,
      // don't assume it. kDefaultMaxJointVelocity is a narrow 2.0 rad/s.
      ASSERT_LT(std::abs(adapter_vel), rtc::kDefaultMaxJointVelocity)
          << "velocity clamp fired at tick " << tick << " ch " << j
          << " — comparison would be invalid";

      EXPECT_TRUE(BitsEqual(adapter_vel, shim_out.target_velocities[uj]))
          << "tick " << tick << " ch " << j << " target_velocity: adapter " << adapter_vel
          << " vs shim " << shim_out.target_velocities[uj];
      EXPECT_TRUE(
          BitsEqual(out.devices[0].trajectory_velocities[uj], shim_out.trajectory_velocities[uj]))
          << "tick " << tick << " ch " << j << " trajectory_velocity";
      EXPECT_TRUE(BitsEqual(out.devices[0].commands[uj], shim_out.commands[uj]))
          << "tick " << tick << " ch " << j << " command (integrated q_des)";

      stats->peak_vel = std::max(stats->peak_vel, std::abs(adapter_vel));
      stats->peak_traj_vel =
          std::max(stats->peak_traj_vel, std::abs(out.devices[0].trajectory_velocities[uj]));

      // The integrator must actually move, or `commands` would be pinning the
      // seed rather than the law that drives it.
      if (tick == 0)
        first_cmd[uj] = out.devices[0].commands[uj];
      else
        stats->peak_cmd_delta =
            std::max(stats->peak_cmd_delta, std::abs(out.devices[0].commands[uj] - first_cmd[uj]));
    }

    // The adapter's window onto the core's input: the cached 3-D pose error,
    // which is the head of the 6-D error in BOTH modes. Pinning it bitwise
    // localises a cross-check failure to either the error (binding) or the law
    // (core) instead of leaving the whole tick suspect.
    const auto adapter_err = ctrl.position_error();
    for (int i = 0; i < 3; ++i) {
      EXPECT_TRUE(BitsEqual(adapter_err[static_cast<std::size_t>(i)], shim_out.position_error[i]))
          << "tick " << tick << " pose-error axis " << i;
      stats->peak_err =
          std::max(stats->peak_err, std::abs(adapter_err[static_cast<std::size_t>(i)]));
    }
  }
}

rtc::ClikController::Gains CrossCheckGains(bool use_6dof, bool use_null_space) {
  rtc::ClikController::Gains g;
  // Six PAIRWISE-DISTINCT gains, not two isotropic blocks. Oracle 2 exists to
  // catch WIRING errors, and a permutation inside one block — kp_translation
  // marshalled as {y,x,z} into TaskVelParams — is bit-for-bit invisible when all
  // three values are equal. Kept at or below the old 2.0 / 1.5 maxima so the
  // adapter's kDefaultMaxJointVelocity clamp stays as inert as before (the
  // per-channel ASSERT_LT in RunCrossCheck still proves it, never assumes it).
  g.kp_translation = {{2.0, 1.4, 1.7}};
  g.kp_rotation = {{1.5, 0.9, 1.2}};
  g.max_damping = 0.05;
  g.singularity_threshold = 0.02;
  g.null_kp = 0.5;
  g.enable_null_space = use_null_space;
  g.control_6dof = use_6dof;
  g.trajectory_speed = 0.05;
  g.trajectory_angular_speed = 0.25;
  return g;
}

// The four (control_6dof × enable_null_space) combinations the adapter can run.
// The null-space task is gated on `use_null_space && !use_6dof`, so the two
// 6-DOF rows must leave it dormant — an asymmetry the shim would be free to get
// wrong if the sweep only ever ran three of the four (plan §S3 R7).
void RunModeCombination(bool use_6dof, bool use_null_space, const std::array<double, 6>& goal) {
  const auto g = CrossCheckGains(use_6dof, use_null_space);

  rtc::ClikController ctrl(Serial7dof(), g);
  ctrl.OnDeviceConfigsSet();  // populates max_joint_velocity_ = kDefaultMaxJointVelocity
  ctrl.set_gains(g);

  ClikShim shim;
  ASSERT_EQ(shim.nv(), 7) << "fixture must be redundant (nv > 6) for a non-trivial null space";

  CrossCheckStats stats;
  RunCrossCheck(ctrl, shim, g, goal, /*retarget_tick=*/3, /*ticks=*/40, /*dt=*/0.002, &stats);

  EXPECT_GT(stats.peak_vel, 1e-4) << "every compared velocity was ~0 — the cross-check is vacuous";
  EXPECT_GT(stats.peak_traj_vel, 1e-6)
      << "the feedforward lane never moved — trajectory_velocities pins nothing";
  EXPECT_GT(stats.peak_err, 1e-5)
      << "the pose error never left zero — the K_p branch was not exercised";
  EXPECT_GT(stats.peak_cmd_delta, 1e-6) << "the integrator never moved — commands pins only a seed";

  if (use_null_space && !use_6dof) {
    EXPECT_GT(shim.null_hits(), 0) << "the null-space branch never fired";
    EXPECT_GT(shim.peak_null(), 1e-6)
        << "the null-space branch fired but contributed ~0 — it pins nothing";
  } else {
    EXPECT_EQ(shim.null_hits(), 0)
        << "the null-space task ran outside 3-DOF mode — the adapter gates it on "
           "(enable_null_space && !control_6dof)";
  }
}

// 6-DOF mode reads all six doubles as [x,y,z, roll,pitch,yaw]; 3-DOF mode reads
// the first three as the TCP position and [3..6) as null-space joint references,
// so the two modes get goals that are sensible under their own interpretation.
std::array<double, 6> Goal6dof() {
  return {0.30, 0.10, 0.45, 0.20, -0.15, 0.35};
}

std::array<double, 6> Goal3dof() {
  return {0.30, 0.10, 0.45, 0.25, 0.20, 0.30};
}

}  // namespace

TEST(TaskVelLaw, CoreDrivenShimMatchesTheAdapterBitwise6dofWithNullSpaceFlag) {
  RunModeCombination(/*use_6dof=*/true, /*use_null_space=*/true, Goal6dof());
}

TEST(TaskVelLaw, CoreDrivenShimMatchesTheAdapterBitwise6dofWithoutNullSpace) {
  RunModeCombination(/*use_6dof=*/true, /*use_null_space=*/false, Goal6dof());
}

TEST(TaskVelLaw, CoreDrivenShimMatchesTheAdapterBitwise3dofWithNullSpace) {
  RunModeCombination(/*use_6dof=*/false, /*use_null_space=*/true, Goal3dof());
}

TEST(TaskVelLaw, CoreDrivenShimMatchesTheAdapterBitwise3dofWithoutNullSpace) {
  RunModeCombination(/*use_6dof=*/false, /*use_null_space=*/false, Goal3dof());
}

// A near-π retarget takes the split branch, so the shim must reproduce the
// two-segment state machine (has_pending_segment_ / pending_duration_) as well
// as the law. 6-DOF only: the adapter computes an angular distance — and hence
// can split — solely in that mode.
TEST(TaskVelLaw, CoreDrivenShimMatchesTheAdapterAcrossAPiRotationSplit) {
  auto g = CrossCheckGains(/*use_6dof=*/true, /*use_null_space=*/false);
  // Softer proportional gains than the straight-retarget cases, and the DEFAULT
  // angular trajectory limits rather than raised ones. Both are forced by the
  // narrow kDefaultMaxJointVelocity = 2.0 rad/s clamp: a near-π rotation drives
  // the pose error toward π (the fixture's measured state sweeps independently
  // of the command, so the error is never worked off), and speeding the segment
  // up to shorten the test raises the feedforward peak as 1.875·Δθ/T. Either one
  // trips the clamp, and a fired clamp voids the bitwise comparison rather than
  // failing it informatively. The gains are law parameters, so this is a choice
  // of operating point, not a weakened assertion — both sides read the same
  // gains, the comparison is still bitwise on every lane, and the non-vacuity
  // guards below still require the law to have visibly done something.
  g.kp_translation = {{0.15, 0.15, 0.15}};
  g.kp_rotation = {{0.15, 0.15, 0.15}};

  rtc::ClikController ctrl(Serial7dof(), g);
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);

  ClikShim shim;
  constexpr int kRetargetTick = 2;
  // 50 Hz keeps the tick budget for a ~6.2 s first segment manageable; dt only
  // parameterises the trajectory sampler and the integrator.
  constexpr double kDt = 0.02;
  // π − 0.05 clears the adapter's kPiSafetyMargin of 0.15.
  const std::array<double, 6> goal =
      GoalRotatedFromTcp(kRetargetTick, kDt, M_PI - 0.05, Eigen::Vector3d(0.3, -0.5, 0.8),
                         Eigen::Vector3d(0.02, 0.0, -0.01));

  CrossCheckStats stats;
  // Segment 1 covers half of (π − 0.05) at the default 0.25 rad/s, i.e. ~6.2 s
  // ≈ 310 ticks. Long enough to cross its end and run into segment 2.
  RunCrossCheck(ctrl, shim, g, goal, kRetargetTick, /*ticks=*/340, kDt, &stats);

  EXPECT_GT(stats.peak_vel, 1e-4) << "every compared velocity was ~0 — the cross-check is vacuous";
  EXPECT_GT(stats.peak_err, 1e-5)
      << "the pose error never left zero — the K_p branch was not exercised";
  // Without these the test would still pass on a straight (single-segment)
  // retarget and would pin none of the state machine it was written for.
  EXPECT_EQ(shim.splits(), 1) << "the π-rotation split branch never fired — goal is not near π";
  EXPECT_GT(shim.transitions(), 0)
      << "segment 1 never ended — the run is too short to reach the transition";
}

// ═══════════════════════════════════════════════════════════════════════════
// RT contract
// ═══════════════════════════════════════════════════════════════════════════

// RT-1: the law runs on the tick. Both forms are fixed-size Eigen throughout, so
// this gate is a structural regression guard, and it takes BOTH sensors to be one
// (see the note above the fixtures):
//   • the operator-new counter — sees a std::vector or a header-inline helper
//     (a bare `new`+immediate-`delete` pair would be elided, so the mutation has
//     to be a real runtime-sized allocation reaching an external sink);
//   • the Eigen tripwire — the only one that can see a fixed-size argument or
//     temporary regressing to a runtime-sized Eigen type, which is this law's
//     most plausible RT-1 regression.
// That neither is vacuous AND that neither substitutes for the other is
// MEASURED, not argued: a `std::vector<double>` inserted into the region below
// trips only the first counter, an `Eigen::VectorXd` trips only the second.
// Eigen's internal::aligned_malloc calls std::malloc directly and never reaches
// operator new, so dropping either sensor leaves a live blind spot.
// Both are armed by RAII, so an early return out of the region cannot leave a
// live gate behind for the rest of the binary.
TEST(TaskVelLaw, IsAllocationFree) {
  auto rng = MakeRng();

  const CoreDraw warm = RandomDraw(rng, 0);
  Vec6 sink6 = RunCore6(warm);  // warm up outside the gate
  Eigen::Vector3d sink3 = RunCore3(warm);

  std::vector<CoreDraw> draws;
  draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial)
    draws.push_back(RandomDraw(rng, trial));

  std::size_t new_calls = 0;
  std::uint64_t eigen_allocs = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (const auto& d : draws) {
      sink6 += RunCore6(d);
      sink3 += RunCore3(d);
    }
    new_calls = heap_gate.count();
    eigen_allocs = eigen_gate.violations();
  }

  EXPECT_EQ(new_calls, 0u) << "the task-velocity law reached operator new on the RT path (RT-1)";
  EXPECT_EQ(eigen_allocs, 0u) << "the task-velocity law made Eigen allocate on the RT path (RT-1)";
  EXPECT_TRUE(std::isfinite(sink6.norm()) && std::isfinite(sink3.norm()))
      << "sink kept the calls from being optimised away";
}
