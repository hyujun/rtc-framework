// ── Golden-vector harness for the task-space acceleration core (#236 S2a) ────
// Second slice of the rtc_controllers pure-library refactor. It follows the
// contract S1 established (test_joint_pd_core.cpp): the claim under test is not
// "the new core is correct" but the stronger "the extraction changed NOTHING",
// so every comparison is bitwise (rtc::testing::BitsEqual), never a tolerance.
//
// Two oracles, deliberately:
//
//   1. PreExtractionInlineForm — a literal, rename-only copy of
//      operational_space_controller.cpp:378-399 at bf684040. DURABLE: it
//      outlives the adapter, so it still pins the core after S7 deletes
//      OperationalSpaceController. It keeps the pinocchio::Motion accessors
//      (.linear()/.angular()) the inline form used, so the binding's switch to
//      Motion::toVector() is pinned as a repack-free view rather than assumed.
//
//   2. The live OperationalSpaceController. TRANSIENT — RETIRED in #298 S7c-2
//      with the adapter (see "Oracle 2 retired" below). Its job was
//      the one thing oracle 1 structurally cannot do: catch a CORRELATED
//      transcription error, plus the WIRING — which six doubles reach the core
//      as e, ν, ν_d and a_ff.
//
// The OSC has no a_task accessor and none is being added to a class scheduled
// for deletion, so oracle 2 can only observe the core through the torque the
// adapter emits: τ = Jᵀ Λ a_task + h + Nᵀτ₀. OscShim below therefore replicates
// the WHOLE remaining pipeline — its own model handle, its own TaskSpaceTrajectory
// (D-T: the law takes a trajectory SAMPLE, so ownership sits in the binding),
// the π-rotation split, the segment transition, and the Λ/τ/Nᵀ tail — and the
// comparison is on the adapter's device-order command array. That makes this
// shim a prototype of the S7 binding, which is exactly what S7 needs and what R5
// of the plan asked for. Since #236 S2b that tail is compliance::TaskDynamics
// plus joint::ComputePostureTorque, so the shim mirrors the adapter's CALLS; the
// literal pre-extraction spelling of that block lives in the tier-2 oracle in
// test_dls_convergence.cpp, not here.
//
// The shim reading J/M/h from a SECOND, independently built RtModelHandle is an
// assumption, not a given, so ReferenceDynamics.IndependentHandlesAgreeBitwise
// below measures it directly (plan §S2 R6) — if that ever fails, every
// cross-check here is void and would need re-basing rather than re-tuning.
//
// Every bitwise suite carries a non-vacuity partner
// (BitwiseComparisonRejectsAReassociatedLaw) proving the comparison can actually
// reject an algebraically equivalent regrouping. Without it the equivalence test
// pins nothing.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"
#include "rtc_controllers/joint/posture_law.hpp"
#include "rtc_controllers/params/osc_params.hpp"
#include "rtc_controllers/task/task_accel_law.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/serial7dof_fixture.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

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

// The RT allocation gate is TWO complementary sensors (shared with the S1/S3a
// suites via rtc_controllers/testing/alloc_gate.hpp):
//   • rtc::testing::ScopedAllocGate counts `operator new`, so it sees a
//     std::vector or a header-inline helper.
//   • rtc::testing::ScopedNoMalloc rides eigen_assert, so it sees EIGEN
//     allocation — which the first gate CANNOT: Eigen's internal::aligned_malloc
//     calls std::malloc directly and never routes through operator new. This
//     core is pure Eigen, so its most likely RT-1 regression (a fixed-size
//     argument or temporary becoming runtime-sized) would otherwise pass green.
// IsAllocationFree arms both.

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;

using rtc::kMaxDeviceChannels;
using rtc::task::ComputeTaskAcceleration;
using rtc::task::TaskAccelParams;
using rtc::testing::BitsEqual;
using rtc::testing::MakeRng;

// The serial_7dof fixture and the measured-state sweep, shared with
// test_task_vel_core.cpp (serial7dof_fixture.hpp). S3a reuses this file's
// independent-handle measurement (plan §S3 R6), which is only valid while both
// files use the SAME fixture — sharing the definition is what makes that
// structural rather than a pair of copies that happen to still agree.
using rtc::testing::FillSweep;
using rtc::testing::MakeHandle;
using rtc::testing::MakeState;
using rtc::testing::Serial7dof;

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1 — literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

// One random tick's worth of core inputs. The trajectory sample is kept as
// pinocchio::Motion, not as a flat 6-vector, because that is the type the
// pre-extraction expression indexed through .linear()/.angular() — keeping it
// is what lets oracle 1 pin the binding's toVector() as a view.
struct CoreDraw {
  std::array<double, 3> kp_pos{};
  std::array<double, 3> kd_pos{};
  std::array<double, 3> kp_rot{};
  std::array<double, 3> kd_rot{};
  Vec6 task_err{Vec6::Zero()};
  Vec6 tcp_vel{Vec6::Zero()};
  pinocchio::Motion traj_vel{pinocchio::Motion::Zero()};
  pinocchio::Motion traj_acc{pinocchio::Motion::Zero()};
};

// EXACTLY the expression as it stood inline in
// OperationalSpaceController::Compute() before #236 S2a lifted it into
// task/task_accel_law.hpp (operational_space_controller.cpp:378-399 at
// bf684040). Rename-only transcription:
//   task_err_              → d.task_err     tcp_vel_          → d.tcp_vel
//   gains.kp_pos/…         → d.kp_pos/…     traj_state_.velocity → d.traj_vel
//   traj_state_.acceleration → d.traj_acc   a_task_           → a_task
// Nothing else moved: same operation order, same association, same Eigen
// expression shape, same materialised Vector3d locals.
Vec6 PreExtractionInlineForm(const CoreDraw& d) {
  const Eigen::Vector3d pos_err = d.task_err.head<3>();
  const Eigen::Vector3d rot_err = d.task_err.tail<3>();

  const Eigen::Vector3d kp_p(d.kp_pos[0], d.kp_pos[1], d.kp_pos[2]);
  const Eigen::Vector3d kd_p(d.kd_pos[0], d.kd_pos[1], d.kd_pos[2]);
  const Eigen::Vector3d kp_r(d.kp_rot[0], d.kp_rot[1], d.kp_rot[2]);
  const Eigen::Vector3d kd_r(d.kd_rot[0], d.kd_rot[1], d.kd_rot[2]);

  Vec6 a_task;
  a_task.head<3>() = kp_p.cwiseProduct(pos_err) +
                     kd_p.cwiseProduct(d.traj_vel.linear() - d.tcp_vel.head<3>()) +
                     d.traj_acc.linear();
  a_task.tail<3>() = kp_r.cwiseProduct(rot_err) +
                     kd_r.cwiseProduct(d.traj_vel.angular() - d.tcp_vel.tail<3>()) +
                     d.traj_acc.angular();
  return a_task;
}

// Drive the core with the same draw — this is the marshalling the binding does.
Vec6 RunCore(const CoreDraw& d) {
  return ComputeTaskAcceleration(TaskAccelParams{d.kp_pos, d.kd_pos, d.kp_rot, d.kd_rot},
                                 d.task_err, d.tcp_vel, d.traj_vel.toVector(),
                                 d.traj_acc.toVector());
}

// Draws sweep the magnitude space the law actually sees: OSC gains are
// acceleration-form and span decades, the pose error is small near convergence
// and O(1) after a retarget, and the feedforward can dominate mid-trajectory.
// Every few trials a term is zeroed so no axis is permanently masked by a
// larger one — a cancellation that only shows up when the terms are comparable
// is precisely what a reassociation changes.
CoreDraw RandomDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> gain(0.5, 800.0);
  std::normal_distribution<double> err(0.0, 0.4);
  std::normal_distribution<double> vel(0.0, 1.5);
  std::normal_distribution<double> acc(0.0, 6.0);

  CoreDraw d;
  for (std::size_t i = 0; i < 3; ++i) {
    d.kp_pos[i] = gain(rng);
    d.kd_pos[i] = gain(rng);
    d.kp_rot[i] = gain(rng);
    d.kd_rot[i] = gain(rng);
  }
  for (int i = 0; i < 6; ++i) {
    d.task_err[i] = err(rng);
    d.tcp_vel[i] = vel(rng);
    d.traj_vel.toVector()[i] = vel(rng);
    d.traj_acc.toVector()[i] = acc(rng);
  }

  switch (trial % 5) {
    case 1:
      d.task_err.setZero();  // converged — the damping/feedforward terms alone
      break;
    case 2:
      d.traj_acc.setZero();  // no feedforward — pure task PD
      break;
    case 3:
      d.traj_vel.setZero();  // static setpoint: ė collapses to −ν
      break;
    case 4:
      // Near-cancelling velocity error: ν_d − ν is a catastrophic subtraction,
      // where the ORDER of the surrounding operations matters most.
      d.traj_vel.toVector() = d.tcp_vel + Vec6::Constant(1e-12);
      break;
    default:
      break;
  }
  return d;
}

// ═══════════════════════════════════════════════════════════════════════════
// Reference dynamics for the cross-check (fixture: serial7dof_fixture.hpp)
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 — the shim (prototype of the S7 integration-layer binding)
// ═══════════════════════════════════════════════════════════════════════════

// Owns the model and the trajectory (STRUCTURE) and calls the core (ALGORITHM).
// Mirrors OperationalSpaceController::Compute() step for step; any divergence
// here is a divergence the future binding would ship.
class OscShim {
 public:
  OscShim() {
    handle_ = MakeHandle(model_);
    tip_ = static_cast<pinocchio::FrameIndex>(model_->nframes - 1);
    nv_ = handle_->nv();

    J_ = Eigen::MatrixXd::Zero(6, nv_);
    Jt_ = Eigen::MatrixXd::Zero(nv_, 6);
    M_ = Eigen::MatrixXd::Zero(nv_, nv_);
    h_ = Eigen::VectorXd::Zero(nv_);
    tau_ = Eigen::VectorXd::Zero(nv_);
    tau0_ = Eigen::VectorXd::Zero(nv_);
    tau0_dev_ = Eigen::VectorXd::Zero(nv_);
    null_tmp_ = Eigen::VectorXd::Zero(nv_);
    q_ref_null_ = Eigen::VectorXd::Zero(nv_);
    llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv_);
    dyn_.Resize(nv_, 6);
  }

  [[nodiscard]] int nv() const { return nv_; }

  // operational_space_controller.cpp:632-639 — the adapter's private static
  // helper, replicated verbatim (ZYX Euler, R = Rz·Ry·Rx).
  static Eigen::Matrix3d RpyToMatrix(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  }

  // One tick. `goal` is non-null exactly on the tick the adapter drains a queued
  // target — the caller keeps the two in lock-step, as the adapter's own drain
  // ordering (FK first, trajectory re-init after) requires.
  // Returns the unclamped torque in DEVICE channel order.
  // The controller-local E-STOP latch, mirroring TriggerEstop()/ClearEstop().
  // Deliberately NOT merged with any global-E-STOP flag — same as the adapter,
  // where `estopped_` is cleared only by an explicit ClearEstop().
  void TriggerEstop() { estopped_ = true; }

  void ClearEstop() { estopped_ = false; }

  Eigen::VectorXd Step(const rtc::params::OscParams& g,
                       const rtc::ControllerState& state, const std::array<double, 6>* goal) {
    // ── Step 0: the E-STOP early return (adapter :193-212) ─────────────────
    // The ORDER inside this branch is the contract, not an implementation
    // detail: the posture-gate window is cleared BEFORE the return, so a held
    // tick reports the gate it actually ran (closed) instead of inheriting the
    // previous active tick's `true` for the whole stop. That flag is the only
    // observable a numerically inert gate has, so a mutation that kept the
    // posture task running through an E-STOP would otherwise leave both the
    // torque lanes and the window looking exactly as they do now (S5 M4).
    if (estopped_) {
      last_nullspace_active_ = false;
      // A held tick is a discontinuity — the next active tick re-seeds the pose
      // target and the trajectory from the measurement (adapter :693).
      initialized_ = false;
      return EstopHold(g, state);
    }

    // ── Step 1: copy joint state into buffers ──────────────────────────────
    const auto& dev0 = state.devices[0];
    std::array<double, kMaxDeviceChannels> q_buf{};
    std::array<double, kMaxDeviceChannels> v_buf{};
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_buf[ui] = dev0.positions[ui];
      v_buf[ui] = dev0.velocities[ui];
    }
    std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv_));
    std::span<const double> v_span(v_buf.data(), static_cast<std::size_t>(nv_));

    // ── Step 2/3: FK + Jacobian, then ν = J·q̇ ─────────────────────────────
    handle_->ComputeJacobians(q_span);
    handle_->GetFrameJacobian(tip_, pinocchio::LOCAL_WORLD_ALIGNED, J_);
    Eigen::Map<const Eigen::VectorXd> v_eigen(v_buf.data(), nv_);
    tcp_vel_.noalias() = J_ * v_eigen;

    const double dt = state.dt;
    const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_);

    // First-tick self-init (adapter: the !target_initialized_ branch).
    if (!initialized_) {
      trajectory_.initialize(tcp, pinocchio::Motion::Zero(), tcp, pinocchio::Motion::Zero(), 0.01);
      time_ = 0.0;
      has_pending_segment_ = false;
      initialized_ = true;
    }

    // ── Step 3.5: trajectory re-init on a new target (adapter :296-359) ────
    if (goal != nullptr) {
      goal_pose_.translation() = Eigen::Vector3d((*goal)[0], (*goal)[1], (*goal)[2]);
      goal_pose_.rotation() = RpyToMatrix((*goal)[3], (*goal)[4], (*goal)[5]);

      const Eigen::Vector3d start_pos = tcp.translation();
      const Eigen::Vector3d goal_pos = goal_pose_.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      const double T_speed_trans = trans_dist / g.trajectory_speed;
      const double T_vel_trans =
          (g.max_traj_velocity > 0.0) ? (1.875 * trans_dist / g.max_traj_velocity) : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      constexpr double kPiSafetyMargin = 0.15;
      const Eigen::AngleAxisd aa(tcp.rotation().transpose() * goal_pose_.rotation());
      const double angular_dist = aa.angle();
      const Eigen::Vector3d rot_axis = aa.axis();

      const double T_speed_rot = angular_dist / g.trajectory_angular_speed;
      const double T_vel_rot = (g.max_traj_angular_velocity > 0.0)
                                   ? (1.875 * angular_dist / g.max_traj_angular_velocity)
                                   : 0.0;
      duration = std::max({duration, T_speed_rot, T_vel_rot});

      const bool split_trajectory = (angular_dist > M_PI - kPiSafetyMargin);

      if (split_trajectory) {
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid =
            tcp.rotation() * Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

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

        trajectory_.initialize(tcp, pinocchio::Motion::Zero(), mid_pose, pinocchio::Motion::Zero(),
                               dur1);
        pending_goal_pose_ = goal_pose_;
        pending_duration_ = dur1;
        has_pending_segment_ = true;
        ++splits_;
      } else {
        trajectory_.initialize(tcp, pinocchio::Motion::Zero(), goal_pose_,
                               pinocchio::Motion::Zero(), duration);
        has_pending_segment_ = false;
      }
      time_ = 0.0;
    }

    // Sample, then advance (adapter :361-362) — the order is load-bearing.
    traj_state_ = trajectory_.compute(time_, dt);
    time_ += dt;

    // Segment transition, π-rotation defense (adapter :364-370).
    if (has_pending_segment_ && time_ >= trajectory_.duration()) {
      pinocchio::SE3 mid_pose = trajectory_.compute(trajectory_.duration()).pose;
      trajectory_.initialize(mid_pose, pinocchio::Motion::Zero(), pending_goal_pose_,
                             pinocchio::Motion::Zero(), pending_duration_);
      time_ = 0.0;
      has_pending_segment_ = false;
      ++transitions_;
    }

    // ── Step 4: pose error (definition owned by the binding, not the law) ──
    task_err_ = rtc::math::se3::computePoseError(tcp, traj_state_.pose,
                                                 rtc::math::se3::ErrorType::SplitWorld);

    // ── Step 5: THE CORE ───────────────────────────────────────────────────
    a_task_ = ComputeTaskAcceleration(TaskAccelParams{g.kp_pos, g.kd_pos, g.kp_rot, g.kd_rot},
                                      task_err_, tcp_vel_, traj_state_.velocity.toVector(),
                                      traj_state_.acceleration.toVector());

    // ── Steps 6-9: joint dynamics, Λ, τ, Nᵀ. Since #236 S2b these are the
    //     shared helpers, so this shim mirrors the adapter's CALLS rather than
    //     a replicated inline block. The literal pre-extraction form now lives
    //     in the tier-2 oracle in test_dls_convergence.cpp, which is where it
    //     belongs: a cross-check shim that re-derives the law cannot also be
    //     the witness that the law did not change. ────────────────────────────
    handle_->ComputeMassMatrix(q_span);
    M_ = handle_->GetMassMatrix();
    M_.triangularView<Eigen::StrictlyLower>() =
        M_.triangularView<Eigen::StrictlyUpper>().transpose();
    handle_->ComputeNonLinearEffects(q_span, v_span);
    h_ = handle_->GetNonLinearEffects();

    bool dyn_ok = true;
    llt_M_.compute(M_);
    dyn_ok = dyn_ok && (llt_M_.info() == Eigen::Success);

    if (dyn_ok) {
      Jt_.noalias() = J_.transpose();
      const rtc::compliance::TaskDynamics::Result r =
          dyn_.Compute(J_, llt_M_, g.singularity_threshold, std::max(1e-4, g.max_damping));
      dyn_ok = r.ok;
      last_sigma_min_ = r.sigma_min;
      last_lambda_sq_ = r.lambda_sq;
    }
    last_dyn_ok_ = dyn_ok;

    if (dyn_ok) {
      const Vec6 F = dyn_.LambdaS() * a_task_;
      tau_.noalias() = Jt_ * F;
      tau_ += h_;

      last_nullspace_active_ = (g.null_kp != 0.0 || g.null_kd != 0.0) && nv_ > 6;
      if (last_nullspace_active_) {
        const auto nvu = static_cast<std::size_t>(nv_);
        // safe_position_ is empty on a controller with no device config, so
        // the adapter's per-joint reference falls to 0.0 on every channel.
        q_ref_null_.setZero();
        rtc::joint::ComputePostureTorque(
            g.null_kp, g.null_kd,
            rtc::joint::PostureInputs{{q_ref_null_.data(), nvu}, q_span, v_span}, nvu,
            {tau0_dev_.data(), nvu});
        handle_->ReorderInput(std::span<const double>(tau0_dev_.data(), nvu), tau0_);
        dyn_.ProjectNullspace(tau0_, null_tmp_);
        tau_ += null_tmp_;
      }
    } else {
      last_nullspace_active_ = false;
      tau_ = h_;
    }

    // Scatter to device channel order (identity order → memcpy).
    Eigen::VectorXd out = Eigen::VectorXd::Zero(nv_);
    handle_->ReorderOutput(tau_, std::span<double>(out.data(), static_cast<std::size_t>(nv_)));
    return out;
  }

  [[nodiscard]] const Vec6& task_err() const { return task_err_; }

  [[nodiscard]] const Vec6& a_task() const { return a_task_; }

  // Branch counters — a cross-check that never took the π-split path would be
  // vacuous with respect to the state machine it exists to pin.
  [[nodiscard]] int splits() const { return splits_; }

  [[nodiscard]] int transitions() const { return transitions_; }

  // Gate / diagnostic windows. The OSC adapter publishes no diagnostics, so
  // these are the only place a gate that is NUMERICALLY INERT when closed can be
  // observed at all — the S5 M4 lesson (a gate whose closed branch contributes a
  // signed zero leaves every bit lane green). Compared bitwise, tick by tick.
  [[nodiscard]] bool last_dyn_ok() const { return last_dyn_ok_; }

  [[nodiscard]] bool last_nullspace_active() const { return last_nullspace_active_; }

  [[nodiscard]] double last_sigma_min() const { return last_sigma_min_; }

  [[nodiscard]] double last_lambda_sq() const { return last_lambda_sq_; }

 private:
  // τ = ĝ(q) − D·q̇ clamped per joint (E-8, #184), through the same core helper
  // the adapter calls. Coriolis is omitted there and so it is here: at the
  // speeds a safety stop targets the damping term dominates.
  Eigen::VectorXd EstopHold(const rtc::params::OscParams& g,
                            const rtc::ControllerState& state) {
    const auto& dev0 = state.devices[0];
    std::array<double, kMaxDeviceChannels> q_buf{};
    Eigen::VectorXd qdot_dev = Eigen::VectorXd::Zero(nv_);
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_buf[ui] = dev0.positions[ui];
      qdot_dev(i) = dev0.velocities[ui];
    }
    handle_->ComputeGeneralizedGravity(
        std::span<const double>(q_buf.data(), static_cast<std::size_t>(nv_)));
    Eigen::VectorXd grav_dev = Eigen::VectorXd::Zero(nv_);
    handle_->ReorderOutput(handle_->GetGeneralizedGravity(),
                           std::span<double>(grav_dev.data(), static_cast<std::size_t>(nv_)));
    const Eigen::VectorXd t_max = Eigen::VectorXd::Constant(nv_, rtc::kDefaultMaxJointTorque);
    Eigen::VectorXd out = Eigen::VectorXd::Zero(nv_);
    rtc::compliance::GravityCompDampedHold(out, grav_dev, qdot_dev, g.estop_damping, t_max);
    return out;
  }

  bool estopped_{false};

  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  int nv_{0};

  Eigen::MatrixXd J_, Jt_, M_;
  Eigen::VectorXd h_, tau_, tau0_, tau0_dev_, null_tmp_, q_ref_null_;
  rtc::compliance::TaskDynamics dyn_;
  Vec6 task_err_{Vec6::Zero()};
  Vec6 a_task_{Vec6::Zero()};
  Vec6 tcp_vel_{Vec6::Zero()};
  Eigen::LLT<Eigen::MatrixXd> llt_M_;

  bool last_dyn_ok_{false};
  bool last_nullspace_active_{false};
  double last_sigma_min_{0.0};
  double last_lambda_sq_{0.0};

  rtc::trajectory::TaskSpaceTrajectory trajectory_;
  rtc::trajectory::TaskSpaceTrajectory::State traj_state_{};
  double time_{0.0};
  bool initialized_{false};
  bool has_pending_segment_{false};
  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 pending_goal_pose_{pinocchio::SE3::Identity()};
  double pending_duration_{0.0};
  int splits_{0};
  int transitions_{0};
};

// A goal pose sitting exactly `angle` radians of rotation away from the TCP
// orientation at `tick`, returned as the [x,y,z, r,p,y] vector SetDeviceTarget
// takes (ZYX Euler, matching the adapter's RpyToMatrix). Deriving the target
// from the fixture instead of hard-coding Euler angles is what makes the
// π-split branch reachable on purpose: the first version of that test used a
// literal yaw of π−0.02 and silently took the single-segment path, which the
// shim's branch counters caught.
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

  // ComputeJacobians is what the adapter calls, and it refreshes frame
  // placements as a side effect — using it keeps this helper on the adapter's
  // own FK path rather than a parallel one.
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
// Bit-identity against the literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

TEST(TaskAccelLaw, MatchesThePreExtractionInlineFormBitwise) {
  auto rng = MakeRng();
  double peak = 0.0;
  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const Vec6 got = RunCore(d);
    const Vec6 want = PreExtractionInlineForm(d);

    for (int i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(got[i], want[i]))
          << "trial " << trial << " axis " << i << ": " << got[i] << " vs " << want[i];
      peak = std::max(peak, std::abs(got[i]));
    }
  }
  EXPECT_GT(peak, 1e-6) << "every compared axis was ~0 — the draw generator degenerated";
}

// Non-vacuity partner. `kd·(ν_d − ν)` and `kd·ν_d − kd·ν` are the same law
// algebraically and the obvious "distribute the gain" edit; if bit equality
// could not tell them apart, the test above would be pinning nothing.
TEST(TaskAccelLaw, BitwiseComparisonRejectsAReassociatedLaw) {
  auto rng = MakeRng();
  int differing_axes = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);
    const Vec6 got = RunCore(d);

    for (int i = 0; i < 6; ++i) {
      const double kp = (i < 3) ? d.kp_pos[static_cast<std::size_t>(i)]
                                : d.kp_rot[static_cast<std::size_t>(i - 3)];
      const double kd = (i < 3) ? d.kd_pos[static_cast<std::size_t>(i)]
                                : d.kd_rot[static_cast<std::size_t>(i - 3)];
      const double reassociated = kp * d.task_err[i] + kd * d.traj_vel.toVector()[i] -
                                  kd * d.tcp_vel[i] + d.traj_acc.toVector()[i];
      ++compared;
      if (!BitsEqual(got[i], reassociated))
        ++differing_axes;
      EXPECT_NEAR(got[i], reassociated, 1e-6 * (1.0 + std::abs(got[i])))
          << "the mutant must stay algebraically equivalent, else it proves nothing";
    }
  }
  ASSERT_GT(compared, 0) << "no axes compared — the draw generator degenerated";
  EXPECT_GT(differing_axes, 0)
      << "bit comparison cannot distinguish a reassociated law — the equivalence test is vacuous";
}

// ═══════════════════════════════════════════════════════════════════════════
// The assumption the cross-check rests on (plan §S2 R6)
// ═══════════════════════════════════════════════════════════════════════════

// The shim cannot read the adapter's J/M/h — there are no accessors, and adding
// them to a class scheduled for deletion is the wrong direction — so it rebuilds
// them from its own RtModelHandle. That is only a valid oracle if two
// independently built handles agree BIT FOR BIT on the same q/v; a tolerance
// would let the cross-check's bitwise verdict rest on a non-bitwise foundation.
TEST(ReferenceDynamics, IndependentHandlesAgreeBitwise) {
  std::shared_ptr<const pinocchio::Model> model_a;
  std::shared_ptr<const pinocchio::Model> model_b;
  auto a = MakeHandle(model_a);
  auto b = MakeHandle(model_b);
  ASSERT_EQ(a->nv(), b->nv());
  const int nv = a->nv();
  const auto tip_a = static_cast<pinocchio::FrameIndex>(model_a->nframes - 1);
  const auto tip_b = static_cast<pinocchio::FrameIndex>(model_b->nframes - 1);
  ASSERT_EQ(tip_a, tip_b);

  auto rng = MakeRng();
  std::normal_distribution<double> draw(0.0, 0.7);

  double peak_h = 0.0;
  for (int trial = 0; trial < 16; ++trial) {
    std::vector<double> q(static_cast<std::size_t>(nv));
    std::vector<double> v(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i) {
      q[static_cast<std::size_t>(i)] = draw(rng);
      v[static_cast<std::size_t>(i)] = draw(rng);
    }

    Eigen::MatrixXd Ja = Eigen::MatrixXd::Zero(6, nv);
    Eigen::MatrixXd Jb = Eigen::MatrixXd::Zero(6, nv);
    a->ComputeJacobians(q);
    b->ComputeJacobians(q);
    a->GetFrameJacobian(tip_a, pinocchio::LOCAL_WORLD_ALIGNED, Ja);
    b->GetFrameJacobian(tip_b, pinocchio::LOCAL_WORLD_ALIGNED, Jb);

    a->ComputeMassMatrix(q);
    b->ComputeMassMatrix(q);
    const Eigen::MatrixXd Ma = a->GetMassMatrix();
    const Eigen::MatrixXd Mb = b->GetMassMatrix();

    a->ComputeNonLinearEffects(q, v);
    b->ComputeNonLinearEffects(q, v);
    const Eigen::VectorXd ha = a->GetNonLinearEffects();
    const Eigen::VectorXd hb = b->GetNonLinearEffects();

    const pinocchio::SE3& Ta = a->GetFramePlacement(tip_a);
    const pinocchio::SE3& Tb = b->GetFramePlacement(tip_b);

    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < nv; ++c)
        ASSERT_TRUE(BitsEqual(Ja(r, c), Jb(r, c)))
            << "trial " << trial << " J(" << r << "," << c << ")";
    for (int r = 0; r < nv; ++r)
      for (int c = 0; c < nv; ++c)
        ASSERT_TRUE(BitsEqual(Ma(r, c), Mb(r, c)))
            << "trial " << trial << " M(" << r << "," << c << ")";
    for (int i = 0; i < nv; ++i) {
      ASSERT_TRUE(BitsEqual(ha[i], hb[i])) << "trial " << trial << " h(" << i << ")";
      peak_h = std::max(peak_h, std::abs(ha[i]));
    }
    for (int i = 0; i < 3; ++i)
      ASSERT_TRUE(BitsEqual(Ta.translation()[i], Tb.translation()[i]));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        ASSERT_TRUE(BitsEqual(Ta.rotation()(r, c), Tb.rotation()(r, c)));
  }
  // serial_7dof's alternating axes are what make h ≢ 0; on an all-Z arm this
  // whole comparison would be over a field of zeros.
  EXPECT_GT(peak_h, 1e-6) << "reference dynamics were all ~0 — the fixture masked the comparison";
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 retired with the adapter (#298 S7c-2)
// ═══════════════════════════════════════════════════════════════════════════
//
// Three cross-check cases stood here, driving the live
// OperationalSpaceController and OscShim through the same tick sequence and
// comparing every device-0 torque channel plus the cached pose error bit for
// bit:
//
//   TaskAccelLaw.CoreDrivenShimMatchesTheAdapterBitwise
//   TaskAccelLaw.CoreDrivenShimMatchesTheAdapterBitwiseWithNullSpaceTask
//   TaskAccelLaw.CoreDrivenShimMatchesTheAdapterAcrossAPiRotationSplit
//
// VERIFICATION PROVENANCE — task::ComputeTaskAcceleration and
// PreExtractionInlineForm above were verified bitwise against the live
// OperationalSpaceController @ 0f873901 over 40 + 40 + 140 ticks x 7 channels
// (straight retarget, null-space posture task active on the redundant fixture,
// and the near-pi rotation split with both the split and the segment transition
// asserted to have fired), with the adapter's torque clamp asserted inert on
// every compared sample; oracle 2 retired in S7c.
//
// They could not be migrated: oracle 2's unique job was the CORRELATED
// transcription error, which by construction needs the shipped code to compare
// against. See test_joint_pd_core.cpp's equivalent block for the full argument.
//
// OscShim below survives — the posture-gate cases at the end of this file drive
// it shim-only (분류 B), and it remains the reference recipe an S7 binding
// starts from, including the two-segment pi-split state machine that R5 flagged
// as the likeliest place to get the trajectory wiring wrong.

// ═══════════════════════════════════════════════════════════════════════════
// RT contract
// ═══════════════════════════════════════════════════════════════════════════

// RT-1: the law runs on the tick. It is fixed-size Eigen throughout, so this
// gate is a structural regression guard, and it takes both sensors to be one:
// a `std::vector<double>` inserted into the region trips only the operator-new
// counter, an `Eigen::VectorXd` trips only the Eigen tripwire (measured in the
// S3a suite, same gate). A bare `new`+immediate-`delete` pair would be elided,
// so a mutation has to be a real allocation reaching an external sink. Both are
// armed by RAII: an ASSERT_* added inside the region returns from the test, and
// a bare disarm line would then never run — leaving counting on for every later
// test in the binary, where nothing reads it.
// ═══════════════════════════════════════════════════════════════════════════
// The posture gate — shim-only (#236 S7c-2, 분류 B)
// ═══════════════════════════════════════════════════════════════════════════
//
// These two moved here from test_dls_convergence.cpp's OscAdapter suite, which
// retires with OperationalSpaceController. What they pin is NOT adapter
// behaviour: it is the gate's own contract, and the observation window
// (last_nullspace_active()) already existed on this shim — it was only ever
// ASSERTED through the adapter's cross-check, so 분류 A's retirement would have
// taken the coverage with it.
//
// Why a window and not an output lane: mutation M2 (deleting the
// `null_kp != 0 || null_kd != 0` half of the gate) ran the whole suite clean and
// has to. With both gains zero the posture torque is a signed zero, Nᵀ·0 is
// zero, and adding it changes no bit of any torque lane. The flag is the only
// place the branch is visible at all.

TEST(OscShimPostureGate, IsWiredToTheGains) {
  rtc::params::OscParams gains;
  gains.null_kp = 0.0;
  gains.null_kd = 0.0;

  OscShim shim;
  ASSERT_EQ(shim.nv(), 7) << "fixture must be redundant (nv > 6), or the gate is closed anyway";
  auto state = MakeState(7, 0.001);

  FillSweep(state, 7, 0, 0.001);
  (void)shim.Step(gains, state, nullptr);
  EXPECT_FALSE(shim.last_nullspace_active()) << "both posture gains are zero";

  gains.null_kp = 3.0;
  FillSweep(state, 7, 1, 0.001);
  (void)shim.Step(gains, state, nullptr);
  EXPECT_TRUE(shim.last_nullspace_active()) << "a non-zero stiffness must open the gate";

  gains.null_kp = 0.0;
  gains.null_kd = 0.4;
  FillSweep(state, 7, 2, 0.001);
  (void)shim.Step(gains, state, nullptr);
  EXPECT_TRUE(shim.last_nullspace_active()) << "damping alone must open the gate too";

  gains.null_kd = 0.0;
  FillSweep(state, 7, 3, 0.001);
  (void)shim.Step(gains, state, nullptr);
  EXPECT_FALSE(shim.last_nullspace_active()) << "the gate must close again";
}

TEST(OscShimPostureGate, ReportsClosedOnAnEstoppedTick) {
  // The window has to describe THIS tick or it is not a window. The E-STOP
  // branch is an early return that sits before the gate assignment, so unless it
  // clears the flag itself a held tick inherits the previous active tick's
  // `true` and keeps reporting it for the whole stop.
  rtc::params::OscParams gains;
  gains.null_kp = 3.0;  // gate open on the redundant fixture

  OscShim shim;
  auto state = MakeState(7, 0.001);

  FillSweep(state, 7, 0, 0.001);
  (void)shim.Step(gains, state, nullptr);
  ASSERT_TRUE(shim.last_nullspace_active()) << "fixture must open the gate first, or this pins "
                                               "nothing";

  shim.TriggerEstop();
  FillSweep(state, 7, 1, 0.001);
  const Eigen::VectorXd held = shim.Step(gains, state, nullptr);
  EXPECT_FALSE(shim.last_nullspace_active())
      << "an E-STOP tick runs no posture task — the gate observable must say so";
  // Non-vacuity: the held tick still produced a hold, so the assertion above is
  // about the gate rather than about the branch having emitted nothing at all.
  EXPECT_TRUE(held.allFinite());
  EXPECT_GT(held.cwiseAbs().maxCoeff(), 0.0) << "ĝ(q) − D·q̇ collapsed to zero on every joint";

  shim.ClearEstop();
  FillSweep(state, 7, 2, 0.001);
  (void)shim.Step(gains, state, nullptr);
  EXPECT_TRUE(shim.last_nullspace_active()) << "recovery must reopen the gate, not latch it closed";
}

TEST(TaskAccelLaw, IsAllocationFree) {
  auto rng = MakeRng();

  const CoreDraw warm = RandomDraw(rng, 0);
  Vec6 sink = RunCore(warm);  // warm up outside the gate

  std::vector<CoreDraw> draws;
  draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial)
    draws.push_back(RandomDraw(rng, trial));

  std::size_t new_calls = 0;
  std::uint64_t eigen_allocs = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (const auto& d : draws)
      sink += RunCore(d);
    new_calls = heap_gate.count();
    eigen_allocs = eigen_gate.violations();
  }

  EXPECT_EQ(new_calls, 0u)
      << "the task-acceleration law reached operator new on the RT path (RT-1)";
  EXPECT_EQ(eigen_allocs, 0u)
      << "the task-acceleration law made Eigen allocate on the RT path (RT-1)";
  EXPECT_TRUE(std::isfinite(sink.norm())) << "sink kept the calls from being optimised away";
}
