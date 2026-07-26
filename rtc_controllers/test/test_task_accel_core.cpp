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
//   2. The live OperationalSpaceController. TRANSIENT (dies with S7). Its job is
//      the one thing oracle 1 structurally cannot do: catch a CORRELATED
//      transcription error, plus the WIRING — which six doubles reach the core
//      as e, ν, ν_d and a_ff.
//
// The OSC has no a_task accessor and none is being added to a class scheduled
// for deletion, so oracle 2 can only observe the core through the torque the
// adapter emits: τ = Jᵀ Λ a_task + h + Nᵀτ₀. OscShim below therefore replicates
// the WHOLE remaining pipeline — its own model handle, its own TaskSpaceTrajectory
// (D-T: the law takes a trajectory SAMPLE, so ownership sits in the binding),
// the π-rotation split, the segment transition, and the Λ/τ/Nᵀ block that #236
// D-S2 defers to S2b — and the comparison is on the adapter's device-order
// command array. That makes this shim a prototype of the S7 binding, which is
// exactly what S7 needs and what R5 of the plan asked for.
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

#include "rtc_controllers/direct/operational_space_controller.hpp"
#include "rtc_controllers/task/task_accel_law.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"
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
#include <cstdlib>
#include <memory>
#include <new>
#include <random>
#include <span>
#include <string>
#include <vector>

// ── Allocation counter (global new/delete interposition) ────────────────────
// The core returns a fixed-size 6-vector and Eigen never touches its allocator
// for fixed-size expressions, so rtc_base/testing/no_malloc_scope.hpp (an Eigen
// allocation tripwire) would be vacuous here. Global interposition sees every
// heap touch, including one from a header-inline helper.
namespace {
thread_local bool g_alloc_active = false;
thread_local std::size_t g_alloc_count = 0;
}  // namespace

void* operator new(std::size_t n) {
  if (g_alloc_active)
    ++g_alloc_count;
  void* p = std::malloc(n != 0 ? n : 1);
  if (p == nullptr)
    throw std::bad_alloc();
  return p;
}

void* operator new[](std::size_t n) {
  return ::operator new(n);
}

void operator delete(void* p) noexcept {
  std::free(p);
}

void operator delete[](void* p) noexcept {
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;

using rtc::kMaxDeviceChannels;
using rtc::task::ComputeTaskAcceleration;
using rtc::task::TaskAccelParams;
using rtc::testing::BitsEqual;
using rtc::testing::MakeRng;

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
// Fixture + reference dynamics for the cross-check
// ═══════════════════════════════════════════════════════════════════════════

// serial_7dof.urdf: alternating Z/Y joint axes, so g(q) ≢ 0 and the Jacobian is
// well-conditioned (serial_6dof.urdf is all-Z — g ≡ 0 and σ_min ≡ 0, the fixture
// trap recorded in the plan's §제약·함정). nv = 7 > 6 also makes Λ⁻¹ full rank
// and the dynamically-consistent null-space branch meaningful rather than inert.
std::string Serial7dof() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

std::unique_ptr<rtc_urdf_bridge::RtModelHandle> MakeHandle(
    std::shared_ptr<const pinocchio::Model>& model_out) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = Serial7dof();
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  model_out = builder.GetFullModel();
  return std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_out);
}

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
    MinvJt_ = Eigen::MatrixXd::Zero(nv_, 6);
    h_ = Eigen::VectorXd::Zero(nv_);
    tau_ = Eigen::VectorXd::Zero(nv_);
    JbarT_ = Eigen::MatrixXd::Zero(6, nv_);
    NT_ = Eigen::MatrixXd::Zero(nv_, nv_);
    tau0_ = Eigen::VectorXd::Zero(nv_);
    tau0_dev_ = Eigen::VectorXd::Zero(nv_);
    null_tmp_ = Eigen::VectorXd::Zero(nv_);
    llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv_);
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
  Eigen::VectorXd Step(const rtc::OperationalSpaceController::Gains& g,
                       const rtc::ControllerState& state, const std::array<double, 6>* goal) {
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

    // ── Steps 6-9: joint dynamics, Λ, τ, Nᵀ (S2b territory — replicated,
    //     not extracted; see the D-S2 note in task_accel_law.hpp) ───────────
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
      MinvJt_ = Jt_;
      llt_M_.solveInPlace(MinvJt_);

      LambdaInv_.noalias() = J_ * MinvJt_;
      const double lambda = std::max(1e-4, g.damping);
      LambdaInv_.diagonal().array() += lambda * lambda;
      llt6_.compute(LambdaInv_);
      dyn_ok = (llt6_.info() == Eigen::Success);
    }

    if (dyn_ok) {
      F_ = a_task_;
      llt6_.solveInPlace(F_);
      tau_.noalias() = Jt_ * F_;
      tau_ += h_;

      if ((g.null_kp != 0.0 || g.null_kd != 0.0) && nv_ > 6) {
        JbarT_ = MinvJt_.transpose();
        llt6_.solveInPlace(JbarT_);
        NT_.setIdentity();
        NT_.noalias() -= Jt_ * JbarT_;
        for (int i = 0; i < nv_; ++i) {
          const auto ui = static_cast<std::size_t>(i);
          // safe_position_ is empty on a controller with no device config, so
          // the adapter's per-joint reference falls to 0.0 on every channel.
          tau0_dev_[i] = g.null_kp * (0.0 - q_buf[ui]) - g.null_kd * v_buf[ui];
        }
        handle_->ReorderInput(
            std::span<const double>(tau0_dev_.data(), static_cast<std::size_t>(nv_)), tau0_);
        null_tmp_.noalias() = NT_ * tau0_;
        tau_ += null_tmp_;
      }
    } else {
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

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  int nv_{0};

  Eigen::MatrixXd J_, Jt_, M_, MinvJt_, JbarT_, NT_;
  Eigen::VectorXd h_, tau_, tau0_, tau0_dev_, null_tmp_;
  Eigen::Matrix<double, 6, 6> LambdaInv_{Eigen::Matrix<double, 6, 6>::Zero()};
  Vec6 task_err_{Vec6::Zero()};
  Vec6 a_task_{Vec6::Zero()};
  Vec6 F_{Vec6::Zero()};
  Vec6 tcp_vel_{Vec6::Zero()};
  Eigen::LLT<Eigen::MatrixXd> llt_M_;
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt6_;

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

// The measured-state sweep both sides are driven with: slow enough that the
// adapter's output clamp stays inert (asserted per channel, never assumed) but
// moving, so the Jacobian, the Coriolis term and the finite pose error are all
// non-trivial. Shared with the goal-derivation helper below so a test can ask
// "what is the TCP orientation at tick N" without duplicating the formula.
void FillSweep(rtc::ControllerState& state, int nv, int tick, double dt) {
  constexpr double kStep = 0.0004;
  for (int j = 0; j < nv; ++j) {
    const auto uj = static_cast<std::size_t>(j);
    state.devices[0].positions[uj] = 0.12 * (1.0 + 0.2 * j) + kStep * tick * (1.0 + 0.3 * j);
    state.devices[0].velocities[uj] = kStep * (1.0 + 0.3 * j) / dt;
  }
}

rtc::ControllerState MakeState(int nc0, double dt) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nc0;
  state.devices[0].valid = true;
  return state;
}

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
// Cross-check against the live adapter (transient — dies with S7)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// Drive adapter and shim through the same tick sequence and compare device-0
// commands bit for bit.
void RunCrossCheck(rtc::OperationalSpaceController& ctrl, OscShim& shim,
                   const rtc::OperationalSpaceController::Gains& g,
                   const std::array<double, 6>& goal, int retarget_tick, int ticks, double dt,
                   double* peak_out, double* peak_err_out) {
  const int nv = shim.nv();
  auto state = MakeState(nv, dt);
  double peak = 0.0;
  double peak_err = 0.0;

  for (int tick = 0; tick < ticks; ++tick) {
    FillSweep(state, nv, tick, dt);

    if (tick == retarget_tick) {
      ctrl.SetDeviceTarget(0, std::span<const double>(goal));
    }

    const auto out = ctrl.Compute(state);
    const Eigen::VectorXd shim_tau = shim.Step(g, state, tick == retarget_tick ? &goal : nullptr);

    for (int j = 0; j < nv; ++j) {
      const auto uj = static_cast<std::size_t>(j);
      const double adapter = out.devices[0].commands[uj];
      // The adapter clamps after the law; the shim does not (limits are an
      // integration-layer concern). Keeping the clamp provably inert is what
      // makes the bitwise comparison meaningful — assert it, don't assume it.
      ASSERT_LT(std::abs(adapter), rtc::kDefaultMaxJointTorque)
          << "clamp fired at tick " << tick << " ch " << j << " — comparison would be invalid";
      EXPECT_TRUE(BitsEqual(adapter, shim_tau[j]))
          << "tick " << tick << " channel " << j << ": adapter " << adapter << " vs shim "
          << shim_tau[j];
      peak = std::max(peak, std::abs(adapter));
    }

    // The adapter's ONE window onto the core's input: the cached pose error.
    // Pinning it bitwise localises a cross-check failure to either the error
    // (binding) or the law (core) instead of leaving the whole tick suspect.
    const auto adapter_err = ctrl.pose_error();
    for (int i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(adapter_err[static_cast<std::size_t>(i)], shim.task_err()[i]))
          << "tick " << tick << " pose-error axis " << i;
      peak_err = std::max(peak_err, std::abs(adapter_err[static_cast<std::size_t>(i)]));
    }
  }
  *peak_out = peak;
  *peak_err_out = peak_err;
}

rtc::OperationalSpaceController::Gains CrossCheckGains() {
  rtc::OperationalSpaceController::Gains g;
  g.kp_pos = {{40.0, 40.0, 40.0}};
  g.kd_pos = {{8.0, 8.0, 8.0}};
  g.kp_rot = {{20.0, 20.0, 20.0}};
  g.kd_rot = {{4.0, 4.0, 4.0}};
  g.damping = 0.05;
  g.trajectory_speed = 0.05;
  g.trajectory_angular_speed = 0.25;
  return g;
}

}  // namespace

// Straight retarget: no π-split, single segment.
TEST(TaskAccelLaw, CoreDrivenShimMatchesTheAdapterBitwise) {
  rtc::OperationalSpaceController ctrl(Serial7dof(), CrossCheckGains());
  ctrl.OnDeviceConfigsSet();  // populates max_joint_torque_ = kDefaultMaxJointTorque
  const auto g = CrossCheckGains();
  ctrl.set_gains(g);

  OscShim shim;
  ASSERT_EQ(shim.nv(), 7) << "fixture must be redundant (nv > 6) for the null-space branch";

  const std::array<double, 6> goal{0.30, 0.10, 0.45, 0.20, -0.15, 0.35};
  double peak = 0.0;
  double peak_err = 0.0;
  RunCrossCheck(ctrl, shim, g, goal, /*retarget_tick=*/3, /*ticks=*/40, /*dt=*/0.002, &peak,
                &peak_err);
  EXPECT_GT(peak, 1e-3) << "every compared command was ~0 — the cross-check is vacuous";
  EXPECT_GT(peak_err, 1e-4) << "the pose error never left zero — the K_p branch was not exercised";
}

// Same recipe with the dynamically-consistent null-space posture task active
// (nv = 7 > 6 makes Nᵀ non-trivial). This pins the binding's Nᵀ wiring as well
// as the core, which is what S7 will inherit.
TEST(TaskAccelLaw, CoreDrivenShimMatchesTheAdapterBitwiseWithNullSpaceTask) {
  auto g = CrossCheckGains();
  g.null_kp = 2.0;
  g.null_kd = 0.5;

  rtc::OperationalSpaceController ctrl(Serial7dof(), g);
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);

  OscShim shim;
  const std::array<double, 6> goal{0.28, -0.12, 0.40, -0.10, 0.20, 0.15};
  double peak = 0.0;
  double peak_err = 0.0;
  RunCrossCheck(ctrl, shim, g, goal, /*retarget_tick=*/2, /*ticks=*/40, /*dt=*/0.002, &peak,
                &peak_err);
  EXPECT_GT(peak, 1e-3) << "every compared command was ~0 — the cross-check is vacuous";
  EXPECT_GT(peak_err, 1e-4) << "the pose error never left zero — the K_p branch was not exercised";
}

// A near-π retarget takes the split branch, so the shim must reproduce the
// two-segment state machine (has_pending_segment_ / pending_duration_) as well
// as the law. This is the transition R5 of the plan flagged as the place a
// future binding is most likely to get the trajectory wiring wrong.
TEST(TaskAccelLaw, CoreDrivenShimMatchesTheAdapterAcrossAPiRotationSplit) {
  auto g = CrossCheckGains();
  // The split's first segment covers HALF the rotation, and at the default
  // angular speed that lasts longer than any practical tick budget. Raising the
  // two angular limits parameterises the TRAJECTORY, not the law (D-T) — the
  // gains the core reads are untouched.
  g.trajectory_angular_speed = 2.0;
  g.max_traj_angular_velocity = 4.0;

  rtc::OperationalSpaceController ctrl(Serial7dof(), g);
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);

  OscShim shim;
  constexpr int kRetargetTick = 2;
  constexpr double kDt = 0.01;
  // π − 0.05 clears the adapter's kPiSafetyMargin of 0.15.
  const std::array<double, 6> goal =
      GoalRotatedFromTcp(kRetargetTick, kDt, M_PI - 0.05, Eigen::Vector3d(0.3, -0.5, 0.8),
                         Eigen::Vector3d(0.02, 0.0, -0.01));
  double peak = 0.0;
  double peak_err = 0.0;
  // Long enough to cross the first segment's end and run into segment 2.
  RunCrossCheck(ctrl, shim, g, goal, kRetargetTick, /*ticks=*/140, kDt, &peak, &peak_err);
  EXPECT_GT(peak, 1e-3) << "every compared command was ~0 — the cross-check is vacuous";
  EXPECT_GT(peak_err, 1e-4) << "the pose error never left zero — the K_p branch was not exercised";
  // Without these the test would still pass on a straight (single-segment)
  // retarget and would pin none of the state machine it was written for.
  EXPECT_EQ(shim.splits(), 1) << "the π-rotation split branch never fired — goal is not near π";
  EXPECT_GT(shim.transitions(), 0)
      << "segment 1 never ended — the run is too short to reach the transition";
}

// ═══════════════════════════════════════════════════════════════════════════
// RT contract
// ═══════════════════════════════════════════════════════════════════════════

// RT-1: the law runs on the tick. It is fixed-size Eigen throughout, so this
// gate is a structural regression guard — mutation-verified by inserting a
// runtime-sized allocation (a `new`+immediate-`delete` pair would be elided).
TEST(TaskAccelLaw, IsAllocationFree) {
  auto rng = MakeRng();

  const CoreDraw warm = RandomDraw(rng, 0);
  Vec6 sink = RunCore(warm);  // warm up outside the gate

  std::vector<CoreDraw> draws;
  draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial)
    draws.push_back(RandomDraw(rng, trial));

  g_alloc_count = 0;
  g_alloc_active = true;
  for (const auto& d : draws)
    sink += RunCore(d);
  g_alloc_active = false;

  EXPECT_EQ(g_alloc_count, 0u) << "the task-acceleration law allocated on the RT path (RT-1)";
  EXPECT_TRUE(std::isfinite(sink.norm())) << "sink kept the calls from being optimised away";
}
