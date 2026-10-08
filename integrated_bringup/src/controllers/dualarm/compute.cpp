// ── DemoDualArmController: the RT tick ──────────────────────────────────────
//
// Everything in this file runs on the control tick. No allocation, no logging
// outside the throttled forms, no locks; the solve's own allocations are the
// solver backend's (counted by the allocation test, not owned here).

#include "integrated_bringup/controllers/demo_dualarm_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_math/se3/so3.hpp"
#include "rtc_tsid/kinematics/se3_error.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace integrated_bringup {

namespace {

using rtc::ControllerOutput;
using rtc::ControllerState;
using Hold = DualArmDiagLogPod::Hold;
using GoalDrop = DualArmDiagLogPod::GoalDrop;

/// Floor of a trajectory speed at the point of use (NUM-4): a speed divides a
/// distance, and the parameter path is not the only writer of the gains.
constexpr double kMinSpeed = 1e-6;
/// Shortest trajectory: a goal on top of the current reference still gets a
/// well-conditioned polynomial.
constexpr double kMinDuration = 0.01;
/// Peak velocity of a rest-to-rest quintic over its mean velocity.
constexpr double kQuinticPeakRatio = 15.0 / 8.0;
/// A goal whose rotation from the current reference is within this of π is
/// refused: the tangent-space interpolation picks its axis by a sign there, so
/// the path — not just the result — would be decided by rounding.
constexpr double kNearPiMargin = 0.15;

void StorePose(const pinocchio::SE3& pose, std::array<double, 3>& position,
               std::array<double, 4>& quaternion) noexcept {
  const Eigen::Vector3d& t = pose.translation();
  const Eigen::Quaterniond q(pose.rotation());
  position = {t.x(), t.y(), t.z()};
  quaternion = {q.w(), q.x(), q.y(), q.z()};
}

void StorePose(const pinocchio::SE3& pose, rtc::Pose& out) noexcept {
  StorePose(pose, out.position, out.quaternion);
}

void StoreTaskSpace(const pinocchio::SE3& pose,
                    std::array<double, rtc::kTaskSpaceDim>& out) noexcept {
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(pose.rotation());
  out = {pose.translation().x(),
         pose.translation().y(),
         pose.translation().z(),
         rpy[0],
         rpy[1],
         rpy[2]};
}

}  // namespace

// ── Requests from other threads ─────────────────────────────────────────────

void DemoDualArmController::ServiceRequests() noexcept {
  // Activation: a fresh start from wherever the robot is.
  const std::uint32_t generation = ActivationGeneration();
  if (!activation_seen_ || generation != serviced_generation_) {
    serviced_generation_ = generation;
    activation_seen_ = true;
    need_reseed_ = true;
    hand_seeded_ = false;
    // The fault latch is NOT dropped: a deactivate / activate cycle is not the
    // reset service, and must not pass for one.
  }

  // The fault reset, BEFORE the E-STOP edge below: the manager's reset service
  // works during an E-STOP and reads HasLatchedFault() back two ticks later.
  const std::uint32_t fault_epoch = fault_reset_epoch_.load(std::memory_order_acquire);
  if (fault_epoch != serviced_fault_reset_epoch_) {
    serviced_fault_reset_epoch_ = fault_epoch;
    if (fault_latched_.load(std::memory_order_relaxed)) {
      fault_latched_.store(false, std::memory_order_release);
      fault_cause_ = DualArmDiagLogPod::FaultCause::kNone;
      qp_fail_streak_ = 0;
      need_reseed_ = true;
      hand_seeded_ = false;
    }
  }

  // Either E-STOP edge re-seeds: on the trigger the command stops being what
  // the robot follows, on the clear it must start from where the robot is.
  const std::uint32_t estop_epoch = estop_epoch_.load(std::memory_order_acquire);
  if (estop_epoch != serviced_estop_epoch_) {
    serviced_estop_epoch_ = estop_epoch;
    need_reseed_ = true;
    hand_seeded_ = false;
  }
}

// ── Seeding and the evaluation state ────────────────────────────────────────

void DemoDualArmController::UpdateEvalCache() noexcept {
  // The solve is configured to evaluate at the COMMAND state: the cache it
  // reads carries this controller's own command on the body joints. The hand
  // entries stay measured — the hand is commanded in joint space, and the
  // solve does not check them.
  q_eval_ = combined_cache_->q();
  v_eval_ = combined_cache_->v();
  for (int i = 0; i < body_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_eval_[combined_cache_->ext_to_pin_q(i)] = q_cmd_[ui];
    v_eval_[combined_cache_->ext_to_pin_v(i)] = qd_cmd_[ui];
  }
  combined_cache_->cache().Update(q_eval_, v_eval_);
}

void DemoDualArmController::Reseed(const ControllerState& state) noexcept {
  const auto& dev = state.devices[kDualArmBodyDeviceIdx];
  for (int i = 0; i < body_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_cmd_[ui] = dev.positions[ui];
    qd_cmd_[ui] = 0.0;
  }
  // The anchor and the previous velocity go with the command: a carried-over
  // anchor would fail the solve's command-state check, and a carried-over
  // velocity would pull the first solve toward a motion that has ended.
  clik_.ResetAnchor();
  qp_fail_streak_ = 0;

  UpdateEvalCache();
  // The posture target and every task goal become "where things are now":
  // nothing resumes toward a goal set before the seed.
  q_posture_des_ = q_eval_;
  const auto& frames = combined_cache_->cache().registered_frames;
  for (std::size_t k = 0; k < num_tasks_; ++k) {
    TaskRt& task = tasks_[k];
    const pinocchio::SE3 current = frames[static_cast<std::size_t>(task.base_idx)].oMf.actInv(
        frames[static_cast<std::size_t>(task.frame_idx)].oMf);
    task.goal = current;
    task.ref_pose = current;
    task.twist_ff.setZero();
    task.traj_active = false;
    task.traj_time = 0.0;
    task.goal_sequence = 0;
    task.err_norm = 0.0;
  }
  seeded_ = true;
  need_reseed_ = false;
  reseeded_this_tick_ = true;
}

// ── Goals and references ────────────────────────────────────────────────────

void DemoDualArmController::ConsumeTaskGoals(bool apply) noexcept {
  for (std::size_t k = 0; k < num_tasks_; ++k) {
    TaskRt& task = tasks_[k];
    // Loaded unconditionally: reading the sequence and the payload in two
    // steps would let a writer land between them.
    const TaskGoal goal = ingress_[k].box.Load();
    if (goal.sequence == task.seen_sequence) {
      continue;
    }
    // Consumed on EVERY tick, also the ones that cannot apply it: a goal sent
    // during a stop, a latched fault or an unreadable spell must not land when
    // that ends, later and with no operator expecting it.
    task.seen_sequence = goal.sequence;
    if (!IsCurrentGeneration(goal.generation)) {
      ++task.drops[static_cast<std::size_t>(GoalDrop::kStale)];
    } else if (!apply) {
      ++task.drops[static_cast<std::size_t>(GoalDrop::kHeld)];
    } else if (!ApplyTaskGoal(k, goal)) {
      ++task.drops[static_cast<std::size_t>(GoalDrop::kNearPi)];
    }
  }
}

bool DemoDualArmController::ApplyTaskGoal(std::size_t k, const TaskGoal& goal) noexcept {
  TaskRt& task = tasks_[k];
  const Eigen::Vector3d rpy(goal.pose[3], goal.pose[4], goal.pose[5]);
  const pinocchio::SE3 in_goal_frame(rtc::math::se3::RpyToRotationZyx(rpy),
                                     Eigen::Vector3d(goal.pose[0], goal.pose[1], goal.pose[2]));

  // Into the task's base frame — ONCE, here, at this tick's command state:
  //   T_base,des = T_base,ref(q_c) · T_ref,des,   T_base,ref = oMf_base⁻¹ · oMf_ref.
  // From now on the goal is a pose fixed in the base frame; a base that moves
  // afterwards carries the goal with it.
  pinocchio::SE3 in_base = in_goal_frame;
  if (goal.frame_slot >= 0) {
    const int ref_idx = target_frame_idx_[static_cast<std::size_t>(goal.frame_slot)];
    if (ref_idx != task.base_idx) {
      const auto& frames = combined_cache_->cache().registered_frames;
      const pinocchio::SE3 base_from_ref =
          frames[static_cast<std::size_t>(task.base_idx)].oMf.actInv(
              frames[static_cast<std::size_t>(ref_idx)].oMf);
      in_base = base_from_ref.act(in_goal_frame);
    }
  }

  // The trajectory starts at the current REFERENCE (not at the commanded
  // frame pose): a goal that arrives mid-motion continues from where the
  // reference is, at zero velocity.
  const pinocchio::SE3& start = task.ref_pose;
  const Eigen::Matrix3d relative = start.rotation().transpose() * in_base.rotation();
  const double angle = Eigen::AngleAxisd(relative).angle();
  if (!(angle <= std::numbers::pi - kNearPiMargin)) {
    return false;  // also a non-finite angle
  }
  const double distance = (in_base.translation() - start.translation()).norm();
  const double v_lin = std::max(kMinSpeed, gains_tick_.linear_speed);
  const double v_ang = std::max(kMinSpeed, gains_tick_.angular_speed);
  const double duration = std::max({kMinDuration, distance / v_lin, angle / v_ang,
                                    kQuinticPeakRatio * distance / cfg_.linear_speed_max,
                                    kQuinticPeakRatio * angle / cfg_.angular_speed_max});
  task.traj.initialize(start, pinocchio::Motion::Zero(), in_base, pinocchio::Motion::Zero(),
                       duration);
  task.traj_time = 0.0;
  task.traj_active = true;
  task.goal = in_base;
  task.goal_sequence = goal.sequence;
  return true;
}

void DemoDualArmController::AdvanceReference(std::size_t k, double dt) noexcept {
  TaskRt& task = tasks_[k];
  if (!task.traj_active) {
    task.ref_pose = task.goal;
    task.twist_ff.setZero();
    return;
  }
  task.traj_time += dt;
  if (task.traj_time >= task.traj.duration()) {
    task.traj_active = false;
    task.ref_pose = task.goal;
    task.twist_ff.setZero();
    return;
  }
  const auto state = task.traj.compute(task.traj_time, dt);
  task.ref_pose = state.pose;
  // The trajectory's velocity is a BODY twist (the reference frame's own
  // axes). The solve takes the time derivative of the reference pose in the
  // BASE frame's axes — origin velocity and angular velocity — so both halves
  // are rotated by the reference orientation.
  const Eigen::Matrix3d& R = state.pose.rotation();
  task.twist_ff.head<3>() = R * state.velocity.linear();
  task.twist_ff.tail<3>() = R * state.velocity.angular();
}

// ── The solve ───────────────────────────────────────────────────────────────

void DemoDualArmController::LatchFault(DualArmDiagLogPod::FaultCause cause) noexcept {
  if (!fault_latched_.load(std::memory_order_relaxed)) {
    fault_cause_ = cause;
    fault_latched_.store(true, std::memory_order_release);
  }
}

void DemoDualArmController::RunClikTick(const ControllerState& state, double dt) noexcept {
  using Clik = rtc::tsid::ClikReferenceGenerator;
  const double k_max = dt > 0.0 ? 1.0 / dt : 0.0;
  const auto& frames = combined_cache_->cache().registered_frames;

  for (std::size_t k = 0; k < num_tasks_; ++k) {
    AdvanceReference(k, dt);
    TaskRt& task = tasks_[k];
    Clik::FrameTask& frame_task = frame_tasks_[k];
    frame_task.kind = Clik::TaskKind::kSe3;
    frame_task.frame_idx = task.frame_idx;
    frame_task.base_frame_idx = task.base_idx;
    frame_task.placement_des = task.ref_pose;
    frame_task.twist_ff = &task.twist_ff;
    // Bounded where they are used: [0, 1/dt]. A non-finite gain passes through
    // std::clamp unchanged and the solve refuses the call, which is the
    // outcome it should have.
    const double k_lin = std::clamp(gains_tick_.task_gain_linear[k], 0.0, k_max);
    const double k_ang = std::clamp(gains_tick_.task_gain_angular[k], 0.0, k_max);
    frame_task.gain << k_lin, k_lin, k_lin, k_ang, k_ang, k_ang;
    frame_task.weight = task.weight;
    frame_task.fb_lin_max = task.fb_lin_max;
    frame_task.fb_ang_max = task.fb_ang_max;

    // The error the solve is about to feed back, per task — the core reports
    // task 0's only. Same convention, same inputs: the frame in its base
    // frame at the command state, against the reference.
    const pinocchio::SE3 commanded = frames[static_cast<std::size_t>(task.base_idx)].oMf.actInv(
        frames[static_cast<std::size_t>(task.frame_idx)].oMf);
    const Eigen::Matrix<double, 6, 1> error =
        rtc::tsid::ComputeTaskPoseError(commanded, task.ref_pose);
    task.err_norm = error.norm();
    auto& row = tick_.tasks[k];
    row.valid = true;
    row.err_lin = error.head<3>().norm();
    row.err_ang = error.tail<3>().norm();
    StorePose(task.ref_pose, row.ref_pos, row.ref_quat);
    StorePose(commanded, row.cmd_pos, row.cmd_quat);
  }
  for (std::size_t g = 0; g < num_posture_groups_; ++g) {
    (void)clik_.SetPostureGroupGain(static_cast<int>(g),
                                    std::clamp(gains_tick_.posture_gain[g], 0.0, k_max));
  }

  Clik::MultiFrameInput input;
  input.tasks = std::span<const Clik::FrameTask>(frame_tasks_.data(), num_tasks_);
  input.q_posture_des = &q_posture_des_;
  input.dt = dt;
  const bool ok = clik_.Compute(combined_cache_->cache(), input);
  clik_ran_this_tick_ = true;

  if (ok) {
    const Eigen::VectorXd& q_ref = clik_.QRef();
    const Eigen::VectorXd& v_ref = clik_.VRef();
    for (int i = 0; i < body_dof_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_cmd_[ui] = q_ref[combined_cache_->ext_to_pin_q(i)];
      qd_cmd_[ui] = v_ref[combined_cache_->ext_to_pin_v(i)];
    }
    qp_fail_streak_ = 0;
  } else {
    // The previous command stands. The command velocity goes to 0 with it: the
    // solve's own previous velocity is 0 after a failure, and the state it is
    // evaluated at next tick has to say the same.
    for (int i = 0; i < body_dof_; ++i) {
      qd_cmd_[static_cast<std::size_t>(i)] = 0.0;
    }
    ++qp_fail_streak_;
    if (qp_fail_streak_ >= cfg_.max_qp_fail_ticks) {
      LatchFault(DualArmDiagLogPod::FaultCause::kQpFailStreak);
    }
  }

  // The one view of the real robot this controller has (the solve never reads
  // the measured body joints): how far the measurement is from the command.
  const auto& dev = state.devices[kDualArmBodyDeviceIdx];
  double track_err = 0.0;
  for (int i = 0; i < body_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    track_err = std::max(track_err, std::abs(dev.positions[ui] - q_cmd_[ui]));
  }
  tick_.track_err = track_err;
  if (cfg_.track_err_max > 0.0 && track_err > cfg_.track_err_max) {
    LatchFault(DualArmDiagLogPod::FaultCause::kTrackError);
  }
}

// ── The hand ────────────────────────────────────────────────────────────────

void DemoDualArmController::RunHandLane(const ControllerState& state, double dt) noexcept {
  if (hand_dof_ <= 0 || state.num_devices <= kDualArmHandDeviceIdx) {
    return;
  }
  if (estop_active_ || !hand_readable_) {
    return;  // frozen; the seed is retaken after a stop (ServiceRequests)
  }
  const auto& dev = state.devices[kDualArmHandDeviceIdx];
  if (!hand_seeded_) {
    for (int i = 0; i < hand_dof_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      hand_cmd_[ui] = dev.positions[ui];
      hand_goal_[ui] = dev.positions[ui];
    }
    hand_traj_active_ = false;
    hand_traj_time_ = 0.0;
    hand_seeded_ = true;
    return;
  }
  if (!hand_traj_active_) {
    return;
  }
  hand_traj_time_ += dt;
  if (hand_traj_time_ >= hand_traj_.duration()) {
    hand_traj_active_ = false;
    for (int i = 0; i < hand_dof_; ++i) {
      hand_cmd_[static_cast<std::size_t>(i)] = hand_goal_[static_cast<std::size_t>(i)];
    }
    return;
  }
  const auto traj = hand_traj_.compute(hand_traj_time_);
  for (int i = 0; i < hand_dof_; ++i) {
    hand_cmd_[static_cast<std::size_t>(i)] = traj.positions[static_cast<std::size_t>(i)];
  }
}

void DemoDualArmController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                               bool /*is_task*/) noexcept {
  if (device_idx == kDualArmBodyDeviceIdx) {
    // The body group's joint goal is the posture target.
    if (!seeded_ || static_cast<int>(values.size()) != body_dof_) {
      group_goal_rejects_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    for (int i = 0; i < body_dof_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_posture_des_[combined_cache_->ext_to_pin_q(i)] =
          std::clamp(values[ui], body_q_min_[ui], body_q_max_[ui]);
    }
    return;
  }
  if (device_idx != kDualArmHandDeviceIdx || !hand_seeded_ ||
      static_cast<int>(values.size()) != hand_dof_) {
    group_goal_rejects_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  // The hand: a quintic from where its command is now. A goal that arrives
  // mid-motion keeps the current velocity, so the command stays smooth.
  using HandTrajectory = rtc::trajectory::JointSpaceTrajectory<kDualArmMaxHandDof>;
  HandTrajectory::State start{};
  if (hand_traj_active_) {
    start = hand_traj_.compute(hand_traj_time_);
  } else {
    for (int i = 0; i < hand_dof_; ++i) {
      start.positions[static_cast<std::size_t>(i)] = hand_cmd_[static_cast<std::size_t>(i)];
    }
  }
  HandTrajectory::State goal{};
  double max_distance = 0.0;
  for (int i = 0; i < hand_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    hand_goal_[ui] = std::clamp(values[ui], hand_q_min_[ui], hand_q_max_[ui]);
    goal.positions[ui] = hand_goal_[ui];
    max_distance = std::max(max_distance, std::abs(hand_goal_[ui] - start.positions[ui]));
  }
  const double speed = std::max(kMinSpeed, gains_tick_.hand_speed);
  const double duration = std::max(
      {kMinDuration, max_distance / speed, kQuinticPeakRatio * max_distance / cfg_.hand_speed_max});
  hand_traj_.initialize(start, goal, duration);
  hand_traj_time_ = 0.0;
  hand_traj_active_ = true;
}

// ── Measured kinematics: the published transforms and the log's poses ───────

void DemoDualArmController::UpdateMeasuredKinematics(const ControllerState& state,
                                                     ControllerOutput& output) noexcept {
  output.arm_tip_pose_valid = false;
  output.virtual_tcp_pose_valid = false;
  output.task_link_pose_valid.fill(false);
  if (!meas_ready_ || !body_readable_) {
    // Withheld rather than repeated: the cache would hold whatever it had
    // before the outage, which is not this tick's measurement.
    return;
  }
  // A second Data on the control model, at the MEASURED configuration (the
  // cache the solve reads is at the command). Published under `_actual`
  // names, so it has to be what the robot did, not what it was told.
  const pinocchio::Model& model = *combined_cache_->model();
  pinocchio::Data& data = *meas_data_;
  pinocchio::forwardKinematics(model, data, combined_cache_->q());
  const pinocchio::SE3 world_from_root = pinocchio::updateFramePlacement(model, data, root_fid_);

  pinocchio::SE3 root_from_tip = pinocchio::SE3::Identity();
  if (has_tip_) {
    root_from_tip = world_from_root.actInv(pinocchio::updateFramePlacement(model, data, tip_fid_));
    StorePose(root_from_tip, output.arm_tip_pose);
    output.arm_tip_pose_valid = true;
  }

  // Slots [0, num_fingertips_) are the fingertips, the tasks follow — the
  // order on_configure registered the transform slots in.
  if (has_tip_ && hand_readable_ && hand_handle_ &&
      RunHandForwardKinematics(closed_hand_fk_, hand_handle_.get(), hand_q_, state)) {
    for (std::size_t f = 0; f < num_fingertips_; ++f) {
      pinocchio::SE3 hand_root_from_tip;
      if (HandFingertipPoseDispatch(closed_hand_fk_, hand_handle_.get(), fingertip_frame_ids_,
                                    use_hand_root_frame_, hand_root_frame_id_, f,
                                    hand_root_from_tip)) {
        // The hand FK is expressed in the hand tree's root link, which is the
        // tip frame above: one composition, no mount transform in between.
        StorePose(root_from_tip.act(hand_root_from_tip), output.task_link_poses[f]);
        output.task_link_pose_valid[f] = true;
      }
    }
  }
  for (std::size_t k = 0; k < num_tasks_; ++k) {
    const TaskRt& task = tasks_[k];
    const pinocchio::SE3 world_from_frame =
        pinocchio::updateFramePlacement(model, data, task.frame_fid);
    const std::size_t slot = num_fingertips_ + k;
    StorePose(world_from_root.actInv(world_from_frame), output.task_link_poses[slot]);
    output.task_link_pose_valid[slot] = true;

    const pinocchio::SE3 base_from_frame =
        pinocchio::updateFramePlacement(model, data, task.base_fid).actInv(world_from_frame);
    auto& row = tick_.tasks[k];
    row.meas_valid = true;
    StorePose(base_from_frame, row.meas_pos, row.meas_quat);
    if (k == 0) {
      // The device-state log's task columns follow task 0.
      StoreTaskSpace(base_from_frame, output.actual_task_positions);
    }
  }
  if (num_tasks_ > 0 && seeded_) {
    StoreTaskSpace(tasks_[0].goal, output.task_goal_positions);
    StoreTaskSpace(tasks_[0].ref_pose, output.trajectory_task_positions);
    for (std::size_t i = 0; i < output.trajectory_task_velocities.size(); ++i) {
      output.trajectory_task_velocities[i] = tasks_[0].twist_ff[static_cast<Eigen::Index>(i)];
    }
  }
}

// ── Commands ────────────────────────────────────────────────────────────────

void DemoDualArmController::WriteBodyCommand(const ControllerState& state,
                                             ControllerOutput& output) noexcept {
  if (state.num_devices <= kDualArmBodyDeviceIdx) {
    return;
  }
  const auto& dev = state.devices[kDualArmBodyDeviceIdx];
  auto& out = output.devices[kDualArmBodyDeviceIdx];
  out.goal_type = rtc::GoalType::kJoint;
  // Nothing honest to command: an unreadable device, or no command seeded yet.
  // Zero-length is "no update"; a full-width command would be a real one.
  if (!body_readable_ || (!estop_active_ && !seeded_)) {
    rtc::SilenceDeviceOutput(out);
    rtc::HoldTelemetryAtMeasured(out, dev.num_channels, std::span<const double>(dev.positions));
    return;
  }
  const int width = rtc::SelfReportedChannelBound(dev, static_cast<int>(rtc::kMaxDeviceChannels));
  out.num_channels = width;
  for (int i = 0; i < body_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    // Under an E-STOP the measured position (the manager substitutes its own
    // hold on the wire; this is the second layer). Otherwise the command —
    // integrated this tick, or held since the last tick that integrated it.
    out.commands[ui] = estop_active_ ? dev.positions[ui] : q_cmd_[ui];
    out.target_positions[ui] = out.commands[ui];
    out.trajectory_positions[ui] = out.commands[ui];
    out.trajectory_velocities[ui] = (estop_active_ || !clik_ran_this_tick_) ? 0.0 : qd_cmd_[ui];
    out.goal_positions[ui] =
        seeded_ ? q_posture_des_[combined_cache_->ext_to_pin_q(i)] : dev.positions[ui];
  }
  // Wire channels past the model's joints follow their own measurement.
  rtc::FillCommandTail(std::span<double>(out.commands), body_dof_, width,
                       rtc::CommandType::kPosition, std::span<const double>(dev.positions));
}

void DemoDualArmController::WriteHandCommand(const ControllerState& state,
                                             ControllerOutput& output) noexcept {
  if (hand_dof_ <= 0 || state.num_devices <= kDualArmHandDeviceIdx) {
    return;
  }
  const auto& dev = state.devices[kDualArmHandDeviceIdx];
  auto& out = output.devices[kDualArmHandDeviceIdx];
  out.goal_type = rtc::GoalType::kJoint;
  if (!hand_readable_ || (!estop_active_ && !hand_seeded_)) {
    rtc::SilenceDeviceOutput(out);
    rtc::HoldTelemetryAtMeasured(out, dev.num_channels, std::span<const double>(dev.positions));
    return;
  }
  const int width = rtc::SelfReportedChannelBound(dev, static_cast<int>(rtc::kMaxDeviceChannels));
  out.num_channels = width;
  for (int i = 0; i < hand_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    out.commands[ui] = estop_active_ ? dev.positions[ui] : hand_cmd_[ui];
    out.target_positions[ui] = out.commands[ui];
    out.trajectory_positions[ui] = out.commands[ui];
    out.goal_positions[ui] = estop_active_ ? dev.positions[ui] : hand_goal_[ui];
  }
  rtc::FillCommandTail(std::span<double>(out.commands), hand_dof_, width,
                       rtc::CommandType::kPosition, std::span<const double>(dev.positions));
}

// ── The record ──────────────────────────────────────────────────────────────

void DemoDualArmController::FillTickRecord() noexcept {
  tick_.estop = estop_active_;
  tick_.fault_latched = fault_latched_.load(std::memory_order_relaxed);
  tick_.fault_cause = fault_cause_;
  tick_.body_readable = body_readable_;
  tick_.hand_readable = hand_readable_;
  tick_.reseeded = reseeded_this_tick_;
  tick_.clik_ran = clik_ran_this_tick_;
  if (clik_ran_this_tick_) {
    tick_.hold = Hold::kNone;
    const auto& solve = clik_.LastSolve();
    tick_.reached_solve = solve.reached_solve;
    tick_.converged = solve.converged;
    tick_.rejected_input = solve.rejected_input;
    tick_.non_finite = solve.non_finite;
    tick_.command_mismatch = solve.command_mismatch;
    tick_.accel_rows_violated = solve.accel_rows_violated;
    tick_.brake_box_empty = solve.brake_box_empty;
    tick_.status = solve.status;
    tick_.iterations = solve.iterations;
    tick_.solve_time_us = solve.solve_time_us;
    tick_.accel_rows = solve.accel_rows;
    tick_.accel_rows_binding = solve.accel_rows_binding;
    tick_.fb_saturated = solve.fb_saturated;
    tick_.rot_near_pi = solve.rot_near_pi;
    tick_.brake_active = solve.brake_active;
    tick_.brake_static_infeasible = solve.brake_static_infeasible;
  } else if (estop_active_) {
    tick_.hold = Hold::kEstop;
  } else if (!clik_ready_) {
    tick_.hold = Hold::kNoModel;
  } else if (!seeded_) {
    tick_.hold = Hold::kNotSeeded;
  } else if (!body_readable_) {
    tick_.hold = Hold::kUnreadable;
  } else {
    tick_.hold = Hold::kFault;
  }
  tick_.qp_fail_streak = qp_fail_streak_;
  tick_.group_goal_rejects = group_goal_rejects_.load(std::memory_order_relaxed);
  tick_.num_tasks = static_cast<std::uint8_t>(num_tasks_);
  for (std::size_t k = 0; k < num_tasks_; ++k) {
    auto& row = tick_.tasks[k];
    const TaskRt& task = tasks_[k];
    row.traj_active = task.traj_active;
    row.goal_sequence = task.goal_sequence;
    row.ingress_counts[0] =
        static_cast<std::uint32_t>(ingress_[k].accepted.load(std::memory_order_relaxed));
    for (std::size_t r = 1; r < row.ingress_counts.size(); ++r) {
      row.ingress_counts[r] =
          static_cast<std::uint32_t>(ingress_[k].reject_counts[r].load(std::memory_order_relaxed));
    }
    row.drop_counts = task.drops;
  }
  tick_.num_joints = static_cast<std::uint8_t>(body_dof_);
  for (int i = 0; i < body_dof_; ++i) {
    tick_.q_cmd[static_cast<std::size_t>(i)] = q_cmd_[static_cast<std::size_t>(i)];
  }
}

// ── Compute ─────────────────────────────────────────────────────────────────

ControllerOutput DemoDualArmController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoDualArmController::Compute");
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = rtc::CommandType::kPosition;
  output.valid = true;

  // A FRESH record every tick, so a block this tick does not compute is zero
  // rather than the previous tick's (PROC-7 by construction).
  tick_ = DualArmDiagLogPod{};
  tick_.t_relative_s = state.t_relative_s;
  tick_.tick = state.iteration;
  reseeded_this_tick_ = false;
  clik_ran_this_tick_ = false;
  const double dt = state.dt > 0.0 ? state.dt : GetDefaultDt();

  // Read ONCE per tick: re-reading the atomic further down would let one tick
  // act on two different answers.
  estop_active_ = estop_requested_.load(std::memory_order_acquire);
  body_readable_ = clik_ready_ && state.num_devices > kDualArmBodyDeviceIdx &&
                   rtc::IsDeviceReadable(state.devices[kDualArmBodyDeviceIdx], body_dof_);
  hand_readable_ = clik_ready_ && hand_dof_ > 0 && state.num_devices > kDualArmHandDeviceIdx &&
                   rtc::IsDeviceReadable(state.devices[kDualArmHandDeviceIdx], hand_dof_);
  gains_tick_ = gains_lock_.Load();

  ServiceRequests();
  if (clik_ready_) {
    // The measured state, once per tick (a no-op for a group that did not
    // report): the measured kinematics below read it, and the evaluation
    // state takes its hand entries from it.
    combined_cache_->ExtractFullState(state, body_dof_, hand_dof_);
    // The seed is retaken from the measurement as soon as there is one and no
    // stop is in force — also under a latched fault, whose held command would
    // otherwise still be the one from before a stop.
    if (need_reseed_ && !estop_active_ && body_readable_) {
      Reseed(state);
    }
  }
  const bool fault = fault_latched_.load(std::memory_order_relaxed);
  const bool run =
      clik_ready_ && seeded_ && !need_reseed_ && !estop_active_ && !fault && body_readable_;

  // The hand lane's seed comes BEFORE the group mailbox, so a hand goal that
  // arrives on the first readable tick has a command to start from.
  RunHandLane(state, dt);
  if (estop_active_ || fault) {
    // Discarded, not left queued: a goal issued during a stop would otherwise
    // land when the stop clears.
    DiscardPendingTargets();
  } else {
    (void)DrainPendingTargets();
  }

  if (run) {
    UpdateEvalCache();
  }
  ConsumeTaskGoals(run);
  if (run) {
    RunClikTick(state, dt);
  }

  UpdateMeasuredKinematics(state, output);
  WriteBodyCommand(state, output);
  WriteHandCommand(state, output);
  FillTickRecord();

  if (body_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kDualArmBodyDeviceIdx, pod);
    body_state_log_handle_.Push(pod);
  }
  if (hand_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kDualArmHandDeviceIdx, pod);
    hand_state_log_handle_.Push(pod);
  }
  if (diag_log_handle_) {
    diag_log_handle_.Push(tick_);
  }
  // Single exit: every branch above reaches the record and the logs.
  return output;
}

}  // namespace integrated_bringup
