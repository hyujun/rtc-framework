#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/fingertip_counts.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"  // FloorMaxDamping, kMinSigma0
#include "rtc_controllers/gain_floor.hpp"
#include "rtc_controllers/task/task_vel_law.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Phase 1: Read joint states + sensor data ────────────────────────────────
void DemoTaskController::ReadState(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::ReadState");
  // ── Unified kin&dyn: scatter measured joint pos/vel into Pinocchio order
  // (q_curr_full_/v_curr_full_). A raw read + reindex, so it belongs to ReadState;
  // the Compute-scope Stage-1 pinocchio_cache_.Update (just after this call)
  // consumes it. Same gate as the prior Compute-top block so it runs on exactly
  // the same ticks — byte-for-byte. The tip-Jacobian extraction + control pose
  // that used to live here consume the cache and moved to ComputeControl Stage 2
  // (after the Update).
  // arm_readable_ joins that gate (#236 S7b) — ExtractFullState gates internally
  // too, but stating it here keeps this tick's phases readable as one decision.
  if (!estop_active_ && arm_readable_ && combined_cache_.reorder_valid() &&
      task_tcp_frame_idx_ >= 0) {
    combined_cache_.ExtractFullState(state, arm_dof_, hand_dof_);
  }

  // Hand motor data: dev1.motor_positions[], motor_velocities[],
  // motor_efforts[] available via state.devices[1].motor_* (populated from
  // /p1a/motor_states)

  // Hand sensor data (per-fingertip)
  num_active_fingertips_ = 0;
  num_sensor_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    // Shared derivation (joint / task / wbc): active = inference-group count,
    // sensor = raw sensor-lane count. See DeriveFingertipCounts.
    const auto counts = DeriveFingertipCounts(dev1);
    num_active_fingertips_ = counts.active;
    num_sensor_fingertips_ = counts.sensor;

    // One SeqLock read for the whole fingertip loop: the force guard's tunables
    // are the only gains this phase needs, and set_gains() must not change the
    // threshold mid-loop.
    const auto gains_now = gains_lock_.Load();

    for (std::size_t f = 0; f < static_cast<std::size_t>(num_active_fingertips_); ++f) {
      auto& ft = fingertip_data_[f];
      const std::size_t base = f * kHandSensorValuesPerFingertipCapacity;

      for (std::size_t j = 0; j < kHandBaroChannelsCapacity; ++j) {
        ft.baro[j] = dev1.sensor_data[base + j];
      }
      for (std::size_t j = 0; j < 3; ++j) {
        ft.tof[j] = dev1.sensor_data[base + kHandBaroChannelsCapacity + j];
      }

      ft.valid = dev1.inference_enable[f];
      if (ft.valid) {
        const std::size_t ft_base = f * kHandInferenceValuesPerFingertipCapacity;
        ft.contact_flag = dev1.inference_data[ft_base];
        for (std::size_t j = 0; j < 3; ++j) {
          ft.force[j] = dev1.inference_data[ft_base + 1 + j];
          ft.displacement[j] = dev1.inference_data[ft_base + 4 + j];
        }
        ft.force_mag = std::sqrt(ft.force[0] * ft.force[0] + ft.force[1] * ft.force[1] +
                                 ft.force[2] * ft.force[2]);
      } else {
        ft.contact_flag = 0.0f;
        ft.force = {};
        ft.displacement = {};
        ft.force_mag = 0.0f;
      }
      UpdateFingertipForceFilter(f, {gains_now.contact_stop_force_guard_delta_n,
                                     gains_now.contact_stop_force_guard_max_hold_ticks});
    }
  }
}

// ── Per-axis fingertip force LPF ────────────────────────────────────────────
//
// Runs from ReadState for every parsed fingertip, on every tick — including
// E-STOP ticks, where ReadState still decodes real sensor data. Keeping it
// unconditional is what makes the logged filtered column comparable across
// runs and keeps the filter warm so the first post-E-STOP contact is judged on
// a settled signal rather than a ramp.
//
// Invalid fingertip: freeze the delay line (do NOT feed the zeroed force — an
// IIR carries a phantom sample forward for several ticks, the same hazard the
// hand-position filter is gated against) and drop the primed flag so the next
// valid sample seeds instead of ramping. Outputs go to zero to match raw
// `ft.force`, which ReadState has already cleared.
//
// The guard is invalidated on that same branch. Keeping its last-accepted
// triplet across a dropout would compare the re-entry force against a base the
// world has left behind — a lane that drops at 5 N and returns at 0.2 N is a
// 4.8 N step, so the guard would hold the stale 5 N and the filter would seed
// from it (#325 follow-up: that is exactly what the re-entry test pins).
void DemoTaskController::UpdateFingertipForceFilter(
    std::size_t f, const FingertipForceGuardConfig& guard_cfg) noexcept {
  const auto& ft = fingertip_data_[f];
  const std::size_t base = f * 3;
  if (!ft.valid) {
    force_lpf_primed_[f] = false;
    force_guard_[f].Invalidate();
    force_guard_rejected_[f] = 0;
    for (std::size_t j = 0; j < 3; ++j) {
      fingertip_force_filt_[base + j] = 0.0F;
      fingertip_force_guarded_[base + j] = 0.0F;
    }
    fingertip_force_mag_filt_[f] = 0.0F;
    return;
  }

  const std::array<double, 3> raw{static_cast<double>(ft.force[0]),
                                  static_cast<double>(ft.force[1]),
                                  static_cast<double>(ft.force[2])};
  // Delta-spike guard: the LPF is fed `guarded`, never `raw`. `ft.force` itself
  // is untouched, so grasp detection, GraspState, the pull estimator and the
  // CSV raw columns keep the driver's wire value.
  const auto guarded = force_guard_[f].Apply(raw, guard_cfg);
  force_guard_rejected_[f] = guarded.rejected ? 1U : 0U;
  for (std::size_t j = 0; j < 3; ++j) {
    fingertip_force_guarded_[base + j] = static_cast<float>(guarded.value[j]);
  }
  if (!force_lpf_primed_[f]) {
    force_lpf_[f].Reset();
    force_lpf_[f].Seed(guarded.value);
    force_lpf_primed_[f] = true;
  }
  const std::array<double, 3> filt = force_lpf_[f].Apply(guarded.value);
  for (std::size_t j = 0; j < 3; ++j) {
    fingertip_force_filt_[base + j] = static_cast<float>(filt[j]);
  }
  fingertip_force_mag_filt_[f] =
      static_cast<float>(std::sqrt(filt[0] * filt[0] + filt[1] * filt[1] + filt[2] * filt[2]));
}

// ── Virtual TCP computation ─────────────────────────────────────────────────

void DemoTaskController::UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp,
                                          const Gains& gains) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::UpdateVirtualTcp");
  vtcp_valid_ = false;
  // Cleared beside vtcp_valid_ so the two early returns below cannot leave a
  // previous tick's participating set behind (#292).
  vtcp_member_mask_ = 0;
  if (!hand_handle_ || gains.vtcp.mode == VirtualTcpMode::kDisabled)
    return;

  // Build fingertip inputs from hand model FK (closed-chain-consistent when the
  // #121 helper is active; serial hand_handle_ FK otherwise — same value).
  for (std::size_t f = 0; f < kNumFingertips; ++f) {
    pinocchio::SE3 ft_pose;
    vtcp_inputs_[f].active = HandFingertipPose(f, ft_pose);
    if (!vtcp_inputs_[f].active)
      continue;
    vtcp_member_mask_ |= static_cast<std::uint8_t>(1U << f);
    vtcp_inputs_[f].position_in_tcp = ft_pose.translation();
    // Force magnitude for weighted mode (cached in ReadState; 0 when !valid).
    vtcp_inputs_[f].force_magnitude = static_cast<double>(fingertip_data_[f].force_mag);
  }

  // #292 frame identity: which fingertips define this tick's control point.
  // kConstant reads none of them, so its mask is pinned — otherwise a fingertip
  // dropping out would flip a constant-offset vtcp's frame identity for no
  // reason and force a spurious re-seed. kWeighted deliberately tracks
  // membership only: its weight (1 + |F|) drifts continuously, which moves the
  // control point smoothly rather than discontinuously.
  if (gains.vtcp.mode == VirtualTcpMode::kConstant) {
    vtcp_member_mask_ = 0xFF;
  }

  const auto result = ComputeVirtualTcp(gains.vtcp, T_base_tcp, vtcp_inputs_);
  if (result.valid) {
    T_tcp_vtcp_ = result.T_tcp_vtcp;
    vtcp_pose_ = result.world_pose;
    vtcp_valid_ = true;
  }
}

// ── Phase 2: Compute control (CLIK/IK + trajectory + sensor logic) ──────────

void DemoTaskController::ComputeControl(const ControllerState& state, double dt,
                                        const Gains& gains) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::ComputeControl");
  // ── Arm TCP pose: cache once for this tick (Update() ran in Compute() before
  // ComputeControl). ArmTcpPoseFromCache is internally gated (Identity when the
  // cache is unconfigured/non-fresh), so this unconditional read reproduces what
  // the CLIK law and FillLog/FillPublishOutput used to read per-consumer — now a
  // single source. Set before the early-return gate below so the non-CLIK path
  // (frame unregistered / reorder invalid) still leaves a defined value.
  arm_tcp_pose_ = combined_cache_.ArmTcpPoseFromCache(task_tcp_frame_idx_, task_base_frame_idx_);

  // ── E-stop / cache-readiness check (estop_active_ loaded at top of Compute) ─
  // CLIK sources the arm TCP pose + Jacobian from the unified cache; hold (skip
  // CLIK) unless the cache is configured (task_tcp_frame_idx_ >= 0) AND was
  // refreshed this tick (joint_reorder_valid_ → Update() ran — same condition as
  // the Compute()-level Update). Without the reorder guard a config where the tip
  // frame registers but the reorder map fails would run CLIK on an un-refreshed
  // (zero-J / stale-pose) cache. DrainTargetSlot's hold fallback keeps desired_q_
  // seeded from measured in that case so WriteJointCommand holds position.
  // !arm_readable_ joins the hold condition (#265 audit T4/T5): both the
  // desired_q_ re-seed on a new target and the null-space error read
  // dev0.positions nv deep, so an unread joint would enter the CLIK law as 0.
  if (estop_active_ || !arm_readable_ || !combined_cache_.reorder_valid() ||
      task_tcp_frame_idx_ < 0) {
    return;
  }

  // ── Stage 2 (compute control law): consume the model. Extract the arm-joint
  // columns of the tip Jacobian from the Stage-1 cache into the 6×nv_arm CLIK
  // buffer via the ext→Pinocchio velocity map (moved here from ReadState — it
  // consumes the cache, so it belongs after the Update). The gate above already
  // guarantees joint_reorder_valid_ && task_tcp_frame_idx_ >= 0, so no extra guard
  // is needed. arm_handle_ is not read here (retained only for the E-STOP TF path).
  {
    const auto& frames = combined_cache_.cache().registered_frames;
    const auto& tip_J = frames[static_cast<std::size_t>(task_tcp_frame_idx_)].J;
    const int narm = std::min(arm_dof_, static_cast<int>(J_full_.cols()));
    J_full_.setZero();
    for (int i = 0; i < narm; ++i) {
      const auto pv = static_cast<Eigen::Index>(combined_cache_.ext_to_pin_v(i));
      J_full_.col(i) = tip_J.col(pv);
    }
    if (task_base_frame_idx_ >= 0) {
      // Rotate the world-aligned Jacobian into the base frame's orientation
      // (rotation-only, matching the base-relative pose convention below).
      const Eigen::Matrix3d R_root_T =
          frames[static_cast<std::size_t>(task_base_frame_idx_)].oMf.rotation().transpose();
      J_full_.topRows(3) = R_root_T * J_full_.topRows(3);
      J_full_.bottomRows(3) = R_root_T * J_full_.bottomRows(3);
    }
    J_pos_.noalias() = J_full_.topRows(3);
  }

  // `gains` is the tick's single SeqLock snapshot, loaded in Compute() and
  // shared with ComputeSecondary — one atomic read, and the arm half and the
  // hand half of a tick cannot disagree about a gain.
  control_6dof_cached_ = gains.control_6dof;  // reused by Fill* (avoids re-Load)

  const auto& dev0 = state.devices[0];

  // ── Arm TCP pose: cached at the top of ComputeControl (arm_tcp_pose_) ─────
  const pinocchio::SE3& tcp_pose = arm_tcp_pose_;

  // ── Hand FK + Virtual TCP (must run before CLIK) ──────────────────────
  // #121: ComputeHandForwardKinematics runs the closed-chain projection when the
  // hand has loop closure with downstream fingertips, else the serial hand FK;
  // HandFingertipPose returns the hand-root-relative fingertip pose from whichever
  // path is active (byte-for-byte in the serial case). The pose-validity flags
  // were already defaulted to invalid for this tick in Compute() — see the
  // #125 F1 note there for why the reset cannot live inside this gated block.
  if (ComputeHandForwardKinematics(state)) {
    // Fingertip world poses (monitoring — always computed)
    for (std::size_t f = 0; f < kNumFingertips; ++f) {
      pinocchio::SE3 T_hand_ft;
      const bool produced = HandFingertipPose(f, T_hand_ft);
      fingertip_pose_valid_[f] = produced;
      if (produced) {
        const pinocchio::SE3 T_base_ft = tcp_pose.act(T_hand_ft);
        fingertip_positions_[f] = T_base_ft.translation();
        fingertip_rotations_[f] = T_base_ft.rotation();
      }
    }

    // Virtual TCP computation
    UpdateVirtualTcp(tcp_pose, gains);
  }

  // ── Control pose: virtual TCP or tool0 ────────────────────────────────
  pinocchio::SE3 control_pose = tcp_pose;
  bool use_vtcp_frame = false;
  if (vtcp_valid_) {
    control_pose = vtcp_pose_;
    use_vtcp_frame = true;

    // Modify translational Jacobian for offset: J_vtcp_lin = J_tcp_lin -
    // skew(offset) * J_tcp_ang
    const Eigen::Vector3d offset = vtcp_pose_.translation() - tcp_pose.translation();
    skew_buf_ << 0.0, -offset(2), offset(1), offset(2), 0.0, -offset(0), -offset(1), offset(0), 0.0;
    J_full_.topRows(3) -= skew_buf_ * J_full_.bottomRows(3);

    // Rotate full Jacobian from world-aligned frame to vtcp frame
    // J_full_ is LOCAL_WORLD_ALIGNED (world frame) → R_vtcp^T * J for vtcp
    // frame
    const Eigen::Matrix3d R_vtcp_T = vtcp_pose_.rotation().transpose();
    J_full_.topRows(3) = R_vtcp_T * J_full_.topRows(3);
    J_full_.bottomRows(3) = R_vtcp_T * J_full_.bottomRows(3);

    J_pos_.noalias() = J_full_.topRows(3);
  }

  const Eigen::Vector3d ctrl_pos = control_pose.translation();

  tcp_position_ = {ctrl_pos[0], ctrl_pos[1], ctrl_pos[2]};

  // ── Control-frame contract (#292) ──────────────────────────────────────
  // Run the CLIK below only when the goal was authored in the frame being
  // controlled this tick. When a closed-chain hand's FK walk-in converges, the
  // control point moves from tool0 to the virtual TCP in a single tick; without
  // this the trajectory's start and goal stay at the tool0 hold pose and
  // computePoseError hands the whole frame offset (0.21 m plus a ~90° rotation
  // on p1b, which runs control_6dof) to CLIK as task error. Nothing here needs
  // a warm-up gate: while the hand FK is still held, the seed frame and the
  // control frame are BOTH tool0, so the existing CLIK already holds correctly.
  const ControlFrameId cur_id{use_vtcp_frame, vtcp_member_mask_, true};
  switch (ClassifyFrameTransition(
      target_frame_id_, cur_id, target_is_hold_seed_.load(std::memory_order_acquire),
      frame_wait_ticks_.load(std::memory_order_acquire), kVtcpFrameWaitTicks)) {
    case FrameTransition::kNone:
      frame_wait_ticks_.store(0, std::memory_order_release);
      break;

    case FrameTransition::kReseed:
      // No external goal is at stake, so move the hold seed onto the new control
      // point: the task error stays exactly zero and the arm does not move. This
      // is also why a frame that flickers valid/invalid needs no latch or
      // hysteresis — every tick re-seeds to zero error.
      SeedHoldTarget(control_pose, dev0, cur_id);
      target_seqlock_.Store(current_target_slot_);
      break;

    case FrameTransition::kHoldExternal: {
      // Hold the arm and keep new_target_pending_ untouched, so the goal fires
      // unchanged on the tick its frame returns. An EXPLICIT hold: desired_q_
      // tracks the measurement and WriteArmJointCommand integrates a zero dq_
      // onto it. Deliberately not rtc::SilenceDeviceOutput — that means "no
      // update" (the drive keeps its previous setpoint) and answers the F5
      // question, not this one; the arm is readable here.
      frame_wait_ticks_.fetch_add(1, std::memory_order_acq_rel);
      dq_.setZero();
      traj_dq_.setZero();
      const std::size_t nhold = ArmCommandBound(dev0.num_channels);
      for (std::size_t i = 0; i < nhold; ++i) {
        desired_q_[static_cast<Eigen::Index>(i)] = dev0.positions[i];
      }
      return;
    }

    case FrameTransition::kExpireExternal:
      // R1: the authoring frame never came back. Expire the goal and hold the
      // frame that IS available. The rejected alternative — re-tagging the goal
      // into the live frame — is the same silent reinterpretation this contract
      // exists to forbid, just spelled with a timeout.
      RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                           "task goal expired: control frame unavailable for %u ticks "
                           "(goal vtcp=%d, control vtcp=%d) — holding current frame",
                           kVtcpFrameWaitTicks, static_cast<int>(target_frame_id_.is_vtcp),
                           static_cast<int>(cur_id.is_vtcp));
      SeedHoldTarget(control_pose, dev0, cur_id);
      target_seqlock_.Store(current_target_slot_);
      break;

    case FrameTransition::kReplanExternal:
      // Same frame kind, but the participating fingertips changed and moved the
      // control point. The goal is still expressed in this frame and stays
      // valid, so replan from the new control point instead of holding: the goal
      // survives and the trajectory restarts from rest, so there is no jerk.
      target_frame_id_ = cur_id;
      new_target_pending_ = true;
      frame_wait_ticks_.store(0, std::memory_order_release);
      break;

    case FrameTransition::kBindExternal:
      // First tick the frames agree. An off-RT goal cannot know the
      // participating set, so bind it here; new_target_pending_ is already true
      // and initialises the trajectory below, so nothing else is owed.
      target_frame_id_ = cur_id;
      frame_wait_ticks_.store(0, std::memory_order_release);
      break;
  }

  // ── §6.5 damped pseudoinverse (#282) ───────────────────────────────────
  // Formed HERE, ahead of the trajectory block, because its failure branch is a
  // HOLD and a hold must not advance the clock it is holding against. Nothing
  // between this point and the Solve() calls below feeds Compute(): it reads J
  // alone, and J_full_ / J_pos_ are final once the vtcp block above has run — so
  // the ok path is unchanged and only the !ok path moves, from after
  // `trajectory_time_ += dt` to before it. Sitting after `traj_state_` was
  // recomputed, the hold let the reference run away for the whole outage and
  // handed the recovery tick the entire accumulated gap as task error.
  //
  // BOTH floors are applied here and not only in LoadConfig, because
  // `set_gains()` and the parameter callback write the Gains POD straight into
  // the SeqLock and bypass configure entirely — NUM-1's and NUM-2's
  // point-of-use half, whose first in-tree consumer this is. σ₀ needs it as much
  // as λ_max does: σ₀ ≤ 0 makes AdaptiveDampingSquared return λ² = 0 for every
  // σ_min, disarming §6.5 at every pose including a genuinely singular one,
  // where J Jᵀ is only positive SEMI-definite and Eigen's LLT still reports
  // Success — so nothing else in the chain would report it either.
  const double lambda_max = rtc::compliance::FloorMaxDamping(gains.max_damping);
  const double sigma0 = rtc::compliance::FloorSigma0(gains.singularity_threshold);
  const rtc::compliance::DifferentialIk::Result ik =
      gains.control_6dof ? ik_6d_.Compute(J_full_, sigma0, lambda_max)
                         : ik_3d_.Compute(J_pos_, sigma0, lambda_max);
  if (!ik.ok) {
    // A non-finite Jacobian — a NaN joint state that reached FK. J⁺ and N still
    // hold the PREVIOUS tick's contents by contract, so solving with them would
    // command a plausible-looking velocity derived from a stale inverse. Hold
    // explicitly instead, the same shape the frame-transition kHoldExternal
    // branch above uses: zero dq_, let WriteArmJointCommand integrate it onto a
    // desired_q_ that tracks the measurement, and leave new_target_pending_ and
    // trajectory_time_ untouched so a pending goal fires unchanged on the tick
    // the Jacobian returns. Deliberately not rtc::SilenceDeviceOutput — that
    // answers the F5 question ("was the device readable"), and the device IS
    // readable here; the Jacobian is not.
    //
    // The NaN is deliberately NOT scrubbed on its way into desired_q_. It
    // reaches the command, urtc::ValidateControllerOutput rejects the whole
    // output at the actuator boundary, and ~100 ms of consecutive rejects
    // escalates to E-STOP — the correct answer to a joint state that has stopped
    // being a number. Copying only the finite entries would keep the command
    // plausible and leave a stuck encoder running behind a throttled WARN.
    dq_.setZero();
    traj_dq_.setZero();
    const std::size_t nhold = ArmCommandBound(dev0.num_channels);
    for (std::size_t i = 0; i < nhold; ++i) {
      desired_q_[static_cast<Eigen::Index>(i)] = dev0.positions[i];
    }
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "task CLIK held: non-finite Jacobian (control_6dof=%d) — "
                         "J+ would be stale, holding arm at measurement",
                         static_cast<int>(gains.control_6dof));
    return;
  }

  // ── Task-space trajectory ──────────────────────────────────────────────
  if (new_target_pending_) {
    {
      pinocchio::SE3 start_pose = control_pose;
      pinocchio::SE3 goal_pose;

      if (gains.control_6dof) {
        goal_pose = tcp_target_pose_;
      } else {
        goal_pose = start_pose;
        goal_pose.translation() =
            Eigen::Vector3d(current_target_slot_.tcp_target[0], current_target_slot_.tcp_target[1],
                            current_target_slot_.tcp_target[2]);
      }

      const Eigen::Vector3d start_pos = start_pose.translation();
      const Eigen::Vector3d goal_pos = goal_pose.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      // Duration from translational trajectory_speed and velocity limit.
      // Quintic rest-to-rest peak velocity = (15/8) * dist / T.
      const double T_speed_trans = trans_dist / gains.trajectory_speed;
      const double T_vel_trans =
          (gains.max_traj_velocity > 0.0) ? (1.875 * trans_dist / gains.max_traj_velocity) : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      // Angular distance via AngleAxisd (stable at θ → π, unlike log3)
      double angular_dist = 0.0;
      Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ();  // fallback
      bool split_trajectory = false;

      if (gains.control_6dof) {
        const Eigen::AngleAxisd aa(start_pose.rotation().transpose() * goal_pose.rotation());
        angular_dist = aa.angle();  // always in [0, π]
        rot_axis = aa.axis();

        const double T_speed_rot = angular_dist / gains.trajectory_angular_speed;
        const double T_vel_rot = (gains.max_traj_angular_velocity > 0.0)
                                     ? (1.875 * angular_dist / gains.max_traj_angular_velocity)
                                     : 0.0;
        duration = std::max({duration, T_speed_rot, T_vel_rot});

        split_trajectory = (angular_dist > M_PI - gains.pi_rotation_margin);
      }

      if (split_trajectory) {
        // ── π-rotation defense: split into 2 rest-to-rest segments ──
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid =
            start_pose.rotation() * Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

        pinocchio::SE3 mid_pose;
        mid_pose.translation() = 0.5 * (start_pos + goal_pos);
        mid_pose.rotation() = R_mid;

        // Segment 1: start → mid (half distances)
        const double half_trans = trans_dist * 0.5;
        const double T1_speed_t = half_trans / gains.trajectory_speed;
        const double T1_vel_t =
            (gains.max_traj_velocity > 0.0) ? (1.875 * half_trans / gains.max_traj_velocity) : 0.0;
        const double T1_speed_r = half_angle / gains.trajectory_angular_speed;
        const double T1_vel_r = (gains.max_traj_angular_velocity > 0.0)
                                    ? (1.875 * half_angle / gains.max_traj_angular_velocity)
                                    : 0.0;
        const double dur1 = std::max({0.01, T1_speed_t, T1_vel_t, T1_speed_r, T1_vel_r});

        // Segment 2: mid → goal (same half distances for symmetric split)
        const double dur2 = dur1;

        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), mid_pose,
                               pinocchio::Motion::Zero(), dur1);
        pending_goal_pose_ = goal_pose;
        pending_duration_ = dur2;
        has_pending_segment_ = true;
      } else {
        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), goal_pose,
                               pinocchio::Motion::Zero(), duration);
        has_pending_segment_ = false;
      }

      trajectory_time_ = 0.0;
      // Initialize desired_q_ from current actual joint positions
      for (int i = 0; i < arm_handle_->nv(); ++i) {
        desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
      }
      new_target_pending_ = false;
    }
  }

  traj_state_ = trajectory_.compute(trajectory_time_, dt);
  trajectory_time_ += dt;

  // ── Segment transition (π-rotation defense) ────────────────────────────
  if (has_pending_segment_ && trajectory_time_ >= trajectory_.duration()) {
    pinocchio::SE3 mid_pose = trajectory_.compute(trajectory_.duration()).pose;
    trajectory_.initialize(mid_pose, pinocchio::Motion::Zero(), pending_goal_pose_,
                           pinocchio::Motion::Zero(), pending_duration_);
    trajectory_time_ = 0.0;
    has_pending_segment_ = false;
  }

  // ── Cartesian error ────────────────────────────────────────────────────
  // BodyLog6 = log6(T_cur⁻¹ T_d): body-frame screw error (preserves the
  // position-rotation coupling), via rtc_math.
  const rtc::math::se3::Iso3 control_iso = rtc::math::se3::toIso3(control_pose);
  const rtc::math::se3::Vec6 e_body = rtc::math::se3::computePoseError(
      control_iso, rtc::math::se3::toIso3(traj_state_.pose), rtc::math::se3::ErrorType::BodyLog6);
  if (use_vtcp_frame) {
    // Jacobian is in vtcp (LOCAL) frame → use the LOCAL error directly.
    pos_error_6d_ = e_body;
  } else {
    // LOCAL_WORLD_ALIGNED Jacobian → transport LOCAL → LWA (rotation-only
    // blockdiag(R,R), NOT the full adjoint).
    pos_error_6d_ = rtc::math::se3::twistLocalToWorld(control_iso, e_body);
  }

  const Eigen::Vector3d p_err = pos_error_6d_.head<3>();

  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = p_err[i];
  }

  // ── Primary task ───────────────────────────────────────────────────────
  // J⁺ and N were formed by the §6.5 Compute() above, ahead of the trajectory
  // block, so that its hold branch could not advance the clock it holds against.
  // The trajectory feedforward velocity (trajectory local → Jacobian frame) is
  // computed once and reused for both the CLIK command dq_ and the log-only
  // feedforward traj_dq_, so the frame rotation runs a single time per tick.
  if (gains.control_6dof) {
    // Feedforward: trajectory local → Jacobian frame (vtcp or world-aligned).
    Eigen::Matrix<double, 6, 1> ff_vel_6d;
    if (use_vtcp_frame) {
      const Eigen::Matrix3d R_vtcp_traj =
          control_pose.rotation().transpose() * traj_state_.pose.rotation();
      ff_vel_6d.head<3>() = R_vtcp_traj * traj_state_.velocity.linear();
      ff_vel_6d.tail<3>() = R_vtcp_traj * traj_state_.velocity.angular();
    } else {
      ff_vel_6d.head<3>() = traj_state_.pose.rotation() * traj_state_.velocity.linear();
      ff_vel_6d.tail<3>() = traj_state_.pose.rotation() * traj_state_.velocity.angular();
    }

    // The shared law, not a local spelling of it (#314). ff_vel_6d is already in
    // the Jacobian frame, which is the frame contract ComputeTaskVelocity states
    // for nu_ff — the rotation above is the caller's half of that contract.
    const Eigen::Matrix<double, 6, 1> task_vel_6d = rtc::task::ComputeTaskVelocity(
        rtc::task::TaskVelParams{gains.kp_translation, gains.kp_rotation}, pos_error_6d_,
        ff_vel_6d);
    // Two-argument Solve, not Solve(twist, zero, out): the 6-DOF branch has no
    // posture task at all (the null-space block below is gated on
    // `!control_6dof`), and `+= N·0` is a full nv×nv product whose result is not
    // the identity on a signed zero. Adding it would be both RT work and a
    // bitwise change for nothing.
    ik_6d_.Solve(task_vel_6d, dq_);
    ik_6d_.Solve(ff_vel_6d, traj_dq_);
  } else {
    Eigen::Vector3d ff_lin;
    if (use_vtcp_frame) {
      const Eigen::Matrix3d R_vtcp_traj =
          control_pose.rotation().transpose() * traj_state_.pose.rotation();
      ff_lin = R_vtcp_traj * traj_state_.velocity.linear();
    } else {
      ff_lin = traj_state_.pose.rotation() * traj_state_.velocity.linear();
    }
    // Translation-only counterpart of the call above (#314). Takes the
    // translation gains DIRECTLY: this branch commands no orientation, and the
    // signature says so.
    const Eigen::Vector3d task_vel =
        rtc::task::ComputeTranslationVelocity(gains.kp_translation, pos_error_, ff_lin);

    // ── Null-space secondary task ────────────────────────────────────────
    // Folded into this branch (#282): the gate WAS `enable_null_space &&
    // !control_6dof`, and inside the else the second half is already true.
    // q̇ = J⁺·task_vel + N·q̇_null now happens in one Solve — the old spelling
    // computed null_err − J⁺(J·null_err) to avoid materialising the nv×nv
    // projector, and there is nothing left to avoid because Compute() forms N
    // either way.
    if (gains.enable_null_space) {
      // Runs nv deep, and that is not the #172 hazard it resembles. In the range
      // this could over-read — [num_channels, nv), which arm_readable_ confines
      // to [arm_dof_, nv) — BOTH operands are the untouched zero-init: the only
      // two writers of null_target (the self-init seed and the target dispatch)
      // are bounded by arm_dof_, so the difference is 0 − 0 and the projector
      // receives exactly the zero it would receive from a narrower loop. Adding
      // a bound plus a setZero() here would buy a per-tick nv-wide write on the
      // RT path and change no value (#282 review finding 5).
      for (Eigen::Index i = 0; i < arm_handle_->nv(); ++i) {
        qdot_null_[i] = current_target_slot_.null_target[static_cast<std::size_t>(i)] -
                        dev0.positions[static_cast<std::size_t>(i)];
      }
      // NUM-6 at the point of use (#277). This gate never reads the gain, so
      // the floor goes straight on the gain rather than in front of the gate.
      // (The retired ClikController adapter placed it identically — #236 S7c
      // deleted the class, not the placement rule, whose SSoT is
      // agent_docs/modification-guide.md §Adding a New Controller item 5.)
      // LoadConfig and the `null_kp` parameter callback floor it too; this half
      // covers neither, because the gain reaches the SeqLock POD from both and
      // a negative K_p drives the posture AWAY from its target while N hides
      // that from the Cartesian task.
      //
      // The gain now scales q̇_null BEFORE the projection instead of the
      // projected result after it. N is linear, so the two agree algebraically
      // and differ only in rounding; the shared law takes q̇_null as its
      // argument, which is what fixes the order.
      qdot_null_ *= rtc::FloorNonNegativeGain(gains.null_kp);
      ik_3d_.Solve(task_vel, qdot_null_, dq_);
    } else {
      ik_3d_.Solve(task_vel, dq_);
    }
    ik_3d_.Solve(ff_lin, traj_dq_);
  }
}

// ── Phase 2b: secondary (hand) lane ─────────────────────────────────────────
//
// Split out of ComputeControl for the F5 gate (#236 S7b). ComputeControl holds
// the ARM on a tick device 0 cannot be read on, and the hand must not be held
// with it — §3.7's "secondary passthrough 유지": a hand does not stop being
// commandable because the arm's state went missing, and WriteJointCommand keeps
// commanding device 1 on a silenced tick. While these blocks lived inside
// ComputeControl its early return froze hand_computed_, and on a controller that
// had never seen a readable tick that froze value is the zero-init — a real
// position command to the hand's origin, which is the hazard the gate exists to
// prevent, moved one device over.
//
// Reads nothing arm-derived: the hand trajectory, the grasp FSM and the ToF
// snapshot all source device 1 and the fingertip sensors. `gains` is the same
// per-tick SeqLock snapshot ComputeControl gets — loaded once in Compute() and
// passed to both, so the two halves of a tick cannot see different gains.
void DemoTaskController::ComputeSecondary(const ControllerState& state, double dt,
                                          const Gains& gains) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::ComputeSecondary");

  // The grasp mode this hand lane runs under, cached for the output fills (which
  // have no Gains of their own) and compared for the race closure below. Both
  // live on this side of Compute()'s `!estop_active_` gate on purpose: an E-STOP
  // tick must not consume the edge that the closure is the only consumer of.
  // Tick 1 can report a spurious edge (the member starts at kContactStop while
  // the config may say otherwise); the only consumer is latch-gated and the latch
  // cannot be set before a tick has run, so that edge is unreachable in practice.
  const bool grasp_mode_changed = gains.grasp_hand_mode != grasp_hand_mode_cached_;
  grasp_hand_mode_cached_ = gains.grasp_hand_mode;

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];

    // hand_readable_ joins the pending flag rather than replacing the enclosing
    // guard (#291): this seed is the only dev1 read in ComputeSecondary, and it
    // runs hand_dof_ deep, so a narrow hand would start the ramp from a phantom
    // origin and drive the unreported fingers origin-ward over `duration`.
    // Scoping it here — not early-returning — keeps the trajectory evaluation
    // below running, which is what WriteJointCommand's silenced-arm branch
    // relies on to keep hand_computed_ tracking. The flag stays pending, so the
    // seed is retried the tick the hand becomes readable (the arm's
    // arm_target_initialized_ deferral, #265 audit T1/T2, on the hand axis).
    if (hand_new_target_pending_ && hand_readable_) {
      trajectory::JointSpaceTrajectory<kDemoTaskMaxHandDof>::State start_state;
      trajectory::JointSpaceTrajectory<kDemoTaskMaxHandDof>::State goal_state;

      double max_dist = 0.0;
      for (int i = 0; i < hand_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        start_state.positions[idx] = dev1.positions[idx];
        start_state.velocities[idx] = 0.0;
        start_state.accelerations[idx] = 0.0;

        goal_state.positions[idx] = current_target_slot_.targets[1][idx];
        goal_state.velocities[idx] = 0.0;
        goal_state.accelerations[idx] = 0.0;

        max_dist = std::max(max_dist,
                            std::abs(current_target_slot_.targets[1][idx] - dev1.positions[idx]));
      }

      const double T_speed = max_dist / gains.hand_trajectory_speed;
      const double T_vel = (gains.hand_max_traj_velocity > 0.0)
                               ? (1.875 * max_dist / gains.hand_max_traj_velocity)
                               : 0.0;
      const double duration = std::max({0.01, T_speed, T_vel});
      hand_trajectory_.initialize(start_state, goal_state, duration);
      hand_trajectory_time_ = 0.0;
      hand_new_target_pending_ = false;
    }

    const auto hand_traj = hand_trajectory_.compute(hand_trajectory_time_);
    hand_trajectory_time_ += dt;

    for (int i = 0; i < hand_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      hand_computed_.positions[idx] = hand_traj.positions[idx];
      hand_computed_.velocities[idx] = hand_traj.velocities[idx];
    }
  }

  // ── Grasp detection + ContactStopHand (500Hz) ────────────────────────
  {
    FillGraspSensorAggregates(gains);
    const float force_thresh = grasp_state_.force_threshold;
    const float max_force = grasp_state_.max_force;
    const int active_count = grasp_state_.num_active_contacts;

    // Periodic grasp status snapshot (2s throttle, debug only).
    // NOTE: throttled logging on the 500Hz path — the rare allocation
    // inside rclcpp logging macros is acceptable at this interval.
    RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "[grasp] type=%s active=%d/%d max_force=%.2fN thresh=%.2fN phase=%d",
                         GraspHandModeName(gains.grasp_hand_mode), active_count,
                         num_active_fingertips_, static_cast<double>(max_force),
                         static_cast<double>(force_thresh),
                         grasp_controller_ ? static_cast<int>(grasp_controller_->phase()) : -1);

    // Hand grasp control: force_pi (adaptive PI) or contact_stop (binary freeze).
    // Named once, because the race closure below has to ask "will the hold run
    // this tick?" and get exactly the answer the branch itself uses.
    //
    // The `!= kNone` form is deliberate: a "force_pi" config whose grasp
    // controller failed to build (null) must still fall into the contact_stop
    // safety freeze, not silently become a no-op.
    const bool run_force_pi = grasp_controller_ && gains.grasp_hand_mode == GraspHandMode::kForcePi;
    const bool run_contact_stop = !run_force_pi && gains.grasp_hand_mode != GraspHandMode::kNone;

    // ── Race closure for the runtime mode switch ──────────────────────────
    // Sibling of the joint controller's; the full rationale lives there. In
    // short: the parameter callback refuses a mode change unless the hand is
    // quiet, but it reads the latch through an atomic mirror and cannot hold the
    // RT thread still, so contact can engage between its check and its Store.
    // The next tick would then stop running the hold while hand_computed_ reverts
    // to a trajectory still aimed at the PRE-CONTACT goal — a resumed squeeze on
    // a held object. Re-seed the goal to the hold position before dropping the
    // latch, so the trajectory taking over is already a standing hold.
    //
    // WARN, not INFO: a quiet gate that works means this never fires, so the
    // message appearing at all is the signal that the gate has a hole.
    if (grasp_mode_changed && contact_latched_ && !run_contact_stop) {
      trajectory::JointSpaceTrajectory<kDemoTaskMaxHandDof>::State hold_state;
      for (int i = 0; i < hand_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        current_target_slot_.targets[1][idx] = hand_hold_position_[idx];
        // This tick too, not just the next one: the hand trajectory block ran
        // earlier in ComputeSecondary, so hand_computed_ already holds a step
        // toward the old goal.
        hand_computed_.positions[idx] = hand_hold_position_[idx];
        hand_computed_.velocities[idx] = 0.0;
        hold_state.positions[idx] = hand_hold_position_[idx];
        hold_state.velocities[idx] = 0.0;
        hold_state.accelerations[idx] = 0.0;
      }
      target_seqlock_.Store(current_target_slot_);
      // Duration only has to be non-zero: start state == goal state, so the
      // quintic is a constant. Same value DrainTargetSlot's hold-seed uses.
      constexpr double kHoldSeedDuration = 0.01;
      hand_trajectory_.initialize(hold_state, hold_state, kHoldSeedDuration);
      hand_trajectory_time_ = 0.0;
      // The goal now equals the trajectory, so nothing is left pending.
      hand_new_target_pending_ = false;
      contact_latched_ = false;
      RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                           "[grasp] mode left contact_stop while LATCHED -> re-seeded hand hold "
                           "(the quiet gate should have refused this); now type=%s",
                           GraspHandModeName(gains.grasp_hand_mode));
    }

    if (run_force_pi) {
      std::array<double, rtc::grasp::kMaxGraspFingers> f_raw{};
      for (int f = 0; f < num_grasp_fingers_; ++f) {
        f_raw[static_cast<std::size_t>(f)] =
            static_cast<double>(grasp_state_.force_magnitude[static_cast<std::size_t>(f)]);
      }

      const auto commands = grasp_controller_->Update(
          std::span<const double>(f_raw.data(), static_cast<std::size_t>(num_grasp_fingers_)), dt);

      // Phase-transition log: rare event (gated by phase change), but still
      // throttled as a defensive RT-safety net in case the FSM oscillates.
      const auto cur_phase = static_cast<uint8_t>(grasp_controller_->phase());
      // Mirror for the non-RT quiet gate (rationale on the member). Stored here
      // rather than beside the latch mirror because this is the only place the
      // FSM is stepped, so it is the only place the value can have changed.
      grasp_phase_pub_.store(cur_phase, std::memory_order_release);
      if (cur_phase != prev_grasp_phase_) {
        RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                             "[force_pi] phase %u -> %u target_force=%.2fN", prev_grasp_phase_,
                             cur_phase, grasp_controller_->target_force());
        prev_grasp_phase_ = cur_phase;
      }

      if (grasp_controller_->phase() != rtc::grasp::GraspPhase::kIdle) {
        for (int f = 0; f < commands.num_fingers; ++f) {
          for (int j = 0; j < commands.dof[static_cast<std::size_t>(f)]; ++j) {
            const auto mi = static_cast<std::size_t>(
                finger_joint_map_[static_cast<std::size_t>(f)][static_cast<std::size_t>(j)]);
            hand_computed_.positions[mi] =
                commands.q[static_cast<std::size_t>(f)][static_cast<std::size_t>(j)];
            hand_computed_.velocities[mi] = 0.0;
          }
        }
      }
    } else if (run_contact_stop) {
      // grasp_controller_type=="none": no hand intervention — trajectory output
      // passes through untouched.  GraspState aggregation/publishing below is
      // unaffected (BT IsGrasped/IsForceAbove still observe contact). Why the
      // predicate is negated rather than `== kContactStop`: see run_contact_stop.
      //
      // ContactStopHand: 힘 감지 시 hand trajectory 출력을 현재 위치로 동결
      // → BT tick(50ms) 사이에도 과도한 hand closure 방지
      //
      // Release-phase gate: 사용자가 손을 여는 방향으로 goal을 내린 경우에는
      // 접촉 잔존 힘이 있더라도 freeze 를 skip 해야 손이 열린다.
      // per-finger gate: release_gate_sign_[f] 방향으로 target-actual 이 움직이면
      // 해당 finger 는 이완 중. 모든 gate finger 가 이완 방향일 때만 "release 의도".
      //   sign > 0: 각도 증가 = loosening (e.g. thumb CMC_FE) → target > actual
      //   sign < 0: 각도 감소 = loosening (e.g. index/middle MCP_FE) → target < actual
      // E-STOP (arm or hand) drops the latch so a post-clear resume honours
      // fresh trajectory goals instead of an old hold. The filter keeps tracking.
      if (estopped_.load(std::memory_order_acquire) ||
          hand_estopped_.load(std::memory_order_acquire)) {
        contact_latched_ = false;
      }

      // hand_readable_, not `valid` (#291). Two distinct hazards live in this
      // block and the gate is the only thing that closes either:
      //   - the LPF below is an IIR. Feeding it a phantom 0 per unreported
      //     finger does not merely spoil this tick's hand_filt — the sample
      //     persists in the filter state and decays over the following ticks,
      //     so the contamination outlives the tick that produced it.
      //   - the release gate reads dev1.positions[gi] at a MODEL index, so an
      //     unreported gi yields a phantom `d` that flips a BRANCH
      //     (release_phase → drop the contact latch), not just a number.
      // The pre-existing `else if (contact_latched_)` below is already the
      // right answer for "no usable hand state": hold the last commanded
      // position rather than let the trajectory drive. Widening the predicate
      // routes the narrow-hand case into it. hand_hold_position_ cannot replay
      // a zero-init here — contact_latched_ is only ever set inside this
      // readable branch, so an always-unreadable hand leaves it false and the
      // else-if a no-op.
      if (hand_readable_) {
        const auto& dev1 = state.devices[1];

        // LPF the measured hand position. Run every readable tick (not just
        // while latched) so the IIR state stays warm and latch entry has no
        // transient.
        std::array<double, kDemoTaskMaxHandDof> hand_meas{};
        for (int i = 0; i < hand_dof_; ++i) {
          hand_meas[static_cast<std::size_t>(i)] = dev1.positions[static_cast<std::size_t>(i)];
        }
        const std::array<double, kDemoTaskMaxHandDof> hand_filt = hand_pos_filter_.Apply(hand_meas);

        // First 3 gate errors are surfaced in the diagnostic log (padded with 0).
        std::array<double, 3> gate_err{};
        bool release_phase = num_release_gates_ > 0;
        for (int f = 0; f < num_release_gates_; ++f) {
          const auto gi = release_gate_idx_[static_cast<std::size_t>(f)];
          const double d = current_target_slot_.targets[1][gi] - dev1.positions[gi];
          const bool releasing = (release_gate_sign_[static_cast<std::size_t>(f)] >= 0)
                                     ? (d > gains.contact_stop_release_eps)
                                     : (d < -gains.contact_stop_release_eps);
          release_phase = release_phase && releasing;
          if (f < 3) {
            gate_err[static_cast<std::size_t>(f)] = d;
          }
        }

        // Filtered aggregates, not the raw ones published on GraspState: this
        // is the only consumer that acts on a threshold crossing, so a single
        // noisy sample must not be able to latch a freeze. The published
        // force_magnitude / max_force stay raw (BT contract unchanged).
        const bool contact_now =
            (contact_stop_active_count_ > 0 && contact_stop_max_force_ > force_thresh);

        if (release_phase) {
          // User is opening the hand: drop the latch and let the trajectory drive.
          contact_latched_ = false;
          RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                               "[contact_stop] SKIP (release) dgate=[%+.3f,%+.3f,%+.3f]",
                               gate_err[0], gate_err[1], gate_err[2]);
        } else {
          if (contact_now) {
            contact_latched_ = true;
          }
          if (contact_latched_) {
            // Hybrid hold: while contact is present, track the LPF'd position
            // (compliant) and snapshot it as the hold target; once contact drops
            // keep the last hold target fixed (no random-walk drift) until the
            // release gate fires.
            for (int i = 0; i < hand_dof_; ++i) {
              const auto idx = static_cast<std::size_t>(i);
              if (contact_now) {
                hand_hold_position_[idx] = hand_filt[idx];
              }
              hand_computed_.positions[idx] = hand_hold_position_[idx];
              hand_computed_.velocities[idx] = 0.0;
            }
            // gate errors (target - actual) encode both the actual position and
            // the overshoot beyond target, enough to diagnose contact_stop engage.
            // fmax/active are the FILTERED aggregates the branch above actually
            // read; fmax_raw is carried alongside so a log line still shows how
            // far the unfiltered signal ran ahead of the decision.
            RCLCPP_INFO_THROTTLE(
                logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                "[contact_stop] %s active=%d fmax=%.2fN fmax_raw=%.2fN "
                "dgate=[%+.3f,%+.3f,%+.3f]",
                contact_now ? "FREEZE" : "HOLD", contact_stop_active_count_,
                static_cast<double>(contact_stop_max_force_), static_cast<double>(max_force),
                gate_err[0], gate_err[1], gate_err[2]);
          }
        }
      } else if (contact_latched_) {
        // Hand-state dropout while latched: hold the last commanded position
        // rather than let the trajectory drive the hand (safety).
        for (int i = 0; i < hand_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          hand_computed_.positions[idx] = hand_hold_position_[idx];
          hand_computed_.velocities[idx] = 0.0;
        }
      }
    }

    // Publish the latch for the non-RT quiet gate (rationale on the member).
    // Unconditional and outside every branch on purpose: the latch is written on
    // several paths (contact engage, release gate, hand E-STOP, the race closure
    // above) and a mirror maintained per-path would be one refactor away from
    // missing one — and a missed path fails in the direction that opens the mode
    // switch on a held object.
    contact_latched_pub_.store(contact_latched_, std::memory_order_release);

    // ── In-plane pull-force estimate (#167) ─────────────────────────────
    // Feeds the 3-axis fingertip forces (before the |F| collapse in
    // ReadState) plus the per-tip FK rotations/positions cached this tick.
    // The baseline snapshot arms on the grasp_detected rising edge; the
    // result rides grasp_state_.pull to the owned GraspState SeqLock (no
    // new topic).
    if (pull_wiring_.enabled()) {
      // Own span so the estimator's latency is separable from the rest of
      // ComputeControl in Perfetto — WBC has had one since #167, joint/task
      // were only covered by the enclosing controller span (#234 P-20).
      RTC_TRACE_SCOPE("DemoTaskController::UpdatePullEstimate");
      StageFkPullTickAndPublish(pull_wiring_, std::span<const FingertipSensorData>(fingertip_data_),
                                std::span<const Eigen::Matrix3d>(fingertip_rotations_),
                                std::span<const Eigen::Vector3d>(fingertip_positions_),
                                std::span<const bool>(fingertip_pose_valid_),
                                num_active_fingertips_, grasp_state_.grasp_detected, dt,
                                grasp_state_.pull);
    }
  }

  // ── ToF snapshot (3 fingers × 2 sensors: tof[1]=A, tof[2]=B) ───────────
  {
    constexpr int kNumTofFingers = 3;     // thumb, index, middle (hand-specific)
    constexpr int kSensorsPerFinger = 2;  // A/B pair
    constexpr double kMmToM = 0.001;
    tof_snapshot_ = {};

    // Gate on BOTH counts: num_sensor_fingertips_ ensures a real raw sensor
    // lane exists (not force-only junk); num_active_fingertips_ ensures
    // ReadState actually refreshed fingertip_data_[0..2].tof this tick (its
    // populate loop is bounded by the inference-group count, which can be < the
    // sensor-lane count on an asymmetric stream).
    if (hand_handle_ && num_sensor_fingertips_ >= kNumTofFingers &&
        num_active_fingertips_ >= kNumTofFingers) {
      tof_snapshot_.num_fingers = kNumTofFingers;
      tof_snapshot_.sensors_per_finger = kSensorsPerFinger;

      for (int f = 0; f < kNumTofFingers; ++f) {
        const auto fi = static_cast<std::size_t>(f);
        const auto& ft = fingertip_data_[fi];
        const int si = f * kSensorsPerFinger;

        // tof[1] → sensor A, tof[2] → sensor B (tof[0] 제외)
        const double d_a = static_cast<double>(ft.tof[1]) * kMmToM;
        const double d_b = static_cast<double>(ft.tof[2]) * kMmToM;
        tof_snapshot_.distances[static_cast<std::size_t>(si)] = d_a;
        tof_snapshot_.distances[static_cast<std::size_t>(si + 1)] = d_b;
        tof_snapshot_.valid[static_cast<std::size_t>(si)] = (d_a > 0.0);
        tof_snapshot_.valid[static_cast<std::size_t>(si + 1)] = (d_b > 0.0);

        // Fingertip SE3 pose → position + quaternion
        auto& pose = tof_snapshot_.tip_poses[fi];
        const auto& pos = fingertip_positions_[fi];
        pose.position = {pos[0], pos[1], pos[2]};
        const Eigen::Quaterniond quat(fingertip_rotations_[fi]);
        pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
      }
      tof_snapshot_.populated = true;
    }
  }
}

// ── Phase 3a: Write joint command (wire-bound only) ──────────────────────────
//
// Task controller wire pipeline:
//   1. Clamp dq_ in-place by per-joint velocity limits (dq_ is the canonical
//      clamped velocity; both log and publish read it back).
//   2. Integrate desired_q_ ← desired_q_ + dq_ * dt (member, persists across
//      ticks; the integrator state).
//   3. commands ← desired_q_.
// E-STOP early-return preserved here so ComputeEstop's output is the wire
// command and the caller still gets log/publish fills via FillLogOutput/
// FillPublishOutput in normal ticks (E-STOP path runs its own log push in the
// dispatcher above).

ControllerOutput DemoTaskController::WriteJointCommand(const ControllerState& state,
                                                       double dt) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::WriteJointCommand");
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;
  output.valid = true;

  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.goal_type = GoalType::kTask;
  if (!arm_readable_) {
    // F5: no honest arm command this tick, so none is issued. Zero-length is
    // "no update" — the drive holds its previous setpoint. nc0 zeros would be a
    // real command to the origin (§3.7). The shared hand block below still runs
    // — ComputeSecondary keeps hand_computed_ tracking on a silenced tick, so
    // what it commands is the hand's own trajectory, not a frozen buffer.
    rtc::SilenceDeviceOutput(out0);
    // The log lane is bounded by the DEVICE's channel count, not this output's,
    // so an untouched reference row would be recorded as zeros and read as a
    // command to the origin. Report where the drive is parked instead.
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
  } else {
    out0.num_channels = nc0;
    WriteArmJointCommand(state, out0, dt);
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto& out1 = output.devices[1];
    out1.goal_type = GoalType::kJoint;
    if (!hand_readable_) {
      // F5 on the SECONDARY axis (#291), the same answer the arm gets above and
      // for the same reason: hand_computed_ was withheld this tick, so the only
      // thing left to command would be a stale or zero-init buffer. Zero-length
      // is "no update" — the hand holds its previous setpoint. nc1 zeros would
      // be a real command to the origin, i.e. every finger slammed open.
      rtc::SilenceDeviceOutput(out1);
      // The log lane is bounded by the DEVICE's channel count, so an untouched
      // reference row would be recorded as zeros and read as exactly that
      // origin command. Report where the fingers are parked instead.
      rtc::HoldTelemetryAtMeasured(out1, nc1, dev1.positions);
    } else {
      out1.num_channels = nc1;
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        out1.commands[i] = hand_computed_.positions[i];
      }
      rtc::utils::ClampRange(out1.commands, nc1, std::span<const double>(device_position_lower_[1]),
                             std::span<const double>(device_position_upper_[1]), -6.2832, 6.2832);
    }
  }

  return output;
}

// Arm half of the wire command. Split out so the silenced branch above and the
// readable branch share ONE hand block rather than two copies of a device
// command lane that must never drift apart.
void DemoTaskController::WriteArmJointCommand(const ControllerState& state, rtc::DeviceOutput& out0,
                                              double dt) noexcept {
  const auto& dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;

  // Model-dimension bound. dq_ / desired_q_ / traj_dq_ are nv-wide Eigen
  // vectors while nc0 is whatever the device reported on the wire
  // (WriteJointStateToCache sets num_channels from the JointState message
  // length, not from joint_state_names) — a device reporting more channels
  // than the model DOF must never index past them (issue #172 OOB; the sibling
  // bindings guard the same hazard with rtc::ModelChannelBound, which
  // ArmCommandBound wraps).
  const std::size_t nq = ArmCommandBound(nc0);
  // Clamp dq_ in-place so log + publish read the same canonical clamped value.
  for (std::size_t i = 0; i < nq; ++i) {
    const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    dq_[static_cast<Eigen::Index>(i)] = std::clamp(dq_[static_cast<Eigen::Index>(i)], -lim, lim);
  }
  for (std::size_t i = 0; i < nq; ++i) {
    desired_q_[static_cast<Eigen::Index>(i)] += dq_[static_cast<Eigen::Index>(i)] * dt;
    out0.commands[i] = desired_q_[static_cast<Eigen::Index>(i)];
  }
  // Channels in [nq, nc0) have no model joint behind them. Hold them at the
  // measured position rather than leaving the fresh-zero init: on a position
  // lane 0.0 is "go to the origin", not "stay put" (DemoJointController's
  // E-STOP tail makes the same distinction via rtc::FillCommandTail, and the
  // backends' WriteSafeCommand comments call out the identical trap).
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = dev0.positions[i];
  }
}

// ── Phase 3b: Fill log output ────────────────────────────────────────────────

void DemoTaskController::FillLogOutput(const ControllerState& state, ControllerOutput& output,
                                       double dt) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::FillLogOutput");
  if (estop_active_) {
    // ComputeEstop path skipped log-fill in legacy WriteOutput too — but the
    // controller-owned publish state must still describe THIS tick (#234 P-1),
    // otherwise the publish thread reships the pre-E-STOP body under the
    // current stamp. The tail PushPullEstimatorLog then writes the matching
    // valid=0 CSV row.
    FillEstopPublishState(dt);
    return;
  }
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  if (arm_readable_) {
    // Same model-dimension bound as WriteJointCommand — these read the identical
    // nv-wide buffers, so the telemetry lane must not index further than the wire
    // lane does (issue #172).
    const std::size_t nq = ArmCommandBound(nc0);
    for (std::size_t i = 0; i < nq; ++i) {
      out0.trajectory_positions[i] = desired_q_[static_cast<Eigen::Index>(i)];
      out0.trajectory_velocities[i] = traj_dq_[static_cast<Eigen::Index>(i)];
    }
  } else {
    // No arm reference this tick — desired_q_ still holds the last readable
    // tick's values (#265 audit T7), so publishing it would date-stamp stale
    // numbers. Report the parked position instead of leaving a row of zeros
    // that reads as a command to the origin (rtc_controller_interface/
    // device_readability.hpp, HoldTelemetryAtMeasured).
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
  }
  // The goal survives the gate: a target the operator set does not stop
  // existing because the arm went unreadable, and it is the one column on a
  // silenced row that should still say what was asked for.
  for (std::size_t i = 0; i < 3; ++i) {
    out0.goal_positions[i] = current_target_slot_.tcp_target[i];
  }
  const std::size_t ngoal =
      std::min(static_cast<std::size_t>(nc0), current_target_slot_.null_target.size());
  for (std::size_t i = 3; i < ngoal; ++i) {
    out0.goal_positions[i] = current_target_slot_.null_target[i];
  }

  // Arm TCP cached in ComputeControl (arm_tcp_pose_); reused here — no re-read.
  const pinocchio::SE3& tcp_current = arm_tcp_pose_;
  pinocchio::SE3 log_pose = vtcp_valid_ ? vtcp_pose_ : tcp_current;
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(log_pose.rotation());
  output.actual_task_positions[0] = log_pose.translation().x();
  output.actual_task_positions[1] = log_pose.translation().y();
  output.actual_task_positions[2] = log_pose.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.task_goal_positions[0] = current_target_slot_.tcp_target[0];
  output.task_goal_positions[1] = current_target_slot_.tcp_target[1];
  output.task_goal_positions[2] = current_target_slot_.tcp_target[2];
  if (control_6dof_cached_) {
    Eigen::Vector3d goal_rpy = pinocchio::rpy::matrixToRpy(tcp_target_pose_.rotation());
    output.task_goal_positions[3] = goal_rpy[0];
    output.task_goal_positions[4] = goal_rpy[1];
    output.task_goal_positions[5] = goal_rpy[2];
  } else {
    output.task_goal_positions[3] = rpy[0];
    output.task_goal_positions[4] = rpy[1];
    output.task_goal_positions[5] = rpy[2];
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto& out1 = output.devices[1];
    if (hand_readable_) {
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        out1.trajectory_positions[i] = hand_computed_.positions[i];
        out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      }
    } else {
      // No hand reference this tick — hand_computed_ still holds the last
      // readable tick's values (#291, the hand's T7), so publishing it would
      // date-stamp stale numbers. Report the parked position instead of leaving
      // a row of zeros that reads as a command to the origin.
      rtc::HoldTelemetryAtMeasured(out1, nc1, dev1.positions);
    }
    // The goal survives the gate on this axis too: a finger target the operator
    // set does not stop existing because the hand went unreadable.
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }

  // grasp_hand_mode_cached_, not a fresh Load: this publishes what the control
  // law produced, so it must agree with the branch ComputeSecondary took.
  if (grasp_controller_ && grasp_hand_mode_cached_ == GraspHandMode::kForcePi) {
    grasp_state_.grasp_phase = static_cast<uint8_t>(grasp_controller_->phase());
    grasp_state_.grasp_target_force = static_cast<float>(grasp_controller_->target_force());
    const auto fs = grasp_controller_->finger_states();
    const std::size_t n = std::min(fs.size(), grasp_state_.finger_s.size());
    for (std::size_t idx = 0; idx < n; ++idx) {
      grasp_state_.finger_s[idx] = static_cast<float>(fs[idx].s);
      grasp_state_.finger_filtered_force[idx] = static_cast<float>(fs[idx].f_measured);
      grasp_state_.finger_force_error[idx] =
          static_cast<float>(fs[idx].f_desired - fs[idx].f_measured);
    }
  }
  grasp_state_lock_.Store(grasp_state_);
  tof_snapshot_lock_.Store(tof_snapshot_);
}

// ── E-STOP publish state (#234 P-1) ─────────────────────────────────────────
//
// Rationale in the header. Same policy as joint/wbc: sensor-derived fields
// (refreshed by ReadState this tick) stay, control-law-derived fields are
// neutralized, and the pull estimate runs its E-STOP tick.

// Sensor-derived grasp aggregates. Sourced from fingertip_data_, which
// ReadState refreshes every tick including E-STOP — hence shared by
// ComputeControl and FillEstopPublishState. Contains no control-law output.
void DemoTaskController::FillGraspSensorAggregates(const Gains& gains) noexcept {
  // Capability-aware: sensor A path → native_prob AND force; sensor B → force
  // only. ft.valid (inference_enable) guards both — stale ticks contribute 0
  // to active_count.
  const float contact_thresh = gains.grasp_contact_threshold;
  const float force_thresh = gains.grasp_force_threshold;

  float max_force = 0.0F;
  int active_count = 0;
  // Same reduction over the LPF'd magnitudes. Computed here rather than in the
  // contact_stop block so both aggregates come out of one pass over the same
  // per-fingertip validity decisions and cannot drift apart.
  float max_force_filt = 0.0F;
  int active_count_filt = 0;

  for (int f = 0; f < num_active_fingertips_; ++f) {
    const auto idx = static_cast<std::size_t>(f);
    const auto& ft = fingertip_data_[idx];
    const float mag = ft.force_mag;  // cached in ReadState
    const float mag_filt = fingertip_force_mag_filt_[idx];

    grasp_state_.force_magnitude[idx] = mag;
    // contact_flag publish policy mirrors joint/wbc: sensor A → native
    // probability (smooth sigmoid), sensor B → derived binary so BT
    // consumers see consistent >0.5 semantics across robots.
    grasp_state_.contact_flag[idx] =
        has_native_contact_ ? ft.contact_flag : ((ft.valid && mag > force_thresh) ? 1.0F : 0.0F);
    grasp_state_.inference_valid[idx] = ft.valid;

    if (mag > max_force) {
      max_force = mag;
    }
    if (mag_filt > max_force_filt) {
      max_force_filt = mag_filt;
    }
    const bool native_path = has_native_contact_ && ft.valid;
    const bool force_active = ft.valid && (mag > force_thresh);
    const bool active =
        native_path ? (ft.contact_flag > contact_thresh && force_active) : force_active;
    if (active) {
      ++active_count;
    }
    // contact_flag is untouched by the force LPF (it is the native sigmoid, a
    // separate lane), so only the force term differs between the two counts.
    const bool force_active_filt = ft.valid && (mag_filt > force_thresh);
    const bool active_filt =
        native_path ? (ft.contact_flag > contact_thresh && force_active_filt) : force_active_filt;
    if (active_filt) {
      ++active_count_filt;
    }
  }
  contact_stop_max_force_ = max_force_filt;
  contact_stop_active_count_ = active_count_filt;
  grasp_state_.num_fingertips = num_active_fingertips_;
  grasp_state_.num_active_contacts = active_count;
  grasp_state_.max_force = max_force;
  grasp_state_.force_threshold = force_thresh;
  grasp_state_.min_fingertips_for_grasp = gains.grasp_min_fingertips;
  grasp_state_.grasp_detected = (active_count >= gains.grasp_min_fingertips);
}

void DemoTaskController::FillEstopPublishState(double dt) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::FillEstopPublishState");
  // Sensors keep streaming under E-STOP and ReadState keeps decoding them, so
  // the aggregates are real data for this tick — publish them.
  FillGraspSensorAggregates(gains_lock_.Load());
  grasp_state_.finger_s.fill(0.0F);
  grasp_state_.finger_filtered_force.fill(0.0F);
  grasp_state_.finger_force_error.fill(0.0F);
  StageEstopPullTick(pull_wiring_, dt, grasp_state_.pull);
  grasp_state_lock_.Store(grasp_state_);
  tof_snapshot_lock_.Store(tof_snapshot_);
}

// ── Phase 3c: Fill publish output ────────────────────────────────────────────

void DemoTaskController::FillPublishOutput(const ControllerState& state, ControllerOutput& output,
                                           double /*dt*/) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::FillPublishOutput");
  if (estop_active_) {
    return;
  }
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  const std::size_t nnull =
      std::min(static_cast<std::size_t>(nc0), current_target_slot_.null_target.size());
  if (arm_readable_) {
    // Model-dimension bound, as in WriteJointCommand / FillLogOutput (issue #172).
    const std::size_t nq = ArmCommandBound(nc0);
    for (std::size_t i = 0; i < nq; ++i) {
      out0.target_velocities[i] = dq_[static_cast<Eigen::Index>(i)];
      out0.trajectory_positions[i] = desired_q_[static_cast<Eigen::Index>(i)];
      out0.trajectory_velocities[i] = traj_dq_[static_cast<Eigen::Index>(i)];
    }
    for (std::size_t i = 0; i < 3; ++i) {
      out0.target_positions[i] = traj_state_.pose.translation()[static_cast<Eigen::Index>(i)];
    }
    for (std::size_t i = 3; i < nnull; ++i) {
      out0.target_positions[i] = current_target_slot_.null_target[i];
    }
  } else {
    // Same rule as FillLogOutput: withhold the stale reference, report the
    // parked position rather than a row of zeros.
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out0.goal_positions[i] = current_target_slot_.tcp_target[i];
  }
  for (std::size_t i = 3; i < nnull; ++i) {
    out0.goal_positions[i] = current_target_slot_.null_target[i];
  }

  // Arm TCP cached in ComputeControl (arm_tcp_pose_); reused here — no re-read.
  const pinocchio::SE3& tcp_current = arm_tcp_pose_;
  pinocchio::SE3 log_pose = vtcp_valid_ ? vtcp_pose_ : tcp_current;
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(log_pose.rotation());
  output.actual_task_positions[0] = log_pose.translation().x();
  output.actual_task_positions[1] = log_pose.translation().y();
  output.actual_task_positions[2] = log_pose.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.task_goal_positions[0] = current_target_slot_.tcp_target[0];
  output.task_goal_positions[1] = current_target_slot_.tcp_target[1];
  output.task_goal_positions[2] = current_target_slot_.tcp_target[2];
  if (control_6dof_cached_) {
    Eigen::Vector3d goal_rpy = pinocchio::rpy::matrixToRpy(tcp_target_pose_.rotation());
    output.task_goal_positions[3] = goal_rpy[0];
    output.task_goal_positions[4] = goal_rpy[1];
    output.task_goal_positions[5] = goal_rpy[2];
  } else {
    output.task_goal_positions[3] = rpy[0];
    output.task_goal_positions[4] = rpy[1];
    output.task_goal_positions[5] = rpy[2];
  }

  // TF source poses.
  {
    const Eigen::Vector3d& trans = tcp_current.translation();
    const Eigen::Quaterniond quat(tcp_current.rotation());
    output.arm_tip_pose.position = {trans.x(), trans.y(), trans.z()};
    output.arm_tip_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
    // Withheld on a silenced tick: the cache was not refreshed from this tick's
    // state, so arm_tcp_pose_ is history, not a measurement (#125 F1's rule).
    // This fill no longer early-returns on !arm_readable_ (the hand lanes below
    // must keep publishing), so the flag has to carry the gate itself.
    output.arm_tip_pose_valid = arm_readable_;
  }
  if (vtcp_valid_) {
    const Eigen::Vector3d& trans = vtcp_pose_.translation();
    const Eigen::Quaterniond quat(vtcp_pose_.rotation());
    output.virtual_tcp_pose.position = {trans.x(), trans.y(), trans.z()};
    output.virtual_tcp_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
    output.virtual_tcp_pose_valid = true;
  } else {
    output.virtual_tcp_pose_valid = false;
  }
  for (std::size_t f = 0; f < kNumFingertips; ++f) {
    // Gate on both a resolved serial frame id AND a pose actually produced this
    // tick: a downstream (loop) tip holds no pose until the closed chain first
    // converges, so publishing its zero-init cache would snap the fingertip TF
    // to the base origin (#125 F1).
    if (fingertip_frame_ids_[f] != 0 && fingertip_pose_valid_[f]) {
      const Eigen::Vector3d& trans = fingertip_positions_[f];
      const Eigen::Quaterniond quat(fingertip_rotations_[f]);
      output.task_link_poses[f].position = {trans.x(), trans.y(), trans.z()};
      output.task_link_poses[f].quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
      output.task_link_pose_valid[f] = true;
    } else {
      output.task_link_pose_valid[f] = false;
    }
  }

  // Task-space trajectory reference (publish-only — no log POD reads these).
  {
    const Eigen::Vector3d traj_pos = traj_state_.pose.translation();
    Eigen::Vector3d traj_rpy = pinocchio::rpy::matrixToRpy(traj_state_.pose.rotation());
    output.trajectory_task_positions[0] = traj_pos[0];
    output.trajectory_task_positions[1] = traj_pos[1];
    output.trajectory_task_positions[2] = traj_pos[2];
    output.trajectory_task_positions[3] = traj_rpy[0];
    output.trajectory_task_positions[4] = traj_rpy[1];
    output.trajectory_task_positions[5] = traj_rpy[2];

    output.trajectory_task_velocities[0] = traj_state_.velocity.linear()[0];
    output.trajectory_task_velocities[1] = traj_state_.velocity.linear()[1];
    output.trajectory_task_velocities[2] = traj_state_.velocity.linear()[2];
    output.trajectory_task_velocities[3] = traj_state_.velocity.angular()[0];
    output.trajectory_task_velocities[4] = traj_state_.velocity.angular()[1];
    output.trajectory_task_velocities[5] = traj_state_.velocity.angular()[2];
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto& out1 = output.devices[1];
    if (hand_readable_) {
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        out1.target_positions[i] = hand_computed_.positions[i];
        out1.target_velocities[i] = hand_computed_.velocities[i];
        out1.trajectory_positions[i] = hand_computed_.positions[i];
        out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      }
    } else {
      // Stale hand_computed_ must not be date-stamped as this tick's reference
      // (#291) — park the reference lanes at the measurement. Velocities stay
      // at their fresh zero: a parked finger is not moving.
      rtc::HoldTelemetryAtMeasured(out1, nc1, dev1.positions);
    }
    // The goal survives the gate on this axis too: a finger target the operator
    // set does not stop existing because the hand went unreadable.
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }
}

// ── E-STOP compute path ──────────────────────────────────────────────────────

ControllerOutput DemoTaskController::ComputeEstop(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::ComputeEstop");
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  // safe_position_ is kDemoTaskMaxArmDof (32) wide and only [0, arm_dof_) is
  // configured, while nc0 may reach kMaxDeviceChannels (64). The loop used to
  // run to nc0 — an out-of-range read past 32, and a ramp toward
  // safe_position_[i] == 0.0 (the origin) on every channel between arm_dof_ and
  // it (#265 audit T8). The bound is the OOB fix; the tail policy is what those
  // extra channels actually get. The gate is what refuses a narrow device: a
  // `q + clamp(q_safe - q)` ramp is only a ramp toward safety while q is real.
  const int nq = rtc::ModelChannelBound(nc0, arm_dof_);
  if (arm_readable_) {
    out0.num_channels = nc0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nq); ++i) {
      const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
      out0.commands[i] =
          dev0.positions[i] + std::clamp(safe_position_[i] - dev0.positions[i], -lim, lim) *
                                  ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
    }
    // No configured safe position past the model: hold where the joint is.
    rtc::FillCommandTail(out0.commands, nq, nc0, command_type_, dev0.positions);
    // Telemetry mirrors the ramp, as demo_joint and demo_wbc already do on this
    // lane. This controller used to write `commands` alone, so its E-STOP CSV
    // row showed a moving command against a zero reference (#236 S7b review).
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      out0.goal_positions[i] =
          (i < static_cast<std::size_t>(nq)) ? safe_position_[i] : out0.commands[i];
      out0.target_positions[i] = out0.commands[i];
      out0.trajectory_positions[i] = out0.commands[i];
    }
  } else {
    rtc::SilenceDeviceOutput(out0);
    // A silenced E-STOP tick is still a logged tick: report the parked position
    // rather than a row of zeros that reads as a command to the origin.
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
  }

  // Hand: hold current position during E-Stop
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto& out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.commands[i] = dev1.positions[i];
      out1.goal_positions[i] = dev1.positions[i];
      out1.target_positions[i] = dev1.positions[i];
      out1.trajectory_positions[i] = dev1.positions[i];
    }
  }

  // ── TF source poses (E-STOP path keeps arm tip tf alive) ───────────────
  // arm_readable_ joins the guard: FK on a device narrower than the model is FK
  // at the ZERO configuration, and a pose derived from joints nobody reported
  // must not be broadcast as a measurement (#125 F1's rule).
  if (arm_readable_ && arm_handle_) {
    std::span<const double> q_span(dev0.positions.data(), static_cast<std::size_t>(nc0));
    arm_handle_->ComputeForwardKinematics(q_span);
    pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
    }
    const Eigen::Vector3d& trans = tcp.translation();
    const Eigen::Quaterniond quat(tcp.rotation());
    output.arm_tip_pose.position = {trans.x(), trans.y(), trans.z()};
    output.arm_tip_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
    output.arm_tip_pose_valid = true;
  }
  output.virtual_tcp_pose_valid = false;
  for (std::size_t f = 0; f < output.task_link_pose_valid.size(); ++f) {
    output.task_link_pose_valid[f] = false;
  }

  return output;
}

}  // namespace integrated_bringup
