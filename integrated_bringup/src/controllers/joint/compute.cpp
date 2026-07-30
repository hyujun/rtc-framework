#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/fingertip_counts.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"

#include <algorithm>
#include <cmath>
#include <span>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Phase 1: Read joint states + sensor data ────────────────────────────────

void DemoJointController::ReadState(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::ReadState");
  // ── Unified kin&dyn: scatter measured joint pos/vel into Pinocchio order
  // (q_curr_full_/v_curr_full_). A raw read + reindex, so it belongs to ReadState;
  // ComputeControl Stage 1 consumes it via pinocchio_cache_.Update. Same gate as
  // the prior Compute-top block (non-E-STOP, reorder map ready, arm TCP frame
  // registered) so it runs on exactly the same ticks — byte-for-byte.
  //
  // arm_readable_ joins that gate (#236 S7b). ExtractFullState scatters into
  // the PERSISTENT q_curr_full_/v_curr_full_, and its own min(arm_dof, nc0)
  // bound does not help there: the slots it skips keep their previous value —
  // zero on the first tick — so the model would run at a partially ZERO
  // configuration that is numerically identical to reading the unreported
  // channels outright (#265 comment 2 §2). The bound stays for OOB; the gate is
  // what keeps unread state out of the model.
  if (!estop_active_ && arm_readable_ && combined_cache_.reorder_valid() &&
      arm_tcp_frame_idx_ >= 0) {
    combined_cache_.ExtractFullState(state, arm_dof_, hand_dof_);
  }

  // Robot arm joint positions (used for FK logging in WriteOutput)
  const auto& dev0 = state.devices[0];
  (void)dev0;  // positions accessed directly via span in WriteOutput

  // Hand motor data: dev1.motor_positions[], motor_velocities[],
  // motor_efforts[] available via state.devices[1].motor_* (populated from
  // /p1a/motor_states)

  // Hand sensor data (per-fingertip).
  // ToF (raw distances) is still read from sensor_data for the publish-only
  // tof_snapshot path; baro is no longer consumed. From inference_data we
  // take fx/fy/fz (sensor union slots 1..3 — populated by both sensor A and
  // B backends) plus slot 0 contact_flag *only when has_native_contact_*
  // (sensor A path; backends without native contact zero-fill the slot).
  // Slots 4..6 (displacement) remain unconsumed in joint controller.
  num_active_fingertips_ = 0;
  num_sensor_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    // Shared derivation (joint / task / wbc): active = inference-group count,
    // sensor = raw sensor-lane count. See DeriveFingertipCounts.
    const auto counts = DeriveFingertipCounts(dev1);
    num_active_fingertips_ = counts.active;
    num_sensor_fingertips_ = counts.sensor;

    const auto gains_now = gains_lock_.Load();
    const float force_threshold = gains_now.grasp_force_threshold;
    const float contact_threshold = gains_now.grasp_contact_threshold;

    for (int f = 0; f < num_active_fingertips_; ++f) {
      auto& ft = fingertip_data_[static_cast<std::size_t>(f)];
      const int base = f * kHandSensorValuesPerFingertipCapacity;

      for (std::size_t j = 0; j < 3; ++j) {
        ft.tof[j] =
            dev1.sensor_data[static_cast<std::size_t>(base) + kHandBaroChannelsCapacity + j];
      }

      ft.valid = dev1.inference_enable[static_cast<std::size_t>(f)];
      if (ft.valid) {
        const int ft_base = f * kHandInferenceValuesPerFingertipCapacity;
        for (int j = 0; j < 3; ++j) {
          ft.force[static_cast<std::size_t>(j)] =
              dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
        }
        // Capability + runtime gate: native_path requires both yaml-declared
        // capability AND per-fingertip inferencer freshness.
        const bool native_path =
            has_native_contact_ && dev1.inference_enable[static_cast<std::size_t>(f)];
        ft.contact_flag =
            native_path ? dev1.inference_data[static_cast<std::size_t>(ft_base)] : 0.0F;
        const float fx = ft.force[0];
        const float fy = ft.force[1];
        const float fz = ft.force[2];
        const float mag = std::sqrt(fx * fx + fy * fy + fz * fz);
        ft.force_mag = mag;
        const bool force_active = (mag > force_threshold);
        // Sensor A (native): require both native prob AND force threshold.
        // Sensor B (force-only): force threshold alone.
        ft.in_contact =
            native_path ? (ft.contact_flag > contact_threshold && force_active) : force_active;
      } else {
        ft.force = {};
        ft.force_mag = 0.0F;
        ft.contact_flag = 0.0F;
        ft.in_contact = false;
      }
      UpdateFingertipForceFilter(static_cast<std::size_t>(f),
                                 {gains_now.contact_stop_force_guard_delta_n,
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
void DemoJointController::UpdateFingertipForceFilter(
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

void DemoJointController::UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp,
                                           const Gains& gains) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::UpdateVirtualTcp");
  vtcp_valid_ = false;
  if (!hand_handle_ || gains.vtcp.mode == VirtualTcpMode::kDisabled)
    return;

  // Build fingertip inputs from hand model FK (already computed in
  // ComputeControl). Closed-chain-consistent when the #121 helper is active;
  // serial hand_handle_ FK otherwise — same value.
  for (std::size_t f = 0; f < kNumFingertips; ++f) {
    pinocchio::SE3 ft_pose;
    vtcp_inputs_[f].active = HandFingertipPose(f, ft_pose);
    if (!vtcp_inputs_[f].active)
      continue;
    vtcp_inputs_[f].position_in_tcp = ft_pose.translation();
    // Force magnitude for weighted mode (cached in ReadState; 0 when !valid).
    vtcp_inputs_[f].force_magnitude = static_cast<double>(fingertip_data_[f].force_mag);
  }

  const auto result = ComputeVirtualTcp(gains.vtcp, T_base_tcp, vtcp_inputs_);
  if (result.valid) {
    vtcp_pose_ = result.world_pose;
    vtcp_valid_ = true;
  }
}

// ── Phase 2: Compute control (trajectory + sensor-based logic) ──────────────

void DemoJointController::ComputeControl(const ControllerState& state, double dt) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::ComputeControl");
  // ── Stage 1 (compute model): refresh the combined-model cache from the state
  // ReadState scattered into q_curr_full_/v_curr_full_ this tick. Reached only on
  // non-E-STOP ticks (Compute() E-STOP-early-returns before here), so the prior
  // !estop_active_ guard is implied; gate on cache readiness only. Stage 2 (the
  // trajectory control law below + the arm_tcp_pose_ FK) consumes it. Same gate
  // and same q_curr_full_ as the prior Compute-top Update — byte-for-byte.
  // arm_readable_ gates it for the same reason ReadState's scatter is gated:
  // refreshing the model from a state this tick could not read would put the
  // partially-ZERO configuration into every consumer downstream of the cache.
  if (arm_readable_ && combined_cache_.reorder_valid() && arm_tcp_frame_idx_ >= 0) {
    combined_cache_.Update();
  }

  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  // The grasp mode rides that same snapshot; cache it for the output fills,
  // which have no Gains of their own (rationale on the member).
  grasp_hand_mode_cached_ = gains.grasp_hand_mode;
  const auto& dev0 = state.devices[0];

  // ── Robot arm trajectory ────────────────────────────────────────────────
  // current_target_slot_ was refreshed by DrainTargetSlot() at the start of
  // Compute(); the flags are RT-thread-only.
  //
  // The whole block is behind the F5 gate (#265 audit J2). Its start state is
  // dev0.positions read arm_dof_ deep, so on a device narrower than the arm the
  // unreported joints would enter the quintic as 0 and the arm would be
  // trajectory-planned toward the origin. Skipping leaves robot_new_target_
  // pending_ latched, so a goal that arrived during the outage is planned from
  // the MEASURED state on the first readable tick rather than being lost.
  if (arm_readable_) {
    if (robot_new_target_pending_) {
      trajectory::JointSpaceTrajectory<kDemoJointMaxArmDof>::State start_state;
      trajectory::JointSpaceTrajectory<kDemoJointMaxArmDof>::State goal_state;

      double max_dist = 0.0;
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        start_state.positions[idx] = dev0.positions[idx];
        start_state.velocities[idx] = 0.0;
        start_state.accelerations[idx] = 0.0;

        goal_state.positions[idx] = current_target_slot_.targets[0][idx];
        goal_state.velocities[idx] = 0.0;
        goal_state.accelerations[idx] = 0.0;

        max_dist = std::max(max_dist,
                            std::abs(current_target_slot_.targets[0][idx] - dev0.positions[idx]));
      }

      // Duration from trajectory_speed, then enforce max trajectory velocity.
      // Quintic rest-to-rest peak velocity = (15/8) * max_dist / T.
      const double T_speed = max_dist / gains.robot_trajectory_speed;
      const double T_vel = (gains.robot_max_traj_velocity > 0.0)
                               ? (1.875 * max_dist / gains.robot_max_traj_velocity)
                               : 0.0;
      const double duration = std::max({0.01, T_speed, T_vel});
      robot_trajectory_.initialize(start_state, goal_state, duration);
      robot_trajectory_time_ = 0.0;
      robot_new_target_pending_ = false;
    }

    const auto robot_traj = robot_trajectory_.compute(robot_trajectory_time_);
    robot_trajectory_time_ += dt;

    for (int i = 0; i < arm_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      robot_computed_.positions[idx] = robot_traj.positions[idx];
      robot_computed_.velocities[idx] = robot_traj.velocities[idx];
    }
    // Channels the device reports but the trajectory does not model — the
    // `nc0 > arm_dof_` direction, which is a NORMAL input (num_channels is the
    // wire length). Leaving them unwritten is a fresh zero, and this is a
    // position lane, so that reads as "go to the origin" on every extra channel
    // (#265 audit J3). robot_computed_ is what all three output fills below
    // copy, so filling the tail HERE is what makes their `i < nc0` loops mean
    // what they say. Velocities stay 0: a held joint is not moving.
    rtc::FillCommandTail(robot_computed_.positions, arm_dof_, dev0.num_channels, command_type_,
                         dev0.positions);
  }

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];

    // hand_readable_ joins the pending flag rather than replacing the enclosing
    // guard (#291): this seed is the only dev1 read in the hand trajectory
    // block, and it runs hand_dof_ deep, so a narrow hand would start the ramp
    // from a phantom origin and drive the unreported fingers origin-ward over
    // `duration`. Scoping it here — not early-returning — keeps the trajectory
    // evaluation below running. The flag stays pending, so the seed is retried
    // the tick the hand becomes readable (the arm's arm_target_initialized_
    // deferral, #265 audit T1/T2, on the hand axis).
    if (hand_new_target_pending_ && hand_readable_) {
      trajectory::JointSpaceTrajectory<kDemoJointMaxHandDof>::State start_state;
      trajectory::JointSpaceTrajectory<kDemoJointMaxHandDof>::State goal_state;

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

  // ── Arm FK: base → tip (from the unified combined-model cache) ────────
  // The cache was Updated at the top of Compute for this non-E-STOP tick; read
  // its registered arm TCP frame (world tip, base-relative when a root frame is
  // registered). arm_handle_ is no longer read here (retained only for the
  // E-STOP TF path). Unconfigured cache → Identity, matching the prior
  // null-arm_handle_ default so grasp/sensor-only unit fixtures stay unchanged.
  // Cached for the Fill* output-composition phase (WriteJointCommand /
  // FillLogOutput do not recompute it).
  arm_tcp_pose_ = combined_cache_.ArmTcpPoseFromCache(arm_tcp_frame_idx_, arm_base_frame_idx_);

  // ── Hand fingertip FK (tree model) — base-to-fingertip ──────────────
  // #121: closed-chain projection when the hand has loop closure with downstream
  // fingertips, else the serial hand FK; HandFingertipPose returns the
  // hand-root-relative fingertip pose either way (byte-for-byte when serial).
  // Default to invalid each tick so a tick where hand FK fails entirely (e.g.
  // the hand device drops out) withholds the fingertip TF rather than
  // republishing a prior tick's cached pose as valid — matches wbc, whose gate
  // sits inside `if (ComputeHandFingertipFk(...))` on a fresh output (#125 F1).
  fingertip_pose_valid_.fill(false);
  if (ComputeHandForwardKinematics(state)) {
    // Chain: T_base_fingertip = T_base_tcp * T_hand_fingertip
    for (std::size_t f = 0; f < kNumFingertips; ++f) {
      pinocchio::SE3 T_hand_ft;
      const bool produced = HandFingertipPose(f, T_hand_ft);
      fingertip_pose_valid_[f] = produced;
      if (produced) {
        const pinocchio::SE3 T_base_ft = arm_tcp_pose_.act(T_hand_ft);
        fingertip_positions_[f] = T_base_ft.translation();
        fingertip_rotations_[f] = T_base_ft.rotation();
      }
    }

    // Virtual TCP computation (uses hand FK data computed above)
    UpdateVirtualTcp(arm_tcp_pose_, gains);
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

    // Hand grasp control: force_pi (adaptive PI) or contact_stop (binary
    // freeze)
    if (grasp_controller_ && gains.grasp_hand_mode == GraspHandMode::kForcePi) {
      // Build force input: one force magnitude per grasp finger.
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
      if (cur_phase != prev_grasp_phase_) {
        RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                             "[force_pi] phase %u -> %u target_force=%.2fN", prev_grasp_phase_,
                             cur_phase, grasp_controller_->target_force());
        prev_grasp_phase_ = cur_phase;
      }

      if (grasp_controller_->phase() != rtc::grasp::GraspPhase::kIdle) {
        // Force PI 활성: 매핑된 finger joint 들만 덮어쓰기; 나머지 hand joint
        // (예: ring)은 trajectory 출력 그대로 유지.
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
    } else if (gains.grasp_hand_mode != GraspHandMode::kNone) {
      // grasp_controller_type=="none": no hand intervention — trajectory output
      // passes through untouched.  GraspState aggregation/publishing below is
      // unaffected (BT IsGrasped/IsForceAbove still observe contact).  The
      // negated form (!= "none") is deliberate: a "force_pi" config whose
      // grasp controller failed to build (null) must still fall into the
      // contact_stop safety freeze, not silently become a no-op.
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
        std::array<double, kDemoJointMaxHandDof> hand_meas{};
        for (int i = 0; i < hand_dof_; ++i) {
          hand_meas[static_cast<std::size_t>(i)] = dev1.positions[static_cast<std::size_t>(i)];
        }
        const std::array<double, kDemoJointMaxHandDof> hand_filt =
            hand_pos_filter_.Apply(hand_meas);

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

    // ── In-plane pull-force estimate (#167) ─────────────────────────────
    // Feeds the 3-axis fingertip forces (before the |F| collapse above) plus
    // the per-tip FK rotations/positions cached this tick. The baseline
    // snapshot arms on the grasp_detected rising edge; the result rides
    // grasp_state_.pull to the owned GraspState SeqLock (no new topic).
    if (pull_wiring_.enabled()) {
      // Own span so the estimator's latency is separable from the rest of
      // ComputeControl in Perfetto — WBC has had one since #167, joint/task
      // were only covered by the enclosing controller span (#234 P-20).
      RTC_TRACE_SCOPE("DemoJointController::UpdatePullEstimate");
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
// Populates ONLY fields the DeviceBackend::WriteCommand consumes (verified via
// audit of MujocoNativeBackend / UrDriverNativeBackend / UdpHandNativeBackend
// — all three read slot.commands, slot.num_channels, and command_type).
// ClampRange lives here because the clamp's job is to make `commands` safe
// just before the wire.

ControllerOutput DemoJointController::WriteJointCommand(const ControllerState& state,
                                                        double /*dt*/) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::WriteJointCommand");
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;
  output.valid = true;

  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  if (arm_readable_) {
    // Every channel in [0, nc0) is real: ComputeControl filled [0, arm_dof_)
    // from the trajectory and the rest from the tail policy.
    out0.num_channels = nc0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      out0.commands[i] = robot_computed_.positions[i];
    }
    rtc::utils::ClampRange(out0.commands, nc0, std::span<const double>(device_position_lower_[0]),
                           std::span<const double>(device_position_upper_[0]), -6.2832, 6.2832);
  } else {
    // F5: this tick has no honest arm command, so it issues none. Zero-length is
    // "no update" — the drive holds its previous setpoint. Emitting nc0 zeros
    // instead would be a real command to the origin (§3.7). The reference lanes
    // are filled by FillLog/FillPublishOutput so the logged row does not read as
    // that very command.
    rtc::SilenceDeviceOutput(out0);
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

// ── Phase 3b: Fill log output ────────────────────────────────────────────────
//
// Populates fields the DeviceStateLogPod reads (verified via
// integrated_bringup/logging/pod_fill.hpp: out.commands[i] — already filled by
// WriteJointCommand — plus out.goal_positions, out.trajectory_positions,
// out.trajectory_velocities, output.actual_task_positions,
// output.task_goal_positions). FK happens here because actual_task_positions
// needs it. SeqLock store of grasp/tof staging buffers also belongs to the
// log/publish path, not the wire.

void DemoJointController::FillLogOutput(const ControllerState& state, ControllerOutput& output,
                                        double /*dt*/) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::FillLogOutput");
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  // Telemetry follows the wire: on a tick that issued no arm command there is
  // no arm trajectory to report either, and robot_computed_ still holds the
  // last readable tick's values. Publishing those would date-stamp stale
  // numbers as this tick's reference.
  if (arm_readable_) {
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      out0.trajectory_positions[i] = robot_computed_.positions[i];
      out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    }
  } else {
    // Withholding is not the same as writing zeros. The log POD copies these
    // arrays bounded by the DEVICE's channel count, not the output's, and it
    // has no field for the emitted width — so an untouched row is recorded as
    // zeros and reads as a command to the origin, on the one failure mode this
    // gate exists for. Report where the drive is parked instead (see
    // rtc_controller_interface/device_readability.hpp, HoldTelemetryAtMeasured).
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
  }
  // The goal survives the gate — a latched hold target does not stop being the
  // target because this tick could not be read.
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.goal_positions[i] = current_target_slot_.targets[0][i];
  }

  // Arm FK was computed once in ComputeControl and cached in arm_tcp_pose_.
  const pinocchio::SE3& log_pose = vtcp_valid_ ? vtcp_pose_ : arm_tcp_pose_;
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(log_pose.rotation());
  output.actual_task_positions[0] = log_pose.translation().x();
  output.actual_task_positions[1] = log_pose.translation().y();
  output.actual_task_positions[2] = log_pose.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];
  // Joint mode: no explicit task goal from GUI, mirror FK result.
  output.task_goal_positions = output.actual_task_positions;

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
  // law produced, so it must agree with the branch ComputeControl took.
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
  // Per-controller SeqLock handoff to the publish thread (Phase 4c).
  grasp_state_lock_.Store(grasp_state_);
  tof_snapshot_lock_.Store(tof_snapshot_);
}

// ── E-STOP publish state (#234 P-1) ─────────────────────────────────────────
//
// Rationale in the header. Mirrors FillLogOutput's store, minus everything the
// control law did not run this tick.

// Sensor-derived grasp aggregates. Sourced from fingertip_data_, which
// ReadState refreshes every tick including E-STOP — hence shared by
// ComputeControl and FillEstopPublishState. Contains no control-law output.
void DemoJointController::FillGraspSensorAggregates(const Gains& gains) noexcept {
  // Capability-aware: ReadState has already encoded the sensor path in
  // ft.in_contact (sensor A → native_prob+force AND, sensor B → force only).
  // grasp_contact_threshold is consulted only on sensor A paths there.
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
    // contact_flag publish policy: sensor A path → native probability so
    // downstream consumers see the smooth sigmoid value; sensor B path →
    // derived binary (1/0 from in_contact). Both work with the BT
    // convention `contact_flag > 0.5f`.
    grasp_state_.contact_flag[idx] =
        has_native_contact_ ? ft.contact_flag : (ft.in_contact ? 1.0F : 0.0F);
    grasp_state_.inference_valid[idx] = ft.valid;

    if (mag > max_force) {
      max_force = mag;
    }
    if (ft.in_contact) {
      ++active_count;
    }
    if (mag_filt > max_force_filt) {
      max_force_filt = mag_filt;
    }
    // ft.in_contact is ReadState's raw-force verdict, so the filtered count has
    // to re-run that decision here. contact_flag is untouched by the force LPF
    // (separate native lane) and ReadState already zeroes it off the native
    // path, so only the force term differs between the two counts.
    const bool native_path = has_native_contact_ && ft.valid;
    const bool force_active_filt = ft.valid && (mag_filt > gains.grasp_force_threshold);
    const bool active_filt =
        native_path ? (ft.contact_flag > gains.grasp_contact_threshold && force_active_filt)
                    : force_active_filt;
    if (active_filt) {
      ++active_count_filt;
    }
  }
  contact_stop_max_force_ = max_force_filt;
  contact_stop_active_count_ = active_count_filt;
  grasp_state_.num_fingertips = num_active_fingertips_;
  grasp_state_.num_active_contacts = active_count;
  grasp_state_.max_force = max_force;
  grasp_state_.force_threshold = gains.grasp_force_threshold;
  grasp_state_.min_fingertips_for_grasp = gains.grasp_min_fingertips;
  grasp_state_.grasp_detected = (active_count >= gains.grasp_min_fingertips);
}

void DemoJointController::FillEstopPublishState(double dt) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::FillEstopPublishState");
  // Sensors keep streaming under E-STOP and ReadState keeps decoding them, so
  // the aggregates are real data for this tick — publish them.
  FillGraspSensorAggregates(gains_lock_.Load());
  // Force-PI per-finger diagnostics: the grasp controller was not stepped, so
  // last tick's servo error is not this tick's. Zero rather than freeze.
  grasp_state_.finger_s.fill(0.0F);
  grasp_state_.finger_filtered_force.fill(0.0F);
  grasp_state_.finger_force_error.fill(0.0F);
  StageEstopPullTick(pull_wiring_, dt, grasp_state_.pull);
  grasp_state_lock_.Store(grasp_state_);
  tof_snapshot_lock_.Store(tof_snapshot_);
}

// ── Phase 3c: Fill publish output ────────────────────────────────────────────
//
// Populates fields the publish snapshot / owned_topics consume — verified via
// rtc_controller_manager/src/rt_controller_node_rt_loop.cpp:147-218 (snapshot
// build) and integrated_bringup/src/support/owned_topics.cpp:384-405 (TF
// publish). target_positions / target_velocities go to gc.* slots;
// goal_positions / trajectory_* are also mirrored here so reading this
// method alone reveals everything the publish path needs (overlap with
// FillLogOutput is intentional — write twice rather than couple ordering).
// Pose fields drive the kRobotTransforms TF publisher.

void DemoJointController::FillPublishOutput(const ControllerState& state, ControllerOutput& output,
                                            double /*dt*/) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::FillPublishOutput");
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  // Same reason as FillLogOutput: a silenced arm has no reference to publish,
  // and the parked position is what the row should show instead of zeros.
  if (arm_readable_) {
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      out0.target_positions[i] = robot_computed_.positions[i];
      out0.target_velocities[i] = robot_computed_.velocities[i];
      out0.trajectory_positions[i] = robot_computed_.positions[i];
      out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    }
  } else {
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
  }
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.goal_positions[i] = current_target_slot_.targets[0][i];
  }

  // Arm FK cached in ComputeControl (arm_tcp_pose_); reused here — no recompute.
  const pinocchio::SE3& tcp = arm_tcp_pose_;
  const pinocchio::SE3& log_pose = vtcp_valid_ ? vtcp_pose_ : tcp;
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(log_pose.rotation());
  output.actual_task_positions[0] = log_pose.translation().x();
  output.actual_task_positions[1] = log_pose.translation().y();
  output.actual_task_positions[2] = log_pose.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];
  output.task_goal_positions = output.actual_task_positions;

  // TF source poses for kRobotTransforms publish. Withheld on a silenced tick:
  // arm_tcp_pose_ then holds whatever the cache had before the outage — or
  // identity if there was never a readable tick — and neither is this tick's
  // measurement (#125 F1's rule, applied to the F5 gate).
  {
    const Eigen::Vector3d& t = tcp.translation();
    const Eigen::Quaterniond q(tcp.rotation());
    output.arm_tip_pose.position = {t.x(), t.y(), t.z()};
    output.arm_tip_pose.quaternion = {q.w(), q.x(), q.y(), q.z()};
    output.arm_tip_pose_valid = arm_readable_;
  }
  if (vtcp_valid_) {
    const Eigen::Vector3d& t = vtcp_pose_.translation();
    const Eigen::Quaterniond q(vtcp_pose_.rotation());
    output.virtual_tcp_pose.position = {t.x(), t.y(), t.z()};
    output.virtual_tcp_pose.quaternion = {q.w(), q.x(), q.y(), q.z()};
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
      const Eigen::Vector3d& t = fingertip_positions_[f];
      const Eigen::Quaterniond q(fingertip_rotations_[f]);
      output.task_link_poses[f].position = {t.x(), t.y(), t.z()};
      output.task_link_poses[f].quaternion = {q.w(), q.x(), q.y(), q.z()};
      output.task_link_pose_valid[f] = true;
    } else {
      output.task_link_pose_valid[f] = false;
    }
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
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
      rtc::HoldTelemetryAtMeasured(out1, nc1, state.devices[1].positions);
    }
    // The goal survives the gate on this axis too: a finger target the operator
    // set does not stop existing because the hand went unreadable.
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }
}

ControllerOutput DemoJointController::ComputeEstop(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::ComputeEstop");
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;

  // Robot arm: move toward safe position with velocity limit.
  //
  // F5 covers this lane too (#236 E-8, approved). The ramp is
  // `q + clamp(q_safe - q)`, which is only a ramp toward safety while q is a
  // real measurement; with an unreadable device the unreported joints read 0
  // and this would ramp them to safe_position_[i] from a fabricated origin.
  // The answer is the same silence Compute() gives — NOT nc0 zeros, and not an
  // nc0-length ramp (§3.7).
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  // safe_position_ is kDemoJointMaxArmDof wide and only [0, arm_dof_) is
  // configured, while nc0 may reach kMaxDeviceChannels — twice as far. The loop
  // used to run to nc0, so `nc0 > kDemoJointMaxArmDof` read out of range and
  // `arm_dof_ <= i < kDemoJointMaxArmDof` ramped toward safe_position_[i] ==
  // 0.0, i.e. toward the origin (#265 audit J5). The bound is the OOB fix; the
  // tail policy is what those extra channels actually get.
  const int nq = rtc::ModelChannelBound(nc0, arm_dof_);
  if (arm_readable_) {
    out0.num_channels = nc0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nq); ++i) {
      const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
      out0.commands[i] =
          dev0.positions[i] + std::clamp(safe_position_[i] - dev0.positions[i], -lim, lim) *
                                  ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
      out0.goal_positions[i] = safe_position_[i];
      out0.target_positions[i] = out0.commands[i];
      out0.trajectory_positions[i] = out0.commands[i];
    }
    // No configured safe position past the model: hold where the joint is.
    rtc::FillCommandTail(out0.commands, nq, nc0, command_type_, dev0.positions);
    for (std::size_t i = static_cast<std::size_t>(nq); i < static_cast<std::size_t>(nc0); ++i) {
      out0.goal_positions[i] = out0.commands[i];
      out0.target_positions[i] = out0.commands[i];
      out0.trajectory_positions[i] = out0.commands[i];
    }
  } else {
    rtc::SilenceDeviceOutput(out0);
    // A silenced E-STOP tick is still a logged tick — FillLogOutput does not run
    // on this lane, so the parked-position fill has to happen here.
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

  // FK for task-space logging (same as normal path). arm_handle_ is always
  // non-null in production; the guard mirrors ComputeControl so the empty-URDF
  // unit fixture (null arm_handle_) can drive ComputeEstop without a crash.
  // arm_readable_ joins it: FK on a device narrower than the model is FK at the
  // ZERO configuration, and a pose derived from joints nobody reported is not a
  // pose worth broadcasting.
  pinocchio::SE3 tcp = pinocchio::SE3::Identity();
  if (arm_readable_ && arm_handle_) {
    std::span<const double> q_span(dev0.positions.data(), static_cast<std::size_t>(nc0));
    arm_handle_->ComputeForwardKinematics(q_span);
    tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
    }
  }
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];
  output.task_goal_positions = output.actual_task_positions;

  // ── TF source poses (E-STOP path keeps tf alive for RViz) ─────────────
  // Arm tip raw FK still valid; virtual TCP / fingertip poses are not
  // updated under E-STOP (hand hold), so keep their valid flags false.
  {
    const Eigen::Vector3d& t = tcp.translation();
    const Eigen::Quaterniond q(tcp.rotation());
    output.arm_tip_pose.position = {t.x(), t.y(), t.z()};
    output.arm_tip_pose.quaternion = {q.w(), q.x(), q.y(), q.z()};
    // Withheld rather than published as identity when the FK above did not run
    // — the same rule the fingertip TF already follows (#125 F1), and the one
    // DemoTaskController's E-STOP path already applies to a null arm_handle_:
    // a consumer must not be handed the base origin as though it were a
    // measurement.
    output.arm_tip_pose_valid = arm_readable_ && (arm_handle_ != nullptr);
  }
  output.virtual_tcp_pose_valid = false;
  for (std::size_t f = 0; f < output.task_link_pose_valid.size(); ++f) {
    output.task_link_pose_valid[f] = false;
  }

  return output;
}

}  // namespace integrated_bringup
