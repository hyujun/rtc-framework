#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/utils/clamp_commands.hpp"

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
  // Robot arm joint positions (used for FK logging in WriteOutput)
  const auto& dev0 = state.devices[0];
  (void)dev0;  // positions accessed directly via span in WriteOutput

  // Hand motor data: dev1.motor_positions[], motor_velocities[],
  // motor_efforts[] available via state.devices[1].motor_* (populated from
  // /hand/motor_states)

  // Hand sensor data (per-fingertip).
  // ToF (raw distances) is still read from sensor_data for the publish-only
  // tof_snapshot path; baro is no longer consumed. From inference_data we
  // take fx/fy/fz (sensor union slots 1..3 — populated by both sensor A and
  // B backends) plus slot 0 contact_flag *only when has_native_contact_*
  // (sensor A path; backends without native contact zero-fill the slot).
  // Slots 4..6 (displacement) remain unconsumed in joint controller.
  num_active_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / kHandSensorValuesPerFingertipCapacity;
    num_active_fingertips_ = std::min(num_fingertips, static_cast<int>(rtc::kMaxSensorGroups));

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
    }
  }
}

// ── Virtual TCP computation ─────────────────────────────────────────────────

void DemoJointController::UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp,
                                           const Gains& gains) noexcept {
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
  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  const auto& dev0 = state.devices[0];

  // ── Robot arm trajectory ────────────────────────────────────────────────
  // current_target_slot_ was refreshed by DrainTargetSlot() at the start of
  // Compute(); the flags are RT-thread-only.
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

      max_dist =
          std::max(max_dist, std::abs(current_target_slot_.targets[0][idx] - dev0.positions[idx]));
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

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];

    if (hand_new_target_pending_) {
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

  // ── Arm FK: base → tip (computed once per tick; cached for Fill*) ──────
  // WriteJointCommand / FillLogOutput do not touch arm_handle_, so the cached
  // pose stays valid for the whole output-composition phase.
  {
    const int nc0 = dev0.num_channels;
    std::span<const double> q_span(dev0.positions.data(), static_cast<std::size_t>(nc0));
    arm_handle_->ComputeForwardKinematics(q_span);
    arm_tcp_pose_ = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      arm_tcp_pose_ = arm_handle_->GetFramePlacement(root_frame_id_).actInv(arm_tcp_pose_);
    }
  }

  // ── Hand fingertip FK (tree model) — base-to-fingertip ──────────────
  // #121: closed-chain projection when the hand has loop closure with downstream
  // fingertips, else the serial hand FK; HandFingertipPose returns the
  // hand-root-relative fingertip pose either way (byte-for-byte when serial).
  if (ComputeHandForwardKinematics(state)) {
    // Chain: T_base_fingertip = T_base_tcp * T_hand_fingertip
    for (std::size_t f = 0; f < kNumFingertips; ++f) {
      pinocchio::SE3 T_hand_ft;
      if (HandFingertipPose(f, T_hand_ft)) {
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
    // Capability-aware: ReadState has already encoded the sensor path in
    // ft.in_contact (sensor A → native_prob+force AND, sensor B → force only).
    // grasp_contact_threshold is consulted only on sensor A paths there.
    const float force_thresh = gains.grasp_force_threshold;
    const int min_fingers = gains.grasp_min_fingertips;

    float max_force = 0.0f;
    int active_count = 0;

    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const auto& ft = fingertip_data_[idx];
      const float mag = ft.force_mag;  // cached in ReadState

      grasp_state_.force_magnitude[idx] = mag;
      // contact_flag publish policy: sensor A path → native probability so
      // downstream consumers see the smooth sigmoid value; sensor B path →
      // derived binary (1/0 from in_contact). Both work with the BT
      // convention `contact_flag > 0.5f`.
      grasp_state_.contact_flag[idx] =
          has_native_contact_ ? ft.contact_flag : (ft.in_contact ? 1.0F : 0.0F);
      grasp_state_.inference_valid[idx] = ft.valid;

      if (mag > max_force)
        max_force = mag;
      if (ft.in_contact) {
        ++active_count;
      }
    }
    grasp_state_.num_fingertips = num_active_fingertips_;
    grasp_state_.num_active_contacts = active_count;
    grasp_state_.max_force = max_force;
    grasp_state_.force_threshold = force_thresh;
    grasp_state_.min_fingertips_for_grasp = min_fingers;
    grasp_state_.grasp_detected = (active_count >= min_fingers);

    // Periodic grasp status snapshot (2s throttle, debug only).
    // NOTE: throttled logging on the 500Hz path — the rare allocation
    // inside rclcpp logging macros is acceptable at this interval.
    RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "[grasp] type=%s active=%d/%d max_force=%.2fN thresh=%.2fN phase=%d",
                         grasp_controller_type_.c_str(), active_count, num_active_fingertips_,
                         static_cast<double>(max_force), static_cast<double>(force_thresh),
                         grasp_controller_ ? static_cast<int>(grasp_controller_->phase()) : -1);

    // Hand grasp control: force_pi (adaptive PI) or contact_stop (binary
    // freeze)
    if (grasp_controller_ && grasp_controller_type_ == "force_pi") {
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
    } else {
      // ContactStopHand: 힘 감지 시 hand trajectory 출력을 현재 위치로 동결
      // → BT tick(50ms) 사이에도 과도한 hand closure 방지
      //
      // Release-phase gate: 사용자가 손을 여는 방향으로 goal을 내린 경우에는
      // 접촉 잔존 힘이 있더라도 freeze 를 skip 해야 손이 열린다.
      // per-finger gate: release_gate_sign_[f] 방향으로 target-actual 이 움직이면
      // 해당 finger 는 이완 중. 모든 gate finger 가 이완 방향일 때만 "release 의도".
      //   sign > 0: 각도 증가 = loosening (e.g. thumb CMC_FE) → target > actual
      //   sign < 0: 각도 감소 = loosening (e.g. index/middle MCP_FE) → target < actual
      if (state.num_devices > 1 && state.devices[1].valid) {
        const auto& dev1 = state.devices[1];

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

        if (release_phase) {
          RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                               "[contact_stop] SKIP (release) dgate=[%+.3f,%+.3f,%+.3f]",
                               gate_err[0], gate_err[1], gate_err[2]);
        } else if (active_count > 0 && max_force > force_thresh) {
          for (int i = 0; i < hand_dof_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            hand_computed_.positions[idx] = dev1.positions[idx];
            hand_computed_.velocities[idx] = 0.0;
          }
          // gate errors (target - actual) encode both the actual position and
          // the overshoot beyond target, enough to diagnose contact_stop engage.
          RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                               "[contact_stop] FREEZE active=%d fmax=%.2fN "
                               "dgate=[%+.3f,%+.3f,%+.3f]",
                               active_count, static_cast<double>(max_force), gate_err[0],
                               gate_err[1], gate_err[2]);
        }
      }
    }
  }

  // ── ToF snapshot (3 fingers × 2 sensors: tof[1]=A, tof[2]=B) ───────────
  {
    constexpr int kNumTofFingers = 3;     // thumb, index, middle (hand-specific)
    constexpr int kSensorsPerFinger = 2;  // A/B pair
    constexpr double kMmToM = 0.001;
    tof_snapshot_ = {};

    if (hand_handle_ && num_active_fingertips_ >= kNumTofFingers) {
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
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;
  output.valid = true;

  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = robot_computed_.positions[i];
  }
  rtc::utils::ClampRange(out0.commands, nc0, std::span<const double>(device_position_lower_[0]),
                         std::span<const double>(device_position_upper_[0]), -6.2832, 6.2832);

  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.commands[i] = hand_computed_.positions[i];
    }
    rtc::utils::ClampRange(out1.commands, nc1, std::span<const double>(device_position_lower_[1]),
                           std::span<const double>(device_position_upper_[1]), -6.2832, 6.2832);
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
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
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
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }

  if (grasp_controller_ && grasp_controller_type_ == "force_pi") {
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
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = robot_computed_.positions[i];
    out0.target_velocities[i] = robot_computed_.velocities[i];
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
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

  // TF source poses for kRobotTransforms publish.
  {
    const Eigen::Vector3d& t = tcp.translation();
    const Eigen::Quaterniond q(tcp.rotation());
    output.arm_tip_pose.position = {t.x(), t.y(), t.z()};
    output.arm_tip_pose.quaternion = {q.w(), q.x(), q.y(), q.z()};
    output.arm_tip_pose_valid = true;
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
    if (fingertip_frame_ids_[f] != 0) {
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
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.target_positions[i] = hand_computed_.positions[i];
      out1.target_velocities[i] = hand_computed_.velocities[i];
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }
}

ControllerOutput DemoJointController::ComputeEstop(const ControllerState& state) noexcept {
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;

  // Robot arm: move toward safe position with velocity limit
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    out0.commands[i] =
        dev0.positions[i] + std::clamp(safe_position_[i] - dev0.positions[i], -lim, lim) *
                                ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
    out0.goal_positions[i] = safe_position_[i];
    out0.target_positions[i] = out0.commands[i];
    out0.trajectory_positions[i] = out0.commands[i];
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

  // FK for task-space logging (same as normal path)
  std::span<const double> q_span(dev0.positions.data(), static_cast<std::size_t>(nc0));
  arm_handle_->ComputeForwardKinematics(q_span);
  pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
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
    output.arm_tip_pose_valid = true;
  }
  output.virtual_tcp_pose_valid = false;
  for (std::size_t f = 0; f < output.task_link_pose_valid.size(); ++f) {
    output.task_link_pose_valid[f] = false;
  }

  return output;
}

}  // namespace integrated_bringup
