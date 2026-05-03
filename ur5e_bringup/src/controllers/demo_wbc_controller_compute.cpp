#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "ur5e_bringup/logging/pod_fill.hpp"

#include <algorithm>
#include <cmath>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace ur5e_bringup {

// ── Phase 1: Read state ─────────────────────────────────────────────────────
void DemoWbcController::ReadState(const ControllerState& state) noexcept {
  // Parse fingertip F/T inference data + raw sensor channels for
  // contact detection (kClosure -> kHold) and anomaly monitoring (kHold).
  // Layout mirrors DemoJointController::ReadState for consistency.
  num_active_fingertips_ = 0;
  if (state.num_devices <= 1 || !state.devices[1].valid) {
    return;
  }

  const auto& dev1 = state.devices[1];
  const int num_sensor_ch = dev1.num_sensor_channels;
  const int num_fingertips =
      (rtc::kSensorValuesPerFingertip > 0) ? (num_sensor_ch / rtc::kSensorValuesPerFingertip) : 0;
  num_active_fingertips_ = std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

  const double inv_dt = (state.dt > 0.0) ? (1.0 / state.dt) : 500.0;

  for (int f = 0; f < num_active_fingertips_; ++f) {
    auto& ft = fingertip_data_[static_cast<std::size_t>(f)];
    const int base = f * rtc::kSensorValuesPerFingertip;

    for (std::size_t j = 0; j < rtc::kBarometerCount; ++j) {
      ft.baro[j] = dev1.sensor_data[static_cast<std::size_t>(base) + j];
    }
    for (std::size_t j = 0; j < 3; ++j) {
      ft.tof[j] = dev1.sensor_data[static_cast<std::size_t>(base) + rtc::kBarometerCount + j];
    }

    ft.valid = dev1.inference_enable[static_cast<std::size_t>(f)];
    if (ft.valid) {
      const int ft_base = f * rtc::kFTValuesPerFingertip;
      ft.contact_flag = dev1.inference_data[static_cast<std::size_t>(ft_base)];
      for (int j = 0; j < 3; ++j) {
        ft.force[static_cast<std::size_t>(j)] =
            dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
        ft.displacement[static_cast<std::size_t>(j)] =
            dev1.inference_data[static_cast<std::size_t>(ft_base + 4 + j)];
      }
      const float fx = ft.force[0];
      const float fy = ft.force[1];
      const float fz = ft.force[2];
      ft.force_magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
    } else {
      ft.contact_flag = 0.0f;
      ft.force = {};
      ft.displacement = {};
      ft.force_magnitude = 0.0f;
    }

    // df/dt with EMA smoothing; skip on first tick to avoid startup spike
    if (force_rate_initialized_) {
      const float raw_rate =
          static_cast<float>((ft.force_magnitude - ft.prev_force_magnitude) * inv_dt);
      ft.force_rate = force_rate_alpha_ * raw_rate + (1.0f - force_rate_alpha_) * ft.force_rate;
    } else {
      ft.force_rate = 0.0f;
    }
    ft.prev_force_magnitude = ft.force_magnitude;
  }
  force_rate_initialized_ = true;
}

// ── Phase 2: Compute control ─────────────────────────────────────────────────


// ── Phase 2: Compute control (phase dispatch) ───────────────────────────────

void DemoWbcController::ComputeControl(const ControllerState& state, double dt) noexcept {
  UpdatePhase(state);

  // Keep MPC state fresh across all phases: HandlerMPCThread::Solve rejects
  // dim-mismatched snapshots, so non-TSID phases (kIdle/kApproach/kRetreat/
  // kRelease) would otherwise starve the solver and leave mpc_timing_log.csv
  // with only the header.
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    ExtractFullState(state);
    const uint64_t now_ns = static_cast<uint64_t>(state.iteration) * 2'000'000ULL;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
  }

  switch (phase_) {
    case WbcPhase::kIdle:
    case WbcPhase::kApproach:
    case WbcPhase::kRetreat:
    case WbcPhase::kRelease:
      // ComputePositionMode(dt);
      // break;

    case WbcPhase::kPreGrasp:
    case WbcPhase::kClosure:
    case WbcPhase::kHold:
      if (tsid_initialized_) {
        ComputeTSIDPosition(state, dt);
      } else {
        ComputePositionMode(dt);  // Fallback if TSID not available
      }
      break;

    case WbcPhase::kFallback:
      ComputeFallback();
      break;
  }
}

// ── FSM ──────────────────────────────────────────────────────────────────────


// ── Position-mode and TSID solvers ──────────────────────────────────────────

void DemoWbcController::ComputePositionMode(double dt) noexcept {
  // Robot arm trajectory
  robot_trajectory_time_ += dt;
  const auto rstate =
      robot_trajectory_.compute(std::min(robot_trajectory_time_, robot_trajectory_.duration()));
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_computed_.positions[i] = rstate.positions[i];
    robot_computed_.velocities[i] = rstate.velocities[i];
  }

  // Hand trajectory
  hand_trajectory_time_ += dt;
  const auto hstate =
      hand_trajectory_.compute(std::min(hand_trajectory_time_, hand_trajectory_.duration()));
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    hand_computed_.positions[i] = hstate.positions[i];
    hand_computed_.velocities[i] = hstate.velocities[i];
  }
}


void DemoWbcController::ComputeTSIDPosition(const ControllerState& state, double dt) noexcept {
  // 1. Extract full state (sensor values, every tick)
  ExtractFullState(state);

  // 2. Update Pinocchio cache (M, h, g, Jacobians)
  pinocchio_cache_.update(q_curr_full_, v_curr_full_, contact_state_);

  // 2b. MPC reference injection.
  //
  // Publish the current RT state to the MPC thread, then try to consume
  // the freshest MPC solution. If we have a valid interpolated reference,
  // it replaces the self-regularising hold target on the next line. If
  // not (MPC disabled / not yet publishing / too many stale cycles), we
  // fall through to the TSID self-hold behaviour.
  bool mpc_ref_valid = false;
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    const uint64_t now_ns = static_cast<uint64_t>(state.iteration) * 2'000'000ULL;  // 500 Hz tick
    rtc::mpc::InterpMeta meta;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
    mpc_ref_valid =
        mpc_manager_.ComputeReference(q_curr_full_, v_curr_full_, now_ns, mpc_q_ref_, mpc_v_ref_,
                                      mpc_a_ff_, mpc_lambda_ref_, mpc_u_fb_, meta);
  }

  // 3. Set posture reference (regularization toward MPC q_ref if valid,
  //    else toward current position for self-holding behaviour).
  if (mpc_ref_valid) {
    control_ref_.q_des = mpc_q_ref_;
    control_ref_.v_des = mpc_v_ref_;
    // TSID will combine a_ff with task PD correction. u_fb from Riccati
    // is additive acceleration feedback on the actuated joints only.
    control_ref_.a_des = mpc_a_ff_;
    const int n_fb =
        std::min(static_cast<int>(mpc_u_fb_.size()), static_cast<int>(control_ref_.a_des.size()));
    control_ref_.a_des.head(n_fb) += mpc_u_fb_.head(n_fb);
  } else {
    control_ref_.q_des = q_curr_full_;
    control_ref_.v_des.setZero();
    control_ref_.a_des.setZero();
  }

  // 4. Build ControlState
  ctrl_state_.q = q_curr_full_;
  ctrl_state_.v = v_curr_full_;
  ctrl_state_.timestamp_ns = state.iteration;

  // 5. TSID solve
  tsid_output_ =
      tsid_controller_.compute(ctrl_state_, control_ref_, pinocchio_cache_, contact_state_);

  // 6. QP failure handling
  if (!tsid_output_.qp_converged) {
    ++qp_fail_count_;
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ur5e_bringup::logging::kThrottleSlowMs,
                         "[wbc] QP failed (%d/%d), solve=%.0fus", qp_fail_count_,
                         max_qp_fail_before_fallback_, tsid_output_.solve_time_us);

    if (qp_fail_count_ >= max_qp_fail_before_fallback_) {
      phase_ = WbcPhase::kFallback;
      ComputeFallback();
      return;
    }
    // This tick: hold last valid output
    return;
  }
  qp_fail_count_ = 0;

  // 7. Semi-implicit Euler integration: a → v → q
  const auto& a = tsid_output_.a_opt;

  // v_next = v_curr + a · dt
  v_next_full_.noalias() = v_curr_full_ + a * dt;

  // Velocity clamp
  v_next_full_ = v_next_full_.cwiseMax(-v_limit_).cwiseMin(v_limit_);

  // q_next = q_curr + v_next · dt
  q_next_full_.noalias() = q_curr_full_ + v_next_full_ * dt;

  // Position clamp (safety net, should not trigger under normal TSID)
  q_next_full_ = q_next_full_.cwiseMax(q_min_clamped_).cwiseMin(q_max_clamped_);

  // 8. Map Pinocchio order → device order
  for (int i = 0; i < kArmDof; ++i) {
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[static_cast<std::size_t>(i)]);
    robot_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    robot_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }
  for (int i = 0; i < kHandDof; ++i) {
    const auto ext_i = static_cast<std::size_t>(kArmDof + i);
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[ext_i]);
    hand_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    hand_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }

  RCLCPP_INFO_THROTTLE(logger_, log_clock_, ur5e_bringup::logging::kThrottleSlowMs,
                       "[wbc] TSID solve=%.0fus qp_ok phase=%d", tsid_output_.solve_time_us,
                       static_cast<int>(phase_));
}


void DemoWbcController::ComputeFallback() noexcept {
  // Hold last computed positions (already in robot_computed_/hand_computed_)
  // Set velocities to zero
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_computed_.velocities[i] = 0.0;
  }
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    hand_computed_.velocities[i] = 0.0;
  }
}

// ── Phase 3: Write output ────────────────────────────────────────────────────


// ── Phase 3: Write output ────────────────────────────────────────────────────

ControllerOutput DemoWbcController::WriteOutput(const ControllerState& state) noexcept {
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;

  // ── Robot arm output ──────────────────────────────────────────────────
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = robot_computed_.positions[i];
    out0.target_positions[i] = robot_computed_.positions[i];
    out0.target_velocities[i] = robot_computed_.velocities[i];
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    out0.goal_positions[i] = device_targets_[0][i];
  }
  rtc::utils::ClampRange(out0.commands, nc0, std::span<const double>(device_position_lower_[0]),
                         std::span<const double>(device_position_upper_[0]), -6.2832, 6.2832);

  // ── Task-space logging (FK) ───────────────────────────────────────────
  if (arm_handle_) {
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

    // Task goal = TCP goal if valid, else mirror actual
    if (tcp_goal_valid_) {
      Eigen::Vector3d grpy = pinocchio::rpy::matrixToRpy(tcp_goal_.rotation());
      output.task_goal_positions[0] = tcp_goal_.translation().x();
      output.task_goal_positions[1] = tcp_goal_.translation().y();
      output.task_goal_positions[2] = tcp_goal_.translation().z();
      output.task_goal_positions[3] = grpy[0];
      output.task_goal_positions[4] = grpy[1];
      output.task_goal_positions[5] = grpy[2];
    } else {
      output.task_goal_positions = output.actual_task_positions;
    }
  }

  // ── Hand output ───────────────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;

    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.commands[i] = hand_computed_.positions[i];
      out1.target_positions[i] = hand_computed_.positions[i];
      out1.target_velocities[i] = hand_computed_.velocities[i];
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = device_targets_[1][i];
    }
    rtc::utils::ClampRange(out1.commands, nc1, std::span<const double>(device_position_lower_[1]),
                           std::span<const double>(device_position_upper_[1]), -6.2832, 6.2832);
  }

  // ── WBC state (per-fingertip aggregates + FSM phase) ─────────────────
  {
    auto& ws = output.wbc_state;
    ws.phase = static_cast<uint8_t>(phase_);
    ws.num_fingertips = num_active_fingertips_;
    int active_count = 0;
    float max_force = 0.0F;
    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const auto& ft = fingertip_data_[idx];
      const float mag = ft.force_magnitude;
      ws.force_magnitude[idx] = mag;
      ws.contact_flag[idx] = ft.contact_flag;
      // Use per-axis displacement magnitude (single scalar for the topic).
      const float d0 = ft.displacement[0];
      const float d1 = ft.displacement[1];
      const float d2 = ft.displacement[2];
      ws.displacement[idx] = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
      if (ft.contact_flag > 0.5F) {
        ++active_count;
      }
      if (mag > max_force) {
        max_force = mag;
      }
    }
    ws.num_active_contacts = active_count;
    ws.max_force = max_force;
    const auto g = gains_lock_.Load();
    ws.grasp_target_force = static_cast<float>(g.grasp_target_force);
    ws.min_fingertips_for_grasp = 2;  // WBC default; no YAML override yet
    ws.grasp_detected = (active_count >= ws.min_fingertips_for_grasp);
    ws.tsid_solver_ok = tsid_initialized_ && (qp_fail_count_ == 0);
    ws.qp_fail_count = qp_fail_count_;
    // tsid_solve_us: not measured in WBC yet — informational, leave 0.
  }

  output.valid = true;
  return output;
}

// ── Target management ────────────────────────────────────────────────────────


// ── E-STOP compute path ──────────────────────────────────────────────────────

ControllerOutput DemoWbcController::ComputeEstop(const ControllerState& state) noexcept {
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.valid = true;
  output.command_type = command_type_;

  // Arm: ramp toward safe position with per-joint velocity limit (matches
  // DemoJoint/DemoTask ComputeEstop pattern — instant jump risks hardware
  // damage on real UR5e at high E-STOP delta).
  auto& out0 = output.devices[0];
  out0.num_channels = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    out0.commands[i] =
        dev0.positions[i] + std::clamp(safe_position_[i] - dev0.positions[i], -lim, lim) * dt;
    out0.target_positions[i] = out0.commands[i];
  }

  // Hold current position (hand)
  if (state.num_devices > 1 && state.devices[1].valid) {
    auto& out1 = output.devices[1];
    out1.num_channels = state.devices[1].num_channels;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(state.devices[1].num_channels) && i < kMaxDeviceChannels;
         ++i) {
      out1.commands[i] = state.devices[1].positions[i];
      out1.target_positions[i] = state.devices[1].positions[i];
    }
  }

  return output;
}

// ── Utility ──────────────────────────────────────────────────────────────────


// ── Helpers (full state extraction, TCP error, MPC timing log) ──────────────

void DemoWbcController::ExtractFullState(const ControllerState& state) noexcept {
  if (!joint_reorder_valid_) {
    return;
  }

  const auto& dev0 = state.devices[0];
  // Arm joints: external [0..5] → Pinocchio order
  for (int i = 0; i < kArmDof; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
    q_curr_full_[pq] = dev0.positions[eidx];
    v_curr_full_[pv] = dev0.velocities[eidx];
  }

  // Hand joints: external [6..15] → Pinocchio order
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    for (int i = 0; i < kHandDof; ++i) {
      const auto eidx = static_cast<std::size_t>(kArmDof + i);
      const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
      const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
      q_curr_full_[pq] = dev1.positions[static_cast<std::size_t>(i)];
      v_curr_full_[pv] = dev1.velocities[static_cast<std::size_t>(i)];
    }
  }
}


double DemoWbcController::ComputeTcpError(const pinocchio::SE3& target) noexcept {
  if (!arm_handle_) {
    return 1e10;
  }
  const pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
  return (tcp.translation() - target.translation()).norm();
}

// ── Controller-owned topic lifecycle ──────────────────────────────────────

void DemoWbcController::LogMpcSolveTimingTick() noexcept {
  if (!mpc_manager_.Enabled()) {
    return;
  }

  // Drain every pending per-MPC-tick sample into the CSV. One row per
  // tick preserves full sampling-rate granularity; the file grows at
  // (MPC frequency × session_seconds) rows. Producer is the MPCThread
  // main loop (see rtc_mpc/thread/mpc_thread.cpp::OnTick).
  if (mpc_thread_) {
    mpc_thread_->TimingProducer().Drain(
        [this](const rtc::RtTickTimingSample& s) { mpc_timing_logger_.Log(s); });
  }

  // Periodic aggregate INFO so tmux-watchers see progress without
  // tail-ing the CSV. The window is computed by MPCSolutionManager over
  // its 256-sample ring (handler-side solve_duration_ns); 10 s INFO
  // cadence keeps the console readable across a 10-minute pilot session.
  const auto stats = mpc_manager_.GetSolveStats();
  static constexpr std::uint32_t kInfoEveryNTicks = 10;
  if (++mpc_timing_tick_ % kInfoEveryNTicks == 0) {
    RCLCPP_INFO(logger_,
                "[mpc_timing] count=%lu window=%u p50=%.2fms p99=%.2fms "
                "max=%.2fms",
                static_cast<unsigned long>(stats.count), static_cast<unsigned>(stats.window),
                static_cast<double>(stats.p50_ns) / 1e6, static_cast<double>(stats.p99_ns) / 1e6,
                static_cast<double>(stats.max_ns) / 1e6);
  }
}

}  // namespace ur5e_bringup
