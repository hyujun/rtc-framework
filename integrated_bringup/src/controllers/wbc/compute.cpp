#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

#include <algorithm>
#include <cmath>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Phase 1: Read state ─────────────────────────────────────────────────────
void DemoWbcController::ReadState(const ControllerState& state) noexcept {
  // Read fingertip data from inference_data: slots 1..3 (fx/fy/fz) always,
  // slot 0 (native contact probability) only when has_native_contact_=true
  // (sensor A path; otherwise backend zero-fills the slot). Slots 4..6
  // (displacement) remain unconsumed — deformation guard is stubbed pending
  // sensor HW upgrade. ft.in_contact here decides *grasp* detection (uses
  // gains.grasp_force_threshold + grasp_contact_threshold via capability-
  // aware AND) — separate from phase.cpp's FSM threshold force_contact_threshold_
  // which gates the kClosure→kHold transition on raw force_magnitude directly.
  num_active_fingertips_ = 0;
  if (state.num_devices <= 1 || !state.devices[1].valid) {
    return;
  }

  const auto& dev1 = state.devices[1];
  // Prefer num_inference_groups (set by mujoco_native via ReadSensorState +
  // udp_hand via ReadSensorState). Fall back to deriving from sensor channel
  // stride for the older udp_hand-style mocks that only fill the sensor lane.
  const int num_groups =
      (dev1.num_inference_groups > 0)
          ? dev1.num_inference_groups
          : (kHandSensorValuesPerFingertipCapacity > 0
                 ? static_cast<int>(dev1.num_sensor_channels /
                                    static_cast<int>(kHandSensorValuesPerFingertipCapacity))
                 : 0);
  num_active_fingertips_ = std::min(num_groups, static_cast<int>(rtc::kMaxSensorGroups));

  const double inv_dt = (state.dt > 0.0) ? (1.0 / state.dt) : 500.0;
  const auto gains_now = gains_lock_.Load();
  const float force_threshold = gains_now.grasp_force_threshold;
  const float contact_threshold = gains_now.grasp_contact_threshold;

  for (int f = 0; f < num_active_fingertips_; ++f) {
    auto& ft = fingertip_data_[static_cast<std::size_t>(f)];

    ft.valid = dev1.inference_enable[static_cast<std::size_t>(f)];
    if (ft.valid) {
      const int ft_base = f * kHandInferenceValuesPerFingertipCapacity;
      for (int j = 0; j < 3; ++j) {
        ft.force[static_cast<std::size_t>(j)] =
            dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
      }
      const bool native_path =
          has_native_contact_ && dev1.inference_enable[static_cast<std::size_t>(f)];
      ft.contact_flag =
          native_path ? dev1.inference_data[static_cast<std::size_t>(ft_base)] : 0.0F;
      const float fx = ft.force[0];
      const float fy = ft.force[1];
      const float fz = ft.force[2];
      ft.force_magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
      const bool force_active = (ft.force_magnitude > force_threshold);
      ft.in_contact =
          native_path ? (ft.contact_flag > contact_threshold && force_active) : force_active;
    } else {
      ft.force = {};
      ft.contact_flag = 0.0F;
      ft.force_magnitude = 0.0f;
      ft.in_contact = false;
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
  // dim-mismatched snapshots, so a starved solver would leave
  // mpc_timing_log.csv with only the header. Now redundant with the per-tick
  // WriteState inside ComputeTSIDPosition for the 5 TSID phases; kept for
  // kRelease (which routes through ComputeReleaseMode) and kFallback. See
  // wbc-tsid-uniform-release-preempt plan finding #15 — perf cleanup task.
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    ExtractFullState(state);
    const uint64_t now_ns = static_cast<uint64_t>(state.iteration) * 2'000'000ULL;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
  }

  switch (phase_) {
    case WbcPhase::kIdle:
    case WbcPhase::kApproach:
    case WbcPhase::kPreGrasp:
    case WbcPhase::kClosure:
    case WbcPhase::kHold:
      if (tsid_initialized_) {
        ComputeTSIDPosition(state, dt);
      } else {
        ComputePositionMode(dt);  // Fallback if TSID not available
      }
      break;

    case WbcPhase::kRelease:
      ComputeReleaseMode(state, dt);
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
  for (int i = 0; i < arm_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    robot_computed_.positions[idx] = rstate.positions[idx];
    robot_computed_.velocities[idx] = rstate.velocities[idx];
  }

  // Hand trajectory
  hand_trajectory_time_ += dt;
  const auto hstate =
      hand_trajectory_.compute(std::min(hand_trajectory_time_, hand_trajectory_.duration()));
  for (int i = 0; i < hand_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    hand_computed_.positions[idx] = hstate.positions[idx];
    hand_computed_.velocities[idx] = hstate.velocities[idx];
  }
}

void DemoWbcController::ComputeTSIDPosition(const ControllerState& state, double dt) noexcept {
  // 1. Extract full state (sensor values, every tick)
  ExtractFullState(state);

  // Stage A-5b: progress per-contact activation ramp by dt. ContactState
  // auto-flips the legacy `active : bool` once s_i crosses kActivationDeadband
  // so downstream skip paths (active_contact_vars, task/constraint early
  // outs) stay coherent.
  contact_state_.UpdateActivation(dt);
  contact_state_.RecomputeActive(contact_mgr_config_);

  // 2. Update Pinocchio cache (M, h, g, Jacobians)
  pinocchio_cache_.Update(q_curr_full_, v_curr_full_, contact_state_);

  // 2a. Stage B-5: populate the once-per-tick grasp cache shared across the
  // three object-level tasks (ObjectWrenchTask / InternalForceTask /
  // ObjectSE3Task). Order is invariant per the plan handoff:
  //   cache.Update → UpdateActivation → RecomputeActive →
  //   ActiveLambdaDim → ComputeGraspMatrix(G_workspace_) → grasp_cache_.Compute
  // Tasks must never re-Compute on their own — they only read GPinv/GTPinv/
  // ProjN/Rank from grasp_cache_. n_active=0 (idle/pre_grasp) leaves the
  // cache empty (Rank()=0); object-level tasks then report ResidualDim=0.
  const int n_lambda_active = contact_mgr_.ActiveLambdaDim(contact_state_);
  if (n_lambda_active > 0) {
    auto G_view = grasp_G_workspace_.leftCols(n_lambda_active);
    contact_mgr_.ComputeGraspMatrix(pinocchio_cache_, contact_state_, object_frame_, G_view);
    grasp_cache_.Compute(G_view, n_lambda_active);
  } else {
    grasp_cache_.Compute(grasp_G_workspace_.leftCols(0), 0);
  }

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

  // 3b. Stage A-3 — per-tick ForceReferenceUpdater (kClosure/kHold only).
  // Closes the loop on per-fingertip normal force: λ_des_i_n = PI(f_des - |f_meas_i|).
  // Stage A-4: push along the contact normal (world frame) so the reference
  // and FrictionConeConstraint share the same R_c basis. Default normal=+Z
  // is byte-identical to the original Stage A-3 +Z push.
  if (phase_ == WbcPhase::kClosure || phase_ == WbcPhase::kHold) {
    auto* force_task =
        tsid_initialized_ ? tsid_controller_.Formulation().GetTask("force") : nullptr;
    const int n_lambda = contact_mgr_config_.max_contact_vars;
    if (force_task && n_lambda > 0 && force_lambda_des_.size() == n_lambda) {
      const auto gains_now = gains_lock_.Load();
      const double f_des = gains_now.grasp_target_force;
      force_lambda_des_.setZero();
      int offset = 0;
      int contact_idx = 0;
      for (const auto& c : contact_mgr_config_.contacts) {
        const int cdim = c.contact_dim;
        if (offset + cdim > n_lambda) {
          break;
        }
        // Map contact_idx → fingertip sensor slot 1:1 (configured ordering).
        const bool valid = (contact_idx < num_active_fingertips_) &&
                           fingertip_data_[static_cast<std::size_t>(contact_idx)].valid;
        const double f_meas =
            valid ? static_cast<double>(
                        fingertip_data_[static_cast<std::size_t>(contact_idx)].force_magnitude)
                  : 0.0;
        const double lambda_n = force_ref_updater_.Update(contact_idx, valid, f_des, f_meas, dt);
        // Stage A-4: 3D push along world-frame contact normal n.
        // λ_world_force = lambda_n · n. Moment block (surface) stays zero.
        if (cdim >= 3 && contact_idx < static_cast<int>(contact_state_.contacts.size())) {
          const Eigen::Vector3d& n =
              contact_state_.contacts[static_cast<std::size_t>(contact_idx)].normal;
          force_lambda_des_[offset + 0] = lambda_n * n.x();
          force_lambda_des_[offset + 1] = lambda_n * n.y();
          force_lambda_des_[offset + 2] = lambda_n * n.z();
        }
        offset += cdim;
        ++contact_idx;
      }
      static_cast<rtc::tsid::ForceTask*>(force_task)->SetForceReferences(force_lambda_des_);
    }
  }

  // 3c. TCP SE3 trajectory tick (MPC-disabled mode only). Gated by both
  // tcp_trajectory_active_ (init flag from OnPhaseEnter) AND the per-phase
  // SE3-task-active cache so we don't push references in phases where
  // se3_tcp is YAML-deactivated. Per-tick push of (pose, v, a) replaces the
  // single step-on-entry SetSe3Reference path so the SE3Task sees a smooth
  // ramp instead of a large initial error → aggressive PD acceleration.
  if (tcp_trajectory_active_ &&
      se3_task_active_in_phase_[static_cast<std::size_t>(phase_)]) {
    tcp_trajectory_time_ += dt;
    if (has_pending_tcp_segment_ &&
        tcp_trajectory_time_ >= tcp_trajectory_.duration()) {
      const pinocchio::SE3 mid =
          tcp_trajectory_.compute(tcp_trajectory_.duration()).pose;
      tcp_trajectory_.initialize(mid, pinocchio::Motion::Zero(), pending_tcp_goal_,
                                 pinocchio::Motion::Zero(), pending_tcp_duration_);
      tcp_trajectory_time_ = 0.0;
      has_pending_tcp_segment_ = false;
    }
    tcp_traj_state_ = tcp_trajectory_.compute(
        std::min(tcp_trajectory_time_, tcp_trajectory_.duration()), dt);
    if (auto* se3 = tsid_controller_.Formulation().GetTask("se3_tcp"); se3) {
      static_cast<rtc::tsid::SE3Task*>(se3)->SetSe3Reference(
          tcp_traj_state_.pose,
          tcp_traj_state_.velocity.toVector(),
          tcp_traj_state_.acceleration.toVector());
    }
  }

  // 4. Build ControlState
  ctrl_state_.q = q_curr_full_;
  ctrl_state_.v = v_curr_full_;
  ctrl_state_.timestamp_ns = state.iteration;

  // 5. TSID solve
  tsid_output_ =
      tsid_controller_.Compute(ctrl_state_, control_ref_, pinocchio_cache_, contact_state_);

  // 6. QP failure handling
  if (!tsid_output_.qp_converged) {
    ++qp_fail_count_;
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
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
  for (int i = 0; i < arm_dof_; ++i) {
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[static_cast<std::size_t>(i)]);
    robot_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    robot_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }
  for (int i = 0; i < hand_dof_; ++i) {
    const auto ext_i = static_cast<std::size_t>(arm_dof_ + i);
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[ext_i]);
    hand_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    hand_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }

  // Stage B-5: WBC diagnostic (RT-safe throttled INFO). WBCDiagnostic struct
  // (separate SeqLock publisher) is deferred to a future stage — this single
  // throttled line is the interim observability surface. Format uses only
  // %d / %.2f / %.0f (no fmt::format / to_string / string concat — RT-3
  // throttle exception applies).
  RCLCPP_INFO_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                       "[wbc] solve=%.0fus phase=%d n_act=%d rank_G=%d", tsid_output_.solve_time_us,
                       static_cast<int>(phase_), n_lambda_active, grasp_cache_.Rank());
}

void DemoWbcController::ComputeReleaseMode(const ControllerState& state, double dt) noexcept {
  // Stage 0: TSID holds the arm SE3 at current TCP while ContactState ramps
  // activation toward 0 (set in OnPhaseEnter(kRelease)). Posture preset
  // damps hand at its current pose during this brief window.
  // Stage 1: lazily initialise the finger-open joint trajectory once the
  // contact ramp window has elapsed, then overlay it onto hand_computed_
  // after TSID's tick mapping. The arm continues on TSID SE3 hold.
  release_elapsed_s_ += dt;

  if (tsid_initialized_) {
    ComputeTSIDPosition(state, dt);
  } else {
    ComputePositionMode(dt);
  }

  if (release_stage_ == 0 && release_elapsed_s_ >= release_ramp_sec_) {
    if (state.num_devices > 1 && state.devices[1].valid) {
      const auto gains = gains_lock_.Load();
      trajectory::JointSpaceTrajectory<kMaxHandDof>::State hstart{};
      trajectory::JointSpaceTrajectory<kMaxHandDof>::State hgoal{};
      double hmax = 0.0;
      for (int i = 0; i < hand_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        hstart.positions[idx] = hand_computed_.positions[idx];
        hgoal.positions[idx] = 0.0;
        const double hd = std::abs(hstart.positions[idx]);
        if (hd > hmax) {
          hmax = hd;
        }
      }
      const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
      hand_trajectory_.initialize(hstart, hgoal, hdur);
      hand_trajectory_time_ = 0.0;
    } else {
      // No hand device → ramp window is the entire release sequence.
      release_done_ = true;
    }
    release_stage_ = 1;
  }

  if (release_stage_ == 1 && state.num_devices > 1 && state.devices[1].valid) {
    hand_trajectory_time_ += dt;
    const auto hstate = hand_trajectory_.compute(
        std::min(hand_trajectory_time_, hand_trajectory_.duration()));
    for (int i = 0; i < hand_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      hand_computed_.positions[idx] = hstate.positions[idx];
      hand_computed_.velocities[idx] = hstate.velocities[idx];
    }
    if (hand_trajectory_time_ >= hand_trajectory_.duration()) {
      release_done_ = true;
    }
  }
}

void DemoWbcController::ComputeFallback() noexcept {
  // Hold last computed positions (already in robot_computed_/hand_computed_)
  // Set velocities to zero
  for (int i = 0; i < arm_dof_; ++i) {
    robot_computed_.velocities[static_cast<std::size_t>(i)] = 0.0;
  }
  for (int i = 0; i < hand_dof_; ++i) {
    hand_computed_.velocities[static_cast<std::size_t>(i)] = 0.0;
  }
}

// ── Phase 3a: Write joint command (wire-bound only) ──────────────────────────

ControllerOutput DemoWbcController::WriteJointCommand(const ControllerState& state) noexcept {
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

void DemoWbcController::FillLogOutput(const ControllerState& state,
                                      ControllerOutput& output) noexcept {
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    out0.goal_positions[i] = current_target_slot_.targets[0][i];
  }

  // FK + actual_task_positions + task_goal_positions (log POD reads both).
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

  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }

  // WBC state aggregates (per-fingertip + FSM phase). Staging buffer feeds
  // both the SeqLock publish path and GetWbcStateForTesting().
  {
    auto& ws = wbc_state_;
    ws.phase = static_cast<uint8_t>(phase_);
    ws.num_fingertips = num_active_fingertips_;
    int active_count = 0;
    float max_force = 0.0F;
    const auto g = gains_lock_.Load();
    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const auto& ft = fingertip_data_[idx];
      const float mag = ft.force_magnitude;
      ws.force_magnitude[idx] = mag;
      // contact_flag: native sigmoid probability if backend provides it
      // (sensor A path), else derived binary 1/0 from ft.in_contact.
      // displacement: native (slots 4..6) if backend provides it (sensor A);
      // else 0 — controller-side deformation derive is stubbed.
      ws.contact_flag[idx] =
          has_native_contact_ ? ft.contact_flag : (ft.in_contact ? 1.0F : 0.0F);
      ws.displacement[idx] = 0.0F;
      if (ft.in_contact) {
        ++active_count;
      }
      if (mag > max_force) {
        max_force = mag;
      }
    }
    ws.num_active_contacts = active_count;
    ws.max_force = max_force;
    ws.grasp_target_force = static_cast<float>(g.grasp_target_force);
    ws.min_fingertips_for_grasp = g.grasp_min_fingertips;
    ws.grasp_detected = (active_count >= ws.min_fingertips_for_grasp);
    ws.tsid_solver_ok = tsid_initialized_ && (qp_fail_count_ == 0);
    ws.qp_fail_count = qp_fail_count_;
  }
  // SeqLock store = two atomic stores + memcpy (wait-free, RT-safe).
  // Read by PublishNonRtSnapshot.
  wbc_state_lock_.Store(wbc_state_);
}

// ── Phase 3c: Fill publish output ────────────────────────────────────────────
//
// WBC's publish surface is narrower than joint/task: only arm_tip_pose for
// kRobotTransforms (no fingertip / virtual_tcp poses — WBC compute doesn't
// produce them). target_*/trajectory_*/goal_positions are also mirrored
// here so the publish snapshot reflects the canonical RT-tick values.

void DemoWbcController::FillPublishOutput(const ControllerState& state,
                                          ControllerOutput& output) noexcept {
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

  if (arm_handle_) {
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

    // TF source: arm tip only. Fingertip / virtual_tcp frames are not
    // produced by WBC compute — those slots stay invalid.
    {
      const Eigen::Vector3d& trans = tcp.translation();
      const Eigen::Quaterniond quat(tcp.rotation());
      output.arm_tip_pose.position = {trans.x(), trans.y(), trans.z()};
      output.arm_tip_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
      output.arm_tip_pose_valid = true;
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
  for (int i = 0; i < arm_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const double lim = (idx < device_max_velocity_[0].size()) ? device_max_velocity_[0][idx] : 2.0;
    out0.commands[idx] =
        dev0.positions[idx] + std::clamp(safe_position_[idx] - dev0.positions[idx], -lim, lim) * dt;
    out0.target_positions[idx] = out0.commands[idx];
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
  // Arm joints: external [0..arm_dof_-1] → Pinocchio order
  for (int i = 0; i < arm_dof_; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
    q_curr_full_[pq] = dev0.positions[eidx];
    v_curr_full_[pv] = dev0.velocities[eidx];
  }

  // Hand joints: external [arm_dof_..full_dof_-1] → Pinocchio order
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    for (int i = 0; i < hand_dof_; ++i) {
      const auto eidx = static_cast<std::size_t>(arm_dof_ + i);
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

}  // namespace integrated_bringup
