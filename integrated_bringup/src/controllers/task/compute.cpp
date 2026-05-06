#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/utils/clamp_commands.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/spatial/log.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Phase 1: Read joint states + sensor data ────────────────────────────────
void DemoTaskController::ReadState(const ControllerState& state) noexcept {
  // Robot arm joint positions → FK + Jacobians via arm_handle_
  const auto& dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;
  std::span<const double> q_span(dev0.positions.data(), static_cast<std::size_t>(nc0));
  arm_handle_->ComputeJacobians(q_span);
  arm_handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  if (use_root_frame_) {
    const Eigen::Matrix3d R_root_T = arm_handle_->GetFrameRotation(root_frame_id_).transpose();
    J_full_.topRows(3) = R_root_T * J_full_.topRows(3);
    J_full_.bottomRows(3) = R_root_T * J_full_.bottomRows(3);
  }
  J_pos_.noalias() = J_full_.topRows(3);

  // Initialize target on first call
  pinocchio::SE3 tcp_pose = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    tcp_pose = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_pose);
  }
  const Eigen::Vector3d tcp = tcp_pose.translation();
  if (!target_initialized_) {
    tcp_target_pose_ = tcp_pose;
    tcp_target_ = {tcp[0], tcp[1], tcp[2]};
    target_initialized_ = true;
  }

  tcp_position_ = {tcp[0], tcp[1], tcp[2]};

  // Hand motor data: dev1.motor_positions[], motor_velocities[],
  // motor_efforts[] available via state.devices[1].motor_* (populated from
  // /hand/motor_states)

  // Hand sensor data (per-fingertip)
  num_active_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / kHandSensorValuesPerFingertipCapacity;
    num_active_fingertips_ = std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

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
      } else {
        ft.contact_flag = 0.0f;
        ft.force = {};
        ft.displacement = {};
      }
    }
  }
}

// ── Virtual TCP computation ─────────────────────────────────────────────────

void DemoTaskController::UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp,
                                          const Gains& gains) noexcept {
  vtcp_valid_ = false;
  if (!hand_handle_ || gains.vtcp.mode == VirtualTcpMode::kDisabled)
    return;

  // Build fingertip inputs from hand model FK
  for (std::size_t f = 0; f < kNumFingertips; ++f) {
    vtcp_inputs_[f].active = (fingertip_frame_ids_[f] != 0);
    if (!vtcp_inputs_[f].active)
      continue;
    auto ft_pose = hand_handle_->GetFramePlacement(fingertip_frame_ids_[f]);
    if (use_hand_root_frame_) {
      ft_pose = hand_handle_->GetFramePlacement(hand_root_frame_id_).actInv(ft_pose);
    }
    vtcp_inputs_[f].position_in_tcp = ft_pose.translation();
    // Force magnitude for weighted mode
    const auto& ft = fingertip_data_[f];
    vtcp_inputs_[f].force_magnitude =
        ft.valid
            ? static_cast<double>(std::sqrt(ft.force[0] * ft.force[0] + ft.force[1] * ft.force[1] +
                                            ft.force[2] * ft.force[2]))
            : 0.0;
  }

  const auto result = ComputeVirtualTcp(gains.vtcp, T_base_tcp, vtcp_inputs_);
  if (result.valid) {
    T_tcp_vtcp_ = result.T_tcp_vtcp;
    vtcp_pose_ = result.world_pose;
    vtcp_valid_ = true;
  }
}

// ── Phase 2: Compute control (CLIK/IK + trajectory + sensor logic) ──────────

void DemoTaskController::ComputeControl(const ControllerState& state, double dt) noexcept {
  // ── E-stop check ───────────────────────────────────────────────────────
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    return;
  }

  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();

  const auto& dev0 = state.devices[0];

  // ── Arm TCP pose ──────────────────────────────────────────────────────
  pinocchio::SE3 tcp_pose = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    tcp_pose = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_pose);
  }

  // ── Hand FK + Virtual TCP (must run before CLIK) ──────────────────────
  if (hand_handle_ && state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const auto hand_nq = static_cast<std::size_t>(hand_handle_->nq());
    for (std::size_t i = 0; i < hand_nq; ++i) {
      hand_q_[static_cast<Eigen::Index>(i)] = dev1.positions[i];
    }
    hand_handle_->ComputeForwardKinematics(std::span<const double>(hand_q_.data(), hand_nq));

    // Fingertip world poses (monitoring — always computed)
    for (std::size_t f = 0; f < kNumFingertips; ++f) {
      if (fingertip_frame_ids_[f] != 0) {
        auto T_hand_ft = hand_handle_->GetFramePlacement(fingertip_frame_ids_[f]);
        if (use_hand_root_frame_) {
          T_hand_ft = hand_handle_->GetFramePlacement(hand_root_frame_id_).actInv(T_hand_ft);
        }
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

  // ── Task-space trajectory ──────────────────────────────────────────────
  if (new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      pinocchio::SE3 start_pose = control_pose;
      pinocchio::SE3 goal_pose;

      if (gains.control_6dof) {
        goal_pose = tcp_target_pose_;
      } else {
        goal_pose = start_pose;
        goal_pose.translation() = Eigen::Vector3d(tcp_target_[0], tcp_target_[1], tcp_target_[2]);
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
      new_target_.store(false, std::memory_order_relaxed);
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
  pinocchio::SE3 T_current_desired = control_pose.actInv(traj_state_.pose);
  pinocchio::Motion twist_error = pinocchio::log6(T_current_desired);

  Eigen::Vector3d p_err;
  Eigen::Vector3d r_err;
  if (use_vtcp_frame) {
    // Jacobian is in vtcp frame → error must also be in vtcp frame (local)
    // log6 returns twist in local (control_pose = vtcp) frame — use directly
    p_err = twist_error.linear();
    r_err = twist_error.angular();
  } else {
    // Default: world-aligned frame (matches LOCAL_WORLD_ALIGNED Jacobian)
    // Use log6 for both linear and angular to preserve position-rotation
    // coupling (V⁻¹p)
    p_err = control_pose.rotation() * twist_error.linear();
    r_err = control_pose.rotation() * twist_error.angular();
  }

  pos_error_6d_.head<3>() = p_err;
  pos_error_6d_.tail<3>() = r_err;

  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = p_err[i];
  }

  // ── Damped pseudoinverse & Primary task ────────────────────────────────
  if (gains.control_6dof) {
    JJt_6d_.noalias() = J_full_ * J_full_.transpose();
    JJt_6d_.diagonal().array() += gains.damping * gains.damping;
    ldlt_6d_.compute(JJt_6d_);
    JJt_inv_6d_.noalias() = ldlt_6d_.solve(Eigen::Matrix<double, 6, 6>::Identity());
    Jpinv_6d_.noalias() = J_full_.transpose() * JJt_inv_6d_;

    Eigen::Matrix<double, 6, 1> kp_vec_6d;
    for (std::size_t i = 0; i < 3; ++i) {
      kp_vec_6d[static_cast<Eigen::Index>(i)] = gains.kp_translation[i];
      kp_vec_6d[static_cast<Eigen::Index>(i + 3)] = gains.kp_rotation[i];
    }

    Eigen::Matrix<double, 6, 1> task_vel_6d = kp_vec_6d.cwiseProduct(pos_error_6d_);
    if (use_vtcp_frame) {
      // Trajectory velocity is in trajectory-pose local frame.
      // Jacobian is in current vtcp frame → rotate trajectory local → vtcp
      // frame.
      const Eigen::Matrix3d R_vtcp_traj =
          control_pose.rotation().transpose() * traj_state_.pose.rotation();
      task_vel_6d.head<3>() += R_vtcp_traj * traj_state_.velocity.linear();
      task_vel_6d.tail<3>() += R_vtcp_traj * traj_state_.velocity.angular();
    } else {
      // Trajectory velocity is in trajectory-pose local frame.
      // Jacobian is LOCAL_WORLD_ALIGNED → rotate trajectory local → world.
      task_vel_6d.head<3>() += traj_state_.pose.rotation() * traj_state_.velocity.linear();
      task_vel_6d.tail<3>() += traj_state_.pose.rotation() * traj_state_.velocity.angular();
    }

    dq_.noalias() = Jpinv_6d_ * task_vel_6d;
  } else {
    JJt_.noalias() = J_pos_ * J_pos_.transpose();
    JJt_.diagonal().array() += gains.damping * gains.damping;
    ldlt_.compute(JJt_);
    JJt_inv_.noalias() = ldlt_.solve(Eigen::Matrix3d::Identity());
    Jpinv_.noalias() = J_pos_.transpose() * JJt_inv_;

    Eigen::Vector3d kp_vec(gains.kp_translation[0], gains.kp_translation[1],
                           gains.kp_translation[2]);
    // Feedforward: trajectory local → Jacobian frame (vtcp or world-aligned)
    Eigen::Vector3d ff_vel;
    if (use_vtcp_frame) {
      const Eigen::Matrix3d R_vtcp_traj =
          control_pose.rotation().transpose() * traj_state_.pose.rotation();
      ff_vel = R_vtcp_traj * traj_state_.velocity.linear();
    } else {
      ff_vel = traj_state_.pose.rotation() * traj_state_.velocity.linear();
    }
    Eigen::Vector3d task_vel = kp_vec.cwiseProduct(pos_error_) + ff_vel;
    dq_.noalias() = Jpinv_ * task_vel;
  }

  // ── Feedforward-only trajectory velocity (for logging) ────────────────
  if (gains.control_6dof) {
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
    traj_dq_.noalias() = Jpinv_6d_ * ff_vel_6d;
  } else {
    Eigen::Vector3d ff_lin;
    if (use_vtcp_frame) {
      const Eigen::Matrix3d R_vtcp_traj =
          control_pose.rotation().transpose() * traj_state_.pose.rotation();
      ff_lin = R_vtcp_traj * traj_state_.velocity.linear();
    } else {
      ff_lin = traj_state_.pose.rotation() * traj_state_.velocity.linear();
    }
    traj_dq_.noalias() = Jpinv_ * ff_lin;
  }

  // ── Null-space secondary task ──────────────────────────────────────────
  if (gains.enable_null_space && !gains.control_6dof) {
    N_.setIdentity();
    N_.noalias() -= Jpinv_ * J_pos_;

    for (Eigen::Index i = 0; i < arm_handle_->nv(); ++i) {
      null_err_[i] =
          null_target_[static_cast<std::size_t>(i)] - dev0.positions[static_cast<std::size_t>(i)];
    }
    null_dq_.noalias() = N_ * null_err_;
    null_dq_ *= gains.null_kp;
    dq_ += null_dq_;
  }

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];

    if (hand_new_target_.load(std::memory_order_acquire)) {
      trajectory::JointSpaceTrajectory<kHandMotorCount>::State start_state;
      trajectory::JointSpaceTrajectory<kHandMotorCount>::State goal_state;

      double max_dist = 0.0;
      for (std::size_t i = 0; i < kHandMotorCount; ++i) {
        start_state.positions[i] = dev1.positions[i];
        start_state.velocities[i] = 0.0;
        start_state.accelerations[i] = 0.0;

        goal_state.positions[i] = device_targets_[1][i];
        goal_state.velocities[i] = 0.0;
        goal_state.accelerations[i] = 0.0;

        max_dist = std::max(max_dist, std::abs(device_targets_[1][i] - dev1.positions[i]));
      }

      const double T_speed = max_dist / gains.hand_trajectory_speed;
      const double T_vel = (gains.hand_max_traj_velocity > 0.0)
                               ? (1.875 * max_dist / gains.hand_max_traj_velocity)
                               : 0.0;
      const double duration = std::max({0.01, T_speed, T_vel});
      hand_trajectory_.initialize(start_state, goal_state, duration);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }

    const auto hand_traj = hand_trajectory_.compute(hand_trajectory_time_);
    hand_trajectory_time_ += dt;

    for (std::size_t i = 0; i < kHandMotorCount; ++i) {
      hand_computed_.positions[i] = hand_traj.positions[i];
      hand_computed_.velocities[i] = hand_traj.velocities[i];
    }
  }

  // ── Grasp detection + ContactStopHand (500Hz) ────────────────────────
  {
    const float contact_thresh = gains.grasp_contact_threshold;
    const float force_thresh = gains.grasp_force_threshold;
    const int min_fingers = gains.grasp_min_fingertips;

    float max_force = 0.0f;
    int active_count = 0;

    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const auto& ft = fingertip_data_[idx];
      const float mag = std::sqrt(ft.force[0] * ft.force[0] + ft.force[1] * ft.force[1] +
                                  ft.force[2] * ft.force[2]);

      grasp_state_.force_magnitude[idx] = mag;
      grasp_state_.contact_flag[idx] = ft.contact_flag;
      grasp_state_.inference_valid[idx] = ft.valid;

      if (mag > max_force)
        max_force = mag;
      if (ft.valid && ft.contact_flag > contact_thresh && mag > force_thresh) {
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
      std::array<double, rtc::grasp::kNumGraspFingers> f_raw{};
      for (int f = 0; f < rtc::grasp::kNumGraspFingers; ++f) {
        f_raw[static_cast<std::size_t>(f)] =
            static_cast<double>(grasp_state_.force_magnitude[static_cast<std::size_t>(f)]);
      }

      const auto commands = grasp_controller_->Update(
          std::span<const double, rtc::grasp::kNumGraspFingers>(f_raw), dt);

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
        for (int f = 0; f < rtc::grasp::kNumGraspFingers; ++f) {
          for (int j = 0; j < rtc::grasp::kDoFPerFinger; ++j) {
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
      //   - thumb_cmc_fe: 각도 증가 = loosening → target > actual 이면 release
      //   - index_mcp_fe: 각도 감소 = loosening → target < actual 이면 release
      //   - middle_mcp_fe: 각도 감소 = loosening → target < actual 이면 release
      // 세 조건이 모두 성립할 때만 "release 의도" 로 판단.
      if (state.num_devices > 1 && state.devices[1].valid) {
        const auto& dev1 = state.devices[1];

        const double d_thumb =
            device_targets_[1][hand_idx_thumb_cmc_fe_] - dev1.positions[hand_idx_thumb_cmc_fe_];
        const double d_index =
            device_targets_[1][hand_idx_index_mcp_fe_] - dev1.positions[hand_idx_index_mcp_fe_];
        const double d_middle =
            device_targets_[1][hand_idx_middle_mcp_fe_] - dev1.positions[hand_idx_middle_mcp_fe_];

        const bool thumb_releasing = d_thumb > gains.contact_stop_release_eps;
        const bool index_releasing = d_index < -gains.contact_stop_release_eps;
        const bool middle_releasing = d_middle < -gains.contact_stop_release_eps;
        const bool release_phase = thumb_releasing && index_releasing && middle_releasing;

        if (release_phase) {
          RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                               "[contact_stop] SKIP (release) dthumb_fe=%+.3f "
                               "dindex_fe=%+.3f dmid_fe=%+.3f",
                               d_thumb, d_index, d_middle);
        } else if (active_count > 0 && max_force > force_thresh) {
          for (std::size_t i = 0; i < static_cast<std::size_t>(kHandMotorCount); ++i) {
            hand_computed_.positions[i] = dev1.positions[i];
            hand_computed_.velocities[i] = 0.0;
          }
          // Errors (target - actual) encode both the actual position and the
          // overshoot beyond target in a single number each, so 5 args are
          // enough to diagnose contact_stop engagement.
          const double err_thumb =
              device_targets_[1][hand_idx_thumb_cmc_fe_] - dev1.positions[hand_idx_thumb_cmc_fe_];
          const double err_index =
              device_targets_[1][hand_idx_index_mcp_fe_] - dev1.positions[hand_idx_index_mcp_fe_];
          const double err_middle =
              device_targets_[1][hand_idx_middle_mcp_fe_] - dev1.positions[hand_idx_middle_mcp_fe_];
          RCLCPP_INFO_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleFastMs,
                               "[contact_stop] FREEZE active=%d fmax=%.2fN "
                               "err=[%+.3f,%+.3f,%+.3f]",
                               active_count, static_cast<double>(max_force), err_thumb, err_index,
                               err_middle);
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

// ── Phase 3: Write output ────────────────────────────────────────────────────

ControllerOutput DemoTaskController::WriteOutput(const ControllerState& state, double dt) noexcept {
  // ── E-stop early return ────────────────────────────────────────────────
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  ControllerOutput output;
  output.num_devices = state.num_devices;

  // ── Robot arm output ───────────────────────────────────────────────────
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_velocities[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  // Clamp joint velocities by max velocity limits (symmetric ±max_vel)
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    out0.target_velocities[i] = std::clamp(out0.target_velocities[i], -lim, lim);
  }

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    desired_q_[static_cast<Eigen::Index>(i)] += out0.target_velocities[i] * dt;
    out0.commands[i] = desired_q_[static_cast<Eigen::Index>(i)];
    // Pure trajectory feedforward velocity (without Kp error / null-space)
    out0.trajectory_velocities[i] = traj_dq_[static_cast<Eigen::Index>(i)];
    out0.trajectory_positions[i] = desired_q_[static_cast<Eigen::Index>(i)];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out0.target_positions[i] = traj_state_.pose.translation()[static_cast<Eigen::Index>(i)];
  }
  for (std::size_t i = 3; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = null_target_[i];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out0.goal_positions[i] = tcp_target_[i];
  }
  for (std::size_t i = 3; i < static_cast<std::size_t>(nc0); ++i) {
    out0.goal_positions[i] = null_target_[i];
  }

  // ── Task-space logging ─────────────────────────────────────────────────
  pinocchio::SE3 tcp_current = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    tcp_current = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_current);
  }
  // Log virtual TCP pose when active, otherwise raw TCP
  pinocchio::SE3 log_pose = vtcp_valid_ ? vtcp_pose_ : tcp_current;
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(log_pose.rotation());
  output.actual_task_positions[0] = log_pose.translation().x();
  output.actual_task_positions[1] = log_pose.translation().y();
  output.actual_task_positions[2] = log_pose.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  // ── TF source poses for kRobotTransforms (Phase 3) ─────────────────────
  // Same layout as DemoJointController. Arm tip = raw FK (not vtcp);
  // virtual_tcp slot only valid when fingertip-based vtcp computed.
  {
    const Eigen::Vector3d& trans = tcp_current.translation();
    const Eigen::Quaterniond quat(tcp_current.rotation());
    output.arm_tip_pose.position = {trans.x(), trans.y(), trans.z()};
    output.arm_tip_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
    output.arm_tip_pose_valid = true;
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
    if (fingertip_frame_ids_[f] != 0) {
      const Eigen::Vector3d& trans = fingertip_positions_[f];
      const Eigen::Quaterniond quat(fingertip_rotations_[f]);
      output.fingertip_poses[f].position = {trans.x(), trans.y(), trans.z()};
      output.fingertip_poses[f].quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
      output.fingertip_pose_valid[f] = true;
    } else {
      output.fingertip_pose_valid[f] = false;
    }
  }

  // Task goal target from GUI
  output.task_goal_positions[0] = tcp_target_[0];
  output.task_goal_positions[1] = tcp_target_[1];
  output.task_goal_positions[2] = tcp_target_[2];
  if (gains_lock_.Load().control_6dof) {
    Eigen::Vector3d goal_rpy = pinocchio::rpy::matrixToRpy(tcp_target_pose_.rotation());
    output.task_goal_positions[3] = goal_rpy[0];
    output.task_goal_positions[4] = goal_rpy[1];
    output.task_goal_positions[5] = goal_rpy[2];
  } else {
    output.task_goal_positions[3] = rpy[0];
    output.task_goal_positions[4] = rpy[1];
    output.task_goal_positions[5] = rpy[2];
  }

  // ── Task-space trajectory reference ─────────────────────────────────
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

  // ── Hand output ────────────────────────────────────────────────────────
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

  // Populate force-PI grasp state if active
  if (grasp_controller_ && grasp_controller_type_ == "force_pi") {
    grasp_state_.grasp_phase = static_cast<uint8_t>(grasp_controller_->phase());
    grasp_state_.grasp_target_force = static_cast<float>(grasp_controller_->target_force());
    const auto& fs = grasp_controller_->finger_states();
    for (int f = 0; f < rtc::grasp::kNumGraspFingers; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      grasp_state_.finger_s[idx] = static_cast<float>(fs[idx].s);
      grasp_state_.finger_filtered_force[idx] = static_cast<float>(fs[idx].f_measured);
      grasp_state_.finger_force_error[idx] =
          static_cast<float>(fs[idx].f_desired - fs[idx].f_measured);
    }
  }

  output.command_type = command_type_;
  output.grasp_state = grasp_state_;
  output.tof_snapshot = tof_snapshot_;
  return output;
}

// ── E-STOP compute path ──────────────────────────────────────────────────────

ControllerOutput DemoTaskController::ComputeEstop(const ControllerState& state) noexcept {
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    out0.commands[i] =
        dev0.positions[i] + std::clamp(safe_position_[i] - dev0.positions[i], -lim, lim) *
                                ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
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
  if (arm_handle_) {
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
  for (std::size_t f = 0; f < output.fingertip_pose_valid.size(); ++f) {
    output.fingertip_pose_valid[f] = false;
  }

  return output;
}

}  // namespace integrated_bringup
