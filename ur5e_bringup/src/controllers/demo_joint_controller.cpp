#include "ur5e_bringup/controllers/demo_joint_controller.hpp"

#include "ur5e_bringup/controllers/demo_shared_config.hpp"

#include <algorithm> // std::copy, std::clamp
#include <cmath>     // std::sqrt

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace ur5e_bringup {

DemoJointController::DemoJointController(std::string_view urdf_path)
    : DemoJointController(urdf_path, Gains{}) {}

DemoJointController::DemoJointController(std::string_view urdf_path,
                                         Gains gains)
    : gains_lock_(gains), urdf_path_(urdf_path) {
  // Model is built in LoadConfig() (bridge YAML driven) or InitArmModel().
  // Constructor only stores urdf_path for later use.
}

void DemoJointController::InitArmModel(
    const rtc_urdf_bridge::ModelConfig &config) {
  namespace rub = rtc_urdf_bridge;
  builder_ = std::make_unique<rub::PinocchioModelBuilder>(config);

  // Resolve sub-model name: match primary device name, fallback to "arm"
  const auto primary = GetPrimaryDeviceName();
  std::string model_name = "arm";
  for (const auto &sm : config.sub_models) {
    if (sm.name == primary) {
      model_name = primary;
      break;
    }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(
      builder_->GetReducedModel(model_name));
}

// ── Hand tree-model initialization ──────────────────────────────────────────
void DemoJointController::InitHandModel(
    const rtc_urdf_bridge::ModelConfig & /*config*/) {
  namespace rub = rtc_urdf_bridge;
  // "hand" tree_model: name matches device name
  hand_handle_ =
      std::make_unique<rub::RtModelHandle>(builder_->GetTreeModel("hand"));

  // Set joint reorder mapping: YAML joint_state_names → Pinocchio model order
  if (auto *hand_cfg = GetDeviceNameConfig("hand"); hand_cfg) {
    hand_handle_->SetJointOrder(hand_cfg->joint_state_names);
  }

  // Resolve fingertip frame IDs from tree_model tip_links
  const auto *sys_cfg = GetSystemModelConfig();
  if (sys_cfg) {
    for (const auto &tm : sys_cfg->tree_models) {
      if (tm.name == "hand") {
        if (!tm.root_link.empty()) {
          hand_root_frame_id_ = hand_handle_->GetFrameId(tm.root_link);
          if (hand_root_frame_id_ != 0) {
            use_hand_root_frame_ = true;
          }
        }
        for (std::size_t i = 0;
             i < std::min(tm.tip_links.size(), kNumFingertips); ++i) {
          fingertip_frame_ids_[i] = hand_handle_->GetFrameId(tm.tip_links[i]);
        }
        break;
      }
    }
  }

  // Pre-allocate hand joint vector
  hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());

  // Initialize position/rotation buffers
  for (auto &p : fingertip_positions_)
    p = Eigen::Vector3d::Zero();
  for (auto &r : fingertip_rotations_)
    r = Eigen::Matrix3d::Identity();
}

void DemoJointController::OnDeviceConfigsSet() {
  if (auto *cfg = GetDeviceNameConfig("ur5e"); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) { // 0 = universe (not found)
        tip_frame_id_ = fid;
      }
    }
    if (cfg->urdf && !cfg->urdf->root_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->root_link);
      if (fid != 0) {
        root_frame_id_ = fid;
        use_root_frame_ = true;
      }
    }
    if (cfg->joint_limits) {
      if (!cfg->joint_limits->max_velocity.empty()) {
        device_max_velocity_[0] = cfg->joint_limits->max_velocity;
      }
      if (!cfg->joint_limits->position_lower.empty()) {
        device_position_lower_[0] = cfg->joint_limits->position_lower;
      }
      if (!cfg->joint_limits->position_upper.empty()) {
        device_position_upper_[0] = cfg->joint_limits->position_upper;
      }
    }
  }
  if (auto *cfg = GetDeviceNameConfig("hand"); cfg && cfg->joint_limits) {
    if (!cfg->joint_limits->max_velocity.empty()) {
      device_max_velocity_[1] = cfg->joint_limits->max_velocity;
    }
    if (!cfg->joint_limits->position_lower.empty()) {
      device_position_lower_[1] = cfg->joint_limits->position_lower;
    }
    if (!cfg->joint_limits->position_upper.empty()) {
      device_position_upper_[1] = cfg->joint_limits->position_upper;
    }
  }
  // Fallback defaults
  for (auto &v : device_max_velocity_) {
    if (v.empty())
      v.assign(kMaxDeviceChannels, 2.0);
  }
  for (auto &v : device_position_lower_) {
    if (v.empty())
      v.assign(kMaxDeviceChannels, -6.2832);
  }
  for (auto &v : device_position_upper_) {
    if (v.empty())
      v.assign(kMaxDeviceChannels, 6.2832);
  }
}

ControllerOutput
DemoJointController::Compute(const ControllerState &state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ReadState(state);
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }
  ComputeControl(state, dt);
  return WriteOutput(state, dt);
}

// ── Phase 1: Read joint states + sensor data ────────────────────────────────

void DemoJointController::ReadState(const ControllerState &state) noexcept {
  // Robot arm joint positions (used for FK logging in WriteOutput)
  const auto &dev0 = state.devices[0];
  (void)dev0; // positions accessed directly via span in WriteOutput

  // Hand motor data: dev1.motor_positions[], motor_velocities[],
  // motor_efforts[] available via state.devices[1].motor_* (populated from
  // /hand/motor_states)

  // Hand sensor data (per-fingertip)
  num_active_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto &dev1 = state.devices[1];
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / rtc::kSensorValuesPerFingertip;
    num_active_fingertips_ =
        std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

    for (int f = 0; f < num_active_fingertips_; ++f) {
      auto &ft = fingertip_data_[static_cast<std::size_t>(f)];
      const int base = f * rtc::kSensorValuesPerFingertip;

      for (std::size_t j = 0; j < rtc::kBarometerCount; ++j) {
        ft.baro[j] = dev1.sensor_data[static_cast<std::size_t>(base) + j];
      }
      for (std::size_t j = 0; j < 3; ++j) {
        ft.tof[j] = dev1.sensor_data[static_cast<std::size_t>(base) +
                                     rtc::kBarometerCount + j];
      }

      ft.valid = dev1.inference_enable[static_cast<std::size_t>(f)];
      if (ft.valid) {
        const int ft_base = f * rtc::kFTValuesPerFingertip;
        ft.contact_flag =
            dev1.inference_data[static_cast<std::size_t>(ft_base)];
        for (int j = 0; j < 3; ++j) {
          ft.force[static_cast<std::size_t>(j)] =
              dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
          ft.displacement[static_cast<std::size_t>(j)] =
              dev1.inference_data[static_cast<std::size_t>(ft_base + 4 + j)];
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

void DemoJointController::UpdateVirtualTcp(const pinocchio::SE3 &T_base_tcp,
                                           const Gains &gains) noexcept {
  vtcp_valid_ = false;
  if (!hand_handle_ || gains.vtcp.mode == VirtualTcpMode::kDisabled)
    return;

  // Build fingertip inputs from hand model FK (already computed in
  // ComputeControl)
  for (std::size_t f = 0; f < kNumFingertips; ++f) {
    vtcp_inputs_[f].active = (fingertip_frame_ids_[f] != 0);
    if (!vtcp_inputs_[f].active)
      continue;
    auto ft_pose = hand_handle_->GetFramePlacement(fingertip_frame_ids_[f]);
    if (use_hand_root_frame_) {
      ft_pose =
          hand_handle_->GetFramePlacement(hand_root_frame_id_).actInv(ft_pose);
    }
    vtcp_inputs_[f].position_in_tcp = ft_pose.translation();
    // Force magnitude for weighted mode
    const auto &ft = fingertip_data_[f];
    vtcp_inputs_[f].force_magnitude =
        ft.valid ? static_cast<double>(std::sqrt(ft.force[0] * ft.force[0] +
                                                 ft.force[1] * ft.force[1] +
                                                 ft.force[2] * ft.force[2]))
                 : 0.0;
  }

  const auto result = ComputeVirtualTcp(gains.vtcp, T_base_tcp, vtcp_inputs_);
  if (result.valid) {
    vtcp_pose_ = result.world_pose;
    vtcp_valid_ = true;
  }
}

// ── Phase 2: Compute control (trajectory + sensor-based logic) ──────────────

void DemoJointController::ComputeControl(const ControllerState &state,
                                         double dt) noexcept {
  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  const auto &dev0 = state.devices[0];

  // ── Robot arm trajectory ────────────────────────────────────────────────
  if (robot_new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start_state;
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal_state;

      double max_dist = 0.0;
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        start_state.positions[i] = dev0.positions[i];
        start_state.velocities[i] = 0.0;
        start_state.accelerations[i] = 0.0;

        goal_state.positions[i] = device_targets_[0][i];
        goal_state.velocities[i] = 0.0;
        goal_state.accelerations[i] = 0.0;

        max_dist = std::max(
            max_dist, std::abs(device_targets_[0][i] - dev0.positions[i]));
      }

      // Duration from trajectory_speed, then enforce max trajectory velocity.
      // Quintic rest-to-rest peak velocity = (15/8) * max_dist / T.
      const double T_speed = max_dist / gains.robot_trajectory_speed;
      const double T_vel =
          (gains.robot_max_traj_velocity > 0.0)
              ? (1.875 * max_dist / gains.robot_max_traj_velocity)
              : 0.0;
      const double duration = std::max({0.01, T_speed, T_vel});
      robot_trajectory_.initialize(start_state, goal_state, duration);
      robot_trajectory_time_ = 0.0;
      robot_new_target_.store(false, std::memory_order_relaxed);
    }
  }

  const auto robot_traj = robot_trajectory_.compute(robot_trajectory_time_);
  robot_trajectory_time_ += dt;

  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_computed_.positions[i] = robot_traj.positions[i];
    robot_computed_.velocities[i] = robot_traj.velocities[i];
  }

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto &dev1 = state.devices[1];

    if (hand_new_target_.load(std::memory_order_acquire)) {
      std::unique_lock lock(target_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State start_state;
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State goal_state;

        double max_dist = 0.0;
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          start_state.positions[i] = dev1.positions[i];
          start_state.velocities[i] = 0.0;
          start_state.accelerations[i] = 0.0;

          goal_state.positions[i] = device_targets_[1][i];
          goal_state.velocities[i] = 0.0;
          goal_state.accelerations[i] = 0.0;

          max_dist = std::max(
              max_dist, std::abs(device_targets_[1][i] - dev1.positions[i]));
        }

        const double T_speed = max_dist / gains.hand_trajectory_speed;
        const double T_vel =
            (gains.hand_max_traj_velocity > 0.0)
                ? (1.875 * max_dist / gains.hand_max_traj_velocity)
                : 0.0;
        const double duration = std::max({0.01, T_speed, T_vel});
        hand_trajectory_.initialize(start_state, goal_state, duration);
        hand_trajectory_time_ = 0.0;
        hand_new_target_.store(false, std::memory_order_relaxed);
      }
    }

    const auto hand_traj = hand_trajectory_.compute(hand_trajectory_time_);
    hand_trajectory_time_ += dt;

    for (std::size_t i = 0; i < kNumHandMotors; ++i) {
      hand_computed_.positions[i] = hand_traj.positions[i];
      hand_computed_.velocities[i] = hand_traj.velocities[i];
    }
  }

  // ── Hand fingertip FK (tree model) — base-to-fingertip ──────────────
  if (hand_handle_ && state.num_devices > 1 && state.devices[1].valid) {
    const auto &dev1 = state.devices[1];
    const auto hand_nq = static_cast<std::size_t>(hand_handle_->nq());
    for (std::size_t i = 0; i < hand_nq; ++i) {
      hand_q_[static_cast<Eigen::Index>(i)] = dev1.positions[i];
    }
    hand_handle_->ComputeForwardKinematics(
        std::span<const double>(hand_q_.data(), hand_nq));

    // Arm FK: base -> tcp (tool0)
    const int nc0 = dev0.num_channels;
    std::span<const double> q_span(dev0.positions.data(),
                                   static_cast<std::size_t>(nc0));
    arm_handle_->ComputeForwardKinematics(q_span);
    pinocchio::SE3 T_base_tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      T_base_tcp =
          arm_handle_->GetFramePlacement(root_frame_id_).actInv(T_base_tcp);
    }

    // Chain: T_base_fingertip = T_base_tcp * T_hand_fingertip
    for (std::size_t f = 0; f < kNumFingertips; ++f) {
      if (fingertip_frame_ids_[f] != 0) {
        auto T_hand_ft =
            hand_handle_->GetFramePlacement(fingertip_frame_ids_[f]);
        if (use_hand_root_frame_) {
          T_hand_ft = hand_handle_->GetFramePlacement(hand_root_frame_id_)
                          .actInv(T_hand_ft);
        }
        const pinocchio::SE3 T_base_ft = T_base_tcp.act(T_hand_ft);
        fingertip_positions_[f] = T_base_ft.translation();
        fingertip_rotations_[f] = T_base_ft.rotation();
      }
    }

    // Virtual TCP computation (uses hand FK data computed above)
    UpdateVirtualTcp(T_base_tcp, gains);
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
      const auto &ft = fingertip_data_[idx];
      const float mag =
          std::sqrt(ft.force[0] * ft.force[0] + ft.force[1] * ft.force[1] +
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
    RCLCPP_INFO_THROTTLE(
        logger_, log_clock_, ::ur5e_bringup::logging::kThrottleSlowMs,
        "[grasp] type=%s active=%d/%d max_force=%.2fN thresh=%.2fN phase=%d",
        grasp_controller_type_.c_str(), active_count, num_active_fingertips_,
        static_cast<double>(max_force), static_cast<double>(force_thresh),
        grasp_controller_ ? static_cast<int>(grasp_controller_->phase()) : -1);

    // Hand grasp control: force_pi (adaptive PI) or contact_stop (binary
    // freeze)
    if (grasp_controller_ && grasp_controller_type_ == "force_pi") {
      // Build force input: 3 fingers의 force magnitude
      std::array<double, rtc::grasp::kNumGraspFingers> f_raw{};
      for (int f = 0; f < rtc::grasp::kNumGraspFingers; ++f) {
        f_raw[static_cast<std::size_t>(f)] = static_cast<double>(
            grasp_state_.force_magnitude[static_cast<std::size_t>(f)]);
      }

      const auto commands = grasp_controller_->Update(
          std::span<const double, rtc::grasp::kNumGraspFingers>(f_raw), dt);

      // Phase-transition log: rare event (gated by phase change), but still
      // throttled as a defensive RT-safety net in case the FSM oscillates.
      const auto cur_phase = static_cast<uint8_t>(grasp_controller_->phase());
      if (cur_phase != prev_grasp_phase_) {
        RCLCPP_INFO_THROTTLE(
            logger_, log_clock_, ::ur5e_bringup::logging::kThrottleFastMs,
            "[force_pi] phase %u -> %u target_force=%.2fN", prev_grasp_phase_,
            cur_phase, grasp_controller_->target_force());
        prev_grasp_phase_ = cur_phase;
      }

      if (grasp_controller_->phase() != rtc::grasp::GraspPhase::kIdle) {
        // Force PI 활성: fingers 0-2 (joints 0-8) 덮어쓰기
        for (int f = 0; f < rtc::grasp::kNumGraspFingers; ++f) {
          for (int j = 0; j < rtc::grasp::kDoFPerFinger; ++j) {
            const auto mi = static_cast<std::size_t>(
                kFingerJointMap[static_cast<std::size_t>(f)]
                               [static_cast<std::size_t>(j)]);
            hand_computed_.positions[mi] =
                commands.q[static_cast<std::size_t>(f)]
                          [static_cast<std::size_t>(j)];
            hand_computed_.velocities[mi] = 0.0;
          }
        }
        // joint 9 (ring)는 trajectory 출력 그대로 유지
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
        const auto &dev1 = state.devices[1];

        const double d_thumb = device_targets_[1][kHandIdxThumbCmcFe] -
                               dev1.positions[kHandIdxThumbCmcFe];
        const double d_index = device_targets_[1][kHandIdxIndexMcpFe] -
                               dev1.positions[kHandIdxIndexMcpFe];
        const double d_middle = device_targets_[1][kHandIdxMiddleMcpFe] -
                                dev1.positions[kHandIdxMiddleMcpFe];

        const bool thumb_releasing = d_thumb > kContactStopReleaseEps;
        const bool index_releasing = d_index < -kContactStopReleaseEps;
        const bool middle_releasing = d_middle < -kContactStopReleaseEps;
        const bool release_phase =
            thumb_releasing && index_releasing && middle_releasing;

        if (release_phase) {
          RCLCPP_INFO_THROTTLE(logger_, log_clock_,
                               ::ur5e_bringup::logging::kThrottleFastMs,
                               "[contact_stop] SKIP (release) dthumb_fe=%+.3f "
                               "dindex_fe=%+.3f dmid_fe=%+.3f",
                               d_thumb, d_index, d_middle);
        } else if (active_count > 0 && max_force > force_thresh) {
          for (std::size_t i = 0; i < static_cast<std::size_t>(kNumHandMotors);
               ++i) {
            hand_computed_.positions[i] = dev1.positions[i];
            hand_computed_.velocities[i] = 0.0;
          }
          // Errors (target - actual) encode both the actual position and the
          // overshoot beyond target in a single number each, so 5 args are
          // enough to diagnose contact_stop engagement.
          const double err_thumb = device_targets_[1][kHandIdxThumbCmcFe] -
                                   dev1.positions[kHandIdxThumbCmcFe];
          const double err_index = device_targets_[1][kHandIdxIndexMcpFe] -
                                   dev1.positions[kHandIdxIndexMcpFe];
          const double err_middle = device_targets_[1][kHandIdxMiddleMcpFe] -
                                    dev1.positions[kHandIdxMiddleMcpFe];
          RCLCPP_INFO_THROTTLE(logger_, log_clock_,
                               ::ur5e_bringup::logging::kThrottleFastMs,
                               "[contact_stop] FREEZE active=%d fmax=%.2fN "
                               "err=[%+.3f,%+.3f,%+.3f]",
                               active_count, static_cast<double>(max_force),
                               err_thumb, err_index, err_middle);
        }
      }
    }
  }

  // ── ToF snapshot (3 fingers × 2 sensors: tof[1]=A, tof[2]=B) ───────────
  {
    constexpr int kNumTofFingers = 3;    // thumb, index, middle (hand-specific)
    constexpr int kSensorsPerFinger = 2; // A/B pair
    constexpr double kMmToM = 0.001;
    tof_snapshot_ = {};

    if (hand_handle_ && num_active_fingertips_ >= kNumTofFingers) {
      tof_snapshot_.num_fingers = kNumTofFingers;
      tof_snapshot_.sensors_per_finger = kSensorsPerFinger;

      for (int f = 0; f < kNumTofFingers; ++f) {
        const auto fi = static_cast<std::size_t>(f);
        const auto &ft = fingertip_data_[fi];
        const int si = f * kSensorsPerFinger;

        // tof[1] → sensor A, tof[2] → sensor B (tof[0] 제외)
        const double d_a = static_cast<double>(ft.tof[1]) * kMmToM;
        const double d_b = static_cast<double>(ft.tof[2]) * kMmToM;
        tof_snapshot_.distances[static_cast<std::size_t>(si)] = d_a;
        tof_snapshot_.distances[static_cast<std::size_t>(si + 1)] = d_b;
        tof_snapshot_.valid[static_cast<std::size_t>(si)] = (d_a > 0.0);
        tof_snapshot_.valid[static_cast<std::size_t>(si + 1)] = (d_b > 0.0);

        // Fingertip SE3 pose → position + quaternion
        auto &pose = tof_snapshot_.tip_poses[fi];
        const auto &pos = fingertip_positions_[fi];
        pose.position = {pos[0], pos[1], pos[2]};
        const Eigen::Quaterniond quat(fingertip_rotations_[fi]);
        pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
      }
      tof_snapshot_.populated = true;
    }
  }
}

// ── Phase 3: Write output ────────────────────────────────────────────────────

ControllerOutput DemoJointController::WriteOutput(const ControllerState &state,
                                                  double /*dt*/) noexcept {
  ControllerOutput output;
  output.num_devices = state.num_devices;

  // ── Robot arm output ────────────────────────────────────────────────────
  const auto &dev0 = state.devices[0];
  auto &out0 = output.devices[0];
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
  ClampCommands(out0.commands, nc0, device_position_lower_[0],
                device_position_upper_[0]);

  // ── Forward kinematics for task-space logging ──────────────────────────
  std::span<const double> q_span(dev0.positions.data(),
                                 static_cast<std::size_t>(nc0));
  arm_handle_->ComputeForwardKinematics(q_span);
  pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
  }
  // Log virtual TCP pose when active, otherwise raw TCP
  const pinocchio::SE3 &log_pose = vtcp_valid_ ? vtcp_pose_ : tcp;
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(log_pose.rotation());

  output.actual_task_positions[0] = log_pose.translation().x();
  output.actual_task_positions[1] = log_pose.translation().y();
  output.actual_task_positions[2] = log_pose.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  // Joint mode: no explicit task goal from GUI, mirror FK result
  output.task_goal_positions = output.actual_task_positions;

  // ── Hand output ────────────────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto &out1 = output.devices[1];
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
    ClampCommands(out1.commands, nc1, device_position_lower_[1],
                  device_position_upper_[1]);
  }

  // Populate force-PI grasp state if active
  if (grasp_controller_ && grasp_controller_type_ == "force_pi") {
    grasp_state_.grasp_phase = static_cast<uint8_t>(grasp_controller_->phase());
    grasp_state_.grasp_target_force =
        static_cast<float>(grasp_controller_->target_force());
    const auto &fs = grasp_controller_->finger_states();
    for (int f = 0; f < rtc::grasp::kNumGraspFingers; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      grasp_state_.finger_s[idx] = static_cast<float>(fs[idx].s);
      grasp_state_.finger_filtered_force[idx] =
          static_cast<float>(fs[idx].f_measured);
      grasp_state_.finger_force_error[idx] =
          static_cast<float>(fs[idx].f_desired - fs[idx].f_measured);
    }
  }

  output.command_type = command_type_;
  output.grasp_state = grasp_state_;
  output.tof_snapshot = tof_snapshot_;
  return output;
}

void DemoJointController::SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices)
    return;
  const int n = std::min(static_cast<int>(target.size()), kMaxDeviceChannels);
  {
    std::lock_guard lock(target_mutex_);
    for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
      device_targets_[static_cast<std::size_t>(device_idx)][i] = target[i];
    }
  }
  if (device_idx == 0) {
    robot_new_target_.store(true, std::memory_order_release);
  } else if (device_idx == 1) {
    hand_new_target_.store(true, std::memory_order_release);
  }
}

void DemoJointController::InitializeHoldPosition(
    const ControllerState &state) noexcept {
  std::lock_guard lock(target_mutex_);

  // Robot
  {
    const auto &dev0 = state.devices[0];
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State hold_state;
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      device_targets_[0][i] = dev0.positions[i];
      hold_state.positions[i] = dev0.positions[i];
      hold_state.velocities[i] = 0.0;
      hold_state.accelerations[i] = 0.0;
    }
    robot_trajectory_.initialize(hold_state, hold_state, 0.01);
    robot_trajectory_time_ = 0.0;
    robot_new_target_.store(false, std::memory_order_relaxed);
  }

  // Hand
  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices);
       ++d) {
    const auto &dev = state.devices[d];
    if (!dev.valid)
      continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(dev.num_channels) &&
                            i < kMaxDeviceChannels;
         ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
    if (d == 1) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hold_state;
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hold_state.positions[i] = dev.positions[i];
        hold_state.velocities[i] = 0.0;
        hold_state.accelerations[i] = 0.0;
      }
      hand_trajectory_.initialize(hold_state, hold_state, 0.01);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }
  }
}

void DemoJointController::ClampCommands(
    std::array<double, kMaxDeviceChannels> &commands, int n,
    const std::vector<double> &lower,
    const std::vector<double> &upper) noexcept {
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lo = (i < lower.size()) ? lower[i] : -6.2832;
    const double hi = (i < upper.size()) ? upper[i] : 6.2832;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoJointController::LoadConfig(const YAML::Node &cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }

  // ── Build arm model from system model config or bridge YAML ──────────────
  namespace rub = rtc_urdf_bridge;
  const auto *sys_cfg = GetSystemModelConfig();
  if (sys_cfg && !sys_cfg->urdf_path.empty() && !sys_cfg->sub_models.empty()) {
    // System-level ModelConfig (top-level "urdf:" YAML section)
    InitArmModel(*sys_cfg);
  } else if (cfg["model_config"]) {
    // Fallback: separate model config YAML file (backward compatibility)
    const auto yaml_name = cfg["model_config"].as<std::string>();
    const auto yaml_path =
        ament_index_cpp::get_package_share_directory("ur5e_bringup") +
        "/config/" + yaml_name;
    auto model_cfg = rub::PinocchioModelBuilder::LoadModelConfig(yaml_path);
    model_cfg.urdf_path = urdf_path_;
    InitArmModel(model_cfg);
  } else if (!urdf_path_.empty()) {
    // Fallback: arm-only URDF, no sub-model extraction
    rub::ModelConfig model_cfg;
    model_cfg.urdf_path = urdf_path_;
    model_cfg.root_joint_type = "fixed";
    model_cfg.sub_models.push_back({"arm", "base_link", "tool0"});
    InitArmModel(model_cfg);
  }

  // ── Build hand tree-model if configured ─────────────────────────────
  if (sys_cfg && !sys_cfg->tree_models.empty()) {
    InitHandModel(*sys_cfg);
  }

  auto g = gains_lock_.Load();
  if (cfg["robot_trajectory_speed"]) {
    g.robot_trajectory_speed =
        std::max(1e-6, cfg["robot_trajectory_speed"].as<double>());
  }
  if (cfg["hand_trajectory_speed"]) {
    g.hand_trajectory_speed =
        std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
  }
  if (cfg["robot_max_traj_velocity"]) {
    g.robot_max_traj_velocity = cfg["robot_max_traj_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    g.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── Shared params: defaults from demo_shared.yaml, overridden by cfg ──
  DemoSharedConfig shared;
  LoadDemoSharedYamlFile(shared);
  ApplyDemoSharedConfig(cfg, shared);

  g.vtcp = shared.vtcp;
  g.grasp_contact_threshold = shared.grasp_contact_threshold;
  g.grasp_force_threshold = shared.grasp_force_threshold;
  g.grasp_min_fingertips = shared.grasp_min_fingertips;
  gains_lock_.Store(g);
  grasp_controller_type_ = shared.grasp_controller_type;

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ =
        (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }

  BuildGraspController(shared, 1.0 / GetDefaultDt(), grasp_controller_);
}

void DemoJointController::UpdateGainsFromMsg(
    std::span<const double> gains) noexcept {
  // layout: [robot_trajectory_speed, hand_trajectory_speed,
  //          robot_max_traj_velocity, hand_max_traj_velocity,
  //          grasp_contact_threshold, grasp_force_threshold,
  //          grasp_min_fingertips,
  //          grasp_command, grasp_target_force] = 9 values
  auto g = gains_lock_.Load();
  if (gains.size() >= 1) {
    g.robot_trajectory_speed = std::max(1e-6, gains[0]);
  }
  if (gains.size() >= 2) {
    g.hand_trajectory_speed = std::max(1e-6, gains[1]);
  }
  if (gains.size() >= 3) {
    g.robot_max_traj_velocity = gains[2];
  }
  if (gains.size() >= 4) {
    g.hand_max_traj_velocity = gains[3];
  }
  if (gains.size() >= 5) {
    g.grasp_contact_threshold = static_cast<float>(gains[4]);
  }
  if (gains.size() >= 6) {
    g.grasp_force_threshold = static_cast<float>(gains[5]);
  }
  if (gains.size() >= 7) {
    g.grasp_min_fingertips = static_cast<int>(gains[6]);
  }
  gains_lock_.Store(g);
  // grasp_command: 0=none, 1=grasp, 2=release
  if (gains.size() >= 8) {
    const int cmd = static_cast<int>(gains[7]);
    if (cmd == 1 || cmd == 2) {
      if (!grasp_controller_) {
        RCLCPP_WARN_THROTTLE(
            logger_, log_clock_, ::ur5e_bringup::logging::kThrottleSlowMs,
            "[grasp] %s command ignored: grasp_controller_type='%s' "
            "(require 'force_pi' in YAML to enable Grasp/Release buttons)",
            (cmd == 1) ? "Grasp" : "Release", grasp_controller_type_.c_str());
      } else if (cmd == 1) {
        const double target_force = (gains.size() >= 9) ? gains[8] : 0.0;
        const auto phase_before =
            static_cast<unsigned>(grasp_controller_->phase());
        grasp_controller_->CommandGrasp(target_force);
        RCLCPP_INFO(logger_,
                    "CommandGrasp target=%.2fN type=%s phase_before=%u",
                    target_force, grasp_controller_type_.c_str(), phase_before);
      } else {
        const auto phase_before =
            static_cast<unsigned>(grasp_controller_->phase());
        grasp_controller_->CommandRelease();
        RCLCPP_INFO(logger_, "CommandRelease type=%s phase_before=%u",
                    grasp_controller_type_.c_str(), phase_before);
      }
    }
  }
}

std::vector<double> DemoJointController::GetCurrentGains() const noexcept {
  // layout: [robot_trajectory_speed, hand_trajectory_speed,
  //          robot_max_traj_velocity, hand_max_traj_velocity,
  //          grasp_contact_threshold, grasp_force_threshold,
  //          grasp_min_fingertips,
  //          grasp_command, grasp_target_force] = 9 values
  const auto g = gains_lock_.Load();
  return {
      g.robot_trajectory_speed,
      g.hand_trajectory_speed,
      g.robot_max_traj_velocity,
      g.hand_max_traj_velocity,
      static_cast<double>(g.grasp_contact_threshold),
      static_cast<double>(g.grasp_force_threshold),
      static_cast<double>(g.grasp_min_fingertips),
      0.0, // grasp_command (read-only: always 0)
      grasp_controller_ ? grasp_controller_->target_force() : 0.0,
  };
}

// ── E-STOP ──────────────────────────────────────────────────────────────────

void DemoJointController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void DemoJointController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool DemoJointController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void DemoJointController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

ControllerOutput
DemoJointController::ComputeEstop(const ControllerState &state) noexcept {
  const auto &dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;

  // Robot arm: move toward safe position with velocity limit
  auto &out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim =
        (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    out0.commands[i] =
        dev0.positions[i] +
        std::clamp(kSafePosition[i] - dev0.positions[i], -lim, lim) *
            ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
    out0.goal_positions[i] = kSafePosition[i];
    out0.target_positions[i] = out0.commands[i];
    out0.trajectory_positions[i] = out0.commands[i];
  }

  // Hand: hold current position during E-Stop
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto &dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto &out1 = output.devices[1];
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
  std::span<const double> q_span(dev0.positions.data(),
                                 static_cast<std::size_t>(nc0));
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

  return output;
}

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────
// Delegates to ur5e_bringup::owned_topics helpers so the 3 demo controllers
// share a single implementation; only storage lives in the subclass.

RTControllerInterface::CallbackReturn DemoJointController::on_configure(
    const rclcpp_lifecycle::State &prev,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node &yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    CreateOwnedTopics(*this, owned_topics_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "DemoJointController on_configure failed: %s",
                 e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(logger_, "DemoJointController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn
DemoJointController::on_activate(const rclcpp_lifecycle::State &prev) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoJointController::on_deactivate(
    const rclcpp_lifecycle::State &prev) noexcept {
  DeactivateOwnedTopics(prev, owned_topics_);
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn
DemoJointController::on_cleanup(const rclcpp_lifecycle::State &prev) noexcept {
  ResetOwnedTopics(owned_topics_);
  return RTControllerInterface::on_cleanup(prev);
}

void DemoJointController::PublishNonRtSnapshot(
    const rtc::PublishSnapshot &snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

} // namespace ur5e_bringup
