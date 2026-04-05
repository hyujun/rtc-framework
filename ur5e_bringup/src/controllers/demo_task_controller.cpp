// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_bringup/controllers/demo_task_controller.hpp"

#include <algorithm>
#include <cmath>     // std::sqrt
#include <cstddef>

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/spatial/log.hpp>
#pragma GCC diagnostic pop

namespace ur5e_bringup
{

// ── Constructor ─────────────────────────────────────────────────────────────

DemoTaskController::DemoTaskController(std::string_view urdf_path, Gains gains)
: gains_(gains), urdf_path_(urdf_path)
{
  // Model is built in LoadConfig() (bridge YAML driven) or InitArmModel().
  // Constructor only stores urdf_path for later use.
}

void DemoTaskController::InitArmModel(
  const rtc_urdf_bridge::ModelConfig & config)
{
  namespace rub = rtc_urdf_bridge;
  builder_ = std::make_unique<rub::PinocchioModelBuilder>(config);

  // Resolve sub-model name: match primary device name, fallback to "arm"
  const auto primary = GetPrimaryDeviceName();
  std::string model_name = "arm";
  for (const auto& sm : config.sub_models) {
    if (sm.name == primary) { model_name = primary; break; }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(
    builder_->GetReducedModel(model_name));

  const int nv = arm_handle_->nv();

  // Pre-allocate all Eigen buffers to their final sizes.
  q_ = Eigen::VectorXd::Zero(nv);
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  J_pos_ = Eigen::MatrixXd::Zero(3, nv);
  JJt_ = Eigen::Matrix3d::Zero();
  JJt_inv_ = Eigen::Matrix3d::Zero();
  Jpinv_ = Eigen::MatrixXd::Zero(nv, 3);
  N_ = Eigen::MatrixXd::Identity(nv, nv);
  dq_ = Eigen::VectorXd::Zero(nv);
  traj_dq_ = Eigen::VectorXd::Zero(nv);
  null_err_ = Eigen::VectorXd::Zero(nv);
  null_dq_ = Eigen::VectorXd::Zero(nv);
  pos_error_ = Eigen::Vector3d::Zero();

  JJt_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  JJt_inv_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  Jpinv_6d_ = Eigen::MatrixXd::Zero(nv, 6);
  pos_error_6d_ = Eigen::Matrix<double, 6, 1>::Zero();
}

// ── Hand tree-model initialization ──────────────────────────────────────────
void DemoTaskController::InitHandModel(
  const rtc_urdf_bridge::ModelConfig & /*config*/)
{
  namespace rub = rtc_urdf_bridge;
  hand_handle_ = std::make_unique<rub::RtModelHandle>(
    builder_->GetTreeModel("hand"));

  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg) {
    for (const auto& tm : sys_cfg->tree_models) {
      if (tm.name == "hand") {
        if (!tm.root_link.empty()) {
          hand_root_frame_id_ = hand_handle_->GetFrameId(tm.root_link);
          if (hand_root_frame_id_ != 0) {
            use_hand_root_frame_ = true;
          }
        }
        for (std::size_t i = 0; i < std::min(tm.tip_links.size(), kNumFingertips); ++i) {
          fingertip_frame_ids_[i] = hand_handle_->GetFrameId(tm.tip_links[i]);
        }
        break;
      }
    }
  }

  hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());
  for (auto& p : fingertip_positions_) p = Eigen::Vector3d::Zero();
  for (auto& r : fingertip_rotations_) r = Eigen::Matrix3d::Identity();
}

void DemoTaskController::OnDeviceConfigsSet()
{
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) {  // 0 = universe (not found)
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
  if (auto* cfg = GetDeviceNameConfig("hand"); cfg && cfg->joint_limits) {
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
  for (auto& v : device_max_velocity_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, 2.0);
  }
  for (auto& v : device_position_lower_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, -6.2832);
  }
  for (auto& v : device_position_upper_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, 6.2832);
  }
}

// ── Virtual TCP computation ─────────────────────────────────────────────────

void DemoTaskController::ComputeVirtualTcp(
  const pinocchio::SE3& T_base_tcp) noexcept
{
  vtcp_valid_ = false;
  if (!hand_handle_ || gains_.virtual_tcp_mode == VirtualTcpMode::kDisabled) return;

  switch (gains_.virtual_tcp_mode) {
    case VirtualTcpMode::kCentroid: {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      int count = 0;
      for (std::size_t f = 0; f < kNumFingertips; ++f) {
        if (fingertip_frame_ids_[f] != 0) {
          auto ft_pose = hand_handle_->GetFramePlacement(fingertip_frame_ids_[f]);
          if (use_hand_root_frame_) {
            ft_pose = hand_handle_->GetFramePlacement(hand_root_frame_id_).actInv(ft_pose);
          }
          sum += ft_pose.translation();
          ++count;
        }
      }
      if (count == 0) return;
      T_tcp_vtcp_.translation() = sum / static_cast<double>(count);
      T_tcp_vtcp_.rotation().setIdentity();
      break;
    }
    case VirtualTcpMode::kWeighted: {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      double total_weight = 0.0;
      for (std::size_t f = 0; f < kNumFingertips; ++f) {
        if (fingertip_frame_ids_[f] == 0) continue;
        const auto& ft = fingertip_data_[f];
        double w = 1.0;  // base weight (ensures non-zero even without contact)
        if (ft.valid) {
          w += static_cast<double>(std::sqrt(
              ft.force[0]*ft.force[0] +
              ft.force[1]*ft.force[1] +
              ft.force[2]*ft.force[2]));
        }
        auto ft_pose = hand_handle_->GetFramePlacement(fingertip_frame_ids_[f]);
        if (use_hand_root_frame_) {
          ft_pose = hand_handle_->GetFramePlacement(hand_root_frame_id_).actInv(ft_pose);
        }
        sum += w * ft_pose.translation();
        total_weight += w;
      }
      if (total_weight <= 0.0) return;
      T_tcp_vtcp_.translation() = sum / total_weight;
      T_tcp_vtcp_.rotation().setIdentity();
      break;
    }
    case VirtualTcpMode::kConstant: {
      T_tcp_vtcp_.translation() = Eigen::Vector3d(
        gains_.virtual_tcp_offset[0],
        gains_.virtual_tcp_offset[1],
        gains_.virtual_tcp_offset[2]);
      T_tcp_vtcp_.rotation().setIdentity();
      break;
    }
    default: return;
  }

  vtcp_pose_ = T_base_tcp.act(T_tcp_vtcp_);
  vtcp_valid_ = true;
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput DemoTaskController::Compute(
  const ControllerState & state) noexcept
{
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ReadState(state);
  ComputeControl(state, dt);
  return WriteOutput(state, dt);
}

// ── Phase 1: Read joint states + sensor data ────────────────────────────────

void DemoTaskController::ReadState(const ControllerState & state) noexcept
{
  // Robot arm joint positions → FK + Jacobians via arm_handle_
  const auto & dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;
  std::span<const double> q_span(dev0.positions.data(),
    static_cast<std::size_t>(nc0));
  arm_handle_->ComputeJacobians(q_span);
  arm_handle_->GetFrameJacobian(tip_frame_id_,
    pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  if (use_root_frame_) {
    const Eigen::Matrix3d R_root_T =
      arm_handle_->GetFrameRotation(root_frame_id_).transpose();
    J_full_.topRows(3)    = R_root_T * J_full_.topRows(3);
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

  // Hand motor data: dev1.motor_positions[], motor_velocities[], motor_efforts[]
  // available via state.devices[1].motor_* (populated from /hand/motor_states)

  // Hand sensor data (per-fingertip)
  num_active_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / rtc::kSensorValuesPerFingertip;
    num_active_fingertips_ = std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

    for (std::size_t f = 0; f < static_cast<std::size_t>(num_active_fingertips_); ++f) {
      auto & ft = fingertip_data_[f];
      const std::size_t base = f * rtc::kSensorValuesPerFingertip;

      for (std::size_t j = 0; j < rtc::kBarometerCount; ++j) {
        ft.baro[j] = dev1.sensor_data[base + j];
      }
      for (std::size_t j = 0; j < 3; ++j) {
        ft.tof[j] =
            dev1.sensor_data[base + rtc::kBarometerCount + j];
      }

      ft.valid = dev1.inference_enable[f];
      if (ft.valid) {
        const std::size_t ft_base = f * rtc::kFTValuesPerFingertip;
        ft.contact_flag = dev1.inference_data[ft_base];
        for (std::size_t j = 0; j < 3; ++j) {
          ft.force[j] =
              dev1.inference_data[ft_base + 1 + j];
          ft.displacement[j] =
              dev1.inference_data[ft_base + 4 + j];
        }
      } else {
        ft.contact_flag = 0.0f;
        ft.force = {};
        ft.displacement = {};
      }
    }
  }
}

// ── Phase 2: Compute control (IK/CLIK + trajectory + sensor-based logic) ────

void DemoTaskController::ComputeControl(
  const ControllerState & state, double dt) noexcept
{
  // ── E-stop check ───────────────────────────────────────────────────────
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) { return; }

  const auto & dev0 = state.devices[0];

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
    hand_handle_->ComputeForwardKinematics(
      std::span<const double>(hand_q_.data(), hand_nq));

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
    ComputeVirtualTcp(tcp_pose);
  }

  // ── Control pose: virtual TCP or tool0 ────────────────────────────────
  pinocchio::SE3 control_pose = tcp_pose;
  if (vtcp_valid_) {
    control_pose = vtcp_pose_;

    // Modify Jacobian: J_vtcp_lin = J_tcp_lin - skew(offset) * J_tcp_ang
    const Eigen::Vector3d offset =
        vtcp_pose_.translation() - tcp_pose.translation();
    skew_buf_ <<       0.0, -offset(2),  offset(1),
                 offset(2),        0.0, -offset(0),
                -offset(1),  offset(0),        0.0;
    J_full_.topRows(3) -= skew_buf_ * J_full_.bottomRows(3);
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

      if (gains_.control_6dof) {
        goal_pose = tcp_target_pose_;
      } else {
        goal_pose = start_pose;
        goal_pose.translation() = Eigen::Vector3d(tcp_target_[0], tcp_target_[1], tcp_target_[2]);
      }

      const double trans_dist = (goal_pose.translation() - start_pose.translation()).norm();
      // Duration from translational trajectory_speed and velocity limit.
      // Quintic rest-to-rest peak velocity = (15/8) * dist / T.
      const double T_speed_trans = trans_dist / gains_.trajectory_speed;
      const double T_vel_trans = (gains_.max_traj_velocity > 0.0)
          ? (1.875 * trans_dist / gains_.max_traj_velocity)
          : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      if (gains_.control_6dof) {
        double angular_dist = pinocchio::log3(start_pose.rotation().transpose() *
            goal_pose.rotation()).norm();
        const double T_speed_rot = angular_dist / gains_.trajectory_angular_speed;
        const double T_vel_rot = (gains_.max_traj_angular_velocity > 0.0)
            ? (1.875 * angular_dist / gains_.max_traj_angular_velocity)
            : 0.0;
        duration = std::max({duration, T_speed_rot, T_vel_rot});
      }

      trajectory_.initialize(start_pose, pinocchio::Motion::Zero(),
                             goal_pose, pinocchio::Motion::Zero(),
                             duration);

      trajectory_time_ = 0.0;
      new_target_.store(false, std::memory_order_relaxed);
    }
  }

  traj_state_ = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  // ── Cartesian error ────────────────────────────────────────────────────
  pinocchio::SE3 T_current_desired = control_pose.actInv(traj_state_.pose);
  pinocchio::Motion twist_error = pinocchio::log6(T_current_desired);

  Eigen::Vector3d p_err = traj_state_.pose.translation() - ctrl_pos;
  Eigen::Vector3d r_err = control_pose.rotation() * twist_error.angular();

  pos_error_6d_.head<3>() = p_err;
  pos_error_6d_.tail<3>() = r_err;

  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = p_err[i];
  }

  // ── Damped pseudoinverse & Primary task ────────────────────────────────
  if (gains_.control_6dof) {
    JJt_6d_.noalias() = J_full_ * J_full_.transpose();
    JJt_6d_.diagonal().array() += gains_.damping * gains_.damping;
    ldlt_6d_.compute(JJt_6d_);
    JJt_inv_6d_.noalias() = ldlt_6d_.solve(Eigen::Matrix<double, 6, 6>::Identity());
    Jpinv_6d_.noalias() = J_full_.transpose() * JJt_inv_6d_;

    Eigen::Matrix<double, 6, 1> kp_vec_6d;
    for (std::size_t i = 0; i < 3; ++i) {
      kp_vec_6d[static_cast<Eigen::Index>(i)] = gains_.kp_translation[i];
      kp_vec_6d[static_cast<Eigen::Index>(i + 3)] = gains_.kp_rotation[i];
    }

    Eigen::Matrix<double, 6, 1> task_vel_6d = kp_vec_6d.cwiseProduct(pos_error_6d_);
    task_vel_6d.head<3>() += traj_state_.velocity.linear();
    task_vel_6d.tail<3>() += control_pose.rotation() * traj_state_.velocity.angular();

    dq_.noalias() = Jpinv_6d_ * task_vel_6d;
  } else {
    JJt_.noalias() = J_pos_ * J_pos_.transpose();
    JJt_.diagonal().array() += gains_.damping * gains_.damping;
    ldlt_.compute(JJt_);
    JJt_inv_.noalias() = ldlt_.solve(Eigen::Matrix3d::Identity());
    Jpinv_.noalias() = J_pos_.transpose() * JJt_inv_;

    Eigen::Vector3d kp_vec(gains_.kp_translation[0], gains_.kp_translation[1], gains_.kp_translation[2]);
    Eigen::Vector3d task_vel = kp_vec.cwiseProduct(pos_error_) + traj_state_.velocity.linear();
    dq_.noalias() = Jpinv_ * task_vel;
  }

  // ── Feedforward-only trajectory velocity (for logging) ────────────────
  if (gains_.control_6dof) {
    Eigen::Matrix<double, 6, 1> ff_vel_6d;
    ff_vel_6d.head<3>() = traj_state_.velocity.linear();
    ff_vel_6d.tail<3>() = control_pose.rotation() * traj_state_.velocity.angular();
    traj_dq_.noalias() = Jpinv_6d_ * ff_vel_6d;
  } else {
    traj_dq_.noalias() = Jpinv_ * traj_state_.velocity.linear();
  }

  // ── Null-space secondary task ──────────────────────────────────────────
  if (gains_.enable_null_space && !gains_.control_6dof) {
    N_.setIdentity();
    N_.noalias() -= Jpinv_ * J_pos_;

    for (Eigen::Index i = 0; i < arm_handle_->nv(); ++i) {
      null_err_[i] = null_target_[static_cast<std::size_t>(i)] -
        dev0.positions[static_cast<std::size_t>(i)];
    }
    null_dq_.noalias() = N_ * null_err_;
    null_dq_ *= gains_.null_kp;
    dq_ += null_dq_;
  }

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];

    if (hand_new_target_.load(std::memory_order_acquire)) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State start_state;
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State goal_state;

      double max_dist = 0.0;
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        start_state.positions[i]     = dev1.positions[i];
        start_state.velocities[i]    = 0.0;
        start_state.accelerations[i] = 0.0;

        goal_state.positions[i]     = device_targets_[1][i];
        goal_state.velocities[i]    = 0.0;
        goal_state.accelerations[i] = 0.0;

        max_dist = std::max(max_dist,
          std::abs(device_targets_[1][i] - dev1.positions[i]));
      }

      const double T_speed = max_dist / gains_.hand_trajectory_speed;
      const double T_vel = (gains_.hand_max_traj_velocity > 0.0)
          ? (1.875 * max_dist / gains_.hand_max_traj_velocity)
          : 0.0;
      const double duration = std::max({0.01, T_speed, T_vel});
      hand_trajectory_.initialize(start_state, goal_state, duration);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }

    const auto hand_traj = hand_trajectory_.compute(hand_trajectory_time_);
    hand_trajectory_time_ += dt;

    for (std::size_t i = 0; i < kNumHandMotors; ++i) {
      hand_computed_.positions[i] = hand_traj.positions[i];
      hand_computed_.velocities[i] = hand_traj.velocities[i];
    }
  }

  // ── Grasp detection + ContactStopHand (500Hz) ────────────────────────
  {
    const float contact_thresh = gains_.grasp_contact_threshold;
    const float force_thresh   = gains_.grasp_force_threshold;
    const int   min_fingers    = gains_.grasp_min_fingertips;

    float max_force = 0.0f;
    int active_count = 0;

    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const auto& ft = fingertip_data_[idx];
      const float mag = std::sqrt(
          ft.force[0]*ft.force[0] +
          ft.force[1]*ft.force[1] +
          ft.force[2]*ft.force[2]);

      grasp_state_.force_magnitude[idx] = mag;
      grasp_state_.contact_flag[idx]    = ft.contact_flag;
      grasp_state_.inference_valid[idx] = ft.valid;

      if (mag > max_force) max_force = mag;
      if (ft.valid && ft.contact_flag > contact_thresh && mag > force_thresh) {
        ++active_count;
      }
    }
    grasp_state_.num_fingertips           = num_active_fingertips_;
    grasp_state_.num_active_contacts      = active_count;
    grasp_state_.max_force                = max_force;
    grasp_state_.force_threshold          = force_thresh;
    grasp_state_.min_fingertips_for_grasp = min_fingers;
    grasp_state_.grasp_detected           = (active_count >= min_fingers);

    // ContactStopHand: 힘 감지 시 hand trajectory 출력을 현재 위치로 동결
    // → BT tick(50ms) 사이에도 과도한 hand closure 방지
    if (active_count > 0 && max_force > force_thresh) {
      if (state.num_devices > 1 && state.devices[1].valid) {
        const auto& dev1 = state.devices[1];
        for (std::size_t i = 0; i < static_cast<std::size_t>(kNumHandMotors); ++i) {
          hand_computed_.positions[i] = dev1.positions[i];
          hand_computed_.velocities[i] = 0.0;
        }
      }
    }
  }

  // ── ToF snapshot (3 fingers × 2 sensors: tof[1]=A, tof[2]=B) ───────────
  {
    constexpr int kNumTofFingers = 3;          // thumb, index, middle (hand-specific)
    constexpr int kSensorsPerFinger = 2;       // A/B pair
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
        tof_snapshot_.distances[static_cast<std::size_t>(si)]     = d_a;
        tof_snapshot_.distances[static_cast<std::size_t>(si + 1)] = d_b;
        tof_snapshot_.valid[static_cast<std::size_t>(si)]     = (d_a > 0.0);
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

ControllerOutput DemoTaskController::WriteOutput(
  const ControllerState & state, double dt) noexcept
{
  // ── E-stop early return ────────────────────────────────────────────────
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  ControllerOutput output;
  output.num_devices = state.num_devices;

  // ── Robot arm output ───────────────────────────────────────────────────
  const auto & dev0 = state.devices[0];
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_velocities[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  ClampCommands(out0.target_velocities, nc0, device_position_lower_[0], device_position_upper_[0]);

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = dev0.positions[i] + out0.target_velocities[i] * dt;
    // Pure trajectory feedforward velocity (without Kp error / null-space)
    out0.trajectory_velocities[i] = traj_dq_[static_cast<Eigen::Index>(i)];
    // Trajectory-implied joint position = current + feedforward * dt
    out0.trajectory_positions[i] = dev0.positions[i] + out0.trajectory_velocities[i] * dt;
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

  // Task goal target from GUI
  output.task_goal_positions[0] = tcp_target_[0];
  output.task_goal_positions[1] = tcp_target_[1];
  output.task_goal_positions[2] = tcp_target_[2];
  if (gains_.control_6dof) {
    Eigen::Vector3d goal_rpy = pinocchio::rpy::matrixToRpy(
        tcp_target_pose_.rotation());
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
    auto & out1 = output.devices[1];
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
    ClampCommands(out1.commands, nc1, device_position_lower_[1], device_position_upper_[1]);
  }

  output.command_type = command_type_;
  output.grasp_state = grasp_state_;
  output.tof_snapshot = tof_snapshot_;
  return output;
}

void DemoTaskController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) return;
  if (device_idx == 0) {
    std::lock_guard lock(target_mutex_);
    if (gains_.control_6dof) {
      if (target.size() >= 6) {
        tcp_target_[0] = target[0];
        tcp_target_[1] = target[1];
        tcp_target_[2] = target[2];

        double r = target[3];
        double p = target[4];
        double y = target[5];

        Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

        tcp_target_pose_.translation() << target[0], target[1], target[2];
        tcp_target_pose_.rotation() = q.matrix();
      }
    } else {
      const std::size_t n = std::min(target.size(), static_cast<std::size_t>(kNumRobotJoints));
      for (std::size_t i = 0; i < std::min(n, std::size_t{3}); ++i) {
        tcp_target_[i] = target[i];
      }
      for (std::size_t i = 3; i < n; ++i) {
        null_target_[i] = target[i];
      }
    }
    new_target_.store(true, std::memory_order_release);
  } else {
    const std::size_t n = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < n; ++i) {
      device_targets_[static_cast<std::size_t>(device_idx)][i] = target[i];
    }
    if (device_idx == 1) {
      hand_new_target_.store(true, std::memory_order_release);
    }
  }
}

void DemoTaskController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  const auto & dev0 = state.devices[0];
  std::span<const double> q_span(dev0.positions.data(),
    static_cast<std::size_t>(dev0.num_channels));
  arm_handle_->ComputeForwardKinematics(q_span);
  pinocchio::SE3 tcp_pose = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    tcp_pose = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_pose);
  }

  // Virtual TCP: compute hold_pose from fingertip kinematics if enabled
  pinocchio::SE3 hold_pose = tcp_pose;
  if (hand_handle_ && state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const auto hand_nq = static_cast<std::size_t>(hand_handle_->nq());
    for (std::size_t i = 0; i < hand_nq; ++i) {
      hand_q_[static_cast<Eigen::Index>(i)] = dev1.positions[i];
    }
    hand_handle_->ComputeForwardKinematics(
      std::span<const double>(hand_q_.data(), hand_nq));
    ComputeVirtualTcp(tcp_pose);
    if (vtcp_valid_) { hold_pose = vtcp_pose_; }
  }

  std::lock_guard lock(target_mutex_);
  tcp_target_pose_ = hold_pose;
  tcp_target_ = {hold_pose.translation()[0],
                 hold_pose.translation()[1],
                 hold_pose.translation()[2]};
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    null_target_[i] = dev0.positions[i];
  }
  target_initialized_ = true;
  new_target_.store(false, std::memory_order_relaxed);

  trajectory_.initialize(hold_pose, pinocchio::Motion::Zero(),
                         hold_pose, pinocchio::Motion::Zero(), 0.01);
  trajectory_time_ = 0.0;

  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid) continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(dev.num_channels) && i < kMaxDeviceChannels; ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
    if (d == 1) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hold_state;
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hold_state.positions[i]     = dev.positions[i];
        hold_state.velocities[i]    = 0.0;
        hold_state.accelerations[i] = 0.0;
      }
      hand_trajectory_.initialize(hold_state, hold_state, 0.01);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }
  }
}

std::string_view DemoTaskController::Name() const noexcept
{
  return "DemoTaskController";
}

void DemoTaskController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_release);
}

void DemoTaskController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
}

bool DemoTaskController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void DemoTaskController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput DemoTaskController::ComputeEstop(
  const ControllerState & state) noexcept
{
  const auto & dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim = (i < device_max_velocity_[0].size()) ? device_max_velocity_[0][i] : 2.0;
    out0.commands[i] = dev0.positions[i] +
      std::clamp(kSafePosition[i] - dev0.positions[i], -lim, lim) *
      ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
  }

  // Hand: hold current position during E-Stop
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto & out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.commands[i] = dev1.positions[i];
      out1.goal_positions[i] = dev1.positions[i];
      out1.target_positions[i] = dev1.positions[i];
      out1.trajectory_positions[i] = dev1.positions[i];
    }
  }

  return output;
}

void DemoTaskController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n,
  const std::vector<double>& lower,
  const std::vector<double>& upper) noexcept
{
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lo = (i < lower.size()) ? lower[i] : -6.2832;
    const double hi = (i < upper.size()) ? upper[i] :  6.2832;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoTaskController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {return;}

  // ── Build arm model from system model config or bridge YAML ──────────────
  namespace rub = rtc_urdf_bridge;
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg && !sys_cfg->urdf_path.empty() && !sys_cfg->sub_models.empty()) {
    // System-level ModelConfig (top-level "urdf:" YAML section)
    InitArmModel(*sys_cfg);
  } else if (cfg["model_config"]) {
    // Fallback: separate model config YAML file (backward compatibility)
    const auto yaml_name = cfg["model_config"].as<std::string>();
    const auto yaml_path =
      ament_index_cpp::get_package_share_directory("ur5e_bringup")
      + "/config/" + yaml_name;
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

  // CLIK gains — translation / rotation separated
  if (cfg["kp_translation"] && cfg["kp_translation"].IsSequence()) {
    std::size_t n = std::min<std::size_t>(cfg["kp_translation"].size(), 3);
    for (std::size_t i = 0; i < n; ++i) {
      gains_.kp_translation[i] = cfg["kp_translation"][i].as<double>();
    }
  }
  if (cfg["kp_rotation"] && cfg["kp_rotation"].IsSequence()) {
    std::size_t n = std::min<std::size_t>(cfg["kp_rotation"].size(), 3);
    for (std::size_t i = 0; i < n; ++i) {
      gains_.kp_rotation[i] = cfg["kp_rotation"][i].as<double>();
    }
  }
  if (cfg["damping"]) {gains_.damping = cfg["damping"].as<double>();}
  if (cfg["null_kp"]) {gains_.null_kp = cfg["null_kp"].as<double>();}
  if (cfg["enable_null_space"]) {gains_.enable_null_space = cfg["enable_null_space"].as<bool>();}
  if (cfg["control_6dof"]) {gains_.control_6dof = cfg["control_6dof"].as<bool>();}

  // Trajectory speed
  if (cfg["trajectory_speed"]) {gains_.trajectory_speed = cfg["trajectory_speed"].as<double>();}
  if (cfg["trajectory_angular_speed"]) {
    gains_.trajectory_angular_speed = cfg["trajectory_angular_speed"].as<double>();
  }
  if (cfg["hand_trajectory_speed"]) {
    gains_.hand_trajectory_speed = cfg["hand_trajectory_speed"].as<double>();
  }
  if (cfg["max_traj_velocity"]) {
    gains_.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    gains_.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    gains_.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── Virtual TCP configuration ──────────────────────────────────────────
  if (cfg["virtual_tcp_mode"]) {
    const auto mode_str = cfg["virtual_tcp_mode"].as<std::string>();
    if (mode_str == "centroid") {
      gains_.virtual_tcp_mode = VirtualTcpMode::kCentroid;
    } else if (mode_str == "weighted") {
      gains_.virtual_tcp_mode = VirtualTcpMode::kWeighted;
    } else if (mode_str == "constant") {
      gains_.virtual_tcp_mode = VirtualTcpMode::kConstant;
    } else {
      gains_.virtual_tcp_mode = VirtualTcpMode::kDisabled;
    }
  }
  if (cfg["virtual_tcp_offset"] && cfg["virtual_tcp_offset"].IsSequence()) {
    for (std::size_t i = 0; i < 3 && i < cfg["virtual_tcp_offset"].size(); ++i) {
      gains_.virtual_tcp_offset[i] = cfg["virtual_tcp_offset"][i].as<double>();
    }
  }

  // Grasp detection parameters
  if (cfg["grasp_contact_threshold"]) {
    gains_.grasp_contact_threshold = cfg["grasp_contact_threshold"].as<float>();
  }
  if (cfg["grasp_force_threshold"]) {
    gains_.grasp_force_threshold = cfg["grasp_force_threshold"].as<float>();
  }
  if (cfg["grasp_min_fingertips"]) {
    gains_.grasp_min_fingertips = cfg["grasp_min_fingertips"].as<int>();
  }

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void DemoTaskController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //          enable_null_space(0/1), control_6dof(0/1),
  //          trajectory_speed, trajectory_angular_speed,
  //          hand_trajectory_speed, max_traj_velocity,
  //          max_traj_angular_velocity, hand_max_traj_velocity,
  //          grasp_contact_threshold, grasp_force_threshold,
  //          grasp_min_fingertips] = 19
  if (gains.size() < 10) {return;}
  for (std::size_t i = 0; i < 3; ++i) {
    gains_.kp_translation[i] = gains[i];
    gains_.kp_rotation[i] = gains[3 + i];
  }
  gains_.damping = gains[6];
  gains_.null_kp = gains[7];
  gains_.enable_null_space = gains[8] > 0.5;
  gains_.control_6dof = gains[9] > 0.5;

  if (gains.size() >= 11) {gains_.trajectory_speed = gains[10];}
  if (gains.size() >= 12) {gains_.trajectory_angular_speed = gains[11];}
  if (gains.size() >= 13) {gains_.hand_trajectory_speed = gains[12];}
  if (gains.size() >= 14) {gains_.max_traj_velocity = gains[13];}
  if (gains.size() >= 15) {gains_.max_traj_angular_velocity = gains[14];}
  if (gains.size() >= 16) {gains_.hand_max_traj_velocity = gains[15];}
  if (gains.size() >= 17) {gains_.grasp_contact_threshold = static_cast<float>(gains[16]);}
  if (gains.size() >= 18) {gains_.grasp_force_threshold = static_cast<float>(gains[17]);}
  if (gains.size() >= 19) {gains_.grasp_min_fingertips = static_cast<int>(gains[18]);}
}

std::vector<double> DemoTaskController::GetCurrentGains() const noexcept
{
  // layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //          enable_null_space(0/1), control_6dof(0/1),
  //          trajectory_speed, trajectory_angular_speed,
  //          hand_trajectory_speed, max_traj_velocity,
  //          max_traj_angular_velocity, hand_max_traj_velocity,
  //          grasp_contact_threshold, grasp_force_threshold,
  //          grasp_min_fingertips] = 19
  std::vector<double> v;
  v.reserve(19);
  v.insert(v.end(), gains_.kp_translation.begin(), gains_.kp_translation.end());
  v.insert(v.end(), gains_.kp_rotation.begin(), gains_.kp_rotation.end());
  v.push_back(gains_.damping);
  v.push_back(gains_.null_kp);
  v.push_back(gains_.enable_null_space ? 1.0 : 0.0);
  v.push_back(gains_.control_6dof ? 1.0 : 0.0);
  v.push_back(gains_.trajectory_speed);
  v.push_back(gains_.trajectory_angular_speed);
  v.push_back(gains_.hand_trajectory_speed);
  v.push_back(gains_.max_traj_velocity);
  v.push_back(gains_.max_traj_angular_velocity);
  v.push_back(gains_.hand_max_traj_velocity);
  v.push_back(static_cast<double>(gains_.grasp_contact_threshold));
  v.push_back(static_cast<double>(gains_.grasp_force_threshold));
  v.push_back(static_cast<double>(gains_.grasp_min_fingertips));
  return v;
}

}  // namespace ur5e_bringup
