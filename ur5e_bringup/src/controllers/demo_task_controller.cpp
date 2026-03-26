// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_bringup/controllers/demo_task_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <pinocchio/math/rpy.hpp>

namespace ur5e_bringup
{

// ── Constructor ─────────────────────────────────────────────────────────────

DemoTaskController::DemoTaskController(std::string_view urdf_path, Gains gains)
: data_(pinocchio::Model{}), gains_(gains)
{
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_ = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);

  // Pre-allocate all Eigen buffers to their final sizes.
  q_ = Eigen::VectorXd::Zero(model_.nv);
  J_full_ = Eigen::MatrixXd::Zero(6, model_.nv);
  J_pos_ = Eigen::MatrixXd::Zero(3, model_.nv);
  JJt_ = Eigen::Matrix3d::Zero();
  JJt_inv_ = Eigen::Matrix3d::Zero();
  Jpinv_ = Eigen::MatrixXd::Zero(model_.nv, 3);
  N_ = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
  dq_ = Eigen::VectorXd::Zero(model_.nv);
  traj_dq_ = Eigen::VectorXd::Zero(model_.nv);
  null_err_ = Eigen::VectorXd::Zero(model_.nv);
  null_dq_ = Eigen::VectorXd::Zero(model_.nv);
  pos_error_ = Eigen::Vector3d::Zero();

  JJt_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  JJt_inv_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  Jpinv_6d_ = Eigen::MatrixXd::Zero(model_.nv, 6);
  pos_error_6d_ = Eigen::Matrix<double, 6, 1>::Zero();
}

void DemoTaskController::OnDeviceConfigsSet()
{
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      if (model_.existFrame(cfg->urdf->tip_link)) {
        auto fid = model_.getFrameId(cfg->urdf->tip_link);
        end_id_ = model_.frames[fid].parentJoint;
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
  // Robot arm joint positions → Eigen vector + FK + Jacobians
  const auto & dev0 = state.devices[0];
  for (Eigen::Index i = 0; i < model_.nv; ++i) {
    q_[i] = dev0.positions[static_cast<std::size_t>(i)];
  }

  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::getJointJacobian(model_, data_, end_id_,
                               pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  J_pos_.noalias() = J_full_.topRows(3);

  // Initialize target on first call
  const pinocchio::SE3 tcp_pose = data_.oMi[end_id_];
  const Eigen::Vector3d tcp = tcp_pose.translation();
  if (!target_initialized_) {
    tcp_target_pose_ = tcp_pose;
    tcp_target_ = {tcp[0], tcp[1], tcp[2]};
    target_initialized_ = true;
  }

  tcp_position_ = {tcp[0], tcp[1], tcp[2]};

  // Hand sensor data (per-fingertip)
  num_active_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / rtc::kSensorValuesPerFingertip;
    num_active_fingertips_ = std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

    for (int f = 0; f < num_active_fingertips_; ++f) {
      auto & ft = fingertip_data_[static_cast<std::size_t>(f)];
      const int base = f * rtc::kSensorValuesPerFingertip;

      for (int j = 0; j < static_cast<int>(rtc::kBarometerCount); ++j) {
        ft.baro[static_cast<std::size_t>(j)] = dev1.sensor_data[base + j];
      }
      for (int j = 0; j < 3; ++j) {
        ft.tof[static_cast<std::size_t>(j)] =
            dev1.sensor_data[base + static_cast<int>(rtc::kBarometerCount) + j];
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

  // ── Task-space trajectory ──────────────────────────────────────────────
  const pinocchio::SE3 tcp_pose = data_.oMi[end_id_];
  const Eigen::Vector3d tcp = tcp_pose.translation();

  if (new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      pinocchio::SE3 start_pose = tcp_pose;
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
  pinocchio::SE3 T_current_desired = tcp_pose.actInv(traj_state_.pose);
  pinocchio::Motion twist_error = pinocchio::log6(T_current_desired);

  Eigen::Vector3d p_err = traj_state_.pose.translation() - tcp;
  Eigen::Vector3d r_err = tcp_pose.rotation() * twist_error.angular();

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
    task_vel_6d.tail<3>() += tcp_pose.rotation() * traj_state_.velocity.angular();

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
    ff_vel_6d.tail<3>() = tcp_pose.rotation() * traj_state_.velocity.angular();
    traj_dq_.noalias() = Jpinv_6d_ * ff_vel_6d;
  } else {
    traj_dq_.noalias() = Jpinv_ * traj_state_.velocity.linear();
  }

  // ── Null-space secondary task ──────────────────────────────────────────
  if (gains_.enable_null_space && !gains_.control_6dof) {
    N_.setIdentity();
    N_.noalias() -= Jpinv_ * J_pos_;

    for (Eigen::Index i = 0; i < model_.nv; ++i) {
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

  // ── Sensor-based control logic (확장 포인트) ────────────────────────────
  // fingertip_data_[0..num_active_fingertips_-1] 에 파싱된 센서 데이터 사용 가능
  // dq_, hand_computed_ 보정 가능
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

  for (int i = 0; i < nc0; ++i) {
    out0.target_velocities[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  ClampCommands(out0.target_velocities, nc0, device_position_lower_[0], device_position_upper_[0]);

  for (int i = 0; i < nc0; ++i) {
    out0.commands[i] = dev0.positions[i] + out0.target_velocities[i] * dt;
    // Pure trajectory feedforward velocity (without Kp error / null-space)
    out0.trajectory_velocities[i] = traj_dq_[static_cast<Eigen::Index>(i)];
    // Trajectory-implied joint position = current + feedforward * dt
    out0.trajectory_positions[i] = dev0.positions[i] + out0.trajectory_velocities[i] * dt;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out0.target_positions[i] = traj_state_.pose.translation()[static_cast<Eigen::Index>(i)];
  }
  for (int i = 3; i < nc0; ++i) {
    out0.target_positions[i] = null_target_[i];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out0.goal_positions[i] = tcp_target_[i];
  }
  for (int i = 3; i < nc0; ++i) {
    out0.goal_positions[i] = null_target_[i];
  }

  // ── Task-space logging ─────────────────────────────────────────────────
  const pinocchio::SE3 & tcp_current = data_.oMi[end_id_];
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp_current.rotation());
  output.actual_task_positions[0] = tcp_current.translation().x();
  output.actual_task_positions[1] = tcp_current.translation().y();
  output.actual_task_positions[2] = tcp_current.translation().z();
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

  // ── Hand output ────────────────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto & out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;

    for (int i = 0; i < nc1; ++i) {
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
      const int n = std::min(static_cast<int>(target.size()), static_cast<int>(kNumRobotJoints));
      for (int i = 0; i < std::min(n, 3); ++i) {
        tcp_target_[i] = target[i];
      }
      for (int i = 3; i < n; ++i) {
        null_target_[i] = target[i];
      }
    }
    new_target_.store(true, std::memory_order_release);
  } else {
    const int n = std::min(static_cast<int>(target.size()), kMaxDeviceChannels);
    for (int i = 0; i < n; ++i) {
      device_targets_[device_idx][i] = target[i];
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
  for (Eigen::Index i = 0; i < model_.nv; ++i) {
    q_[i] = dev0.positions[static_cast<std::size_t>(i)];
  }
  pinocchio::forwardKinematics(model_, data_, q_);
  const pinocchio::SE3 & tcp_pose = data_.oMi[end_id_];

  std::lock_guard lock(target_mutex_);
  tcp_target_pose_ = tcp_pose;
  tcp_target_ = {tcp_pose.translation()[0],
                 tcp_pose.translation()[1],
                 tcp_pose.translation()[2]};
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    null_target_[i] = dev0.positions[i];
  }
  target_initialized_ = true;
  new_target_.store(false, std::memory_order_relaxed);

  trajectory_.initialize(tcp_pose, pinocchio::Motion::Zero(),
                         tcp_pose, pinocchio::Motion::Zero(), 0.01);
  trajectory_time_ = 0.0;

  for (int d = 1; d < state.num_devices; ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid) continue;
    for (int i = 0; i < dev.num_channels && i < kMaxDeviceChannels; ++i) {
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
  for (int i = 0; i < nc0; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lim = (ui < device_max_velocity_[0].size()) ? device_max_velocity_[0][ui] : 2.0;
    out0.commands[i] = dev0.positions[i] +
      std::clamp(kSafePosition[i] - dev0.positions[i], -lim, lim) *
      ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
  }
  return output;
}

void DemoTaskController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n,
  const std::vector<double>& lower,
  const std::vector<double>& upper) noexcept
{
  for (int i = 0; i < n; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lo = (ui < lower.size()) ? lower[ui] : -6.2832;
    const double hi = (ui < upper.size()) ? upper[ui] :  6.2832;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoTaskController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {return;}

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
  //          max_traj_angular_velocity, hand_max_traj_velocity] = 16
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
}

std::vector<double> DemoTaskController::GetCurrentGains() const noexcept
{
  // layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //          enable_null_space(0/1), control_6dof(0/1),
  //          trajectory_speed, trajectory_angular_speed,
  //          hand_trajectory_speed, max_traj_velocity,
  //          max_traj_angular_velocity, hand_max_traj_velocity] = 16
  std::vector<double> v;
  v.reserve(16);
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
  return v;
}

}  // namespace ur5e_bringup
