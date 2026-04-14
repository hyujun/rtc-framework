// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "rtc_controllers/indirect/clik_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

namespace rtc
{

// ── Constructor ─────────────────────────────────────────────────────────────

ClikController::ClikController(std::string_view urdf_path, Gains gains)
: gains_(gains)
{
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";

  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  model_ptr_ = builder.GetFullModel();
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);

  // Default: use the last frame in the model as tip
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);

  const int nv = handle_->nv();

  // Pre-allocate all Eigen buffers to their final sizes.
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  J_pos_ = Eigen::MatrixXd::Zero(3, nv);
  JJt_ = Eigen::Matrix3d::Zero();
  JJt_inv_ = Eigen::Matrix3d::Zero();
  Jpinv_ = Eigen::MatrixXd::Zero(nv, 3);
  N_ = Eigen::MatrixXd::Identity(nv, nv);
  dq_ = Eigen::VectorXd::Zero(nv);
  desired_q_ = Eigen::VectorXd::Zero(nv);
  traj_dq_ = Eigen::VectorXd::Zero(nv);
  null_err_ = Eigen::VectorXd::Zero(nv);
  null_dq_ = Eigen::VectorXd::Zero(nv);
  pos_error_ = Eigen::Vector3d::Zero();

  JJt_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  JJt_inv_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  Jpinv_6d_ = Eigen::MatrixXd::Zero(nv, 6);
  pos_error_6d_ = Eigen::Matrix<double, 6, 1>::Zero();
}

void ClikController::OnDeviceConfigsSet()
{
  const auto primary = GetPrimaryDeviceName();
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) {
        tip_frame_id_ = fid;
      }
    }
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
    }
    if (!cfg->safe_position.empty()) {
      safe_position_ = cfg->safe_position;
      // Also use safe_position as null-space reference if available
      for (std::size_t i = 0; i < std::min(cfg->safe_position.size(),
           null_target_.size()); ++i) {
        null_target_[i] = cfg->safe_position[i];
      }
    }
  }
  if (max_joint_velocity_.empty()) {
    max_joint_velocity_.assign(kMaxDeviceChannels, kDefaultMaxJointVelocity);
  }
  if (safe_position_.empty()) {
    safe_position_.assign(kMaxDeviceChannels, 0.0);
  }
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput ClikController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const int nv = handle_->nv();

  // ── Step 1: copy joint state into q buffer ──────────────────────────────
  const auto & dev0 = state.devices[0];
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    q_buf[static_cast<std::size_t>(i)] = dev0.positions[static_cast<std::size_t>(i)];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));

  // ── Step 2: FK + Jacobians ───────────────────────────────────────────────
  // ComputeJacobians performs FK internally and updates frame placements.
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  // Translational Jacobian: rows 0..2
  J_pos_.noalias() = J_full_.topRows(3);

  // ── Step 3: Cartesian position/orientation error ──────────────────────────
  const pinocchio::SE3 & tcp_pose = handle_->GetFramePlacement(tip_frame_id_);
  const Eigen::Vector3d tcp = tcp_pose.translation();

  if (!target_initialized_) {
    tcp_target_pose_ = tcp_pose;
    // Keep internal target synchronized
    tcp_target_ = {tcp[0], tcp[1], tcp[2]};
    target_initialized_ = true;
  }

  // Protect target read + trajectory init with target_mutex_.
  // try_lock so RT thread never blocks — skip on contention, handle next tick.
  if (new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      pinocchio::SE3 start_pose = tcp_pose;
      pinocchio::SE3 goal_pose;

      if (gains_.control_6dof) {
        goal_pose = tcp_target_pose_;
      } else {
        goal_pose = start_pose; // keep current rotation
        goal_pose.translation() = Eigen::Vector3d(tcp_target_[0], tcp_target_[1], tcp_target_[2]);
      }

      const Eigen::Vector3d start_pos = start_pose.translation();
      const Eigen::Vector3d goal_pos = goal_pose.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      // Duration from translational trajectory_speed and velocity limit.
      // Quintic rest-to-rest peak velocity = (15/8) * dist / T.
      const double T_speed_trans = trans_dist / gains_.trajectory_speed;
      const double T_vel_trans = (gains_.max_traj_velocity > 0.0)
          ? (1.875 * trans_dist / gains_.max_traj_velocity)
          : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      // Angular distance via AngleAxisd (stable at θ → π, unlike log3)
      constexpr double kPiSafetyMargin = 0.15;  // rad ≈ 8.6°
      double angular_dist = 0.0;
      Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ();  // fallback
      bool split_trajectory = false;

      if (gains_.control_6dof) {
        const Eigen::AngleAxisd aa(
            start_pose.rotation().transpose() * goal_pose.rotation());
        angular_dist = aa.angle();       // always in [0, π]
        rot_axis = aa.axis();

        const double T_speed_rot = angular_dist / gains_.trajectory_angular_speed;
        const double T_vel_rot = (gains_.max_traj_angular_velocity > 0.0)
            ? (1.875 * angular_dist / gains_.max_traj_angular_velocity)
            : 0.0;
        duration = std::max({duration, T_speed_rot, T_vel_rot});

        split_trajectory = (angular_dist > M_PI - kPiSafetyMargin);
      }

      if (split_trajectory) {
        // ── π-rotation defense: split into 2 rest-to-rest segments ──
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid = start_pose.rotation() *
            Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

        pinocchio::SE3 mid_pose;
        mid_pose.translation() = 0.5 * (start_pos + goal_pos);
        mid_pose.rotation() = R_mid;

        // Segment 1: start → mid (half distances)
        const double half_trans = trans_dist * 0.5;
        const double T1_speed_t = half_trans / gains_.trajectory_speed;
        const double T1_vel_t = (gains_.max_traj_velocity > 0.0)
            ? (1.875 * half_trans / gains_.max_traj_velocity) : 0.0;
        const double T1_speed_r = half_angle / gains_.trajectory_angular_speed;
        const double T1_vel_r = (gains_.max_traj_angular_velocity > 0.0)
            ? (1.875 * half_angle / gains_.max_traj_angular_velocity) : 0.0;
        const double dur1 = std::max({0.01, T1_speed_t, T1_vel_t, T1_speed_r, T1_vel_r});

        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(),
                               mid_pose, pinocchio::Motion::Zero(), dur1);
        pending_goal_pose_ = goal_pose;
        pending_duration_ = dur1;  // symmetric split
        has_pending_segment_ = true;
      } else {
        trajectory_.initialize(start_pose, pinocchio::Motion::Zero(),
                               goal_pose, pinocchio::Motion::Zero(),
                               duration);
        has_pending_segment_ = false;
      }

      trajectory_time_ = 0.0;
      // Initialize desired_q_ from current actual joint positions
      for (int i = 0; i < nv; ++i) {
        desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
      }
      new_target_.store(false, std::memory_order_relaxed);
    }
  }

  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  traj_state_ = trajectory_.compute(trajectory_time_, dt);
  trajectory_time_ += dt;

  // ── Segment transition (π-rotation defense) ────────────────────────────
  if (has_pending_segment_ && trajectory_time_ >= trajectory_.duration()) {
    pinocchio::SE3 mid_pose = trajectory_.compute(trajectory_.duration()).pose;
    trajectory_.initialize(mid_pose, pinocchio::Motion::Zero(),
                           pending_goal_pose_, pinocchio::Motion::Zero(),
                           pending_duration_);
    trajectory_time_ = 0.0;
    has_pending_segment_ = false;
  }

  tcp_position_ = {tcp[0], tcp[1], tcp[2]};

  // Positional error is trajectory - current
  // Rotation error is computed using log6/log3
  pinocchio::SE3 T_current_desired = tcp_pose.actInv(traj_state_.pose); // relative pose
  pinocchio::Motion twist_error = pinocchio::log6(T_current_desired);

  // The twist_error is the spatial velocity needed in the LOCAL frame to reach target.
  // Converting it to the LOCAL_WORLD_ALIGNED frame (where J_full_ is expressed).
  Eigen::Vector3d p_err = tcp_pose.rotation() * twist_error.linear();
  Eigen::Vector3d r_err = tcp_pose.rotation() * twist_error.angular();

  pos_error_6d_.head<3>() = p_err;
  pos_error_6d_.tail<3>() = r_err;

  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = p_err[i];
  }

  // ── Step 4 & 5: Damped pseudoinverse & Primary task ──────────────────────
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
    // Feedforward: trajectory local → world-aligned via R_trajectory (not R_current)
    task_vel_6d.head<3>() += traj_state_.pose.rotation() * traj_state_.velocity.linear();
    task_vel_6d.tail<3>() += traj_state_.pose.rotation() * traj_state_.velocity.angular();

    dq_.noalias() = Jpinv_6d_ * task_vel_6d;
  } else {
    // 3D version
    JJt_.noalias() = J_pos_ * J_pos_.transpose();
    JJt_.diagonal().array() += gains_.damping * gains_.damping;
    ldlt_.compute(JJt_);
    JJt_inv_.noalias() = ldlt_.solve(Eigen::Matrix3d::Identity());
    Jpinv_.noalias() = J_pos_.transpose() * JJt_inv_;

    Eigen::Vector3d kp_vec(gains_.kp_translation[0], gains_.kp_translation[1], gains_.kp_translation[2]);
    Eigen::Vector3d task_vel = kp_vec.cwiseProduct(pos_error_) +
        traj_state_.pose.rotation() * traj_state_.velocity.linear();
    dq_.noalias() = Jpinv_ * task_vel;
  }

  // ── Feedforward-only trajectory velocity (for logging) ────────────────
  if (gains_.control_6dof) {
    Eigen::Matrix<double, 6, 1> ff_vel_6d;
    ff_vel_6d.head<3>() = traj_state_.pose.rotation() * traj_state_.velocity.linear();
    ff_vel_6d.tail<3>() = traj_state_.pose.rotation() * traj_state_.velocity.angular();
    traj_dq_.noalias() = Jpinv_6d_ * ff_vel_6d;
  } else {
    traj_dq_.noalias() = Jpinv_ * (traj_state_.pose.rotation() * traj_state_.velocity.linear());
  }

  // ── Step 6: Null-space secondary task ────────────────────────────────────
  if (gains_.enable_null_space && !gains_.control_6dof) {
    N_.setIdentity();
    N_.noalias() -= Jpinv_ * J_pos_;

    for (int i = 0; i < nv; ++i) {
      null_err_[static_cast<Eigen::Index>(i)] = null_target_[static_cast<std::size_t>(i)] -
        dev0.positions[static_cast<std::size_t>(i)];
    }
    null_dq_.noalias() = N_ * null_err_;
    null_dq_ *= gains_.null_kp;
    dq_ += null_dq_;
  }

  // ── Step 7: Clamp joint velocity and integrate ────────────────────────────
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_velocities[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  ClampVelocity(out0.target_velocities, nc0);

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
  // goal_positions: task-space goal in [0..2], null-space goal in [3..5]
  for (std::size_t i = 0; i < 3; ++i) {
    out0.goal_positions[i] = tcp_target_[i];
  }
  for (std::size_t i = 3; i < static_cast<std::size_t>(nc0); ++i) {
    out0.goal_positions[i] = null_target_[i];
  }

  // Device 1+ : pass-through goals
  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto & devN = state.devices[d];
    auto & outN = output.devices[d];
    const int ncN = devN.num_channels;
    outN.num_channels = ncN;
    for (std::size_t i = 0; i < static_cast<std::size_t>(ncN); ++i) {
      outN.commands[i] = device_targets_[d][i];
      outN.target_positions[i] = device_targets_[d][i];
      outN.goal_positions[i] = device_targets_[d][i];
    }
  }

  const pinocchio::SE3 & tcp_current = handle_->GetFramePlacement(tip_frame_id_);
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp_current.rotation());
  output.actual_task_positions[0] = tcp_current.translation().x();
  output.actual_task_positions[1] = tcp_current.translation().y();
  output.actual_task_positions[2] = tcp_current.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  // Task-space goal target
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

  // Task-space trajectory reference
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

  output.command_type = command_type_;
  return output;
}

void ClikController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) return;
  if (device_idx == 0) {
    std::lock_guard lock(target_mutex_);
    if (gains_.control_6dof) {
      // 6-DOF target mode: [x, y, z, roll, pitch, yaw]
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
      // 3-DOF target mode: [x, y, z, null3, null4, ...]
      const auto nv = static_cast<std::size_t>(handle_->nv());
      const std::size_t n = std::min(target.size(), nv);
      for (std::size_t i = 0; i < std::min(n, std::size_t{3}); ++i) {
        tcp_target_[i] = target[i];
      }
      for (std::size_t i = 3; i < n; ++i) {
        null_target_[i] = target[i];
      }
    }
    new_target_.store(true, std::memory_order_release);
  } else {
    const auto ud = static_cast<std::size_t>(device_idx);
    const std::size_t n = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < n; ++i) {
      device_targets_[ud][i] = target[i];
    }
  }
}

void ClikController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  const auto & dev0 = state.devices[0];
  const int nv = handle_->nv();

  // Compute current TCP position/pose via FK and set as target
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    q_buf[static_cast<std::size_t>(i)] = dev0.positions[static_cast<std::size_t>(i)];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  handle_->ComputeForwardKinematics(q_span);
  const pinocchio::SE3 & tcp_pose = handle_->GetFramePlacement(tip_frame_id_);

  // Initialize desired_q_ from current actual joint positions
  for (int i = 0; i < nv; ++i) {
    desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
  }

  std::lock_guard lock(target_mutex_);
  tcp_target_pose_ = tcp_pose;
  tcp_target_ = {tcp_pose.translation()[0],
                 tcp_pose.translation()[1],
                 tcp_pose.translation()[2]};
  // null-space target: initialize to current joint positions
  for (int i = 0; i < nv; ++i) {
    null_target_[static_cast<std::size_t>(i)] = dev0.positions[static_cast<std::size_t>(i)];
  }
  target_initialized_ = true;
  new_target_.store(false, std::memory_order_relaxed);

  // Initialize stationary trajectory (start == goal)
  trajectory_.initialize(tcp_pose, pinocchio::Motion::Zero(),
                         tcp_pose, pinocchio::Motion::Zero(), 0.01);
  trajectory_time_ = 0.0;
  has_pending_segment_ = false;

  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid) continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(dev.num_channels) && i < static_cast<std::size_t>(kMaxDeviceChannels); ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
  }
}

std::string_view ClikController::Name() const noexcept
{
  return "ClikController";
}

void ClikController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_release);
}

void ClikController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
}

bool ClikController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void ClikController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput ClikController::ComputeEstop(
  const ControllerState & state) noexcept
{
  const auto & dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim = (i < max_joint_velocity_.size()) ? max_joint_velocity_[i] : kDefaultMaxJointVelocity;
    const double sp = (i < safe_position_.size()) ? safe_position_[i] : 0.0;
    out0.commands[i] = dev0.positions[i] +
      std::clamp(sp - dev0.positions[i], -lim, lim) *
      ((state.dt > 0.0) ? state.dt : GetDefaultDt());
  }
  return output;
}

void ClikController::ClampVelocity(
  std::array<double, kMaxDeviceChannels>& dq, int n) const noexcept
{
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lim = (i < max_joint_velocity_.size()) ? max_joint_velocity_[i] : kDefaultMaxJointVelocity;
    dq[i] = std::clamp(dq[i], -lim, lim);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void ClikController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {return;}

  // CLIK gains — translation / rotation separated
  auto load3 = [](const YAML::Node & n, std::array<double, 3> & arr) {
      if (n && n.IsSequence() && n.size() >= 3) {
        for (std::size_t i = 0; i < 3; ++i) {
          arr[i] = n[i].as<double>();
        }
      }
    };
  load3(cfg["kp_translation"], gains_.kp_translation);
  load3(cfg["kp_rotation"], gains_.kp_rotation);

  if (cfg["damping"]) {gains_.damping = cfg["damping"].as<double>();}
  if (cfg["null_kp"]) {gains_.null_kp = cfg["null_kp"].as<double>();}
  if (cfg["enable_null_space"]) {gains_.enable_null_space = cfg["enable_null_space"].as<bool>();}
  if (cfg["control_6dof"]) {gains_.control_6dof = cfg["control_6dof"].as<bool>();}

  // Trajectory speed
  if (cfg["trajectory_speed"]) {gains_.trajectory_speed = cfg["trajectory_speed"].as<double>();}
  if (cfg["trajectory_angular_speed"]) {
    gains_.trajectory_angular_speed = cfg["trajectory_angular_speed"].as<double>();
  }

  // Trajectory velocity limits
  if (cfg["max_traj_velocity"]) {
    gains_.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    gains_.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void ClikController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //          enable_null_space(0/1), control_6dof(0/1),
  //          trajectory_speed, trajectory_angular_speed,
  //          max_traj_velocity, max_traj_angular_velocity] = 14
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
  if (gains.size() >= 13) {gains_.max_traj_velocity = gains[12];}
  if (gains.size() >= 14) {gains_.max_traj_angular_velocity = gains[13];}
}

std::vector<double> ClikController::GetCurrentGains() const noexcept
{
  // layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //          enable_null_space(0/1), control_6dof(0/1),
  //          trajectory_speed, trajectory_angular_speed,
  //          max_traj_velocity, max_traj_angular_velocity] = 14
  std::vector<double> v;
  v.reserve(14);
  v.insert(v.end(), gains_.kp_translation.begin(), gains_.kp_translation.end());
  v.insert(v.end(), gains_.kp_rotation.begin(), gains_.kp_rotation.end());
  v.push_back(gains_.damping);
  v.push_back(gains_.null_kp);
  v.push_back(gains_.enable_null_space ? 1.0 : 0.0);
  v.push_back(gains_.control_6dof ? 1.0 : 0.0);
  v.push_back(gains_.trajectory_speed);
  v.push_back(gains_.trajectory_angular_speed);
  v.push_back(gains_.max_traj_velocity);
  v.push_back(gains_.max_traj_angular_velocity);
  return v;
}

}  // namespace rtc
