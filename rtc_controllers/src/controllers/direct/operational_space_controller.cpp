// ── Includes: project header first, then C++ stdlib
// ────────────────────────────
#include "rtc_controllers/direct/operational_space_controller.hpp"

#include "rtc_base/utils/device_passthrough.hpp"

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

namespace rtc {

// ── Constructor ─────────────────────────────────────────────────────────────

OperationalSpaceController::OperationalSpaceController(
    std::string_view urdf_path, Gains gains)
    : gains_lock_(gains) {
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
  q_ = Eigen::VectorXd::Zero(nv);
  v_ = Eigen::VectorXd::Zero(nv);
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  Jpinv_ = Eigen::MatrixXd::Zero(nv, 6);
  dq_ = Eigen::VectorXd::Zero(nv);
  traj_dq_ = Eigen::VectorXd::Zero(nv);
  JJt_.setZero();
  task_err_.setZero();
  task_vel_.setZero();
  tcp_vel_.setZero();
}

void OperationalSpaceController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  if (auto *cfg = GetDeviceNameConfig(primary); cfg) {
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

ControllerOutput
OperationalSpaceController::Compute(const ControllerState &state) noexcept {
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const int nv = handle_->nv();

  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  const bool use_gravity = gains.enable_gravity_compensation;

  // ── Step 1: copy joint state into buffers ───────────────────────────────
  const auto &dev0 = state.devices[0];
  std::array<double, kMaxDeviceChannels> q_buf{};
  std::array<double, kMaxDeviceChannels> v_buf{};
  for (int i = 0; i < nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_buf[ui] = dev0.positions[ui];
    v_buf[ui] = dev0.velocities[ui];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  std::span<const double> v_span(v_buf.data(), static_cast<std::size_t>(nv));

  // ── Step 2: FK + full Jacobian ────────────────────────────────────────────
  // ComputeJacobians performs FK internally and updates frame placements.
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
                            J_full_);

  // ── Step 3: current task-space velocity  tcp_vel = J * dq ────────────────
  Eigen::Map<const Eigen::VectorXd> v_eigen(v_buf.data(), nv);
  tcp_vel_.noalias() = J_full_ * v_eigen;

  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const pinocchio::SE3 &tcp = handle_->GetFramePlacement(tip_frame_id_);

  // ── Step 3.5: initialise trajectory on new target (after FK) ─────────────
  if (new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      const Eigen::Vector3d start_pos = tcp.translation();
      const Eigen::Vector3d goal_pos = goal_pose_.translation();
      const double trans_dist = (goal_pos - start_pos).norm();

      // Duration from translational trajectory_speed and velocity limit.
      // Quintic rest-to-rest peak velocity = (15/8) * dist / T.
      const double T_speed_trans = trans_dist / gains.trajectory_speed;
      const double T_vel_trans =
          (gains.max_traj_velocity > 0.0)
              ? (1.875 * trans_dist / gains.max_traj_velocity)
              : 0.0;
      double duration = std::max({0.01, T_speed_trans, T_vel_trans});

      // Angular distance via AngleAxisd (stable at θ → π, unlike log3)
      constexpr double kPiSafetyMargin = 0.15; // rad ≈ 8.6°
      const Eigen::AngleAxisd aa(tcp.rotation().transpose() *
                                 goal_pose_.rotation());
      const double angular_dist = aa.angle(); // always in [0, π]
      const Eigen::Vector3d rot_axis = aa.axis();

      const double T_speed_rot = angular_dist / gains.trajectory_angular_speed;
      const double T_vel_rot =
          (gains.max_traj_angular_velocity > 0.0)
              ? (1.875 * angular_dist / gains.max_traj_angular_velocity)
              : 0.0;
      duration = std::max({duration, T_speed_rot, T_vel_rot});

      const bool split_trajectory = (angular_dist > M_PI - kPiSafetyMargin);

      if (split_trajectory) {
        // ── π-rotation defense: split into 2 rest-to-rest segments ──
        const double half_angle = angular_dist * 0.5;
        const Eigen::Matrix3d R_mid =
            tcp.rotation() *
            Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

        pinocchio::SE3 mid_pose;
        mid_pose.translation() = 0.5 * (start_pos + goal_pos);
        mid_pose.rotation() = R_mid;

        // Segment 1: start → mid (half distances)
        const double half_trans = trans_dist * 0.5;
        const double T1_speed_t = half_trans / gains.trajectory_speed;
        const double T1_vel_t =
            (gains.max_traj_velocity > 0.0)
                ? (1.875 * half_trans / gains.max_traj_velocity)
                : 0.0;
        const double T1_speed_r = half_angle / gains.trajectory_angular_speed;
        const double T1_vel_r =
            (gains.max_traj_angular_velocity > 0.0)
                ? (1.875 * half_angle / gains.max_traj_angular_velocity)
                : 0.0;
        const double dur1 =
            std::max({0.01, T1_speed_t, T1_vel_t, T1_speed_r, T1_vel_r});

        trajectory_.initialize(tcp, pinocchio::Motion::Zero(), mid_pose,
                               pinocchio::Motion::Zero(), dur1);
        pending_goal_pose_ = goal_pose_;
        pending_duration_ = dur1; // symmetric split
        has_pending_segment_ = true;
      } else {
        trajectory_.initialize(tcp, pinocchio::Motion::Zero(), goal_pose_,
                               pinocchio::Motion::Zero(), duration);
        has_pending_segment_ = false;
      }

      trajectory_time_ = 0.0;
      new_target_.store(false, std::memory_order_relaxed);
    }
  }

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

  // ── Step 4: 6D pose error w.r.t. trajectory setpoint ─────────────────────
  // 3D position error
  const Eigen::Vector3d pos_err =
      traj_state_.pose.translation() - tcp.translation();
  tcp_position_ = {tcp.translation()[0], tcp.translation()[1],
                   tcp.translation()[2]};

  // 3D orientation error via SO(3) logarithm
  const Eigen::Matrix3d R_err =
      traj_state_.pose.rotation() * tcp.rotation().transpose();
  const Eigen::Vector3d rot_err = pinocchio::log3(R_err);

  task_err_.head<3>() = pos_err;
  task_err_.tail<3>() = rot_err;

  // Cache for diagnostics (non-RT reads via pose_error())
  for (int i = 0; i < 6; ++i) {
    pose_error_cache_[static_cast<std::size_t>(i)] = task_err_[i];
  }

  // ── Step 5: desired task-space velocity with feedforward ──────────────────
  Eigen::Vector3d kp_p(gains.kp_pos[0], gains.kp_pos[1], gains.kp_pos[2]);
  Eigen::Vector3d kd_p(gains.kd_pos[0], gains.kd_pos[1], gains.kd_pos[2]);
  Eigen::Vector3d kp_r(gains.kp_rot[0], gains.kp_rot[1], gains.kp_rot[2]);
  Eigen::Vector3d kd_r(gains.kd_rot[0], gains.kd_rot[1], gains.kd_rot[2]);

  task_vel_.head<3>() = kp_p.cwiseProduct(pos_err) +
                        traj_state_.velocity.linear() -
                        kd_p.cwiseProduct(tcp_vel_.head<3>());
  task_vel_.tail<3>() = kp_r.cwiseProduct(rot_err) +
                        traj_state_.velocity.angular() -
                        kd_r.cwiseProduct(tcp_vel_.tail<3>());

  // ── Step 6: Damped pseudoinverse  J^# = J^T (J J^T + λ²I₆)^{−1} ─────────
  JJt_.noalias() = J_full_ * J_full_.transpose();
  JJt_.diagonal().array() += gains.damping * gains.damping;
  lu_.compute(JJt_);
  Jpinv_.noalias() =
      J_full_.transpose() * lu_.solve(Eigen::Matrix<double, 6, 6>::Identity());

  // ── Step 7: joint velocity from task-space velocity ───────────────────────
  dq_.noalias() = Jpinv_ * task_vel_;

  // ── Feedforward-only trajectory velocity (for logging) ────────────────
  {
    Eigen::Matrix<double, 6, 1> ff_vel_6d;
    ff_vel_6d.head<3>() = traj_state_.velocity.linear();
    ff_vel_6d.tail<3>() = traj_state_.velocity.angular();
    traj_dq_.noalias() = Jpinv_ * ff_vel_6d;
  }

  // ── Step 8: optional gravity compensation ────────────────────────────────
  if (use_gravity) {
    handle_->ComputeGeneralizedGravity(q_span);
    dq_ += handle_->GetGeneralizedGravity();
  }

  // ── Step 9: clamp joint velocity and integrate ────────────────────────────
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto &out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kTask;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_velocities[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  ClampVelocity(out0.target_velocities, nc0);

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = dev0.positions[i] + out0.target_velocities[i] * dt;
    // Pure trajectory feedforward velocity (without PD error)
    out0.trajectory_velocities[i] = traj_dq_[static_cast<Eigen::Index>(i)];
    // Trajectory-implied joint position = current + feedforward * dt
    out0.trajectory_positions[i] =
        dev0.positions[i] + out0.trajectory_velocities[i] * dt;
  }
  for (std::size_t i = 0; i < 6; ++i) {
    out0.target_positions[i] = pose_target_[i];
    out0.goal_positions[i] = pose_target_[i];
  }

  rtc::utils::PassthroughSecondaryDevices(state, output, device_targets_);

  Eigen::Vector3d rpy_current = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy_current[0];
  output.actual_task_positions[4] = rpy_current[1];
  output.actual_task_positions[5] = rpy_current[2];

  // Task-space goal target
  {
    Eigen::Vector3d goal_rpy =
        pinocchio::rpy::matrixToRpy(goal_pose_.rotation());
    output.task_goal_positions[0] = pose_target_[0];
    output.task_goal_positions[1] = pose_target_[1];
    output.task_goal_positions[2] = pose_target_[2];
    output.task_goal_positions[3] = goal_rpy[0];
    output.task_goal_positions[4] = goal_rpy[1];
    output.task_goal_positions[5] = goal_rpy[2];
  }

  // Task-space trajectory reference
  {
    const Eigen::Vector3d traj_pos = traj_state_.pose.translation();
    Eigen::Vector3d traj_rpy =
        pinocchio::rpy::matrixToRpy(traj_state_.pose.rotation());
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

void OperationalSpaceController::SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices)
    return;
  if (device_idx == 0) {
    std::lock_guard lock(target_mutex_);
    const std::size_t n = std::min(target.size(), std::size_t{6});
    for (std::size_t i = 0; i < n; ++i) {
      pose_target_[i] = target[i];
    }
    if (n >= 6) {
      goal_pose_.translation() =
          Eigen::Vector3d(target[0], target[1], target[2]);
      goal_pose_.rotation() = RpyToMatrix(target[3], target[4], target[5]);
    }
    new_target_.store(true, std::memory_order_release);
  } else {
    const auto ud = static_cast<std::size_t>(device_idx);
    const std::size_t n =
        std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < n; ++i) {
      device_targets_[ud][i] = target[i];
    }
  }
}

void OperationalSpaceController::InitializeHoldPosition(
    const ControllerState &state) noexcept {
  const auto &dev0 = state.devices[0];
  const int nv = handle_->nv();

  std::array<double, kMaxDeviceChannels> q_buf{};
  for (int i = 0; i < nv; ++i) {
    q_buf[static_cast<std::size_t>(i)] =
        dev0.positions[static_cast<std::size_t>(i)];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  handle_->ComputeJacobians(q_span);
  const pinocchio::SE3 &tcp = handle_->GetFramePlacement(tip_frame_id_);

  // try_lock: called from RT path — never block. Skip on contention (retry next
  // tick).
  std::unique_lock lock(target_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
    return;
  goal_pose_ = tcp;

  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
  pose_target_[0] = tcp.translation()[0];
  pose_target_[1] = tcp.translation()[1];
  pose_target_[2] = tcp.translation()[2];
  pose_target_[3] = rpy[0];
  pose_target_[4] = rpy[1];
  pose_target_[5] = rpy[2];
  new_target_.store(false, std::memory_order_relaxed);

  trajectory_.initialize(tcp, pinocchio::Motion::Zero(), tcp,
                         pinocchio::Motion::Zero(), 0.01);
  trajectory_time_ = 0.0;
  has_pending_segment_ = false;

  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices);
       ++d) {
    const auto &dev = state.devices[d];
    if (!dev.valid)
      continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(dev.num_channels) &&
                            i < static_cast<std::size_t>(kMaxDeviceChannels);
         ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
  }
}

std::string_view OperationalSpaceController::Name() const noexcept {
  return "OperationalSpaceController";
}

void OperationalSpaceController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void OperationalSpaceController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  new_target_.store(true, std::memory_order_relaxed);
}

bool OperationalSpaceController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void OperationalSpaceController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput OperationalSpaceController::ComputeEstop(
    const ControllerState &state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const auto &dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto &out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double lim = (i < max_joint_velocity_.size())
                           ? max_joint_velocity_[i]
                           : kDefaultMaxJointVelocity;
    const double sp = (i < safe_position_.size()) ? safe_position_[i] : 0.0;
    out0.commands[i] =
        dev0.positions[i] + std::clamp(sp - dev0.positions[i], -lim, lim) * dt;
  }
  return output;
}

void OperationalSpaceController::ClampVelocity(
    std::array<double, kMaxDeviceChannels> &dq, int n) const noexcept {
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lim = (i < max_joint_velocity_.size())
                           ? max_joint_velocity_[i]
                           : kDefaultMaxJointVelocity;
    dq[i] = std::clamp(dq[i], -lim, lim);
  }
}

Eigen::Matrix3d OperationalSpaceController::RpyToMatrix(double roll,
                                                        double pitch,
                                                        double yaw) noexcept {
  // ZYX Euler convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// ── Controller registry hooks ────────────────────────────────────────────────

void OperationalSpaceController::LoadConfig(const YAML::Node &cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }
  auto g = gains_lock_.Load();
  auto load3 = [](const YAML::Node &n, std::array<double, 3> &arr) {
    if (n && n.IsSequence() && n.size() == 3) {
      for (std::size_t i = 0; i < 3; ++i) {
        arr[i] = n[i].as<double>();
      }
    }
  };
  load3(cfg["kp_pos"], g.kp_pos);
  load3(cfg["kd_pos"], g.kd_pos);
  load3(cfg["kp_rot"], g.kp_rot);
  load3(cfg["kd_rot"], g.kd_rot);
  if (cfg["damping"]) {
    g.damping = cfg["damping"].as<double>();
  }
  if (cfg["enable_gravity_compensation"]) {
    g.enable_gravity_compensation =
        cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["trajectory_angular_speed"]) {
    g.trajectory_angular_speed =
        std::max(1e-6, cfg["trajectory_angular_speed"].as<double>());
  }
  if (cfg["max_traj_velocity"]) {
    g.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    g.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }
  gains_lock_.Store(g);
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ =
        (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

} // namespace rtc
