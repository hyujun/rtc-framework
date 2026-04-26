#include "rtc_controllers/direct/joint_pd_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"

#include <algorithm>
#include <cstddef>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace rtc {

// ── Constructors ─────────────────────────────────────────────────────────────

JointPDController::JointPDController(std::string_view urdf_path)
    : JointPDController(urdf_path, Gains{}) {}

JointPDController::JointPDController(std::string_view urdf_path, Gains gains)
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
  coriolis_forces_ = Eigen::VectorXd::Zero(nv);
  jacobian_ = Eigen::MatrixXd::Zero(6, nv);

  trajectory_.initialize({}, {}, 0.0);
}

void JointPDController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  if (auto *cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
      max_joint_torque_ = cfg->joint_limits->max_torque;
    }
    if (!cfg->safe_position.empty()) {
      safe_position_ = cfg->safe_position;
    }
  }
  if (max_joint_velocity_.empty()) {
    max_joint_velocity_.assign(kMaxDeviceChannels, kDefaultMaxJointVelocity);
  }
  if (max_joint_torque_.empty()) {
    max_joint_torque_.assign(kMaxDeviceChannels, kDefaultMaxJointTorque);
  }
  if (safe_position_.empty()) {
    safe_position_.assign(kMaxDeviceChannels, 0.0);
  }
}

// ── RTControllerInterface implementation ─────────────────────────────────────

ControllerOutput
JointPDController::Compute(const ControllerState &state) noexcept {
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const auto &dev0 = state.devices[0];

  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  const bool use_gravity = gains.enable_gravity_compensation;
  const bool use_coriolis = gains.enable_coriolis_compensation;

  UpdateDynamics(dev0, gains);

  const int nc0 = dev0.num_channels;

  if (new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      trajectory::JointSpaceTrajectory<kMaxRobotDOF>::State start_state;
      trajectory::JointSpaceTrajectory<kMaxRobotDOF>::State goal_state;

      double max_dist = 0.0;
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
        start_state.positions[i] = dev0.positions[i];
        start_state.velocities[i] = dev0.velocities[i];
        start_state.accelerations[i] = 0.0;

        goal_state.positions[i] = device_targets_[0][i];
        goal_state.velocities[i] = 0.0;
        goal_state.accelerations[i] = 0.0;

        max_dist = std::max(
            max_dist, std::abs(device_targets_[0][i] - dev0.positions[i]));
      }

      const double duration = std::max(0.01, max_dist / gains.trajectory_speed);
      trajectory_.initialize(start_state, goal_state, duration);
      trajectory_time_ = 0.0;
      new_target_.store(false, std::memory_order_relaxed);
    }
  }

  const auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  // PD control + feedforward velocity + optional gravity/Coriolis compensation
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto &out0 = output.devices[0];
  out0.num_channels = nc0;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double e = traj_state.positions[i] - dev0.positions[i];
    const double de = (e - prev_error_[i]) / dt;

    out0.commands[i] = gains.kp[i] * e + gains.kd[i] * de;
    if (command_type_ != CommandType::kTorque) {
      out0.commands[i] += traj_state.velocities[i];
    }

    if (use_gravity) {
      out0.commands[i] += gravity_torques_[i];
    }
    if (use_coriolis) {
      out0.commands[i] += coriolis_forces_[static_cast<Eigen::Index>(i)];
    }

    prev_error_[i] = e;
  }

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = traj_state.positions[i];
    out0.goal_positions[i] = device_targets_[0][i];
    out0.target_velocities[i] = traj_state.velocities[i];
  }

  rtc::utils::PassthroughSecondaryDevices(state, output, device_targets_);

  ClampCommands(out0.commands, nc0, command_type_);

  // TCP pose output (task_positions: [x, y, z, roll, pitch, yaw])
  const pinocchio::SE3 &tcp = handle_->GetFramePlacement(tip_frame_id_);
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.command_type = command_type_;
  return output;
}

void JointPDController::SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices)
    return;
  const std::size_t n =
      std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  if (device_idx == 0) {
    std::lock_guard lock(target_mutex_);
    for (std::size_t i = 0; i < n; ++i) {
      device_targets_[0][i] = target[i];
    }
    new_target_.store(true, std::memory_order_release);
  } else {
    const auto ud = static_cast<std::size_t>(device_idx);
    for (std::size_t i = 0; i < n; ++i) {
      device_targets_[ud][i] = target[i];
    }
  }
}

void JointPDController::InitializeHoldPosition(
    const ControllerState &state) noexcept {
  const auto &dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;
  // try_lock: called from RT path — never block. Skip on contention (retry next
  // tick).
  std::unique_lock lock(target_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
    return;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    device_targets_[0][i] = dev0.positions[i];
  }
  trajectory::JointSpaceTrajectory<kMaxRobotDOF>::State hold_state;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    hold_state.positions[i] = dev0.positions[i];
    hold_state.velocities[i] = 0.0;
    hold_state.accelerations[i] = 0.0;
  }
  trajectory_.initialize(hold_state, hold_state, 0.01);
  trajectory_time_ = 0.0;
  prev_error_ = {};
  new_target_.store(false, std::memory_order_relaxed);

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

// ── E-STOP
// ────────────────────────────────────────────────────────────────────

void JointPDController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void JointPDController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  prev_error_ = {};
  new_target_.store(true, std::memory_order_relaxed);
}

bool JointPDController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void JointPDController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Diagnostic accessors ─────────────────────────────────────────────────────

std::array<double, kMaxRobotDOF>
JointPDController::gravity_torques() const noexcept {
  return gravity_torques_;
}

std::array<double, 3> JointPDController::tcp_position() const noexcept {
  return tcp_position_;
}

// ── Controller registry hooks ────────────────────────────────────────────────

void JointPDController::LoadConfig(const YAML::Node &cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }

  auto g = gains_lock_.Load();
  if (cfg["kp"] && cfg["kp"].IsSequence()) {
    const auto n =
        std::min(cfg["kp"].size(), static_cast<std::size_t>(kMaxRobotDOF));
    for (std::size_t i = 0; i < n; ++i) {
      g.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["kd"] && cfg["kd"].IsSequence()) {
    const auto n =
        std::min(cfg["kd"].size(), static_cast<std::size_t>(kMaxRobotDOF));
    for (std::size_t i = 0; i < n; ++i) {
      g.kd[i] = cfg["kd"][i].as<double>();
    }
  }
  if (cfg["enable_gravity_compensation"]) {
    g.enable_gravity_compensation =
        cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["enable_coriolis_compensation"]) {
    g.enable_coriolis_compensation =
        cfg["enable_coriolis_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  gains_lock_.Store(g);
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ =
        (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void JointPDController::UpdateGainsFromMsg(
    std::span<const double> gains) noexcept {
  // layout: [kp×nv, kd×nv, enable_gravity(0/1), enable_coriolis(0/1),
  // trajectory_speed] where nv = model DOF (e.g. 6 for UR5e)
  const auto nv = static_cast<std::size_t>(handle_->nv());
  if (gains.size() < 2 * nv + 2) {
    return;
  }

  auto g = gains_lock_.Load();
  for (std::size_t i = 0; i < nv; ++i) {
    g.kp[i] = gains[i];
  }
  for (std::size_t i = 0; i < nv; ++i) {
    g.kd[i] = gains[nv + i];
  }
  g.enable_gravity_compensation = gains[2 * nv] > 0.5;
  g.enable_coriolis_compensation = gains[2 * nv + 1] > 0.5;
  if (gains.size() >= 2 * nv + 3) {
    g.trajectory_speed = std::max(1e-6, gains[2 * nv + 2]);
  }
  gains_lock_.Store(g);
}

std::vector<double> JointPDController::GetCurrentGains() const noexcept {
  // layout: [kp×nv, kd×nv, enable_gravity(0/1), enable_coriolis(0/1),
  // trajectory_speed]
  const auto nv = static_cast<std::size_t>(handle_->nv());
  const auto g = gains_lock_.Load();
  std::vector<double> v;
  v.reserve(2 * nv + 3);
  v.insert(v.end(), g.kp.begin(),
           g.kp.begin() + static_cast<std::ptrdiff_t>(nv));
  v.insert(v.end(), g.kd.begin(),
           g.kd.begin() + static_cast<std::ptrdiff_t>(nv));
  v.push_back(g.enable_gravity_compensation ? 1.0 : 0.0);
  v.push_back(g.enable_coriolis_compensation ? 1.0 : 0.0);
  v.push_back(g.trajectory_speed);
  return v;
}

// ── Internal helpers ─────────────────────────────────────────────────────────

ControllerOutput
JointPDController::ComputeEstop(const ControllerState &state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const auto &dev0 = state.devices[0];
  const auto gains = gains_lock_.Load();

  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto &out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double e = safe_position_[i] - dev0.positions[i];
    const double de = (e - prev_error_[i]) / dt;
    out0.commands[i] = gains.kp[i] * e + gains.kd[i] * de;
    prev_error_[i] = e;
  }

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = safe_position_[i];
    out0.goal_positions[i] = safe_position_[i];
  }
  ClampCommands(out0.commands, nc0, command_type_);
  new_target_.store(true, std::memory_order_relaxed);
  return output;
}

void JointPDController::UpdateDynamics(const DeviceState &dev,
                                       const Gains &gains) noexcept {
  const int nv = handle_->nv();
  const std::size_t n = std::min(static_cast<std::size_t>(dev.num_channels),
                                 static_cast<std::size_t>(nv));

  std::array<double, kMaxDeviceChannels> q_buf{};
  std::array<double, kMaxDeviceChannels> v_buf{};
  for (std::size_t i = 0; i < n; ++i) {
    q_buf[i] = dev.positions[i];
    v_buf[i] = dev.velocities[i];
  }
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  std::span<const double> v_span(v_buf.data(), static_cast<std::size_t>(nv));

  // ── Gravity torque g(q) ─────────────────────────────────────────────────
  if (gains.enable_gravity_compensation) {
    handle_->ComputeGeneralizedGravity(q_span);
    const auto &g = handle_->GetGeneralizedGravity();
    for (std::size_t i = 0; i < n; ++i) {
      gravity_torques_[i] = g[static_cast<Eigen::Index>(i)];
    }
  }

  // ── Forward kinematics (FK) ──────────────────────────────────────────────
  handle_->ComputeForwardKinematics(q_span, v_span);

  // ── TCP position cache ───────────────────────────────────────────────────
  const pinocchio::SE3 &tcp = handle_->GetFramePlacement(tip_frame_id_);
  tcp_position_[0] = tcp.translation()[0];
  tcp_position_[1] = tcp.translation()[1];
  tcp_position_[2] = tcp.translation()[2];

  // ── Jacobian ─────────────────────────────────────────────────────────────
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
                            jacobian_);

  // ── Coriolis/centrifugal C(q,v)·v ────────────────────────────────────────
  if (gains.enable_coriolis_compensation) {
    handle_->ComputeCoriolisMatrix(q_span, v_span);
    const auto &C = handle_->GetCoriolisMatrix();
    // Reconstruct v as Eigen vector to compute C * v
    Eigen::Map<const Eigen::VectorXd> v_eigen(v_buf.data(), nv);
    coriolis_forces_.noalias() = C * v_eigen;
  }
}

void JointPDController::ClampCommands(
    std::array<double, kMaxDeviceChannels> &cmds, int n,
    CommandType type) const noexcept {
  const auto &limits =
      (type == CommandType::kTorque) ? max_joint_torque_ : max_joint_velocity_;
  // NOTE: default falls back to kDefaultMaxJointVelocity even for torque mode
  // — preserved here to keep refactor pure. Latent bug; revisit separately.
  rtc::utils::ClampSymmetric(cmds, n, std::span<const double>(limits),
                             kDefaultMaxJointVelocity);
}

} // namespace rtc
