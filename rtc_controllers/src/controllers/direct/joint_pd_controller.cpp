#include "rtc_controllers/direct/joint_pd_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

namespace rtc {

// ── Constructors ─────────────────────────────────────────────────────────────

JointPDController::JointPDController(std::string_view urdf_path)
    : JointPDController(urdf_path, Gains{}) {}

JointPDController::JointPDController(std::string_view urdf_path, Gains gains) : gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";

  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  InitFromModel(builder.GetFullModel());

  trajectory_.initialize({}, {}, 0.0);
}

void JointPDController::InitFromModel(std::shared_ptr<const pinocchio::Model> model) {
  model_ptr_ = std::move(model);
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);

  // Default: use the last frame in the model as tip
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);

  const int nv = handle_->nv();
  // Fail-fast capacity check at model-load time (off-RT): the fixed-size
  // gain/prev_error/trajectory buffers are kMaxRobotDOF-wide, so a larger model
  // would overrun them in Compute/UpdateDynamics. Enforcing it here (not only in
  // LoadConfig) guarantees the invariant even when LoadConfig is never called or
  // is handed a null node (issue #172).
  if (nv > kMaxRobotDOF) {
    throw std::runtime_error(
        "JointPDController: model DOF nv=" + std::to_string(nv) +
        " exceeds fixed capacity kMaxRobotDOF=" + std::to_string(kMaxRobotDOF));
  }
  coriolis_forces_ = Eigen::VectorXd::Zero(nv);
  coriolis_pinocchio_ = Eigen::VectorXd::Zero(nv);
  v_pinocchio_ = Eigen::VectorXd::Zero(nv);
  jacobian_ = Eigen::MatrixXd::Zero(6, nv);
}

void JointPDController::MaybeSelectSubModel() {
  const auto* sys = GetSystemModelConfig();
  if (sys == nullptr || sys->urdf_path.empty() || sys->sub_models.empty()) {
    return;  // no system topology → keep the full model built in the ctor
  }
  const auto primary = GetPrimaryDeviceName();
  if (primary.empty()) {
    return;
  }
  const bool matches =
      std::any_of(sys->sub_models.begin(), sys->sub_models.end(),
                  [&](const rtc_urdf_bridge::SubModelConfig& sm) { return sm.name == primary; });
  if (!matches) {
    return;  // primary device is not a declared sub_model → keep full model
  }
  auto builder = GetSharedModelBuilder();
  if (!builder) {
    // No shared builder injected (e.g. unit test) — build our own from the config.
    builder = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(*sys);
  }
  try {
    InitFromModel(builder->GetReducedModel(primary));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("JointPDController"),
                 "[JointPDController] reduced submodel '%s' unavailable (%s) — keeping full model",
                 primary.c_str(), e.what());
  }
}

void JointPDController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
      max_joint_torque_ = cfg->joint_limits->max_torque;
    }
    if (!cfg->safe_position.empty()) {
      safe_position_ = cfg->safe_position;
    }
    // Register device→model joint reorder (#172 A2): when the device declares
    // joint_state_names, the model consumes device-order q/v and maps g/C·v back
    // to device channel order. Identity order → HasJointReorder()==false →
    // zero-overhead passthrough (existing single-order robots unchanged).
    // A size/name mismatch means Compute would read q from the wrong channels;
    // surface it as an error (this hook is not exception-wrapped by the CM; the
    // RT path is additionally bounded by nq=min(nc0,nv) against OOB).
    const auto js = static_cast<int>(cfg->joint_state_names.size());
    const int nv = handle_->nv();
    if (js == nv) {
      if (!handle_->SetJointOrder(cfg->joint_state_names)) {
        RCLCPP_ERROR(rclcpp::get_logger("JointPDController"),
                     "[JointPDController] primary device '%s' joint_state_names not all in model — "
                     "reorder disabled, falling back to positional (URDF-order) consumption",
                     primary.c_str());
      }
    } else if (js > 0) {
      RCLCPP_ERROR(rclcpp::get_logger("JointPDController"),
                   "[JointPDController] primary device '%s' joint_state_names size=%d != model DOF "
                   "nv=%d — FK/dynamics state mapping will be inconsistent",
                   primary.c_str(), js, nv);
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

ControllerOutput JointPDController::Compute(const ControllerState& state) noexcept {
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const auto& dev0 = state.devices[0];

  // Atomic gains snapshot for the whole tick (SeqLock: torn-read-free).
  const auto gains = gains_lock_.Load();
  const bool use_gravity = gains.enable_gravity_compensation;
  const bool use_coriolis = gains.enable_coriolis_compensation;

  UpdateDynamics(dev0, gains);

  const int nc0 = dev0.num_channels;
  // Model-dimension bound. kp/kd/prev_error_ and the trajectory State are
  // kMaxRobotDOF-wide and the dynamics vectors are nv-wide; a device reporting
  // more channels than the model DOF must never index past them (issue #172
  // OOB — the sibling PController guards the same hazard with `nkp`). Channels
  // in [nq, nc0) have no model joint and receive zero torque (safe no-command).
  const std::size_t nq =
      std::min(static_cast<std::size_t>(nc0), static_cast<std::size_t>(handle_->nv()));

  // ── Target slot maintenance (RT thread is the single SeqLock writer) ──
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;

  // First-tick self-init: seed every device's target slot from the current
  // state so subsequent ticks have a coherent baseline.
  if (!target_initialized_.load(std::memory_order_acquire)) {
    for (std::size_t d = 0; d < static_cast<std::size_t>(state.num_devices); ++d) {
      const auto& dev = state.devices[d];
      if (!dev.valid) {
        continue;
      }
      const std::size_t nch = std::min(static_cast<std::size_t>(dev.num_channels),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i) {
        slot.targets[d][i] = dev.positions[i];
      }
    }
    trajectory::JointSpaceTrajectory<kMaxRobotDOF>::State hold_state;
    for (std::size_t i = 0; i < nq; ++i) {
      hold_state.positions[i] = dev0.positions[i];
      hold_state.velocities[i] = 0.0;
      hold_state.accelerations[i] = 0.0;
    }
    trajectory_.initialize(hold_state, hold_state, 0.01);
    trajectory_time_ = 0.0;
    prev_error_ = {};
    new_target_pending_ = false;
    target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
  }

  // Drain SPSC entries pushed by off-RT SetDeviceTarget callers. Device 0
  // updates trigger a trajectory re-init; secondary devices passthrough.
  PendingTarget pending{};
  while (pending_targets_.Pop(pending)) {
    // Drop targets queued before the current activation (#196 §3) — they were
    // addressed to a controller that is no longer the one running.
    if (!IsCurrentGeneration(pending.generation)) {
      continue;
    }
    const auto didx = static_cast<std::size_t>(pending.device_idx);
    if (didx >= ControllerState::kMaxDevices) {
      continue;
    }
    const std::size_t nch = std::min(static_cast<std::size_t>(pending.num_values),
                                     static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < nch; ++i) {
      slot.targets[didx][i] = pending.values[i];
    }
    if (didx == 0) {
      new_target_pending_ = true;
    }
    slot_dirty = true;
  }

  if (new_target_pending_) {
    trajectory::JointSpaceTrajectory<kMaxRobotDOF>::State start_state;
    trajectory::JointSpaceTrajectory<kMaxRobotDOF>::State goal_state;

    double max_dist = 0.0;
    for (std::size_t i = 0; i < nq; ++i) {
      start_state.positions[i] = dev0.positions[i];
      start_state.velocities[i] = dev0.velocities[i];
      start_state.accelerations[i] = 0.0;

      goal_state.positions[i] = slot.targets[0][i];
      goal_state.velocities[i] = 0.0;
      goal_state.accelerations[i] = 0.0;

      max_dist = std::max(max_dist, std::abs(slot.targets[0][i] - dev0.positions[i]));
    }

    const double duration = std::max(0.01, max_dist / gains.trajectory_speed);
    trajectory_.initialize(start_state, goal_state, duration);
    trajectory_time_ = 0.0;
    new_target_pending_ = false;
  }

  if (slot_dirty) {
    target_seqlock_.Store(slot);
  }

  const auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  // PD control + feedforward velocity + optional gravity/Coriolis compensation
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  out0.num_channels = nc0;

  for (std::size_t i = 0; i < nq; ++i) {
    const double e = traj_state.positions[i] - dev0.positions[i];
    const double de = (e - prev_error_[i]) / dt;

    out0.commands[i] = gains.kp[i] * e + gains.kd[i] * de;
    if (command_type_ == CommandType::kTorque) {
      // Torque mode (N·m): PD torque + optional gravity/Coriolis feedforward.
      if (use_gravity) {
        out0.commands[i] += gravity_torques_[i];
      }
      if (use_coriolis) {
        out0.commands[i] += coriolis_forces_[static_cast<Eigen::Index>(i)];
      }
    } else {
      // Position/velocity mode: trajectory velocity feedforward only. N·m
      // dynamics terms (g, C·v) must NOT be mixed into a non-torque command
      // (issue #172): they are gated out here and rejected in LoadConfig.
      out0.commands[i] += traj_state.velocities[i];
    }

    prev_error_[i] = e;
  }
  // Channels past the model DOF (nc0 > nv) carry no PD/dynamics term: zero
  // torque (passive) or hold the current position in a non-torque command.
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = (command_type_ == CommandType::kTorque) ? 0.0 : dev0.positions[i];
  }

  for (std::size_t i = 0; i < nq; ++i) {
    out0.target_positions[i] = traj_state.positions[i];
    out0.goal_positions[i] = slot.targets[0][i];
    out0.target_velocities[i] = traj_state.velocities[i];
  }
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = dev0.positions[i];
    out0.goal_positions[i] = dev0.positions[i];
    out0.target_velocities[i] = 0.0;
  }

  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  ClampCommands(out0.commands, nc0, command_type_);

  // TCP pose output (task_positions: [x, y, z, roll, pitch, yaw])
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);
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

void JointPDController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) {
    return;
  }
  PendingTarget pending{};
  pending.device_idx = device_idx;
  pending.generation = ActivationGeneration();
  const std::size_t nch = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  pending.num_values = static_cast<int>(nch);
  for (std::size_t i = 0; i < nch; ++i) {
    pending.values[i] = target[i];
  }
  // Off-RT marshal — the RT thread drains pending_targets_ inside Compute()
  // and is the SOLE writer of target_seqlock_. SpscQueue::Push is lock-free
  // and uses newest-drop on overflow, so callbacks never block.
  (void)pending_targets_.Push(pending);
}

// ── E-STOP
// ────────────────────────────────────────────────────────────────────

void JointPDController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void JointPDController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  prev_error_ = {};
  // Force the next Compute() to re-seed the target slot + trajectory from the
  // current device state (single-writer invariant preserved — target_seqlock_
  // is still only ever stored by the RT thread).
  target_initialized_.store(false, std::memory_order_release);
}

bool JointPDController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void JointPDController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Diagnostic accessors ─────────────────────────────────────────────────────

std::array<double, kMaxRobotDOF> JointPDController::gravity_torques() const noexcept {
  return gravity_torques_;
}

std::array<double, 3> JointPDController::tcp_position() const noexcept {
  return tcp_position_;
}

// ── Controller registry hooks ────────────────────────────────────────────────

void JointPDController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  // A1 (#172): now that topic_config_ (device group names) + system model config
  // are both available, switch to the primary device's reduced submodel if one
  // is declared. All subsequent gain-length validation uses the submodel nv.
  MaybeSelectSubModel();
  if (!cfg) {
    return;
  }

  // Model DOF (nv <= kMaxRobotDOF is already guaranteed by the constructor's
  // fail-fast capacity check). Used below to validate config array lengths.
  const int nv = handle_->nv();
  auto g = gains_lock_.Load();
  // `kp`/`kd` length must equal the model DOF — no silent truncation (#172).
  if (cfg["kp"]) {
    if (!cfg["kp"].IsSequence() || cfg["kp"].size() != static_cast<std::size_t>(nv)) {
      throw std::runtime_error("JointPDController: 'kp' must be a sequence of length nv=" +
                               std::to_string(nv));
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(nv); ++i) {
      g.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["kd"]) {
    if (!cfg["kd"].IsSequence() || cfg["kd"].size() != static_cast<std::size_t>(nv)) {
      throw std::runtime_error("JointPDController: 'kd' must be a sequence of length nv=" +
                               std::to_string(nv));
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(nv); ++i) {
      g.kd[i] = cfg["kd"][i].as<double>();
    }
  }
  if (cfg["enable_gravity_compensation"]) {
    g.enable_gravity_compensation = cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["enable_coriolis_compensation"]) {
    g.enable_coriolis_compensation = cfg["enable_coriolis_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
  // Dynamics compensation (g, C·v) produces N·m and is only valid on a torque
  // command. Reject the mixed-unit configuration fail-fast rather than silently
  // dropping the terms at runtime (issue #172).
  if (command_type_ != CommandType::kTorque &&
      (g.enable_gravity_compensation || g.enable_coriolis_compensation)) {
    throw std::runtime_error(
        "JointPDController: enable_gravity_compensation / enable_coriolis_compensation require "
        "command_type: torque (dynamics compensation is N·m and cannot be added to a "
        "position/velocity command)");
  }
  gains_lock_.Store(g);
}

// ── Internal helpers ─────────────────────────────────────────────────────────

ControllerOutput JointPDController::ComputeEstop(const ControllerState& state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();
  const auto& dev0 = state.devices[0];
  const auto gains = gains_lock_.Load();

  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  // Same model-dimension bound as Compute(): kp/kd/prev_error_ are
  // kMaxRobotDOF-wide, and safe_position_ can be a config-supplied vector
  // shorter than nc0 — both must be indexed within bounds on the E-STOP path
  // (issue #172; CLIK guards the identical safe_position_ access).
  const std::size_t nq =
      std::min(static_cast<std::size_t>(nc0), static_cast<std::size_t>(handle_->nv()));

  for (std::size_t i = 0; i < nq; ++i) {
    const double sp = (i < safe_position_.size()) ? safe_position_[i] : 0.0;
    const double e = sp - dev0.positions[i];
    const double de = (e - prev_error_[i]) / dt;
    out0.commands[i] = gains.kp[i] * e + gains.kd[i] * de;
    prev_error_[i] = e;
  }
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = (command_type_ == CommandType::kTorque) ? 0.0 : dev0.positions[i];
  }

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double sp = (i < safe_position_.size()) ? safe_position_[i] : dev0.positions[i];
    out0.target_positions[i] = sp;
    out0.goal_positions[i] = sp;
  }
  ClampCommands(out0.commands, nc0, command_type_);
  // After E-STOP returns to normal flow we re-trigger self-init so the
  // trajectory matches whatever state the robot is in (single-writer-safe:
  // RT thread itself sets this).
  target_initialized_.store(false, std::memory_order_release);
  return output;
}

void JointPDController::UpdateDynamics(const DeviceState& dev, const Gains& gains) noexcept {
  const int nv = handle_->nv();
  const std::size_t n =
      std::min(static_cast<std::size_t>(dev.num_channels), static_cast<std::size_t>(nv));

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
    // g(q) is in Pinocchio order; scatter it back to device channel order so
    // out0.commands[i] (device i) receives its own joint's gravity (#172 A2).
    // Identity order → memcpy, so gravity_torques_ is unchanged for single-order
    // robots (and the gravity_torques() diagnostic stays device-indexed).
    handle_->ReorderOutput(handle_->GetGeneralizedGravity(),
                           std::span<double>(gravity_torques_.data(), n));
  }

  // ── Forward kinematics (FK) ──────────────────────────────────────────────
  handle_->ComputeForwardKinematics(q_span, v_span);

  // ── TCP position cache ───────────────────────────────────────────────────
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);
  tcp_position_[0] = tcp.translation()[0];
  tcp_position_[1] = tcp.translation()[1];
  tcp_position_[2] = tcp.translation()[2];

  // ── Jacobian ─────────────────────────────────────────────────────────────
  handle_->ComputeJacobians(q_span);
  handle_->GetFrameJacobian(tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

  // ── Coriolis/centrifugal C(q,v)·v ────────────────────────────────────────
  if (gains.enable_coriolis_compensation) {
    handle_->ComputeCoriolisMatrix(q_span, v_span);
    const auto& C = handle_->GetCoriolisMatrix();
    // C is Pinocchio-order (nv×nv). Gather v to Pinocchio order for the product,
    // then scatter C·v back to device channel order so out0.commands[i] gets its
    // own joint's Coriolis term (#172 A2). Identity order → both are memcpy.
    handle_->ReorderInput(v_span, v_pinocchio_);
    coriolis_pinocchio_.noalias() = C * v_pinocchio_;
    handle_->ReorderOutput(coriolis_pinocchio_, std::span<double>(coriolis_forces_.data(),
                                                                  static_cast<std::size_t>(nv)));
  }
}

void JointPDController::ClampCommands(std::array<double, kMaxDeviceChannels>& cmds, int n,
                                      CommandType type) const noexcept {
  const auto& limits = (type == CommandType::kTorque) ? max_joint_torque_ : max_joint_velocity_;
  // NOTE: default falls back to kDefaultMaxJointVelocity even for torque mode
  // — preserved here to keep refactor pure. Latent bug; revisit separately.
  rtc::utils::ClampSymmetric(cmds, n, std::span<const double>(limits), kDefaultMaxJointVelocity);
}

}  // namespace rtc
