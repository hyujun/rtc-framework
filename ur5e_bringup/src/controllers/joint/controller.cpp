#include "ur5e_bringup/controllers/demo_joint_controller.hpp"

#include "ur5e_bringup/support/demo_shared_config.hpp"
#include "ur5e_bringup/logging/pod_fill.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

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

DemoJointController::DemoJointController(std::string_view urdf_path, Gains gains)
    : gains_lock_(gains), urdf_path_(urdf_path) {
  // Model is built in LoadConfig() (bridge YAML driven) or InitArmModel().
  // Constructor only stores urdf_path for later use.
}

void DemoJointController::InitArmModel(const rtc_urdf_bridge::ModelConfig& config) {
  namespace rub = rtc_urdf_bridge;
  // Prefer the shared builder injected by RtControllerNode so the URDF is
  // parsed only once across every controller. Fall back to building our own
  // (e.g. for unit tests that bypass CM, or when the shared build failed).
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rub::PinocchioModelBuilder>(config);
  }

  // Resolve sub-model name: match primary device name, fallback to "arm"
  const auto primary = GetPrimaryDeviceName();
  std::string model_name = "arm";
  for (const auto& sm : config.sub_models) {
    if (sm.name == primary) {
      model_name = primary;
      break;
    }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetReducedModel(model_name));
}

// ── Hand tree-model initialization ──────────────────────────────────────────
void DemoJointController::InitHandModel(const rtc_urdf_bridge::ModelConfig& /*config*/) {
  namespace rub = rtc_urdf_bridge;
  // "hand" tree_model: name matches device name
  hand_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetTreeModel("hand"));

  // Set joint reorder mapping: YAML joint_state_names → Pinocchio model order
  if (auto* hand_cfg = GetDeviceNameConfig("hand"); hand_cfg) {
    if (!hand_handle_->SetJointOrder(hand_cfg->joint_state_names)) {
      RCLCPP_WARN(logger_,
                  "DemoJointController: hand SetJointOrder failed — "
                  "joint_state_names not all in Pinocchio model");
    }
  }

  // Resolve fingertip frame IDs from tree_model tip_links
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

  // Pre-allocate hand joint vector
  hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());

  // Initialize position/rotation buffers
  for (auto& p : fingertip_positions_)
    p = Eigen::Vector3d::Zero();
  for (auto& r : fingertip_rotations_)
    r = Eigen::Matrix3d::Identity();
}

void DemoJointController::OnDeviceConfigsSet() {
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
  }
  // Lift L1: per-device joint limits (position + velocity) loaded from
  // device_name_configs_ in topic_config_ group order; missing slots get
  // ±2π / 2 rad/s fallbacks so RT clamping always has valid bounds.
  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             -6.2832, 6.2832, 2.0);

  // Capture joint/sensor names for CSV header expansion (Phase C).
  // Header writers run once at file open in on_configure → these vectors
  // are read non-RT only.
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg) {
    ur5e_joint_names_ = cfg->joint_state_names;
  }
  if (auto* cfg = GetDeviceNameConfig("hand"); cfg) {
    hand_joint_names_ = cfg->joint_state_names;
    hand_motor_names_ = cfg->motor_state_names;
    hand_sensor_names_ = cfg->sensor_names;
  }
}

ControllerOutput DemoJointController::Compute(const ControllerState& state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ReadState(state);
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    // Push minimal logs even in E-STOP so the file shows the gap clearly.
    if (ur5e_state_log_handle_) {
      ur5e::DeviceStateLogPod pod{};
      FillUr5eStateLogPod(state, out, pod);
      ur5e_state_log_handle_.Push(pod);
    }
    if (hand_state_log_handle_) {
      ur5e::DeviceStateLogPod pod{};
      FillHandStateLogPod(state, out, pod);
      hand_state_log_handle_.Push(pod);
    }
    if (hand_sensor_log_handle_) {
      ur5e::DeviceSensorLogPod pod{};
      FillHandSensorLogPod(state, num_active_fingertips_, pod);
      hand_sensor_log_handle_.Push(pod);
    }
    return out;
  }
  ComputeControl(state, dt);
  auto output = WriteOutput(state, dt);

  // ── Phase C: push log PODs to controller-owned channels ────────────────
  // Push site is INSIDE Compute() per Q-ACTIVITY-GATING — inactive
  // controllers' Compute() is never called, so their CSVs do not grow.
  if (ur5e_state_log_handle_) {
    ur5e::DeviceStateLogPod pod{};
    FillUr5eStateLogPod(state, output, pod);
    ur5e_state_log_handle_.Push(pod);
  }
  if (hand_state_log_handle_) {
    ur5e::DeviceStateLogPod pod{};
    FillHandStateLogPod(state, output, pod);
    hand_state_log_handle_.Push(pod);
  }
  if (hand_sensor_log_handle_) {
    ur5e::DeviceSensorLogPod pod{};
    FillHandSensorLogPod(state, num_active_fingertips_, pod);
    hand_sensor_log_handle_.Push(pod);
  }
  return output;
}

// ReadState / UpdateVirtualTcp / ComputeControl / WriteOutput / ComputeEstop
// live in demo_joint_controller_compute.cpp.

void DemoJointController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
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

void DemoJointController::InitializeHoldPosition(const ControllerState& state) noexcept {
  std::lock_guard lock(target_mutex_);

  // Robot
  {
    const auto& dev0 = state.devices[0];
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
  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto& dev = state.devices[d];
    if (!dev.valid)
      continue;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(dev.num_channels) && i < kMaxDeviceChannels; ++i) {
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

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoJointController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }

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
        ament_index_cpp::get_package_share_directory("ur5e_bringup") + "/config/" + yaml_name;
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

  // ── L2: E-STOP arm safe position (required) ──────────────────────────
  {
    const auto sp = ParseArmSafePosition(cfg, kNumRobotJoints, "demo_joint_controller");
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      safe_position_[i] = sp[i];
    }
  }

  auto g = gains_lock_.Load();
  if (cfg["robot_trajectory_speed"]) {
    g.robot_trajectory_speed = std::max(1e-6, cfg["robot_trajectory_speed"].as<double>());
  }
  if (cfg["hand_trajectory_speed"]) {
    g.hand_trajectory_speed = std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
  }
  if (cfg["robot_max_traj_velocity"]) {
    g.robot_max_traj_velocity = cfg["robot_max_traj_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    g.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── FSM tuning (required) ───────────────────────────────────────────
  if (!cfg["fsm"] || !cfg["fsm"].IsMap()) {
    throw std::runtime_error("demo_joint_controller: required 'fsm' section is missing");
  }
  {
    const auto& fsm = cfg["fsm"];
    if (!fsm["contact_stop_release_eps"]) {
      throw std::runtime_error(
          "demo_joint_controller: required 'fsm.contact_stop_release_eps' "
          "is missing");
    }
    const double eps = fsm["contact_stop_release_eps"].as<double>();
    if (!(eps >= 0.0 && eps <= 0.1)) {
      throw std::runtime_error(
          "demo_joint_controller: 'fsm.contact_stop_release_eps' out of range "
          "[0, 0.1]");
    }
    g.contact_stop_release_eps = eps;
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
  finger_joint_map_ = shared.hand_finger_joint_map;
  hand_idx_thumb_cmc_fe_ = static_cast<std::size_t>(shared.hand_idx_thumb_cmc_fe);
  hand_idx_index_mcp_fe_ = static_cast<std::size_t>(shared.hand_idx_index_mcp_fe);
  hand_idx_middle_mcp_fe_ = static_cast<std::size_t>(shared.hand_idx_middle_mcp_fe);

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }

  BuildGraspController(shared, 1.0 / GetDefaultDt(), grasp_controller_);

  // ── Phase C: parse `logs:` section ──────────────────────────────────────
  // Validation only; RegisterLog calls happen in on_configure once
  // device_name_configs_ is populated and joint_names are available.
  parsed_log_entries_.clear();
  if (cfg["logs"]) {
    if (!cfg["logs"].IsSequence()) {
      throw std::runtime_error("DemoJointController: 'logs' must be a sequence");
    }
    for (const auto& entry : cfg["logs"]) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoJointController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      // Closed-set msg_type validation (typo = hard fail at parse time).
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != "rtc_msgs/DeviceSensorLog") {
        throw std::runtime_error("DemoJointController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
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

void DemoJointController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

// on_configure / on_activate / on_deactivate / on_cleanup
//   live in demo_joint_controller_lifecycle.cpp.
// DeclareGainParameters / OnGainParametersSet
//   live in demo_joint_controller_parameters.cpp.

}  // namespace ur5e_bringup
