// ── Includes: project header first, then C++ stdlib
// ────────────────────────────
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_base/utils/clamp_commands.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cmath>  // std::sqrt
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/spatial/log.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Constructor ─────────────────────────────────────────────────────────────

DemoTaskController::DemoTaskController(std::string_view urdf_path, Gains gains)
    : gains_lock_(gains), urdf_path_(urdf_path) {
  // Model is built in LoadConfig() (bridge YAML driven) or InitArmModel().
  // Constructor only stores urdf_path for later use.
}

void DemoTaskController::InitArmModel(const rtc_urdf_bridge::ModelConfig& config) {
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

// ── Hand tree-model initialization ──────────────────────────────────────────
void DemoTaskController::InitHandModel(const rtc_urdf_bridge::ModelConfig& /*config*/) {
  namespace rub = rtc_urdf_bridge;
  // Secondary device name == tree_model name == device_name_configs key
  // (robot-agnostic: e.g. "hand" for ur5e_hand, "leap" for iiwa7_leap).
  const auto secondary = GetSecondaryDeviceName();
  if (secondary.empty()) {
    return;
  }
  hand_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetTreeModel(secondary));

  // Set joint reorder mapping: YAML joint_state_names → Pinocchio model order
  if (auto* hand_cfg = GetDeviceNameConfig(secondary); hand_cfg) {
    if (!hand_handle_->SetJointOrder(hand_cfg->joint_state_names)) {
      RCLCPP_WARN(logger_,
                  "DemoTaskController: secondary device '%s' SetJointOrder failed — "
                  "joint_state_names not all in Pinocchio model",
                  secondary.c_str());
    }
  }

  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg) {
    for (const auto& tm : sys_cfg->tree_models) {
      if (tm.name == secondary) {
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
  for (auto& p : fingertip_positions_)
    p = Eigen::Vector3d::Zero();
  for (auto& r : fingertip_rotations_)
    r = Eigen::Matrix3d::Identity();
}

void DemoTaskController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // ── Runtime DoF resolution ─────────────────────────────────────────────
  // arm_dof_ was set from YAML `estop.arm_safe_position` in LoadConfig.
  // Resolve hand_dof_ from secondary device joint_state_names and
  // cross-validate arm DoF against primary device joint_state_names.
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    const auto js_size = static_cast<int>(cfg->joint_state_names.size());
    if (js_size > 0 && arm_dof_ > 0 && js_size != arm_dof_) {
      RCLCPP_ERROR(logger_,
                   "[task] arm DoF mismatch: estop.arm_safe_position=%d but primary device "
                   "'%s' joint_state_names size=%d",
                   arm_dof_, primary.c_str(), js_size);
    }
  }
  hand_dof_ = 0;
  if (!secondary.empty()) {
    if (auto* cfg = GetDeviceNameConfig(secondary); cfg) {
      hand_dof_ = static_cast<int>(cfg->joint_state_names.size());
    }
  }
  if (hand_dof_ < 0 || hand_dof_ > kDemoTaskMaxHandDof) {
    RCLCPP_ERROR(logger_, "[task] hand DoF %d exceeds capacity kDemoTaskMaxHandDof=%d — clamping",
                 hand_dof_, kDemoTaskMaxHandDof);
    hand_dof_ = std::min(std::max(hand_dof_, 0), kDemoTaskMaxHandDof);
  }

  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
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

  // Phase C: capture joint/sensor names for CSV header expansion.
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    primary_joint_names_ = cfg->joint_state_names;
  }
  if (!secondary.empty()) {
    if (auto* cfg = GetDeviceNameConfig(secondary); cfg) {
      secondary_joint_names_ = cfg->joint_state_names;
      secondary_motor_names_ = cfg->motor_state_names;
      secondary_sensor_names_ = cfg->sensor_names;
    }
  }
}

// ── Virtual TCP computation ─────────────────────────────────────────────────

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput DemoTaskController::Compute(const ControllerState& state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ReadState(state);
  ComputeControl(state, dt);
  auto output = WriteOutput(state, dt);

  // ── Phase C: push log PODs (only from inside Compute()) ──────────────
  if (primary_state_log_handle_) {
    integrated_bringup::DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, /*device_idx=*/0, pod);
    primary_state_log_handle_.Push(pod);
  }
  if (secondary_state_log_handle_) {
    integrated_bringup::DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, /*device_idx=*/1, pod);
    secondary_state_log_handle_.Push(pod);
  }
  if (secondary_sensor_log_handle_) {
    integrated_bringup::DeviceSensorLogPod pod{};
    FillDeviceSensorLogPod(state, /*device_idx=*/1, num_active_fingertips_, pod);
    secondary_sensor_log_handle_.Push(pod);
  }
  return output;
}

// ── Phase 1: Read joint states + sensor data ────────────────────────────────

// ── Phase 2: Compute control (IK/CLIK + trajectory + sensor-based logic) ────

// ── Phase 3: Write output ────────────────────────────────────────────────────

void DemoTaskController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices)
    return;
  if (device_idx == 0) {
    std::lock_guard lock(target_mutex_);
    if (gains_lock_.Load().control_6dof) {
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
      // Layout: target = [tcp_x, tcp_y, tcp_z, null_q_3, null_q_4, ...].
      // Cap total elements at runtime arm_dof_ (kDemoTaskMaxArmDof when
      // arm_dof_ is 0 — pre-LoadConfig path) so null_target_ writes stay
      // in bounds.
      const auto cap = (arm_dof_ > 0) ? static_cast<std::size_t>(arm_dof_)
                                      : static_cast<std::size_t>(kDemoTaskMaxArmDof);
      const std::size_t n = std::min(target.size(), cap);
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

void DemoTaskController::InitializeHoldPosition(const ControllerState& state) noexcept {
  // Fallback DoF when LoadConfig/OnDeviceConfigsSet hasn't populated runtime
  // dimensions (e.g. unit tests that bypass YAML). Derive from device
  // num_channels so the hold/null-target loops still cover active channels.
  if (arm_dof_ == 0 && state.num_devices > 0) {
    arm_dof_ = std::min(state.devices[0].num_channels, kDemoTaskMaxArmDof);
  }
  if (hand_dof_ == 0 && state.num_devices > 1 && state.devices[1].valid) {
    hand_dof_ = std::min(state.devices[1].num_channels, kDemoTaskMaxHandDof);
  }

  const auto& dev0 = state.devices[0];
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
    hand_handle_->ComputeForwardKinematics(std::span<const double>(hand_q_.data(), hand_nq));
    UpdateVirtualTcp(tcp_pose, gains_lock_.Load());
    if (vtcp_valid_) {
      hold_pose = vtcp_pose_;
    }
  }

  // Initialize desired_q_ from current actual joint positions
  if (arm_handle_) {
    for (int i = 0; i < arm_handle_->nv(); ++i) {
      desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
    }
  }

  std::lock_guard lock(target_mutex_);
  tcp_target_pose_ = hold_pose;
  tcp_target_ = {hold_pose.translation()[0], hold_pose.translation()[1],
                 hold_pose.translation()[2]};
  for (int i = 0; i < arm_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    null_target_[idx] = dev0.positions[idx];
  }
  target_initialized_ = true;
  new_target_.store(false, std::memory_order_relaxed);

  trajectory_.initialize(hold_pose, pinocchio::Motion::Zero(), hold_pose, pinocchio::Motion::Zero(),
                         0.01);
  trajectory_time_ = 0.0;
  has_pending_segment_ = false;

  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto& dev = state.devices[d];
    if (!dev.valid)
      continue;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(dev.num_channels) && i < kMaxDeviceChannels; ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
    if (d == 1) {
      trajectory::JointSpaceTrajectory<kDemoTaskMaxHandDof>::State hold_state;
      for (int i = 0; i < hand_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        hold_state.positions[idx] = dev.positions[idx];
        hold_state.velocities[idx] = 0.0;
        hold_state.accelerations[idx] = 0.0;
      }
      hand_trajectory_.initialize(hold_state, hold_state, 0.01);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }
  }
}

std::string_view DemoTaskController::Name() const noexcept {
  return "DemoTaskController";
}

void DemoTaskController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void DemoTaskController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool DemoTaskController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void DemoTaskController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoTaskController::LoadConfig(const YAML::Node& cfg) {
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
        ament_index_cpp::get_package_share_directory("integrated_bringup") + "/config/" + yaml_name;
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

  // ── E-STOP arm safe position (required) ─────────────────────────────
  // Lift L2: estop arm safe position parsing.
  //
  // Runtime arm_dof_ is established from the YAML's `estop.arm_safe_position`
  // length (authoritative at LoadConfig time; device configs arrive later in
  // OnDeviceConfigsSet, which cross-checks joint_state_names size).
  if (!cfg["estop"] || !cfg["estop"]["arm_safe_position"] ||
      !cfg["estop"]["arm_safe_position"].IsSequence()) {
    throw std::runtime_error(
        "demo_task_controller: required 'estop.arm_safe_position' must be a sequence");
  }
  {
    const auto seq_size = cfg["estop"]["arm_safe_position"].size();
    if (seq_size == 0 || seq_size > static_cast<std::size_t>(kDemoTaskMaxArmDof)) {
      throw std::runtime_error("demo_task_controller: 'estop.arm_safe_position' size " +
                               std::to_string(seq_size) + " out of range [1, " +
                               std::to_string(kDemoTaskMaxArmDof) + "]");
    }
    arm_dof_ = static_cast<int>(seq_size);
    const auto sp = ParseArmSafePosition(cfg, seq_size, "demo_task_controller");
    safe_position_.fill(0.0);
    for (std::size_t i = 0; i < seq_size; ++i) {
      safe_position_[i] = sp[i];
    }
  }

  auto g = gains_lock_.Load();
  // CLIK gains — translation / rotation separated
  if (cfg["kp_translation"] && cfg["kp_translation"].IsSequence()) {
    std::size_t n = std::min<std::size_t>(cfg["kp_translation"].size(), 3);
    for (std::size_t i = 0; i < n; ++i) {
      g.kp_translation[i] = cfg["kp_translation"][i].as<double>();
    }
  }
  if (cfg["kp_rotation"] && cfg["kp_rotation"].IsSequence()) {
    std::size_t n = std::min<std::size_t>(cfg["kp_rotation"].size(), 3);
    for (std::size_t i = 0; i < n; ++i) {
      g.kp_rotation[i] = cfg["kp_rotation"][i].as<double>();
    }
  }
  if (cfg["damping"]) {
    g.damping = cfg["damping"].as<double>();
  }
  if (cfg["null_kp"]) {
    g.null_kp = cfg["null_kp"].as<double>();
  }
  if (cfg["enable_null_space"]) {
    g.enable_null_space = cfg["enable_null_space"].as<bool>();
  }
  if (cfg["control_6dof"]) {
    g.control_6dof = cfg["control_6dof"].as<bool>();
  }

  // Trajectory speed
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["trajectory_angular_speed"]) {
    g.trajectory_angular_speed = std::max(1e-6, cfg["trajectory_angular_speed"].as<double>());
  }
  if (cfg["hand_trajectory_speed"]) {
    g.hand_trajectory_speed = std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
  }
  if (cfg["max_traj_velocity"]) {
    g.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    g.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    g.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── FSM / trajectory tuning (required) ──────────────────────────────────
  if (!cfg["fsm"] || !cfg["fsm"].IsMap()) {
    throw std::runtime_error("demo_task_controller: required 'fsm' section is missing");
  }
  {
    const auto& fsm = cfg["fsm"];
    if (!fsm["pi_rotation_margin"]) {
      throw std::runtime_error(
          "demo_task_controller: required 'fsm.pi_rotation_margin' is missing");
    }
    const double pi_margin = fsm["pi_rotation_margin"].as<double>();
    if (!(pi_margin >= 0.0 && pi_margin <= M_PI_2)) {
      throw std::runtime_error(
          "demo_task_controller: 'fsm.pi_rotation_margin' out of range "
          "[0, pi/2]");
    }
    g.pi_rotation_margin = pi_margin;

    if (!fsm["contact_stop_release_eps"]) {
      throw std::runtime_error(
          "demo_task_controller: required 'fsm.contact_stop_release_eps' "
          "is missing");
    }
    const double eps = fsm["contact_stop_release_eps"].as<double>();
    if (!(eps >= 0.0 && eps <= 0.1)) {
      throw std::runtime_error(
          "demo_task_controller: 'fsm.contact_stop_release_eps' out of range "
          "[0, 0.1]");
    }
    g.contact_stop_release_eps = eps;
  }

  // ── Shared params: defaults from demo_shared.yaml, overridden by cfg ──
  // config_variant is declared on the per-controller LifecycleNode by the CM
  // (see rt_controller_node_params.cpp).  Empty → legacy flat layout.
  DemoSharedConfig shared;
  const std::string variant = node_ && node_->has_parameter("config_variant")
                                  ? node_->get_parameter("config_variant").as_string()
                                  : std::string{};
  LoadDemoSharedYamlFile(shared, variant);
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
  parsed_log_entries_.clear();
  if (cfg["logs"]) {
    if (!cfg["logs"].IsSequence()) {
      throw std::runtime_error("DemoTaskController: 'logs' must be a sequence");
    }
    for (const auto& entry : cfg["logs"]) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoTaskController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != "rtc_msgs/DeviceSensorLog") {
        throw std::runtime_error("DemoTaskController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────

void DemoTaskController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

// ── Phase B: gain → ROS 2 parameter declaration & callback ────────────────
//
// Declare every tunable gain as a parameter on the controller's LifecycleNode,
// seeded from the current `gains_lock_` snapshot (which LoadConfig populated
// from YAML). After declare, read each parameter back so any startup overrides
// (--params-file) supersede the YAML defaults, and store the final Gains
// snapshot via gains_lock_.Store() — RT-safe by SeqLock.
//
// Read-only fields (D-2): max_traj_velocity, max_traj_angular_velocity,
// hand_max_traj_velocity. Declared with read_only=true; honour startup
// overrides but reject runtime `ros2 param set`.

}  // namespace integrated_bringup
