#include "rtc_controller_interface/rt_controller_interface.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rtc {

namespace {
// Phase 4: device-wire roles (state / motor_state / sensor_state / joint_command
// / ros2_command) live in `devices.<group>.backend:` (sim.yaml/robot.yaml) and
// are owned by DeviceBackend impls. Controller YAML retains only target /
// robot_target / robot_transforms / digital_twin_state — grasp_state /
// wbc_state / tof_snapshot are now controller-owned via per-controller
// SeqLock and are not declared in YAML.
//
// Phase 4 trailing cleanup: SubscribeRole enum dropped — the only remaining
// value (kTarget) carries no discrimination. The parser still validates the
// `role:` string as documentation + drift detection.
const std::unordered_set<std::string> kSubscribeRoleStrings = {
    "target",
    "goal",  // backward compat
};

const std::unordered_map<std::string, PublishRole> kPublishRoleMap = {
    // Topic-based State/Command/Goal
    {"robot_target", PublishRole::kRobotTarget},
    // backward compat
    {"joint_goal", PublishRole::kRobotTarget},
    {"robot_transforms", PublishRole::kRobotTransforms},
    // Digital twin
    {"digital_twin_state", PublishRole::kDigitalTwinState},
};

// Parse the optional "ownership" field on a subscribe/publish entry.
// Missing → kManager; unknown string → runtime_error.
TopicOwnership ParseOwnership(const YAML::Node& entry) {
  if (!entry["ownership"]) {
    return TopicOwnership::kManager;
  }
  const auto str = entry["ownership"].as<std::string>();
  if (str == "manager") {
    return TopicOwnership::kManager;
  }
  if (str == "controller") {
    return TopicOwnership::kController;
  }
  throw std::runtime_error("Unknown topic ownership: " + str);
}

// Parse subscribe/publish arrays from a YAML device group node (ur5e or hand).
void ParseDeviceTopicGroup(const YAML::Node& group_node, DeviceTopicGroup& out) {
  if (group_node["subscribe"] && group_node["subscribe"].IsSequence()) {
    for (const auto& entry : group_node["subscribe"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      if (kSubscribeRoleStrings.find(role_str) == kSubscribeRoleStrings.end()) {
        throw std::runtime_error("Unknown subscribe role: " + role_str);
      }
      out.subscribe.push_back({topic, ParseOwnership(entry)});
    }
  }

  if (group_node["publish"] && group_node["publish"].IsSequence()) {
    for (const auto& entry : group_node["publish"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      auto it = kPublishRoleMap.find(role_str);
      if (it == kPublishRoleMap.end()) {
        throw std::runtime_error("Unknown publish role: " + role_str);
      }
      int data_size = 0;
      if (entry["data_size"]) {
        data_size = entry["data_size"].as<int>();
      }
      out.publish.push_back({topic, it->second, data_size, ParseOwnership(entry)});
    }
  }
}

}  // namespace

RTControllerInterface::RTControllerInterface() : topic_config_{} {}

RTControllerInterface::~RTControllerInterface() = default;

// ── Lifecycle hooks: default implementations ─────────────────────────────────
//
// noexcept contract: transitions that touch YAML/allocators must catch all
// exceptions and report FAILURE via CallbackReturn.

RTControllerInterface::CallbackReturn RTControllerInterface::PreConfigure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const YAML::Node& yaml_cfg) noexcept {
  if (!node) {
    return CallbackReturn::FAILURE;
  }
  node_ = std::move(node);
  try {
    LoadConfig(yaml_cfg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"), "[%s] LoadConfig failed: %s",
                 std::string(Name()).c_str(), e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"),
                 "[%s] LoadConfig failed: unknown exception", std::string(Name()).c_str());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const YAML::Node& yaml_cfg) noexcept {
  // Idempotent re-entry: when CM drives the 3-pass bring-up, PreConfigure has
  // already stored node_ and parsed yaml_cfg. Direct callers (unit tests,
  // legacy paths) skip PreConfigure, so we still accept the node + reparse
  // here.
  if (!node_) {
    if (!node) {
      return CallbackReturn::FAILURE;
    }
    node_ = std::move(node);
    try {
      LoadConfig(yaml_cfg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"), "[%s] LoadConfig failed: %s",
                   std::string(Name()).c_str(), e.what());
      return CallbackReturn::FAILURE;
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"),
                   "[%s] LoadConfig failed: unknown exception", std::string(Name()).c_str());
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  // Controller-internal init policy: each derived controller seeds its
  // target slot from the current device state on the first Compute() tick
  // after activation. The base has no hold-init responsibility — see the
  // header comment for the rationale (single-writer SeqLock invariant).
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  node_.reset();
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) noexcept {
  return on_cleanup(previous_state);
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

void RTControllerInterface::SetSystemModelConfig(const rtc_urdf_bridge::ModelConfig& config) {
  system_model_config_ = std::make_unique<rtc_urdf_bridge::ModelConfig>(config);
  OnSystemModelConfigSet();
}

const rtc_urdf_bridge::ModelConfig* RTControllerInterface::GetSystemModelConfig() const noexcept {
  return system_model_config_.get();
}

void RTControllerInterface::SetSharedModelBuilder(
    std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder) noexcept {
  shared_model_builder_ = std::move(builder);
}

std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder>
RTControllerInterface::GetSharedModelBuilder() const noexcept {
  return shared_model_builder_;
}

TopicConfig RTControllerInterface::ParseTopicConfig(const YAML::Node& topics_node) {
  // ── Detect deprecated flat format (topics.subscribe exists directly) ──
  if (topics_node["subscribe"] && topics_node["subscribe"].IsSequence()) {
    throw std::runtime_error(
        "Flat topics: format is deprecated. "
        "Migrate to device-group keyed format (e.g. topics.ur5e / "
        "topics.hand). "
        "See config/controllers/ examples for the new format.");
  }

  TopicConfig cfg;

  // ── Dynamic device group parsing: iterate all keys ──
  for (auto it = topics_node.begin(); it != topics_node.end(); ++it) {
    const std::string group_name = it->first.as<std::string>();
    if (!it->second.IsMap()) {
      continue;
    }
    ParseDeviceTopicGroup(it->second, cfg[group_name]);
  }

  return cfg;
}

void RTControllerInterface::DeliverTargetMessage(const std::string& group_name, int device_idx,
                                                 const rtc_msgs::msg::RobotTarget& msg) noexcept {
  const double* data_ptr = nullptr;
  int data_size = 0;

  if (msg.goal_type == "task") {
    data_ptr = msg.task_target.data();
    data_size = static_cast<int>(msg.task_target.size());
  } else {
    if (msg.joint_target.empty()) {
      return;
    }
    data_ptr = msg.joint_target.data();
    data_size = static_cast<int>(msg.joint_target.size());
  }

  std::array<double, kMaxDeviceChannels> reordered{};
  const double* ordered_ptr = data_ptr;
  int ordered_size = data_size;

  if (msg.goal_type != "task" && !msg.joint_names.empty()) {
    auto cfg_it = device_name_configs_.find(group_name);
    if (cfg_it != device_name_configs_.end()) {
      const auto& ref_names = cfg_it->second.joint_state_names;
      if (!ref_names.empty()) {
        for (std::size_t mi = 0; mi < msg.joint_names.size() && mi < msg.joint_target.size();
             ++mi) {
          for (std::size_t ri = 0; ri < ref_names.size(); ++ri) {
            if (msg.joint_names[mi] == ref_names[ri]) {
              reordered[ri] = msg.joint_target[mi];
              break;
            }
          }
        }
        ordered_ptr = reordered.data();
        ordered_size = std::min(static_cast<int>(ref_names.size()), kMaxDeviceChannels);
      }
    }
  }

  const int n = std::min(ordered_size, kMaxDeviceChannels);
  SetDeviceTarget(device_idx, std::span<const double>(ordered_ptr, static_cast<std::size_t>(n)));
}

void RTControllerInterface::LoadDeviceLimitsFromConfig(
    std::array<std::vector<double>, ControllerState::kMaxDevices>& position_lower,
    std::array<std::vector<double>, ControllerState::kMaxDevices>& position_upper,
    std::array<std::vector<double>, ControllerState::kMaxDevices>& max_velocity,
    double default_lower, double default_upper, double default_velocity) const {
  // Walk topic_config_.groups in declaration order so the controller-local
  // device index matches the rest of the controller's data layout
  // (state.devices[i], device_max_velocity_[i], …).
  const std::size_t n_groups =
      std::min<std::size_t>(topic_config_.groups.size(), ControllerState::kMaxDevices);

  for (std::size_t i = 0; i < n_groups; ++i) {
    const auto& [device_name, _group] = topic_config_.groups[i];
    auto cfg_it = device_name_configs_.find(device_name);
    if (cfg_it != device_name_configs_.end() && cfg_it->second.joint_limits) {
      const auto& limits = *cfg_it->second.joint_limits;
      if (!limits.position_lower.empty()) {
        position_lower[i] = limits.position_lower;
      }
      if (!limits.position_upper.empty()) {
        position_upper[i] = limits.position_upper;
      }
      if (!limits.max_velocity.empty()) {
        max_velocity[i] = limits.max_velocity;
      }
    }
  }

  // Fill remaining empty slots (slots without matching DeviceNameConfig, or
  // with empty joint_limits) with caller-supplied fallbacks so RT-path
  // clamping always has valid bounds of length kMaxDeviceChannels.
  for (auto& slot : position_lower) {
    if (slot.empty()) {
      slot.assign(kMaxDeviceChannels, default_lower);
    }
  }
  for (auto& slot : position_upper) {
    if (slot.empty()) {
      slot.assign(kMaxDeviceChannels, default_upper);
    }
  }
  for (auto& slot : max_velocity) {
    if (slot.empty()) {
      slot.assign(kMaxDeviceChannels, default_velocity);
    }
  }
}

std::vector<double> RTControllerInterface::ParseArmSafePosition(
    const YAML::Node& cfg, std::size_t expected_size, const std::string& controller_name) {
  if (!cfg["estop"] || !cfg["estop"].IsMap()) {
    throw std::runtime_error(controller_name + ": required 'estop' section is missing");
  }
  const auto estop_node = cfg["estop"];
  if (!estop_node["arm_safe_position"] || !estop_node["arm_safe_position"].IsSequence()) {
    throw std::runtime_error(controller_name +
                             ": required 'estop.arm_safe_position' must be a sequence");
  }
  const auto safe_seq = estop_node["arm_safe_position"];
  if (safe_seq.size() != expected_size) {
    throw std::runtime_error(controller_name + ": 'estop.arm_safe_position' length " +
                             std::to_string(safe_seq.size()) + " != expected " +
                             std::to_string(expected_size));
  }
  std::vector<double> out;
  out.reserve(expected_size);
  for (std::size_t i = 0; i < expected_size; ++i) {
    out.push_back(safe_seq[i].as<double>());
  }
  return out;
}

void RTControllerInterface::LoadConfig(const YAML::Node& cfg) {
  if (!cfg) {
    return;
  }

  // ── Deprecation warning for removed device enable flags ──
  if (cfg["enable_ur5e"] || cfg["enable_hand"]) {
    std::cerr << "[WARN] enable_ur5e/enable_hand in controller YAML is "
              << "deprecated and ignored. Device activation is now determined "
              << "by the topics section presence.\n";
  }

  // Parse topics section if present; otherwise leave topic_config_ empty.
  // Controllers without YAML topics keep an empty topic_config_; the CM /
  // owned-topics helpers tolerate that (rtc_* stays robot-agnostic — no
  // hardcoded fallback device name lives in the framework).
  if (cfg["topics"]) {
    topic_config_ = ParseTopicConfig(cfg["topics"]);
  }
}

}  // namespace rtc
