#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rtc {

namespace {
const std::unordered_map<std::string, SubscribeRole> kSubscribeRoleMap = {
    {"state", SubscribeRole::kState},
    {"motor_state", SubscribeRole::kMotorState},
    {"sensor_state", SubscribeRole::kSensorState},
    {"target", SubscribeRole::kTarget},
    // backward compat
    {"joint_state", SubscribeRole::kState},
    {"hand_state", SubscribeRole::kState},
    {"goal", SubscribeRole::kTarget},
};

const std::unordered_map<std::string, PublishRole> kPublishRoleMap = {
    // Control Command
    {"joint_command", PublishRole::kJointCommand},
    {"ros2_command", PublishRole::kRos2Command},
    // GUI / Monitoring
    {"gui_position", PublishRole::kGuiPosition},
    // Topic-based State/Command/Goal/Log
    {"robot_target", PublishRole::kRobotTarget},
    // backward compat
    {"joint_goal", PublishRole::kRobotTarget},
    {"device_state_log", PublishRole::kDeviceStateLog},
    {"device_sensor_log", PublishRole::kDeviceSensorLog},
    {"grasp_state", PublishRole::kGraspState},
    {"tof_snapshot", PublishRole::kToFSnapshot},
    // Digital twin
    {"digital_twin_state", PublishRole::kDigitalTwinState},
    // backward compat
    {"position_command", PublishRole::kRos2Command},
    {"torque_command", PublishRole::kRos2Command},
    {"hand_command", PublishRole::kJointCommand},
};

// Infer DeviceCapability bitmask from subscribe roles.
uint16_t InferCapability(const DeviceTopicGroup &group) {
  uint16_t cap = static_cast<uint16_t>(DeviceCapability::kNone);
  for (const auto &entry : group.subscribe) {
    switch (entry.role) {
    case SubscribeRole::kState:
      cap |= static_cast<uint16_t>(DeviceCapability::kJointState);
      break;
    case SubscribeRole::kMotorState:
      cap |= static_cast<uint16_t>(DeviceCapability::kMotorState);
      break;
    case SubscribeRole::kSensorState:
      cap |= static_cast<uint16_t>(DeviceCapability::kSensorData) |
             static_cast<uint16_t>(DeviceCapability::kInference);
      break;
    case SubscribeRole::kTarget:
      break;
    }
  }
  return cap;
}

// Parse the optional "ownership" field on a subscribe/publish entry.
// Missing → kManager; unknown string → runtime_error.
TopicOwnership ParseOwnership(const YAML::Node &entry) {
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
void ParseDeviceTopicGroup(const YAML::Node &group_node,
                           DeviceTopicGroup &out) {
  if (group_node["subscribe"] && group_node["subscribe"].IsSequence()) {
    for (const auto &entry : group_node["subscribe"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      auto it = kSubscribeRoleMap.find(role_str);
      if (it == kSubscribeRoleMap.end()) {
        throw std::runtime_error("Unknown subscribe role: " + role_str);
      }
      out.subscribe.push_back({topic, it->second, ParseOwnership(entry)});
    }
  }

  if (group_node["publish"] && group_node["publish"].IsSequence()) {
    for (const auto &entry : group_node["publish"]) {
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
      out.publish.push_back(
          {topic, it->second, data_size, ParseOwnership(entry)});
    }
  }

  // Auto-infer capability bitmask from subscribe roles.
  out.capability = InferCapability(out);
}

} // namespace

RTControllerInterface::RTControllerInterface()
    : topic_config_(MakeDefaultTopicConfig("ur5e")) {}

RTControllerInterface::~RTControllerInterface() = default;

// ── Lifecycle hooks: default implementations ─────────────────────────────────
//
// noexcept contract: transitions that touch YAML/allocators must catch all
// exceptions and report FAILURE via CallbackReturn.

RTControllerInterface::CallbackReturn RTControllerInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node &yaml_cfg) noexcept {
  if (!node) {
    return CallbackReturn::FAILURE;
  }
  node_ = std::move(node);
  try {
    LoadConfig(yaml_cfg);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] LoadConfig failed: %s",
                 std::string(Name()).c_str(), e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] LoadConfig failed: unknown exception",
                 std::string(Name()).c_str());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) noexcept {
  node_.reset();
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) noexcept {
  return on_cleanup(previous_state);
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

void RTControllerInterface::SetSystemModelConfig(
    const rtc_urdf_bridge::ModelConfig &config) {
  system_model_config_ = std::make_unique<rtc_urdf_bridge::ModelConfig>(config);
  OnSystemModelConfigSet();
}

const rtc_urdf_bridge::ModelConfig *
RTControllerInterface::GetSystemModelConfig() const noexcept {
  return system_model_config_.get();
}

TopicConfig
RTControllerInterface::MakeDefaultTopicConfig(const std::string &device_name) {
  TopicConfig cfg;
  const std::string ns = "/" + device_name;

  cfg[device_name].subscribe = {
      {"/joint_states", SubscribeRole::kState},
      {ns + "/target_joint_positions", SubscribeRole::kTarget},
  };
  cfg[device_name].publish = {
      {ns + "/joint_command", PublishRole::kJointCommand, 0},
      {"/forward_position_controller/commands", PublishRole::kRos2Command, 0},
      {ns + "/gui_position", PublishRole::kGuiPosition, 0},
      {ns + "/robot_target", PublishRole::kRobotTarget, 0},
      {ns + "/state_log", PublishRole::kDeviceStateLog, 0},
  };
  cfg[device_name].capability = InferCapability(cfg[device_name]);

  return cfg;
}

TopicConfig
RTControllerInterface::ParseTopicConfig(const YAML::Node &topics_node) {
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

void RTControllerInterface::LoadConfig(const YAML::Node &cfg) {
  if (!cfg) {
    return;
  }

  // ── Deprecation warning for removed device enable flags ──
  if (cfg["enable_ur5e"] || cfg["enable_hand"]) {
    std::cerr << "[WARN] enable_ur5e/enable_hand in controller YAML is "
              << "deprecated and ignored. Device activation is now determined "
              << "by the topics section presence.\n";
  }

  // Parse topics section if present; otherwise keep the default topic config.
  if (cfg["topics"]) {
    topic_config_ = ParseTopicConfig(cfg["topics"]);
  }
}

} // namespace rtc
