#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rtc
{

namespace
{
const std::unordered_map<std::string, SubscribeRole> kSubscribeRoleMap = {
  {"state",         SubscribeRole::kState},
  {"sensor_state",  SubscribeRole::kSensorState},
  {"target",        SubscribeRole::kTarget},
  // backward compat
  {"joint_state",   SubscribeRole::kState},
  {"hand_state",    SubscribeRole::kState},
  {"goal",          SubscribeRole::kTarget},
};

const std::unordered_map<std::string, PublishRole> kPublishRoleMap = {
  // Control Command
  {"joint_command",      PublishRole::kJointCommand},
  {"ros2_command",       PublishRole::kRos2Command},
  // GUI / Monitoring
  {"gui_position",       PublishRole::kGuiPosition},
  // Topic-based State/Command/Goal/Log
  {"robot_target",       PublishRole::kRobotTarget},
  // backward compat
  {"joint_goal",         PublishRole::kRobotTarget},
  {"device_state_log",   PublishRole::kDeviceStateLog},
  {"device_sensor_log",  PublishRole::kDeviceSensorLog},
  // backward compat
  {"position_command",   PublishRole::kRos2Command},
  {"torque_command",     PublishRole::kRos2Command},
  {"hand_command",       PublishRole::kJointCommand},
};

// Parse subscribe/publish arrays from a YAML device group node (ur5e or hand).
void ParseDeviceTopicGroup(
    const YAML::Node & group_node,
    DeviceTopicGroup & out)
{
  if (group_node["subscribe"] && group_node["subscribe"].IsSequence()) {
    for (const auto & entry : group_node["subscribe"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      auto it = kSubscribeRoleMap.find(role_str);
      if (it == kSubscribeRoleMap.end()) {
        throw std::runtime_error("Unknown subscribe role: " + role_str);
      }
      out.subscribe.push_back({topic, it->second});
    }
  }

  if (group_node["publish"] && group_node["publish"].IsSequence()) {
    for (const auto & entry : group_node["publish"]) {
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
      out.publish.push_back({topic, it->second, data_size});
    }
  }
}

}  // namespace

RTControllerInterface::RTControllerInterface()
: topic_config_(MakeDefaultTopicConfig())
{}

TopicConfig RTControllerInterface::MakeDefaultTopicConfig()
{
  TopicConfig cfg;

  // ── ur5e device group (default topics) ──
  cfg["ur5e"].subscribe = {
    {"/joint_states",                SubscribeRole::kState},
    {"/ur5e/target_joint_positions", SubscribeRole::kTarget},
  };
  cfg["ur5e"].publish = {
    {"/ur5e/joint_command",                   PublishRole::kJointCommand,     kNumRobotJoints},
    {"/forward_position_controller/commands", PublishRole::kRos2Command,      kNumRobotJoints},
    {"/ur5e/gui_position",                    PublishRole::kGuiPosition,      0},
    {"/ur5e/robot_target",                    PublishRole::kRobotTarget,      0},
    {"/ur5e/state_log",                       PublishRole::kDeviceStateLog,   0},
  };

  // hand is NOT included by default — add via YAML topics section to activate.

  return cfg;
}

TopicConfig RTControllerInterface::ParseTopicConfig(const YAML::Node & topics_node)
{
  // ── Detect deprecated flat format (topics.subscribe exists directly) ──
  if (topics_node["subscribe"] && topics_node["subscribe"].IsSequence()) {
    throw std::runtime_error(
        "Flat topics: format is deprecated. "
        "Migrate to device-group keyed format (e.g. topics.ur5e / topics.hand). "
        "See config/controllers/ examples for the new format.");
  }

  TopicConfig cfg;

  // ── Dynamic device group parsing: iterate all keys ──
  for (auto it = topics_node.begin(); it != topics_node.end(); ++it) {
    const std::string group_name = it->first.as<std::string>();
    if (!it->second.IsMap()) { continue; }
    ParseDeviceTopicGroup(it->second, cfg[group_name]);
  }

  return cfg;
}

void RTControllerInterface::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) { return; }

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

}  // namespace rtc
