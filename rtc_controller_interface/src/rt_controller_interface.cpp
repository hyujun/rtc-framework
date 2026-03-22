#include "rtc_controller_interface/rt_controller_interface.hpp"

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
  {"joint_command",     PublishRole::kJointCommand},
  {"ros2_command",      PublishRole::kRos2Command},
  // Logging/Monitoring
  {"task_position",     PublishRole::kTaskPosition},
  {"trajectory_state",  PublishRole::kTrajectoryState},
  {"controller_state",  PublishRole::kControllerState},
  // backward compat
  {"position_command",  PublishRole::kRos2Command},
  {"torque_command",    PublishRole::kRos2Command},
  {"hand_command",      PublishRole::kJointCommand},
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
  cfg.ur5e.subscribe = {
    {"/joint_states",                SubscribeRole::kState},
    {"/ur5e/target_joint_positions", SubscribeRole::kTarget},
  };
  cfg.ur5e.publish = {
    {"/ur5e/joint_command",                   PublishRole::kJointCommand,     kNumRobotJoints},
    {"/forward_position_controller/commands", PublishRole::kRos2Command,      kNumRobotJoints},
    {"/ur5e/current_task_position",           PublishRole::kTaskPosition,     6},
    {"/ur5e/trajectory_state",                PublishRole::kTrajectoryState,  18},
    {"/ur5e/controller_state",                PublishRole::kControllerState,  18},
  };

  // ── hand device group (default topics) ──
  cfg.hand.subscribe = {
    {"/hand/joint_states", SubscribeRole::kState},
  };

  return cfg;
}

TopicConfig RTControllerInterface::ParseTopicConfig(const YAML::Node & topics_node)
{
  // ── Detect deprecated flat format (topics.subscribe exists directly) ──
  if (topics_node["subscribe"] && topics_node["subscribe"].IsSequence()) {
    throw std::runtime_error(
        "Flat topics: format is deprecated. "
        "Migrate to topics.ur5e / topics.hand grouping. "
        "See config/controllers/ examples for the new format.");
  }

  TopicConfig cfg;

  // ── ur5e device group parsing ──
  if (topics_node["ur5e"] && topics_node["ur5e"].IsMap()) {
    ParseDeviceTopicGroup(topics_node["ur5e"], cfg.ur5e);
  }

  // ── hand device group parsing ──
  if (topics_node["hand"] && topics_node["hand"].IsMap()) {
    ParseDeviceTopicGroup(topics_node["hand"], cfg.hand);
  }

  return cfg;
}

void RTControllerInterface::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) { return; }

  // ── Device enable flags (per-controller override) ──
  if (cfg["enable_ur5e"]) {
    per_controller_device_flags_.enable_ur5e = cfg["enable_ur5e"].as<bool>();
  }
  if (cfg["enable_hand"]) {
    per_controller_device_flags_.enable_hand = cfg["enable_hand"].as<bool>();
  }

  // Parse topics section if present; otherwise keep the default topic config.
  if (cfg["topics"]) {
    topic_config_ = ParseTopicConfig(cfg["topics"]);
  }
}

}  // namespace rtc
