#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rtc
{

namespace
{
const std::unordered_map<std::string, SubscribeRole> kSubscribeRoleMap = {
  {"joint_state",  SubscribeRole::kJointState},
  {"hand_state",   SubscribeRole::kHandState},
  {"goal",         SubscribeRole::kGoal},
  {"target",       SubscribeRole::kGoal},       // backward compat
};

const std::unordered_map<std::string, PublishRole> kPublishRoleMap = {
  // Category 3: Control Command
  {"position_command",  PublishRole::kPositionCommand},
  {"torque_command",    PublishRole::kTorqueCommand},
  {"hand_command",      PublishRole::kHandCommand},
  // Category 4: Logging/Monitoring
  {"task_position",     PublishRole::kTaskPosition},
  {"trajectory_state",  PublishRole::kTrajectoryState},
  {"controller_state",  PublishRole::kControllerState},
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
    {"/joint_states",                SubscribeRole::kJointState},
    {"/ur5e/target_joint_positions", SubscribeRole::kGoal},
  };
  cfg.ur5e.publish = {
    {"/forward_position_controller/commands", PublishRole::kPositionCommand,  kNumRobotJoints},
    {"/forward_torque_controller/commands",   PublishRole::kTorqueCommand,    kNumRobotJoints},
    {"/ur5e/current_task_position",           PublishRole::kTaskPosition,     6},
    {"/ur5e/trajectory_state",                PublishRole::kTrajectoryState,  18},
    {"/ur5e/controller_state",                PublishRole::kControllerState,  18},
  };

  // ── hand device group (default topics) ──
  cfg.hand.subscribe = {
    {"/hand/joint_states", SubscribeRole::kHandState},
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
