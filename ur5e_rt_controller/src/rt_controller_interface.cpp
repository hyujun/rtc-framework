#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace ur5e_rt_controller
{

namespace
{
const std::unordered_map<std::string, SubscribeRole> kSubscribeRoleMap = {
  {"joint_state",  SubscribeRole::kJointState},
  {"hand_state",   SubscribeRole::kHandState},
  {"target",       SubscribeRole::kTarget},
};

const std::unordered_map<std::string, PublishRole> kPublishRoleMap = {
  {"position_command", PublishRole::kPositionCommand},
  {"torque_command",   PublishRole::kTorqueCommand},
  {"hand_command",     PublishRole::kHandCommand},
  {"task_position",    PublishRole::kTaskPosition},
};
}  // namespace

RTControllerInterface::RTControllerInterface()
: topic_config_(MakeDefaultTopicConfig())
{}

TopicConfig RTControllerInterface::MakeDefaultTopicConfig()
{
  TopicConfig cfg;
  cfg.subscribe = {
    {"/joint_states",           SubscribeRole::kJointState},
    {"/hand/joint_states",      SubscribeRole::kHandState},
    {"/target_joint_positions", SubscribeRole::kTarget},
  };
  cfg.publish = {
    {"/forward_position_controller/commands", PublishRole::kPositionCommand, kNumRobotJoints},
    {"/forward_torque_controller/commands",   PublishRole::kTorqueCommand,   kNumRobotJoints},
    {"/rt_controller/current_task_position",  PublishRole::kTaskPosition,    6},
  };
  return cfg;
}

TopicConfig RTControllerInterface::ParseTopicConfig(const YAML::Node & topics_node)
{
  TopicConfig cfg;

  if (topics_node["subscribe"] && topics_node["subscribe"].IsSequence()) {
    for (const auto & entry : topics_node["subscribe"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      auto it = kSubscribeRoleMap.find(role_str);
      if (it == kSubscribeRoleMap.end()) {
        throw std::runtime_error("Unknown subscribe role: " + role_str);
      }
      cfg.subscribe.push_back({topic, it->second});
    }
  }

  if (topics_node["publish"] && topics_node["publish"].IsSequence()) {
    for (const auto & entry : topics_node["publish"]) {
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
      cfg.publish.push_back({topic, it->second, data_size});
    }
  }

  return cfg;
}

void RTControllerInterface::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) { return; }

  // Parse topics section if present; otherwise keep the default topic config.
  if (cfg["topics"]) {
    topic_config_ = ParseTopicConfig(cfg["topics"]);
  }
}

}  // namespace ur5e_rt_controller
