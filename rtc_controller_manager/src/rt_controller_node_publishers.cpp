// ── Publisher creation + topic-parameter exposure ───────────────────────────
//
// CM owns exactly two manager-owned publish roles (kJointCommand and
// kRos2Command) plus three fixed publishers (digital twin republish per
// active group, /system/estop_status, /rtc_cm/active_controller_name).
// Controller-output roles (kRobotTarget / kGraspState / kWbcState /
// kToFSnapshot / kRobotTransforms) are owned by each controller's LifecycleNode.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace urtc = rtc;

namespace {

// Build a reorder map from controller command order (`cmd_names`) to device
// state order (`state_names`). Returns an empty vector when no reorder is
// needed — both lists empty, or already aligned. Used by kJointCommand and
// kRos2Command publishers; the empty-vector convention is checked by the RT
// publish path to skip the reorder step.
std::vector<int> BuildPublisherReorderMap(const std::vector<std::string>& state_names,
                                          const std::vector<std::string>& cmd_names) {
  if (state_names.empty() || cmd_names.empty() || state_names == cmd_names) {
    return {};
  }
  std::vector<int> map(cmd_names.size(), -1);
  for (std::size_t ci = 0; ci < cmd_names.size(); ++ci) {
    for (std::size_t si = 0; si < state_names.size(); ++si) {
      if (cmd_names[ci] == state_names[si]) {
        map[ci] = static_cast<int>(si);
        break;
      }
    }
  }
  return map;
}

}  // namespace

void RtControllerNode::CreatePublishers() {
  // BEST_EFFORT + depth 1: minimises DDS overhead on the RT path.
  rclcpp::QoS cmd_qos{1};
  cmd_qos.best_effort();

  // CM owns exactly two publish roles — kJointCommand (controller→HW/sim) and
  // kRos2Command (sim ros2_control forward bridge). Any other role with
  // ownership: manager means the YAML mistakenly assigned a controller-output
  // role (kGraspState / kWbcState / kToFSnapshot / kRobotTarget /
  // kRobotTransforms) to CM; the default branch fails fast so the
  // misconfiguration surfaces immediately instead of dropping the publisher
  // silently.
  for (const auto& tc : controller_topic_configs_) {
    for (const auto& [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;
      for (const auto& entry : group.publish) {
        // Controller-owned publishers are created by the controller's own
        // on_configure — CM does not bind them here.
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        switch (entry.role) {
          case urtc::PublishRole::kJointCommand:
            CreateJointCommandPublisher(entry, group_name, cmd_qos);
            break;
          case urtc::PublishRole::kRos2Command:
            CreateRos2CommandPublisher(entry, group_name);
            break;
          default:
            throw std::logic_error(
                std::string("CreatePublishers: role '") + urtc::PublishRoleToString(entry.role) +
                "' for topic '" + entry.topic_name + "' (group '" + group_name +
                "') is controller-owned only; set ownership: controller in the YAML "
                "or remove the entry.");
        }
      }
    }
  }

  CreateDigitalTwinPublishers();
  CreateFixedSafetyPublishers();
}

void RtControllerNode::CreateJointCommandPublisher(const urtc::PublishTopicEntry& entry,
                                                   const std::string& group_name,
                                                   const rclcpp::QoS& cmd_qos) {
  if (joint_command_publishers_.count(entry.topic_name) > 0) {
    return;
  }
  JointCommandPublisherEntry jce;
  jce.publisher = create_publisher<rtc_msgs::msg::JointCommand>(entry.topic_name, cmd_qos);
  if (auto cfg_it = device_name_configs_.find(group_name); cfg_it != device_name_configs_.end()) {
    const auto& state_names = cfg_it->second.joint_state_names;
    const auto& cmd_names = cfg_it->second.joint_command_names;
    jce.msg.joint_names = cmd_names;
    jce.msg.values.resize(cmd_names.size(), 0.0);
    jce.reorder_map = BuildPublisherReorderMap(state_names, cmd_names);
    if (!jce.reorder_map.empty()) {
      RCLCPP_INFO(get_logger(), "  [%s] kJointCommand reorder map built (%zu → %zu)",
                  group_name.c_str(), state_names.size(), cmd_names.size());
    }
  }
  jce.msg.command_type = "position";
  joint_command_publishers_[entry.topic_name] = std::move(jce);
  RCLCPP_INFO(get_logger(), "  Publish [%s/joint_command]: %s (JointCommand)", group_name.c_str(),
              entry.topic_name.c_str());
}

void RtControllerNode::CreateRos2CommandPublisher(const urtc::PublishTopicEntry& entry,
                                                  const std::string& group_name) {
  if (ros2_command_publishers_.count(entry.topic_name) > 0) {
    return;
  }
  int data_size = entry.data_size;
  if (data_size <= 0) {
    auto cfg_it = device_name_configs_.find(group_name);
    data_size = (cfg_it != device_name_configs_.end())
                    ? static_cast<int>(cfg_it->second.joint_command_names.size())
                    : rtc::kMaxRobotDOF;
  }

  rclcpp::QoS ros2_cmd_qos{1};
  ros2_cmd_qos.reliable();

  Ros2CommandPublisherEntry pe;
  pe.publisher = create_publisher<std_msgs::msg::Float64MultiArray>(entry.topic_name, ros2_cmd_qos);
  pe.msg.data.resize(static_cast<std::size_t>(data_size), 0.0);

  if (auto cfg_it = device_name_configs_.find(group_name); cfg_it != device_name_configs_.end()) {
    const auto& state_names = cfg_it->second.joint_state_names;
    const auto& cmd_names = cfg_it->second.joint_command_names;
    pe.reorder_map = BuildPublisherReorderMap(state_names, cmd_names);
    if (!pe.reorder_map.empty()) {
      RCLCPP_INFO(get_logger(), "  [%s] kRos2Command reorder map built (%zu → %zu)",
                  group_name.c_str(), state_names.size(), cmd_names.size());
    }
  }

  ros2_command_publishers_[entry.topic_name] = std::move(pe);
  RCLCPP_INFO(get_logger(), "  Publish [%s/ros2_command]: %s (size=%d)", group_name.c_str(),
              entry.topic_name.c_str(), data_size);
}

void RtControllerNode::CreateDigitalTwinPublishers() {
  rclcpp::QoS dt_qos{10};
  dt_qos.reliable();
  for (const auto& [group_name, slot] : group_slot_map_) {
    std::string dt_topic = "/rtc_cm/" + group_name + "/joint_states";
    DigitalTwinEntry dte;
    dte.publisher = create_publisher<sensor_msgs::msg::JointState>(dt_topic, dt_qos);
    auto cfg_it = device_name_configs_.find(group_name);
    if (cfg_it != device_name_configs_.end()) {
      const auto& names = cfg_it->second.joint_state_names;
      dte.msg.name.assign(names.begin(), names.end());
      dte.msg.position.resize(names.size(), 0.0);
      dte.msg.velocity.resize(names.size(), 0.0);
      dte.msg.effort.resize(names.size(), 0.0);
    }
    digital_twin_publishers_[dt_topic] = std::move(dte);
    slot_to_dt_topic_[slot] = dt_topic;
    RCLCPP_INFO(get_logger(), "  CM JointState publish: %s (RELIABLE/10)", dt_topic.c_str());
  }
}

void RtControllerNode::CreateFixedSafetyPublishers() {
  // Use standalone rclcpp::create_publisher so these remain regular
  // rclcpp::Publisher (not LifecyclePublisher) — E-STOP status and active
  // controller name must be publishable regardless of lifecycle state.
  estop_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
      this->get_node_topics_interface(), "/system/estop_status", rclcpp::QoS(10));

  rclcpp::QoS latch_qos{1};
  latch_qos.transient_local();
  // /rtc_cm/active_controller_name — owned by rtc_controller_manager
  // (single-CM assumption per locked decision D-A2). Topic name is fixed so
  // the ownership is obvious from its name; no robot-level prefix.
  active_ctrl_name_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
      this->get_node_topics_interface(), "/rtc_cm/active_controller_name", latch_qos);
}

// ── Expose topic configuration as read-only ROS2 parameters ─────────────────
void RtControllerNode::ExposeTopicParameters() {
  for (std::size_t i = 0; i < controllers_.size(); ++i) {
    const auto& tc = controller_topic_configs_[i];
    const std::string prefix = "controllers." + std::string(controllers_[i]->Name());

    for (const auto& [group_name, group] : tc.groups) {
      for (const auto& entry : group.subscribe) {
        const std::string param_name =
            prefix + "." + group_name + ".subscribe." + urtc::SubscribeRoleToString(entry.role);
        if (!has_parameter(param_name)) {
          declare_parameter(param_name, entry.topic_name);
        }
      }
      for (const auto& entry : group.publish) {
        const std::string param_name =
            prefix + "." + group_name + ".publish." + urtc::PublishRoleToString(entry.role);
        if (!has_parameter(param_name)) {
          declare_parameter(param_name, entry.topic_name);
        }
      }
    }
  }

  // ── Read-only guard: reject any runtime mutation of topic parameters ────
  param_callback_handle_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto& p : params) {
          if (p.get_name().rfind("controllers.", 0) == 0) {
            result.successful = false;
            result.reason = "Topic parameters are read-only after initialisation";
            return result;
          }
        }
        result.successful = true;
        return result;
      });

  RCLCPP_INFO(get_logger(),
              "Topic parameters exposed (read-only) — use 'ros2 "
              "param list' to inspect");
}
