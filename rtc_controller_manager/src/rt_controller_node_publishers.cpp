// ── Publisher creation + topic-parameter exposure ───────────────────────────
//
// CM owns exactly two manager-owned publish roles (kJointCommand and
// kRos2Command) plus three fixed publishers (digital twin republish per
// active group, /system/estop_status, /rtc_cm/active_controller_name).
// Controller-output roles (kGuiPosition / kRobotTarget / kGraspState /
// kWbcState / kToFSnapshot) are owned by each controller's LifecycleNode.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <stdexcept>
#include <string>

namespace urtc = rtc;

void RtControllerNode::CreatePublishers() {
  // BEST_EFFORT + depth 1: minimises DDS overhead on the RT path.
  rclcpp::QoS cmd_qos{1};
  cmd_qos.best_effort();

  // Helper: create a publisher for a manager-owned publish entry. CM owns
  // exactly two publish roles — kJointCommand (controller→HW/sim) and
  // kRos2Command (sim ros2_control forward bridge). Any other role here
  // means the YAML mistakenly set ownership: manager on a controller-output
  // role (kGuiPosition / kGraspState / kWbcState / kToFSnapshot /
  // kRobotTarget); fail-fast so the misconfiguration surfaces immediately
  // instead of dropping the publisher silently.
  auto create_pub = [&](const urtc::PublishTopicEntry& entry, const std::string& group_name) {
    switch (entry.role) {
      case urtc::PublishRole::kJointCommand: {
        if (joint_command_publishers_.count(entry.topic_name) > 0) {
          return;
        }
        JointCommandPublisherEntry jce;
        jce.publisher = create_publisher<rtc_msgs::msg::JointCommand>(entry.topic_name, cmd_qos);
        {
          auto cfg_it = device_name_configs_.find(group_name);
          if (cfg_it != device_name_configs_.end()) {
            const auto& state_names = cfg_it->second.joint_state_names;
            const auto& cmd_names = cfg_it->second.joint_command_names;
            jce.msg.joint_names = cmd_names;
            jce.msg.values.resize(cmd_names.size(), 0.0);
            if (!state_names.empty() && !cmd_names.empty() && state_names != cmd_names) {
              jce.reorder_map.resize(cmd_names.size(), -1);
              for (std::size_t ci = 0; ci < cmd_names.size(); ++ci) {
                for (std::size_t si = 0; si < state_names.size(); ++si) {
                  if (cmd_names[ci] == state_names[si]) {
                    jce.reorder_map[ci] = static_cast<int>(si);
                    break;
                  }
                }
              }
              RCLCPP_INFO(get_logger(), "  [%s] kJointCommand reorder map built (%zu → %zu)",
                          group_name.c_str(), state_names.size(), cmd_names.size());
            }
          }
        }
        jce.msg.command_type = "position";
        joint_command_publishers_[entry.topic_name] = std::move(jce);
        RCLCPP_INFO(get_logger(), "  Publish [%s/joint_command]: %s (JointCommand)",
                    group_name.c_str(), entry.topic_name.c_str());
        return;
      }
      case urtc::PublishRole::kRos2Command: {
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
        pe.publisher =
            create_publisher<std_msgs::msg::Float64MultiArray>(entry.topic_name, ros2_cmd_qos);
        pe.msg.data.resize(static_cast<std::size_t>(data_size), 0.0);

        auto cfg_it = device_name_configs_.find(group_name);
        if (cfg_it != device_name_configs_.end()) {
          const auto& state_names = cfg_it->second.joint_state_names;
          const auto& cmd_names = cfg_it->second.joint_command_names;
          if (!state_names.empty() && !cmd_names.empty() && state_names != cmd_names) {
            pe.reorder_map.resize(cmd_names.size(), -1);
            for (std::size_t ci = 0; ci < cmd_names.size(); ++ci) {
              for (std::size_t si = 0; si < state_names.size(); ++si) {
                if (cmd_names[ci] == state_names[si]) {
                  pe.reorder_map[ci] = static_cast<int>(si);
                  break;
                }
              }
            }
            RCLCPP_INFO(get_logger(), "  [%s] kRos2Command reorder map built (%zu → %zu)",
                        group_name.c_str(), state_names.size(), cmd_names.size());
          }
        }

        ros2_command_publishers_[entry.topic_name] = std::move(pe);
        RCLCPP_INFO(get_logger(), "  Publish [%s/ros2_command]: %s (size=%d)", group_name.c_str(),
                    entry.topic_name.c_str(), data_size);
        return;
      }
      default:
        break;
    }
    // Controller-output role with ownership: manager — misconfiguration.
    throw std::logic_error(std::string("CreatePublishers: role '") +
                           urtc::PublishRoleToString(entry.role) + "' for topic '" +
                           entry.topic_name + "' (group '" + group_name +
                           "') is controller-owned only; set ownership: controller in the YAML "
                           "or remove the entry.");
  };

  // ── Create publishers for all active device groups ────────────────────────
  for (const auto& tc : controller_topic_configs_) {
    for (const auto& [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;
      for (const auto& entry : group.publish) {
        // Controller-owned publishers are created by the controller's own
        // on_configure — CM does not bind them here.
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        create_pub(entry, group_name);
      }
    }
  }

  // ── Digital Twin republishers (RELIABLE, depth 10) ──────────────────────
  {
    rclcpp::QoS dt_qos{10};
    dt_qos.reliable();
    for (const auto& [group_name, slot] : group_slot_map_) {
      std::string dt_topic = "/" + group_name + "/digital_twin/joint_states";
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
      RCLCPP_INFO(get_logger(), "  Digital Twin publish: %s (RELIABLE/10)", dt_topic.c_str());
    }
  }

  // ── Fixed safety publishers (always present, non-lifecycle) ────────────────
  // These use the standalone rclcpp::create_publisher to remain as regular
  // rclcpp::Publisher (not LifecyclePublisher). This ensures E-STOP status
  // and controller name can be published regardless of lifecycle state.
  estop_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
      this->get_node_topics_interface(), "/system/estop_status", rclcpp::QoS(10));

  rclcpp::QoS latch_qos{1};
  latch_qos.transient_local();
  // /rtc_cm/active_controller_name — owned by rtc_controller_manager
  // (single-CM assumption per locked decision D-A2). robot_namespace prefix
  // is intentionally absent so the topic ownership is obvious from its name.
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
