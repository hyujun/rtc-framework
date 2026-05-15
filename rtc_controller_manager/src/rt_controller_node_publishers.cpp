// ── Publisher creation + topic-parameter exposure ───────────────────────────
//
// kJointCommand / kRos2Command are owned by DeviceBackend implementations
// (created in CreateDeviceBackends — see device_config.cpp). CM owns three
// fixed publishers: per-group digital twin (RELIABLE republish of measured
// joint state), /system/estop_status, and /rtc_cm/active_controller_name.
// Controller-output roles (kRobotTarget / kGraspState / kWbcState /
// kToFSnapshot / kRobotTransforms) are owned by each controller's
// LifecycleNode.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace urtc = rtc;

void RtControllerNode::CreatePublishers() {
  // CM no longer owns kJointCommand/kRos2Command — those moved into
  // DeviceBackend impls. Reject any manager-ownership entry whose role is
  // outside the controller-output set so YAML mistakes surface early.
  for (const auto& tc : controller_topic_configs_) {
    for (const auto& [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;
      for (const auto& entry : group.publish) {
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        switch (entry.role) {
          case urtc::PublishRole::kJointCommand:
          case urtc::PublishRole::kRos2Command:
            // Owned by the device backend — no-op here. CreateDeviceBackends
            // resolves the corresponding command_topic from the YAML entry.
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
