// ── Publisher creation + topic-parameter exposure ───────────────────────────
//
// kJointCommand / kRos2Command are owned by DeviceBackend implementations
// (created in CreateDeviceBackends — see device_config.cpp). CM owns three
// fixed publishers: per-group digital twin (RELIABLE republish of measured
// joint state), /system/estop_status, and /rtc_cm/active_controller_name.
// kRobotTransforms — the only remaining PublishRole after issue #196 Phase 5
// — is owned by each controller's LifecycleNode (integrated_bringup
// owned_topics.cpp). GraspState / WbcState / ToFSnapshot bypass YAML roles
// and use per-controller SeqLock<T> handoffs.
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <string>
#include <vector>

namespace urtc = rtc;

void RtControllerNode::CreatePublishers() {
  // Phase 4 + issue #138: device-wire command publication is owned by
  // DeviceBackend impls; the controller-output role (kRobotTransforms — the
  // only one left after issue #196 Phase 5) is controller-owned and created on
  // each controller's LifecycleNode (integrated_bringup owned_topics). CM only
  // owns its fixed publishers below.
  CreateDigitalTwinPublishers();
  CreateFixedSafetyPublishers();
}

void RtControllerNode::CreateDigitalTwinPublishers() {
  rclcpp::QoS dt_qos{1};
  dt_qos.reliable();
  for (const auto& [group_name, slot] : group_slot_map_) {
    // Slot is the array index the state-lane callback will raise its dirty
    // bit at, so a group outside the fixed slot range gets no twin rather
    // than an out-of-bounds write later (issue #198 Phase 2).
    if (slot < 0 || slot >= kMaxDevices) {
      RCLCPP_WARN(get_logger(),
                  "  CM JointState publish skipped for group '%s': slot %d outside [0, %d)",
                  group_name.c_str(), slot, kMaxDevices);
      continue;
    }
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
    digital_twin_by_slot_[static_cast<std::size_t>(slot)] = std::move(dte);
    RCLCPP_INFO(get_logger(), "  CM JointState publish: %s (RELIABLE/1, slot %d)", dt_topic.c_str(),
                slot);
  }
}

void RtControllerNode::CreateFixedSafetyPublishers() {
  // Use standalone rclcpp::create_publisher so these remain regular
  // rclcpp::Publisher (not LifecyclePublisher) — E-STOP status and active
  // controller name must be publishable regardless of lifecycle state.
  estop_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
      this->get_node_topics_interface(), "/system/estop_status", rclcpp::QoS(1));

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
      // Phase 4 trailing cleanup: SubscribeRole enum dropped. Only one
      // subscribe lane (target) remains per group, so the parameter name is
      // fixed; the index keeps duplicate target entries (rare but legal)
      // discoverable.
      for (std::size_t si = 0; si < group.subscribe.size(); ++si) {
        const auto& entry = group.subscribe[si];
        std::string param_name = prefix;
        param_name += ".";
        param_name += group_name;
        param_name += ".subscribe.target";
        if (si > 0) {
          param_name += "_";
          param_name += std::to_string(si);
        }
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
