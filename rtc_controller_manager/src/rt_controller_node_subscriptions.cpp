// ── Subscription creation (HW/sim ↔ controller boundary inputs) ─────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <set>

namespace urtc = rtc;

void RtControllerNode::CreateSubscriptions() {
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_sensor_;

  // ── Helper: find DeviceTimeoutEntry index for a given group name ─────────
  auto find_timeout_idx_by_group = [this](const std::string& group) -> int {
    for (std::size_t i = 0; i < device_timeouts_.size(); ++i) {
      if (device_timeouts_[i].group_name == group)
        return static_cast<int>(i);
    }
    return -1;
  };

  // ── Create subscriptions for all active device groups ────────────────────
  //
  // QoS strategy per role:
  //   kState / kSensorState → BEST_EFFORT, depth 2
  //   kTarget → RELIABLE, depth 10
  rclcpp::QoS sensor_sub_qos{2};
  sensor_sub_qos.best_effort();

  std::set<std::string> created_topics;

  for (const auto& tc : controller_topic_configs_) {
    for (const auto& [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;

      const int slot = group_slot_map_[group_name];
      const int dt_idx = find_timeout_idx_by_group(group_name);

      for (const auto& entry : group.subscribe) {
        // Controller-owned subscriptions are created by the controller's own
        // on_configure — CM does not bind them here.
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        if (!created_topics.insert(entry.topic_name).second)
          continue;

        switch (entry.role) {
          case urtc::SubscribeRole::kState: {
            auto sub = create_subscription<sensor_msgs::msg::JointState>(
                entry.topic_name, sensor_sub_qos,
                [this, slot, dt_idx](sensor_msgs::msg::JointState::SharedPtr msg) {
                  DeviceJointStateCallback(slot, std::move(msg));
                  if (dt_idx >= 0) {
                    const auto dti = static_cast<std::size_t>(dt_idx);
                    device_timeouts_[dti].last_update = std::chrono::steady_clock::now();
                    device_timeouts_[dti].received.store(true, std::memory_order_relaxed);
                  }
                },
                sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/state]: %s (slot %d, BEST_EFFORT/2)",
                        group_name.c_str(), entry.topic_name.c_str(), slot);
            break;
          }
          case urtc::SubscribeRole::kMotorState: {
            auto sub = create_subscription<sensor_msgs::msg::JointState>(
                entry.topic_name, sensor_sub_qos,
                [this, slot, dt_idx](sensor_msgs::msg::JointState::SharedPtr msg) {
                  DeviceMotorStateCallback(slot, std::move(msg));
                  if (dt_idx >= 0) {
                    const auto dti = static_cast<std::size_t>(dt_idx);
                    device_timeouts_[dti].last_update = std::chrono::steady_clock::now();
                    device_timeouts_[dti].received.store(true, std::memory_order_relaxed);
                  }
                },
                sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/motor_state]: %s (slot %d, BEST_EFFORT/2)",
                        group_name.c_str(), entry.topic_name.c_str(), slot);
            break;
          }
          case urtc::SubscribeRole::kSensorState: {
            auto sub = create_subscription<rtc_msgs::msg::HandSensorState>(
                entry.topic_name, sensor_sub_qos,
                [this, slot, dt_idx](rtc_msgs::msg::HandSensorState::SharedPtr msg) {
                  HandSensorStateCallback(slot, std::move(msg));
                  if (dt_idx >= 0) {
                    const auto dti = static_cast<std::size_t>(dt_idx);
                    device_timeouts_[dti].last_update = std::chrono::steady_clock::now();
                    device_timeouts_[dti].received.store(true, std::memory_order_relaxed);
                  }
                },
                sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/sensor_state]: %s (slot %d, BEST_EFFORT/2)",
                        group_name.c_str(), entry.topic_name.c_str(), slot);
            break;
          }
          case urtc::SubscribeRole::kTarget: {
            auto sub = create_subscription<rtc_msgs::msg::RobotTarget>(
                entry.topic_name, 10,
                [this, slot](rtc_msgs::msg::RobotTarget::SharedPtr msg) {
                  DeviceTargetCallback(slot, std::move(msg));
                },
                sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/target]: %s (slot %d)", group_name.c_str(),
                        entry.topic_name.c_str(), slot);
            break;
          }
        }
      }
    }
  }

  // Per-controller gain tuning has migrated from /<robot>/{controller_gains,
  // request_gains, current_gains} topics to ROS 2 parameters declared on each
  // controller's own LifecycleNode (Phase D of gain→parameter migration).
  // BT now calls SetActiveControllerGains() and SendGraspCommand() directly
  // against the active controller; CM no longer routes any gain traffic.
}
