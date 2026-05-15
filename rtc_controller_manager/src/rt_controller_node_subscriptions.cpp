// ── Subscription creation (target lane; state/motor/sensor live in backends)
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <set>

namespace urtc = rtc;

void RtControllerNode::CreateSubscriptions() {
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_sensor_;

  std::set<std::string> created_topics;

  // kState / kMotorState / kSensorState are owned by DeviceBackend
  // implementations and bound in CreateDeviceBackends(). CM only binds
  // kTarget here since it is a controller-target lane (RobotTarget),
  // not part of the HW/sim adapter.
  for (const auto& tc : controller_topic_configs_) {
    for (const auto& [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;

      const int slot = group_slot_map_[group_name];

      for (const auto& entry : group.subscribe) {
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        if (entry.role != urtc::SubscribeRole::kTarget)
          continue;
        if (!created_topics.insert(entry.topic_name).second)
          continue;

        CreateTargetSubscription(entry, group_name, slot, sub_options);
      }
    }
  }
}

void RtControllerNode::CreateTargetSubscription(const urtc::SubscribeTopicEntry& entry,
                                                const std::string& group_name, int slot,
                                                const rclcpp::SubscriptionOptions& sub_options) {
  auto sub = create_subscription<rtc_msgs::msg::RobotTarget>(
      entry.topic_name, 10,
      [this, slot](rtc_msgs::msg::RobotTarget::SharedPtr msg) {
        DeviceTargetCallback(slot, std::move(msg));
      },
      sub_options);
  topic_subscriptions_.push_back(sub);
  RCLCPP_INFO(get_logger(), "  Subscribe [%s/target]: %s (slot %d)", group_name.c_str(),
              entry.topic_name.c_str(), slot);
}
