#ifndef UR5E_BRINGUP_CONTROLLERS_OWNED_TOPICS_H_
#define UR5E_BRINGUP_CONTROLLERS_OWNED_TOPICS_H_

// Shared helper for Phase 4 — creates, activates, and publishes the
// controller-owned sub/pub set that the three demo controllers
// (DemoJointController / DemoTaskController / DemoWbcController) share.
//
// Each demo owns one ControllerTopicHandles instance and delegates to the
// free functions here from its lifecycle overrides + PublishNonRtSnapshot.
// Manager-owned entries (ownership == TopicOwnership::kManager) are left to
// RtControllerNode.

#include <array>
#include <cstddef>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <rtc_base/threading/publish_buffer.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <rtc_msgs/msg/wbc_state.hpp>

namespace ur5e_bringup {

// Up to two device groups per demo (ur5e, hand). Expand if/when a demo
// introduces a third group.
inline constexpr std::size_t kMaxOwnedGroups = 2;

struct ControllerTopicHandles {
  // Target subscriptions — one per device group (ur5e, hand).
  std::array<rclcpp::Subscription<rtc_msgs::msg::RobotTarget>::SharedPtr,
             kMaxOwnedGroups>
      target_subs{};

  // Gui-position publishers — one per device group.
  std::array<rclcpp_lifecycle::LifecyclePublisher<
                 rtc_msgs::msg::GuiPosition>::SharedPtr,
             kMaxOwnedGroups>
      gui_pubs{};
  std::array<rtc_msgs::msg::GuiPosition, kMaxOwnedGroups> gui_msgs{};
  // Group index that owns the gui publisher (so PublishNonRtSnapshot can
  // pick the correct snapshot slot). -1 when unused.
  std::array<int, kMaxOwnedGroups> gui_group_idx{{-1, -1}};

  // Grasp + ToF publishers — at most one per demo (hand group).
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::GraspState>::SharedPtr
      grasp_pub{};
  rtc_msgs::msg::GraspState grasp_msg{};
  int grasp_group_idx{-1};

  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::ToFSnapshot>::SharedPtr
      tof_pub{};
  rtc_msgs::msg::ToFSnapshot tof_msg{};
  int tof_group_idx{-1};

  // WBC state publisher — at most one per demo (TSID-based controllers).
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::WbcState>::SharedPtr
      wbc_pub{};
  rtc_msgs::msg::WbcState wbc_msg{};
  int wbc_group_idx{-1};
};

// Walk ctrl.GetTopicConfig().groups and for every entry with
// ownership == TopicOwnership::kController create the matching sub/pub on
// ctrl.get_lifecycle_node(). Subscriptions route through
// ctrl.DeliverTargetMessage(group_name, group_idx, msg). Throws on
// allocation failure — callers must wrap in try/catch.
void CreateOwnedTopics(rtc::RTControllerInterface &ctrl,
                       ControllerTopicHandles &handles);

// Activate / deactivate all LifecyclePublishers held by `handles`. Must be
// called from the matching lifecycle hook so publish() never hits an
// Inactive publisher.
void ActivateOwnedTopics(const rclcpp_lifecycle::State &prev,
                         ControllerTopicHandles &handles) noexcept;
void DeactivateOwnedTopics(const rclcpp_lifecycle::State &prev,
                           ControllerTopicHandles &handles) noexcept;

// Release all handles (subs + pubs) so a subsequent on_configure can rebuild.
void ResetOwnedTopics(ControllerTopicHandles &handles) noexcept;

// Publish controller-owned topics from a snapshot. Called from CM's publish
// thread via RTControllerInterface::PublishNonRtSnapshot. Must be noexcept.
void PublishOwnedTopicsFromSnapshot(const rtc::PublishSnapshot &snap,
                                    ControllerTopicHandles &handles) noexcept;

} // namespace ur5e_bringup

#endif // UR5E_BRINGUP_CONTROLLERS_OWNED_TOPICS_H_
