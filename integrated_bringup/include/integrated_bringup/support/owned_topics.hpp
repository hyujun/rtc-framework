#ifndef UR5E_BRINGUP_SUPPORT_OWNED_TOPICS_HPP_
#define UR5E_BRINGUP_SUPPORT_OWNED_TOPICS_HPP_

// Shared helper for Phase 4 — creates, activates, and publishes the
// controller-owned sub/pub set that the three demo controllers
// (DemoJointController / DemoTaskController / DemoWbcController) share.
//
// Each demo owns one ControllerTopicHandles instance and delegates to the
// free functions here from its lifecycle overrides + PublishNonRtSnapshot.
// Manager-owned entries (ownership == TopicOwnership::kManager) are left to
// RtControllerNode.

#include <rtc_base/threading/publish_buffer.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <rtc_msgs/msg/wbc_state.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace integrated_bringup {

// Up to two device groups per demo (ur5e, hand). Expand if/when a demo
// introduces a third group.
inline constexpr std::size_t kMaxOwnedGroups = 2;

// Upper bound on TFMessage transforms broadcast by a single controller.
// Sized for DemoJoint/Task (6: arm tip + 4 fingertip + virtual_tcp) and
// DemoWbc (4 fingertip + alpha placeholder + headroom for future frames).
inline constexpr std::size_t kMaxControllerTransforms = 16;

// One transform slot — populated at on_configure from the system YAML
// urdf.{sub,tree}_models. The publish thread reads from
// PublishSnapshot::GroupCommandSlot SE3 fields based on `source` + index.
struct TfFrameSlot {
  std::string parent_frame_id;  // pre-allocated string (no resize at publish)
  std::string child_frame_id;

  enum class Source : uint8_t {
    kArmTip,        // group_commands[group_idx].arm_tip_pose
    kHandTip,       // group_commands[group_idx].fingertip_poses[source_index]
    kVirtualTcp,    // group_commands[group_idx].virtual_tcp_pose
    kWbcTipInBase,  // (Phase 3) WBC tree, tip in base frame — slot reserved
    kCustom,        // (D-5) future extension hook
  };

  Source source{Source::kArmTip};
  int group_idx{0};        // PublishSnapshot::group_commands index
  int source_index{0};     // for multi-tip sources (fingertip index)
  bool slot_valid{false};  // controller configured this slot at on_configure
};

struct ControllerTopicHandles {
  // Target subscriptions — one per device group (ur5e, hand).
  std::array<rclcpp::Subscription<rtc_msgs::msg::RobotTarget>::SharedPtr, kMaxOwnedGroups>
      target_subs{};

  // Grasp + ToF publishers — at most one per demo (hand group). Created by
  // the controller via SetupGraspStatePublisher / SetupToFSnapshotPublisher
  // (controller-owned non-RT data is no longer routed through YAML role
  // mappings; the controller decides the topic name and pre-fills the msg).
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::GraspState>::SharedPtr grasp_pub{};
  rtc_msgs::msg::GraspState grasp_msg{};

  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::ToFSnapshot>::SharedPtr tof_pub{};
  rtc_msgs::msg::ToFSnapshot tof_msg{};

  // WBC state publisher — at most one per demo (TSID-based controllers).
  // Created by the controller via SetupWbcStatePublisher.
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::WbcState>::SharedPtr wbc_pub{};
  rtc_msgs::msg::WbcState wbc_msg{};

  // ── Per-controller TF publisher (kRobotTransforms) ────────────────────
  // Single publisher per controller — D-2: controller당 1 토픽. The set of
  // frames is fixed at on_configure (parent/child frame_id + source slot)
  // so the publish path is allocation-free apart from TFMessage vector
  // resize; tf_msg.transforms is pre-reserved in CreateOwnedTopics.
  rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub{};
  tf2_msgs::msg::TFMessage tf_msg{};
  std::array<TfFrameSlot, kMaxControllerTransforms> tf_slots{};
  int num_tf_slots{0};
};

// Walk ctrl.GetTopicConfig().groups and for every entry with
// ownership == TopicOwnership::kController create the matching sub/pub on
// ctrl.get_lifecycle_node(). Subscriptions route through
// ctrl.DeliverTargetMessage(group_name, group_idx, msg). Throws on
// allocation failure — callers must wrap in try/catch.
void CreateOwnedTopics(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles);

// Activate / deactivate all LifecyclePublishers held by `handles`. Must be
// called from the matching lifecycle hook so publish() never hits an
// Inactive publisher.
void ActivateOwnedTopics(const rclcpp_lifecycle::State& prev,
                         ControllerTopicHandles& handles) noexcept;
void DeactivateOwnedTopics(const rclcpp_lifecycle::State& prev,
                           ControllerTopicHandles& handles) noexcept;

// Release all handles (subs + pubs) so a subsequent on_configure can rebuild.
void ResetOwnedTopics(ControllerTopicHandles& handles) noexcept;

// Publish controller-owned topics from a snapshot. Called from CM's publish
// thread via RTControllerInterface::PublishNonRtSnapshot. Must be noexcept.
//
// `grasp` / `wbc` / `tof` carry controller-owned non-RT data that used to ride
// inside `snap.group_commands[gi].{grasp_state,wbc_state,tof_snapshot}`. Each
// controller now owns a per-output SeqLock<T> and passes the freshly-loaded
// snapshot here; pass `nullptr` for any role the controller does not publish.
// `snap` still carries the stamp + group routing + SE3 fields used by
// kRobotTransforms.
void PublishOwnedTopicsFromSnapshot(const rtc::PublishSnapshot& snap,
                                    ControllerTopicHandles& handles,
                                    const rtc::GraspStateData* grasp = nullptr,
                                    const rtc::WbcStateData* wbc = nullptr,
                                    const rtc::ToFSnapshotData* tof = nullptr) noexcept;

// ── Helpers for controllers to register TF frame slots at on_configure ─────
// Each helper appends one TfFrameSlot to handles.tf_slots[] (no-op when the
// slot array is full). frame_id strings are stored by value so the publish
// thread sees stable memory.

// Append `<root>` → `<tip>_actual` slot reading PublishSnapshot
// group_commands[group_idx].arm_tip_pose. Returns false if no room.
bool AppendArmTipSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                      const std::string& child_link, int group_idx);

// Append `<hand_root>` → `<tip>_actual` slot per fingertip, reading
// group_commands[group_idx].fingertip_poses[source_index]. Skips slots when
// `tip_links` exceeds remaining capacity.
void AppendHandTipSlots(ControllerTopicHandles& handles, const std::string& parent_frame,
                        const std::vector<std::string>& tip_links, int group_idx);

// Append `<base>` → `virtual_tcp_actual` slot reading
// group_commands[group_idx].virtual_tcp_pose. group_idx selects which slot
// holds the virtual TCP — typically the arm group (0).
bool AppendVirtualTcpSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                          int group_idx);

// Append a placeholder slot (slot_valid = false) for future activation —
// used by DemoWbcController for the D-5 "alpha" frame.
bool AppendCustomPlaceholderSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                                 const std::string& child_frame);

// ── Controller-owned non-RT publishers (no YAML role mapping) ────────────
// Called from controller on_configure to create the GraspState / WbcState /
// ToFSnapshot LifecyclePublisher with a controller-specified topic name and
// pre-fill the per-finger arrays with the device's sensor names (so the
// publish path never resizes). Each helper is idempotent — a second call
// with a non-empty handle is a no-op.

void SetupGraspStatePublisher(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles,
                              const std::string& topic_name, const std::string& device_group);

void SetupWbcStatePublisher(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles,
                            const std::string& topic_name, const std::string& device_group);

void SetupToFSnapshotPublisher(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles,
                               const std::string& topic_name);

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_OWNED_TOPICS_HPP_
