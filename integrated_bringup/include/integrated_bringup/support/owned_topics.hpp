#ifndef UR5E_BRINGUP_SUPPORT_OWNED_TOPICS_HPP_
#define UR5E_BRINGUP_SUPPORT_OWNED_TOPICS_HPP_

// Shared helper for Phase 4 — creates, activates, and publishes the
// controller-owned sub/pub set that the three demo controllers
// (DemoJointController / DemoTaskController / DemoWbcController) share.
//
// Each demo owns one ControllerTopicHandles instance and delegates to the
// free functions here from its lifecycle overrides + PublishNonRtSnapshot.
// Every controller-YAML topic entry is controller-owned (issue #138); CM's
// own fixed publishers and DeviceBackend device-wire lanes are separate.

#include "integrated_bringup/controllers/catching/traj_input.hpp"
#include "integrated_bringup/controllers/tof_snapshot.hpp"
#include "integrated_bringup/controllers/wbc/wbc_state.hpp"
#include "integrated_bringup/logging/catching_diag_log_pod.hpp"
#include "integrated_bringup/logging/momentum_observer_log_pod.hpp"
#include <rtc_base/threading/publish_buffer.hpp>
#include <rtc_base/threading/seqlock.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>
#include <rtc_controllers/grasp/grasp_state.hpp>
#include <rtc_msgs/msg/catching_state.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/payload_estimate.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <rtc_msgs/msg/wbc_state.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

// ── Per-task goal ingress ────────────────────────────────────────────────────
//
// The `topics:` lane gives a device group ONE RobotTarget subscription, and its
// task payload is one pose. A controller that moves several frames of one
// group needs a goal per frame, so it owns one subscription per task here:
// `<task name>/task_goal`, the same `rtc_msgs/RobotTarget` (goal_type "task",
// [x y z roll pitch yaw], ZYX).
//
// DeliverTargetMessage is not reused: it routes by device group, so it carries
// neither which task a goal is for nor the frame the goal is expressed in, and
// its validation and reject counter are private to the base. This ingress does
// its own, and the three differ on purpose:
//   - a joint goal on a task topic is refused (the group's own topic takes it);
//   - `header.frame_id` is READ. Empty means "the task's base frame"; any other
//     name has to be one the controller registered, and comes out as an index
//     into that list. An unknown name is refused, never read as the base frame —
//     a pose taken in the wrong frame is finite, smooth and wrong;
//   - every refusal is counted by reason, so a log can say why a goal did not
//     land.
//
// Hand-over to the RT tick is a SeqLock per task (single writer: the
// controller node's default callback group, one thread). The payload carries a
// sequence number — the tick applies a goal once — and the activation
// generation the writer observed, so a goal sent while the controller was
// Inactive is dropped by the tick instead of moving the robot on activation.

/// One accepted task goal as the RT tick reads it.
struct TaskGoal {
  /// [x, y, z, roll, pitch, yaw] (m, rad; ZYX) in the frame `frame_slot` names.
  std::array<double, rtc::kTaskSpaceDim> pose{};
  /// Index into the frame-name list the subscription was created with, or −1
  /// for "the task's own base frame" (an empty header.frame_id).
  int frame_slot{-1};
  /// 0 until the first goal; then 1, 2, … per accepted goal of this task.
  std::uint32_t sequence{0};
  /// RTControllerInterface::ActivationGeneration() when the goal arrived.
  std::uint32_t generation{0};
};

static_assert(std::is_trivially_copyable_v<TaskGoal>, "TaskGoal travels through a SeqLock");

/// Why a task goal was refused. kCount is the array size, not a reason.
enum class TaskGoalReject : std::uint8_t {
  kNone = 0,
  kGoalType,      ///< goal_type is not "task"
  kNonFinite,     ///< a pose component is NaN / Inf
  kUnknownFrame,  ///< header.frame_id is not a registered target frame
  kCount,
};

/// Static string for a reason (non-RT log lines; never allocates).
[[nodiscard]] const char* TaskGoalRejectName(TaskGoalReject reason) noexcept;

/// What one task's subscription writes and the controller reads. Owned by the
/// controller at a stable address — the subscription callback keeps a pointer.
struct TaskGoalIngress {
  rtc::SeqLock<TaskGoal> box{};
  std::array<std::atomic<std::uint64_t>, static_cast<std::size_t>(TaskGoalReject::kCount)>
      reject_counts{};
  std::atomic<std::uint64_t> accepted{0};
  /// Writer-side only (the callback group's one thread).
  std::uint32_t next_sequence{0};

  [[nodiscard]] std::uint64_t RejectCount(TaskGoalReject reason) const noexcept {
    return reject_counts[static_cast<std::size_t>(reason)].load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t TotalRejects() const noexcept {
    std::uint64_t total = 0;
    for (const auto& c : reject_counts) {
      total += c.load(std::memory_order_relaxed);
    }
    return total;
  }
};

/// Validate one message (no ROS, no allocation). On kNone `out.pose` and
/// `out.frame_slot` are filled; sequence and generation are the caller's.
[[nodiscard]] TaskGoalReject ParseTaskGoal(const rtc_msgs::msg::RobotTarget& msg,
                                           std::span<const std::string> frame_names,
                                           TaskGoal& out) noexcept;

/// The subscription callback's whole body: ParseTaskGoal, then either stamp
/// (sequence, `generation`) and Store, or count the refusal. Returns the
/// verdict so the caller can log it. Exposed so a test can drive the ingress
/// without a node.
[[nodiscard]] TaskGoalReject DeliverTaskGoal(const rtc_msgs::msg::RobotTarget& msg,
                                             std::span<const std::string> frame_names,
                                             std::uint32_t generation,
                                             TaskGoalIngress& ingress) noexcept;

/// True for `[A-Za-z][A-Za-z0-9_]*`: a name that can be one token of a topic
/// (and of a parameter name). One definition for the config parser and the
/// subscription helper, which both have to refuse the same names.
[[nodiscard]] bool IsTopicToken(std::string_view name) noexcept;

/// One task to subscribe for: the topic is `<name>/task_goal`.
struct TaskGoalSubscriptionRequest {
  std::string name;
  TaskGoalIngress* ingress{nullptr};
};

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
    kHandTip,       // group_commands[group_idx].task_link_poses[source_index]
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

  // Per-task goal subscriptions (CreateTaskGoalSubscriptions) — one per task of
  // a controller that moves several frames of one group. Empty for every
  // controller that takes its task goal through `target_subs`.
  std::vector<rclcpp::Subscription<rtc_msgs::msg::RobotTarget>::SharedPtr> task_goal_subs{};

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

  // Catching state publisher (dynamic_catching D-20) — at most one per demo
  // (the catching controller). Created via SetupCatchingStatePublisher, which
  // pre-sizes every per-joint / per-tip / per-reject array so the publish path
  // writes into existing elements only.
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::CatchingState>::SharedPtr catching_pub{};
  rtc_msgs::msg::CatchingState catching_msg{};

  // Payload estimate publisher (#135 D12) — one per controller that wires a
  // MomentumObserverWiring. Created via SetupPayloadEstimatePublisher, which
  // pre-fills joint_names and sizes `residual` to match, so the publish path
  // only ever writes into existing elements.
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::PayloadEstimate>::SharedPtr payload_pub{};
  rtc_msgs::msg::PayloadEstimate payload_msg{};

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

// Walk ctrl.GetTopicConfig().groups and create the matching sub/pub for every
// entry on ctrl.get_lifecycle_node() (all controller-YAML entries are
// controller-owned, issue #138). Subscriptions route through
// ctrl.DeliverTargetMessage(group_name, group_idx, msg). Throws on
// allocation failure — callers must wrap in try/catch.
void CreateOwnedTopics(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles);

// Create one `<name>/task_goal` subscription per request on the controller's
// LifecycleNode — default callback group (non-RT), QoS depth 1 — and keep them
// in `handles.task_goal_subs` (ResetOwnedTopics releases them). Each callback
// runs DeliverTaskGoal against the request's ingress with `frame_names` (copied
// — the names a goal's header.frame_id may carry) and a throttled warning per
// refusal. Throws when the controller has no node, a request has no ingress, or
// a name is not a valid topic token; callers wrap in try/catch.
void CreateTaskGoalSubscriptions(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles,
                                 std::span<const TaskGoalSubscriptionRequest> requests,
                                 const std::vector<std::string>& frame_names);

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
                                    const rtc::grasp::GraspStateData* grasp = nullptr,
                                    const WbcStateData* wbc = nullptr,
                                    const ToFSnapshotData* tof = nullptr,
                                    const MomentumObserverLogPod* payload = nullptr) noexcept;

// ── Helpers for controllers to register TF frame slots at on_configure ─────
// Each helper appends one TfFrameSlot to handles.tf_slots[] (no-op when the
// slot array is full). frame_id strings are stored by value so the publish
// thread sees stable memory.

// Append `<root>` → `<tip>_actual` slot reading PublishSnapshot
// group_commands[group_idx].arm_tip_pose. Returns false if no room.
bool AppendArmTipSlot(ControllerTopicHandles& handles, const std::string& parent_frame,
                      const std::string& child_link, int group_idx);

// Append `<hand_root>` → `<tip>_actual` slot per fingertip, reading
// group_commands[group_idx].task_link_poses[source_index]. Registers at most
// `max_tips` slots: the controllers only fill kNumFingertips fingertip poses,
// so extra tip_links would register slots that never receive a pose and
// silently never broadcast (#125 F4). Skips further slots when `tip_links`
// exceeds remaining capacity.
void AppendHandTipSlots(ControllerTopicHandles& handles, const std::string& parent_frame,
                        const std::vector<std::string>& tip_links, int group_idx,
                        std::size_t max_tips);

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

// Create the catching state publisher (D-20) and pre-fill everything that is
// fixed for the life of the configuration: the arm joint names the `q_cmd` /
// `q_meas` arrays are in, the fingertip names, and the CloudReject names the
// `input_reject_counts` histogram is indexed by. Every variable-length array
// is sized HERE and never resized afterwards.
//
// The names ride on the wire rather than being assumed by consumers for the
// same reason PayloadEstimate carries `joint_names`: device order and model
// order coincide on some robots and not others, and a consumer that pairs
// them wrongly gets a finite, smooth, wrong answer with no symptom.
void SetupCatchingStatePublisher(rtc::RTControllerInterface& ctrl, ControllerTopicHandles& handles,
                                 const std::string& topic_name,
                                 const std::vector<std::string>& arm_joint_names,
                                 const std::vector<std::string>& tip_names);

// Publish the catching state from one tick's record plus the ingress
// snapshot. Called from the controller's PublishNonRtSnapshot (CM publish
// thread) — must be noexcept.
//
// A SEPARATE ENTRY POINT rather than two more optional parameters on
// PublishOwnedTopicsFromSnapshot: that function's argument list is already
// five roles long, and the catching controller publishes none of the other
// five. `ingress` may be null when the ingress never ran; the input counters
// are then left at whatever the previous message put there, which is what
// they mean.
void PublishCatchingStateFromSnapshot(const rtc::PublishSnapshot& snap,
                                      ControllerTopicHandles& handles,
                                      const CatchingDiagLogPod& tick,
                                      const CatchingIngressSnapshot* ingress) noexcept;

// Create the PayloadEstimate publisher (#135 D12) and pre-fill the two fields
// that are fixed for the life of the configuration: `joint_names` — the arm
// DEVICE order the residual is expressed in — and `payload_frame`, the frame
// the estimated wrench acts at (empty when the payload estimator is not
// configured; the residual half of the message is published either way).
//
// joint_names is carried on the WIRE rather than assumed by consumers because
// the residual's order is the device order and a model Jacobian's columns are
// in pinocchio order. The two coincide on some robots and not others, and a
// consumer that pairs them wrongly gets a finite, smooth, wrong answer with no
// symptom — the same trap the estimator's own preamble opens with.
//
// `residual` is sized here to match joint_names (and never resized afterwards),
// so the publish path writes into existing elements only.
void SetupPayloadEstimatePublisher(rtc::RTControllerInterface& ctrl,
                                   ControllerTopicHandles& handles, const std::string& topic_name,
                                   const std::vector<std::string>& joint_names,
                                   const std::string& payload_frame);

// Stamp the reference frame the GraspState / WbcState / PayloadEstimate *vector*
// payloads are expressed in — the FK reference the controller resolves fingertip rotations
// and positions into, i.e. the arm root link (#234 P-5). Call once from
// on_configure after the Setup*Publisher calls; the string is stored in the
// pre-filled message so the publish thread never touches it.
//
// Without it the pull estimate's force / plane_normal / basis_x go out with an
// empty frame_id and are uninterpretable off-line: the numbers are a 3-D
// vector in *some* frame, and which one differs per robot.
//
// Note this is the payload frame, not a transform frame — header.stamp remains
// the publish wall clock and must not be used for staleness (see
// rtc_base/threading/publish_buffer.hpp).
void SetOwnedStateFrameId(ControllerTopicHandles& handles, const std::string& frame_id);

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_OWNED_TOPICS_HPP_
