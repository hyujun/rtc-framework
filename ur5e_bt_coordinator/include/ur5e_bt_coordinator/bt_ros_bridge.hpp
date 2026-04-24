#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace rtc_bt {

/// Topic health status for watchdog monitoring.
struct TopicHealth {
  std::string name;
  bool received{false};            ///< true if at least one message received
  double seconds_since_last{-1.0}; ///< -1 means never received
  bool healthy{false};             ///< true if within timeout
};

/// Bridges ROS2 topics to/from the BT Blackboard.
///
/// Subscribers cache the latest value from each RELIABLE topic.
/// BT nodes read/write via the public accessors (mutex-protected).
/// Publishers send commands to the RT control layer.
/// Also provides:
///   - Pose library (runtime-configurable hand/arm poses)
///   - Topic health watchdog
class BtRosBridge {
public:
  explicit BtRosBridge(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  // ── Cached state (thread-safe reads) ──────────────────────────────────────

  /// Current TCP pose from /ur5e/gui_position
  Pose6D GetTcpPose() const;

  /// Current arm joint positions from /ur5e/gui_position
  std::vector<double> GetArmJointPositions() const;

  /// Current hand joint positions from /hand/gui_position
  std::vector<double> GetHandJointPositions() const;

  /// Last published hand target (empty if never published)
  std::vector<double> GetLastHandTarget() const;

  /// Cached grasp state from /hand/grasp_state (500Hz pre-computed)
  CachedGraspState GetGraspState() const;

  /// Latest vision object pose from /world_target_info (position only).
  /// Orientation is zeroed; callers should fill from current TCP if needed.
  bool GetObjectPose(Pose6D &pose) const;

  /// Latest world target position from /world_target_info (Polygon).
  /// Returns position only (orientation zeroed). Returns false if topic
  /// not received or all coordinates are zero.
  bool GetWorldTargetPose(Pose6D &pose) const;

  /// Active controller name
  std::string GetActiveController() const;

  /// E-STOP status
  bool IsEstopped() const;

  // ── Gains query (request/response via controller manager) ─────────────────

  /// Request current gains from the active controller.
  /// Publishes a Bool to /{ns}/request_gains, response arrives on
  /// /{ns}/current_gains and is cached internally.
  void RequestCurrentGains();

  /// Return the most recently cached gains (from current_gains topic).
  /// Returns empty vector if no gains have been received yet.
  std::vector<double> GetCachedGains() const;

  /// True if at least one current_gains message has been received
  /// since the last call to RequestCurrentGains().
  bool HasCachedGains() const;

  /// Clear the cached gains (called before a new request to detect fresh
  /// arrival).
  void ClearCachedGains();

  // ── Publishers (send commands) ────────────────────────────────────────────

  /// Publish arm task-space target [x,y,z,roll,pitch,yaw]
  void PublishArmTarget(const Pose6D &target);

  /// Publish arm joint-space target [q0..q5]
  void PublishArmJointTarget(const std::vector<double> &target);

  /// Publish hand motor target [m0..m9]
  void PublishHandTarget(const std::vector<double> &target);

  /// Publish gain update (16-element array)
  void PublishGains(const std::vector<double> &gains);

  /// Publish controller switch command
  void PublishSelectController(const std::string &name);

  // ── Shape estimation ────────────────────────────────────────────────────

  /// Publish trigger command to /shape/trigger ("start", "stop", etc.)
  void PublishShapeTrigger(const std::string &command);

  /// Call /shape/clear service (async, non-blocking)
  void CallShapeClear();

  /// Get the latest cached ShapeEstimate message.
  /// Returns false if no estimate has been received yet.
  bool GetShapeEstimate(shape_estimation_msgs::msg::ShapeEstimate &out) const;

  /// Clear the cached shape estimate (called before a new estimation session).
  void ClearShapeEstimate();

  // ── ToF data collection ─────────────────────────────────────────────────

  /// Start buffering incoming /tof/snapshot messages.
  /// Always clears the buffer and resets state (safe to call even if a
  /// previous collection was interrupted by E-STOP or tree halt).
  void StartToFCollection();

  /// Stop buffering. The collected data remains accessible via
  /// GetCollectedToFData() until the next StartToFCollection() call.
  void StopToFCollection();

  /// Collected ToF snapshots (valid between StopToFCollection and next Start).
  const std::vector<rtc_msgs::msg::ToFSnapshot> &GetCollectedToFData() const;

  /// Number of snapshots currently in the buffer.
  std::size_t GetCollectedToFCount() const;

  // ── Pose library (runtime-configurable) ───────────────────────────────────

  /// Load hand/arm pose overrides from ROS2 parameters (deg → rad conversion).
  /// Parameters format: hand_pose.<name> = [double array], arm_pose.<name> =
  /// [double array]
  void LoadPoseOverrides(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /// Lookup a hand pose by name. Falls back to compile-time defaults.
  const HandPose &GetHandPose(const std::string &name) const;

  /// Lookup an arm pose by name. Falls back to compile-time defaults.
  const ArmPose &GetArmPose(const std::string &name) const;

  /// Get the full hand pose map (for iteration/validation).
  const std::map<std::string, HandPose> &GetHandPoses() const {
    return hand_poses_;
  }

  /// Get the full arm pose map.
  const std::map<std::string, ArmPose> &GetArmPoses() const {
    return arm_poses_;
  }

  // ── Topic health watchdog ─────────────────────────────────────────────────

  /// Get health status for all monitored topics.
  std::vector<TopicHealth> GetTopicHealth(double timeout_s = 2.0) const;

  /// Check if all critical topics (arm_gui, hand_gui) are healthy.
  bool AreTopicsHealthy(double timeout_s = 2.0) const;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // ── Controller-owned topic rewiring (Phase 4) ─────────────────────────────
  // Rebuild arm_gui/hand_gui/grasp_state/tof/arm_target/hand_target sub/pub
  // against the /<ctrl_name>/... namespace. No-op when ctrl_name is empty or
  // unchanged. Protected by controller_topics_mutex_.
  void RewireControllerTopics(const std::string &ctrl_name);
  std::string rewired_controller_;
  mutable std::mutex controller_topics_mutex_;

  // ── Subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<rtc_msgs::msg::GuiPosition>::SharedPtr arm_gui_sub_;
  rclcpp::Subscription<rtc_msgs::msg::GuiPosition>::SharedPtr hand_gui_sub_;
  rclcpp::Subscription<rtc_msgs::msg::GraspState>::SharedPtr grasp_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr
      world_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_ctrl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      current_gains_sub_;
  rclcpp::Subscription<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr
      shape_estimate_sub_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<rtc_msgs::msg::RobotTarget>::SharedPtr arm_target_pub_;
  rclcpp::Publisher<rtc_msgs::msg::RobotTarget>::SharedPtr hand_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gains_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr select_ctrl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_gains_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shape_trigger_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shape_clear_client_;

  // ── Cached state ──────────────────────────────────────────────────────────
  mutable std::mutex state_mutex_;
  Pose6D tcp_pose_;
  std::vector<double> arm_joint_positions_;
  std::vector<double> hand_joint_positions_;
  CachedGraspState grasp_state_;
  Pose6D world_target_pose_;
  bool world_target_valid_{false};
  std::string active_controller_;
  bool estopped_{false};
  std::vector<double> last_hand_target_;
  std::vector<double> cached_gains_;
  bool cached_gains_valid_{false};
  shape_estimation_msgs::msg::ShapeEstimate shape_estimate_;
  bool shape_estimate_valid_{false};

  // ── Topic health timestamps ───────────────────────────────────────────────
  using TimePoint = std::chrono::steady_clock::time_point;
  mutable std::mutex health_mutex_;
  TimePoint arm_gui_last_{};
  bool arm_gui_received_{false};
  TimePoint hand_gui_last_{};
  bool hand_gui_received_{false};
  TimePoint grasp_state_last_{};
  bool grasp_state_received_{false};
  TimePoint world_target_last_{};
  bool world_target_received_{false};
  TimePoint estop_last_{};
  bool estop_received_{false};

  // ── ToF data collection ────────────────────────────────────────────────────
  rclcpp::Subscription<rtc_msgs::msg::ToFSnapshot>::SharedPtr tof_snapshot_sub_;
  mutable std::mutex tof_mutex_; ///< Guards tof_buffer_ only
  std::vector<rtc_msgs::msg::ToFSnapshot> tof_buffer_;
  std::atomic<bool> tof_collecting_{false}; ///< Checked in 500Hz callback

  // ── Pose library ──────────────────────────────────────────────────────────
  std::map<std::string, HandPose> hand_poses_;
  std::map<std::string, ArmPose> arm_poses_;
};

} // namespace rtc_bt
