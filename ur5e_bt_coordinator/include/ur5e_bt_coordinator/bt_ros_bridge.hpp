#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <rtc_msgs/msg/wbc_state.hpp>
#include <rtc_msgs/srv/grasp_command.hpp>
#include <rtc_msgs/srv/list_controllers.hpp>
#include <rtc_msgs/srv/switch_controller.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <std_msgs/msg/bool.hpp>
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

  /// Cached grasp state from /<ctrl>/hand/grasp_state (500Hz pre-computed).
  /// Populated only when the active controller publishes Force-PI grasp
  /// state (DemoJointController / DemoTaskController). Empty/stale when a
  /// WBC controller is active — use GetWbcState() instead.
  CachedGraspState GetGraspState() const;

  /// Cached WBC state from /<ctrl>/hand/wbc_state (500Hz pre-computed).
  /// Populated only when the active controller is a TSID-based WBC
  /// controller (DemoWbcController). Empty/stale when a Force-PI grasp
  /// controller is active — use GetGraspState() instead.
  CachedWbcState GetWbcState() const;

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

  // ── Publishers (send commands) ────────────────────────────────────────────

  /// Publish arm task-space target [x,y,z,roll,pitch,yaw]
  void PublishArmTarget(const Pose6D &target);

  /// Publish arm joint-space target [q0..q5]
  void PublishArmJointTarget(const std::vector<double> &target);

  /// Publish hand motor target [m0..m9]
  void PublishHandTarget(const std::vector<double> &target);

  /// Set ROS 2 parameters atomically on the active controller's LifecycleNode
  /// via the rebound AsyncParametersClient. Sync wrapper — blocks until the
  /// remote node responds or `timeout_s` elapses. Returns false on timeout,
  /// service unavailable, or any-parameter-rejected; populates `message`
  /// with the failure reason.
  bool SetActiveControllerGains(const std::vector<rclcpp::Parameter> &params,
                                double timeout_s, std::string &message);

  /// Issue a one-shot Force-PI grasp command via the active controller's
  /// /<active>/grasp_command srv. `command` uses the rtc_msgs/GraspCommand
  /// constants (GRASP=1, RELEASE=2). `target_force` is ignored for RELEASE.
  /// Sync wrapper — same blocking semantics as SetActiveControllerGains.
  bool SendGraspCommand(uint8_t command, double target_force, double timeout_s,
                        std::string &message);

  /// Request a controller switch via /rtc_cm/switch_controller (sync srv).
  /// Returns true when the service responded ok=true within `timeout_s`.
  /// On false, `message` carries the failure reason (E-STOP active, unknown
  /// name, timeout, ...). Caller (BT switch_controller node) treats the
  /// boolean as a synchronous switch confirmation — no follow-up polling on
  /// /rtc_cm/active_controller_name is required.
  bool RequestSwitchController(const std::string &name, double timeout_s,
                               std::string &message);

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
  rclcpp::Subscription<rtc_msgs::msg::WbcState>::SharedPtr wbc_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr
      world_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_ctrl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr
      shape_estimate_sub_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<rtc_msgs::msg::RobotTarget>::SharedPtr arm_target_pub_;
  rclcpp::Publisher<rtc_msgs::msg::RobotTarget>::SharedPtr hand_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shape_trigger_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shape_clear_client_;

  // ── /rtc_cm/* service clients (Phase 4) ─────────────────────────────────
  // Sync wrappers exposed via RequestSwitchController. list_controllers_
  // currently has no consumer in BT but is created so future diagnostics /
  // health-check nodes can reuse it without re-touching this file.
  rclcpp::Client<rtc_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;
  rclcpp::Client<rtc_msgs::srv::ListControllers>::SharedPtr
      list_controllers_client_;

  // ── Phase C (gain→parameter migration): per-active-controller clients ──
  //
  // Both rebound to the active controller in RewireControllerTopics():
  //   active_param_client_  → /<ctrl_FQN>/{get,set}_parameters[_atomically]
  //   grasp_command_client_ → /<ctrl_ns>/grasp_command
  //
  // The controller's LifecycleNode is created with namespace=/<config_key>
  // and node-name=<config_key>, so its parameter services live under
  // /<config_key>/<config_key>/... while its relative grasp_command srv
  // resolves under /<config_key>/grasp_command.
  rclcpp::AsyncParametersClient::SharedPtr active_param_client_;
  rclcpp::Client<rtc_msgs::srv::GraspCommand>::SharedPtr grasp_command_client_;

  // ── Cached state ──────────────────────────────────────────────────────────
  mutable std::mutex state_mutex_;
  Pose6D tcp_pose_;
  std::vector<double> arm_joint_positions_;
  std::vector<double> hand_joint_positions_;
  CachedGraspState grasp_state_;
  CachedWbcState wbc_state_;
  Pose6D world_target_pose_;
  bool world_target_valid_{false};
  std::string active_controller_;
  bool estopped_{false};
  std::vector<double> last_hand_target_;
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
  TimePoint wbc_state_last_{};
  bool wbc_state_received_{false};
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
