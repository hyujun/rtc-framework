#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"
#include "ur5e_bt_coordinator/finger_resolver.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"
#include "ur5e_bt_coordinator/robot_profile.hpp"
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <rtc_msgs/msg/wbc_state.hpp>
#include <rtc_msgs/srv/grasp_command.hpp>
#include <rtc_msgs/srv/list_controllers.hpp>
#include <rtc_msgs/srv/switch_controller.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <behaviortree_cpp/bt_factory.h>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace rtc_bt {

namespace test {
struct BridgeStateInjector;  // test-only state-inject seam (test/inject_fixture.hpp, issue #154)
}  // namespace test

/// Topic health status for watchdog monitoring.
struct TopicHealth {
  std::string name;
  bool received{false};             ///< true if at least one message received
  double seconds_since_last{-1.0};  ///< -1 means never received
  bool healthy{false};              ///< true if within timeout
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
  /// `profile` selects the arm/hand device groups (topic paths) and joint
  /// widths. The default {ur5e/hand, 6/10-DoF} reproduces the legacy behavior.
  explicit BtRosBridge(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                       RobotProfile profile = RobotProfile{});

  /// Runtime joint widths from the active RobotProfile (default 6 / 10).
  int ArmDof() const { return arm_dof_; }

  int HandDof() const { return hand_dof_; }

  // ── Cached state (thread-safe reads) ──────────────────────────────────────

  /// Current TCP pose from tf2 lookup, in the ACTIVE controller's task frame.
  /// Phase 4: replaces GuiPosition.task_positions consumer; the active
  /// controller broadcasts `<config_key>/transforms` (tf2_msgs/TFMessage),
  /// which `transforms_sub_` feeds into tf_buffer_ manually (the controller
  /// has no /tf publisher, so the bare listener alone would never see it).
  ///
  /// The child frame is NOT fixed (#292): a task controller with
  /// `virtual_tcp_mode` enabled controls a virtual TCP, and reading tool0 while
  /// it does that puts this pose in a different frame than the goals published
  /// through PublishArmTarget — which matters here more than in the GUI,
  /// because MoveToPose / TrackTrajectory judge *convergence* with it, so a
  /// frame mismatch is not a display bug but a goal that can never be reached.
  ///
  /// Always pair with IsTcpPoseValid(). After a controller rewire this returns
  /// the last-known pose, which belongs to the PREVIOUS controller until the
  /// new one broadcasts; treating that as live is how a convergence test
  /// succeeds against a pose nobody is holding.
  Pose6D GetTcpPose() const;

  /// True when GetTcpPose() is a live reading in the active controller's
  /// settled task frame. False before the frame settles, after a rewire until
  /// the new controller broadcasts, and whenever the lookup fails.
  [[nodiscard]] bool IsTcpPoseValid() const;

  /// The task frame GetTcpPose() reads, or empty while unsettled (#292).
  /// Exposed for the watchdog and for nodes that need to say which frame a
  /// waypoint would be compared against.
  [[nodiscard]] std::string GetTaskFrame() const;

  /// Current arm joint positions from /rtc_cm/ur5e/joint_states.
  /// Phase 4: CM publishes the per-group JointState regardless of active
  /// controller, so no rewire needed when controllers switch.
  std::vector<double> GetArmJointPositions() const;

  /// Current hand joint positions from /rtc_cm/hand/joint_states.
  std::vector<double> GetHandJointPositions() const;

  /// True once a structurally usable hand JointState has arrived: names
  /// non-empty, positions the same width as names, and that width equal to the
  /// configured hand DoF. This is a *readiness* query, deliberately distinct
  /// from GetFingerJointIndices() returning empty — the latter also means
  /// "unknown finger" (a config error). A gate waits on readiness; an unknown
  /// finger *after* readiness is a clear failure, not an indefinite wait (#161).
  [[nodiscard]] bool IsHandStateReady() const;

  /// Finger → hand-joint index list, derived at runtime from the hand
  /// joint_states names by prefix-matching `<key>_` (see FingerJointIndices in
  /// hand_pose_config.hpp). Supports variable per-finger DoF without hardcoding.
  /// Returns empty if no joint_states received yet or no name matches `key` —
  /// use IsHandStateReady() to tell those two apart.
  std::vector<int> GetFingerJointIndices(const std::string& key) const;

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
  bool GetObjectPose(Pose6D& pose) const;

  /// Latest world target position from /world_target_info (Polygon).
  /// Returns position only (orientation zeroed). Returns false if topic
  /// not received or all coordinates are zero.
  bool GetWorldTargetPose(Pose6D& pose) const;

  /// Active controller name
  std::string GetActiveController() const;

  /// E-STOP status
  bool IsEstopped() const;

  /// True once RewireControllerTopics has bound the controller-owned target
  /// publishers at least once. Callers use it to hold off work that would
  /// otherwise publish into unbound topics (#158).
  ///
  /// Monotone as far as any reader can tell — but NOT because the pubs survive.
  /// A controller switch resets and recreates them inside the rewire. What
  /// makes the latch hold is that the rewire and its readers all run on the
  /// single-threaded executor's mutually-exclusive callback group, so nothing
  /// can observe the window between the reset and the recreate. Give the rewire
  /// its own callback group, move to a multi-threaded executor, or split it
  /// across callbacks, and this can read false under a running tree — which is
  /// #158 again, with no test to catch it (the coverage only samples after
  /// OnActiveController returns). See the THREADING note on
  /// RewireControllerTopics.
  ///
  /// An empty name is ignored by the rewire outright, so a CM going down cannot
  /// re-close the latch either.
  [[nodiscard]] bool IsControllerWired() const;

  // ── Publishers (send commands) ────────────────────────────────────────────

  /// Publish arm task-space target [x,y,z,roll,pitch,yaw]
  void PublishArmTarget(const Pose6D& target);

  /// Publish arm joint-space target [q0..q5]
  void PublishArmJointTarget(const std::vector<double>& target);

  /// Publish hand motor target [m0..m9]
  void PublishHandTarget(const std::vector<double>& target);

  // ── Async service calls (non-blocking send/poll) ─────────────────────────
  //
  // The BT runs under a SingleThreadedExecutor (main.cpp `rclcpp::spin`) that
  // ticks the tree from a timer callback. A blocking service wait inside a tick
  // would deadlock: the same executor that must dispatch the service RESPONSE
  // is stuck in the tick waiting for it. These methods only SEND (fire the
  // request, return immediately); the caller (a StatefulActionNode) polls the
  // returned future with wait_for(0) across ticks, letting the executor
  // dispatch the response between ticks. See switch_controller / set_gains.

  using SwitchControllerFuture = rclcpp::Client<rtc_msgs::srv::SwitchController>::SharedFuture;
  using GraspCommandFuture = rclcpp::Client<rtc_msgs::srv::GraspCommand>::SharedFuture;
  using SetParametersFuture = std::shared_future<rcl_interfaces::msg::SetParametersResult>;

  /// Handle for an in-flight switch request: the response future plus the
  /// request id needed to cancel it on tree halt. `future.valid()==false` means
  /// the request was NOT sent (service not ready — caller retries or times out).
  struct SwitchRequestHandle {
    SwitchControllerFuture future;
    int64_t request_id{0};
  };

  /// Fire a controller switch via /rtc_cm/switch_controller without blocking.
  /// If the service is not ready this instant, returns an invalid future
  /// (`message` explains) so the caller can retry on a later tick until its own
  /// timeout — replacing the former 200ms blocking grace with poll-based grace.
  SwitchRequestHandle RequestSwitchControllerAsync(const std::string& name, std::string& message);

  /// Cancel an in-flight switch request (from onHalted). No-op for id 0 or an
  /// already-resolved request.
  void CancelSwitchControllerRequest(int64_t request_id);

  /// Fire a set_parameters_atomically on the active controller's rebound
  /// AsyncParametersClient without blocking. Returns an invalid future when
  /// there is no active controller or the parameter service is not ready
  /// (`message` explains); the caller retries/times out. Assumes `params`
  /// non-empty (callers guard the no-op case).
  SetParametersFuture SetActiveControllerGainsAsync(const std::vector<rclcpp::Parameter>& params,
                                                    std::string& message);

  /// Fire a one-shot Force-PI grasp command via the active controller's
  /// /<active>/grasp_command srv without blocking. `command` uses the
  /// rtc_msgs/GraspCommand constants (GRASP=1, RELEASE=2); `target_force` is
  /// ignored for RELEASE. Returns an invalid future when there is no active
  /// controller or the srv is not ready (`message` explains).
  GraspCommandFuture SendGraspCommandAsync(uint8_t command, double target_force,
                                           std::string& message);

  // ── Shape estimation ────────────────────────────────────────────────────

  /// Publish trigger command to /shape/trigger ("start", "stop", etc.)
  void PublishShapeTrigger(const std::string& command);

  /// Call /shape/clear service (async, non-blocking)
  void CallShapeClear();

  /// Get the latest cached ShapeEstimate message.
  /// Returns false if no estimate has been received yet.
  bool GetShapeEstimate(shape_estimation_msgs::msg::ShapeEstimate& out) const;

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
  const std::vector<rtc_msgs::msg::ToFSnapshot>& GetCollectedToFData() const;

  /// Number of snapshots currently in the buffer.
  std::size_t GetCollectedToFCount() const;

  // ── Pose library (runtime-configurable) ───────────────────────────────────

  /// Load hand/arm pose overrides from ROS2 parameters (deg → rad conversion).
  /// Parameters format: hand_pose.<name> = [double array], arm_pose.<name> =
  /// [double array]
  void LoadPoseOverrides(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /// Select the finger-index strategy (Seam C). If any `finger_map.<finger>`
  /// integer-array parameters are present, switch to an ExplicitFingerResolver
  /// built from them; otherwise keep the default PrefixFingerResolver. Call
  /// once in on_configure, before ticking.
  void LoadFingerMap(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /// Lookup a hand pose by name. Falls back to compile-time defaults.
  const HandPose& GetHandPose(const std::string& name) const;

  /// Lookup an arm pose by name. Falls back to compile-time defaults.
  const ArmPose& GetArmPose(const std::string& name) const;

  /// Get the full hand pose map (for iteration/validation).
  const std::map<std::string, HandPose>& GetHandPoses() const { return hand_poses_; }

  /// Get the full arm pose map.
  const std::map<std::string, ArmPose>& GetArmPoses() const { return arm_poses_; }

  // ── Topic health watchdog ─────────────────────────────────────────────────

  /// Get health status for all monitored topics.
  std::vector<TopicHealth> GetTopicHealth(double timeout_s = 2.0) const;

  /// Check if all critical topics (arm_gui, hand_gui) are healthy.
  bool AreTopicsHealthy(double timeout_s = 2.0) const;

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Robot-agnostic topic-name factory + joint widths (from RobotProfile).
  // Immutable after construction, so read without a lock.
  TopicNamer topic_namer_;
  int arm_dof_{kDefaultArmDof};
  int hand_dof_{kDefaultHandDof};

  // ── Controller-owned topic rewiring (Phase 4) ─────────────────────────────
  // Rebuild grasp_state/wbc_state/tof/arm_target/hand_target sub/pub against
  // the /<ctrl_name>/... namespace. No-op when ctrl_name is empty or
  // unchanged. (Phase 4: arm/hand GuiPosition subs are replaced by
  // /rtc_cm/<group>/joint_states which is controller-agnostic, so no rewire on
  // those.)
  //
  // controller_topics_mutex_ guards the fields read cross-method by the async
  // service senders: rewired_controller_ and the two rebindable clients
  // (active_param_client_ / grasp_command_client_). The sub/pub .reset()+create
  // sequence itself is NOT locked and relies on the single-thread executor
  // invariant (see the THREADING note on RewireControllerTopics in the .cpp).
  void RewireControllerTopics(const std::string& ctrl_name);

  /// Run the tf2 lookup for the active task frame and refresh tcp_pose_ /
  /// tcp_pose_valid_. Returns whether this call produced a live reading. Both
  /// GetTcpPose() and IsTcpPoseValid() go through it so neither depends on the
  /// other having been called first (#292).
  bool RefreshTcpPose() const;
  std::string rewired_controller_;
  mutable std::mutex controller_topics_mutex_;

  // ── Subscription handlers ─────────────────────────────────────────────────
  // Extracted callback bodies: every subscription lambda is a one-line forward
  // to exactly one handler, and the test-only BridgeStateInjector (friend
  // below) calls the same handlers — so the DDS-free inject path exercises the
  // identical parsing/mutex/health-stamp code as production, by construction.
  void OnArmJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void OnHandJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void OnWorldTarget(geometry_msgs::msg::Polygon::SharedPtr msg);
  void OnActiveController(std_msgs::msg::String::SharedPtr msg);
  void OnEstop(std_msgs::msg::Bool::SharedPtr msg);
  void OnShapeEstimate(shape_estimation_msgs::msg::ShapeEstimate::SharedPtr msg);
  void OnGraspState(rtc_msgs::msg::GraspState::SharedPtr msg);
  void OnWbcState(rtc_msgs::msg::WbcState::SharedPtr msg);
  void OnTransforms(tf2_msgs::msg::TFMessage::SharedPtr msg);
  void OnToFSnapshot(rtc_msgs::msg::ToFSnapshot::SharedPtr msg);
  friend struct test::BridgeStateInjector;

  // ── Subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr hand_joint_sub_;
  rclcpp::Subscription<rtc_msgs::msg::GraspState>::SharedPtr grasp_state_sub_;
  rclcpp::Subscription<rtc_msgs::msg::WbcState>::SharedPtr wbc_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr world_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_ctrl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr shape_estimate_sub_;

  // ── tf2 listener (TCP pose source, Phase 4) ───────────────────────────
  // The active controller broadcasts `base → tool0_actual` on
  // `<config_key>/transforms` (tf2_msgs/TFMessage), NOT on /tf — so the bare
  // TransformListener below never receives it. `transforms_sub_` (rewired per
  // active controller in RewireControllerTopics) feeds those frames into
  // tf_buffer_ manually so GetTcpPose's lookup resolves without depending on
  // any external /tf re-broadcaster (e.g. rtc_digital_twin). The listener is
  // kept only for any /tf_static frames a bringup might publish.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transforms_sub_;
  std::string tf_parent_frame_{"base"};

  // ── Task-frame selection (#292) ───────────────────────────────────────────
  // Which control point the active controller is actually using, decided from
  // AVAILABILITY rather than from a controller name: the controller broadcasts
  // kVirtualTcpFrame only on ticks it is controlling that point (integrated_
  // bringup owned_topics.cpp gates the slot on virtual_tcp_pose_valid), so
  // observing the frame at all is the evidence. A controller that never
  // publishes it — WBC, or task with the vtcp disabled — settles on the
  // fallback, and no controller list has to be kept in sync here.
  //
  // Mirrors integrated_bringup/demo_gui/task_frame.py, which makes the same
  // decision for the GUI. Deliberately duplicated rather than shared: that one
  // is Python, and this package is robot-specific while that one is not.
  static constexpr const char* kVirtualTcpFrame = "virtual_tcp_actual";
  // TFMessages carrying the fallback but no virtual TCP before the fallback is
  // latched. The window has to OUTLAST the closed-chain FK walk-in, during
  // which even a vtcp-configured controller publishes tool0 alone — latching
  // tool0 there is the exact error this exists to prevent. A measured p1b
  // walk-in is ~93 RT ticks and the controller emits at most one TFMessage per
  // tick (the CM's publish lane coalesces, never duplicates), so a window of N
  // messages spans at least N ticks and 150 clears 93 with margin at any wire
  // rate. Counting messages rather than seconds is what makes that argument
  // hold — the walk-in is ⌈Δ/max_seed_increment⌉ iterations long, so it scales
  // with ticks (the same reason integrated_bringup's kVtcpFrameWaitTicks is a
  // tick count). Kept in sync with demo_gui/task_frame.py DEFAULT_SETTLE_MSGS.
  static constexpr int kTaskFrameSettleMsgs = 150;

  // All four guarded by state_mutex_. tf_child_frame_ is empty until the frame
  // settles; that emptiness is what IsTcpPoseValid() reports as "not live yet".
  std::string tf_fallback_child_frame_{"tool0_actual"};
  std::string tf_child_frame_;
  int tf_fallback_seen_{0};
  // Mutable for the same reason tcp_pose_ is: GetTcpPose() is const and writes
  // both — the freshness of the cache is set by the same lookup that fills it.
  mutable bool tcp_pose_valid_{false};

  // ── Publishers ────────────────────────────────────────────────────────────
  // LifecyclePublisher, deliberately not the rclcpp::Publisher base: publish()
  // is virtual only on the derived type, so a base-typed handle calls straight
  // through to rclcpp::Publisher::publish and silently skips the is_activated()
  // check — an INACTIVE node would still put commands on the wire. The target
  // pubs additionally need an explicit on_activate() at creation; see
  // RewireControllerTopics.
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::RobotTarget>::SharedPtr arm_target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::RobotTarget>::SharedPtr hand_target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr shape_trigger_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shape_clear_client_;

  // ── /rtc_cm/* service clients (Phase 4) ─────────────────────────────────
  // Sync wrappers exposed via RequestSwitchController. list_controllers_
  // currently has no consumer in BT but is created so future diagnostics /
  // health-check nodes can reuse it without re-touching this file.
  rclcpp::Client<rtc_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<rtc_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

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
  // Mutable: GetTcpPose() is const but caches each successful tf2 lookup here
  // so a subsequent lookup failure returns the last-known pose, not zeros.
  mutable Pose6D tcp_pose_;
  std::vector<double> arm_joint_positions_;
  std::vector<double> hand_joint_positions_;
  std::vector<std::string> hand_joint_names_;  // from /rtc_cm/hand/joint_states name field
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
  // Live topic paths for the controller-owned health entries, updated in
  // RewireControllerTopics so the watchdog label matches the real subscribed
  // topic (namespaced by the active controller). Guarded by health_mutex_.
  std::string grasp_state_topic_;
  std::string wbc_state_topic_;
  TimePoint world_target_last_{};
  bool world_target_received_{false};
  TimePoint estop_last_{};
  bool estop_received_{false};

  // ── ToF data collection ────────────────────────────────────────────────────
  rclcpp::Subscription<rtc_msgs::msg::ToFSnapshot>::SharedPtr tof_snapshot_sub_;
  mutable std::mutex tof_mutex_;  ///< Guards tof_buffer_ only
  std::vector<rtc_msgs::msg::ToFSnapshot> tof_buffer_;
  std::atomic<bool> tof_collecting_{false};  ///< Checked in 500Hz callback

  // ── Pose library ──────────────────────────────────────────────────────────
  std::map<std::string, HandPose> hand_poses_;
  std::map<std::string, ArmPose> arm_poses_;

  // ── Finger-index strategy (Seam C) ────────────────────────────────────────
  // Defaults to PrefixFingerResolver; swapped to ExplicitFingerResolver by
  // LoadFingerMap() during on_configure when finger_map.* params exist. Set
  // before activation, so GetFingerJointIndices reads it without extra locking.
  std::unique_ptr<FingerResolver> finger_resolver_;
};

}  // namespace rtc_bt
