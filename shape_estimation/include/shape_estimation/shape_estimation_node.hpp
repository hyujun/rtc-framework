#pragma once

#include "shape_estimation/exploration_motion.hpp"
#include "shape_estimation/fast_shape_classifier.hpp"
#include "shape_estimation/msg_conversions.hpp"
#include "shape_estimation/primitive_fitter.hpp"
#include "shape_estimation/protuberance_detector.hpp"
#include "shape_estimation/rviz_markers.hpp"
#include "shape_estimation/snapshot_history.hpp"
#include "shape_estimation/tof_shape_types.hpp"
#include "shape_estimation/voxel_point_cloud.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_estimation_msgs/action/explore_shape.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace shape_estimation {

class ShapeEstimationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ExploreShape = shape_estimation_msgs::action::ExploreShape;
  using GoalHandleExploreShape = rclcpp_action::ServerGoalHandle<ExploreShape>;

  ShapeEstimationNode();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

private:
  // 상태 머신
  enum class State { kStopped, kRunning, kPaused, kSingleShot };

  // ── 기존 콜백 ─────────────────────────────────────────────────────────────
  void SnapshotCallback(rtc_msgs::msg::ToFSnapshot::SharedPtr msg);
  void TriggerCallback(std_msgs::msg::String::SharedPtr msg);
  void
  ClearCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void VizTimerCallback();

  // 형상 추정 실행 (외부에서 GetPoints() 결과를 전달받아 중복 복사 방지)
  ShapeEstimate EstimateShape(const std::vector<PointWithNormal> &points);

  // 파라미터 선언
  void DeclareParameters();

  // ── 핵심 알고리즘 ──────────────────────────────────────────────────────────
  VoxelPointCloud voxel_cloud_;
  FastShapeClassifier fast_classifier_;
  PrimitiveFitter fitter_;
  ProtuberanceDetector protuberance_detector_;
  SnapshotHistory snapshot_history_;

  // ── 상태
  // ────────────────────────────────────────────────────────────────────
  State state_{State::kStopped};
  ToFSnapshot latest_snapshot_{};
  ShapeEstimate latest_estimate_{};
  ProtuberanceResult latest_protuberance_result_{};
  bool has_snapshot_{false};

  // ── ROS 인터페이스 (기존) ──────────────────────────────────────────────────
  // Subscribers
  rclcpp::Subscription<rtc_msgs::msg::ToFSnapshot>::SharedPtr snapshot_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;

  // Publishers (LifecyclePublisher — gated by lifecycle state)
  rclcpp_lifecycle::LifecyclePublisher<
      shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr estimate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr primitive_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr tof_beams_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr curvature_text_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr protuberance_marker_pub_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // ── 파라미터
  // ────────────────────────────────────────────────────────────────
  std::string frame_id_{"base_link"};
  int min_points_for_fitting_{10};
  double publish_rate_hz_{10.0};

  // Publish rate limiting
  rclcpp::Time last_estimate_pub_time_;

  // RemoveExpired throttle (500Hz에서 매번 호출 방지)
  uint32_t expire_counter_{0};
  static constexpr uint32_t kExpireInterval = 250; // 0.5초 간격 @500Hz

  // ═════════════════════════════════════════════════════════════════════════════
  // 탐색 모션 (enable_exploration: true일 때만 활성화)
  // ═════════════════════════════════════════════════════════════════════════════

  bool enable_exploration_{false};

  /// 탐색 관련 리소스 초기화 (enable_exploration=true일 때만 호출)
  void InitExploration();

  // ── Action Server ──────────────────────────────────────────────────────────
  rclcpp_action::Server<ExploreShape>::SharedPtr action_server_;

  rclcpp_action::GoalResponse
  HandleGoal(const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const ExploreShape::Goal> goal);
  rclcpp_action::CancelResponse
  HandleCancel(const std::shared_ptr<GoalHandleExploreShape> goal_handle);
  void
  HandleAccepted(const std::shared_ptr<GoalHandleExploreShape> goal_handle);

  // ── RT Controller 연동 ────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
      pub_controller_type_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::RobotTarget>::SharedPtr
      pub_robot_target_;

  // ── 피드백 수신 ───────────────────────────────────────────────────────────
  rclcpp::Subscription<rtc_msgs::msg::GuiPosition>::SharedPtr sub_gui_position_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_object_pose_;

  // ── Phase 4: controller namespace rewiring ──────────────────────────────
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_ctrl_sub_;
  std::string active_ctrl_name_;
  void RewireControllerTopics(const std::string &ctrl_name);

  // ── 탐색 시각화 ───────────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr explore_status_pub_;

  // ── 탐색 루프 타이머 ──────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr explore_timer_;

  // ── 컨트롤러 전환 지연 타이머 (one-shot) ──────────────────────────────────
  rclcpp::TimerBase::SharedPtr delayed_start_timer_;

  // ── 탐색 모션 생성기 ──────────────────────────────────────────────────────
  ExplorationMotionGenerator motion_generator_;

  // ── 탐색 상태 ─────────────────────────────────────────────────────────────
  std::shared_ptr<GoalHandleExploreShape> active_goal_handle_;
  bool action_active_{false};
  bool estop_active_{false};
  std::array<double, 6> latest_gui_position_{};
  bool has_gui_position_{false};
  std::array<double, 3> latest_object_position_{};
  bool has_object_position_{false};

  // ── 탐색 설정 ─────────────────────────────────────────────────────────────
  std::string robot_namespace_{"ur5e"};
  std::string controller_name_{"demo_task_controller"};
  int controller_switch_delay_ms_{200};

  // ── 탐색 콜백 ─────────────────────────────────────────────────────────────
  void ExploreLoopCallback();
  void GuiPositionCallback(rtc_msgs::msg::GuiPosition::SharedPtr msg);
  void EstopCallback(std_msgs::msg::Bool::SharedPtr msg);
  void ObjectPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // ── 탐색 유틸 ─────────────────────────────────────────────────────────────
  void StartExploration(const std::array<double, 3> &object_position);
  void StopExploration();
  void SendActionResult(bool success, const std::string &message = "");
  void PublishActionFeedback(ExplorePhase phase, const std::string &status_msg);

  // YAML에서 ExplorationConfig 로드
  ExplorationConfig LoadExplorationConfig();
};

} // namespace shape_estimation
