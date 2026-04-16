#include "shape_estimation/shape_estimation_node.hpp"
#include "shape_estimation/shape_logging.hpp"

#include <sstream>

namespace shape_estimation {

namespace {
auto node_log() { return ::rtc::shape::logging::NodeLogger(); }
}  // namespace

ShapeEstimationNode::ShapeEstimationNode()
    : LifecycleNode("shape_estimation_node") {
  // Lifecycle design: constructor is intentionally empty.
  // All resource allocation happens in on_configure().
}

ShapeEstimationNode::CallbackReturn
ShapeEstimationNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
  DeclareParameters();

  // 파라미터로부터 설정 구성
  VoxelPointCloud::Config cloud_config;
  cloud_config.voxel_resolution_m = get_parameter("voxel_resolution").as_double();
  cloud_config.max_points = static_cast<int>(get_parameter("max_points").as_int());
  cloud_config.expiry_duration_sec = get_parameter("point_expiry_sec").as_double();
  voxel_cloud_ = VoxelPointCloud(cloud_config);

  FastShapeClassifier::Config classifier_config;
  classifier_config.flat_curvature_threshold =
      get_parameter("flat_curvature_threshold").as_double();
  classifier_config.curvature_uniformity_threshold =
      get_parameter("curvature_uniformity_threshold").as_double();
  fast_classifier_ = FastShapeClassifier(classifier_config);

  PrimitiveFitter::Config fitter_config;
  fitter_config.min_confidence = 0.3;
  fitter_ = PrimitiveFitter(fitter_config);

  // 돌출 구조 탐지기 설정
  ProtuberanceConfig prot_config;
  prot_config.residual_threshold =
      get_parameter("protuberance.residual_threshold").as_double();
  prot_config.min_cluster_points = static_cast<uint32_t>(
      get_parameter("protuberance.min_cluster_points").as_int());
  prot_config.cluster_radius =
      get_parameter("protuberance.cluster_radius").as_double();
  prot_config.gap_distance_jump =
      get_parameter("protuberance.gap_distance_jump").as_double();
  prot_config.min_gap_invalid_count = static_cast<uint32_t>(
      get_parameter("protuberance.min_gap_invalid_count").as_int());
  prot_config.curvature_jump_threshold =
      get_parameter("protuberance.curvature_jump_threshold").as_double();
  prot_config.gap_cluster_association_radius =
      get_parameter("protuberance.gap_cluster_association_radius").as_double();
  protuberance_detector_ = ProtuberanceDetector(prot_config);

  frame_id_ = get_parameter("frame_id").as_string();
  min_points_for_fitting_ = static_cast<int>(get_parameter("min_points_for_fitting").as_int());
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  const double viz_rate_hz = get_parameter("viz_rate_hz").as_double();

  // Callback group: MutuallyExclusive로 voxel_cloud_ 접근 직렬화
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  // ── Subscribers ────────────────────────────────────────────────────────────
  snapshot_sub_ = create_subscription<rtc_msgs::msg::ToFSnapshot>(
      "/tof/snapshot",
      rclcpp::SensorDataQoS().keep_last(5),
      [this](rtc_msgs::msg::ToFSnapshot::SharedPtr msg) {
        SnapshotCallback(std::move(msg));
      },
      sub_options);

  trigger_sub_ = create_subscription<std_msgs::msg::String>(
      "/shape/trigger",
      rclcpp::QoS(5).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        TriggerCallback(std::move(msg));
      });

  // ── Publishers (LifecyclePublisher) ────────────────────────────────────────
  estimate_pub_ = create_publisher<shape_estimation_msgs::msg::ShapeEstimate>(
      "/shape/estimate", rclcpp::QoS(10).reliable());

  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/shape/point_cloud", rclcpp::SensorDataQoS());

  primitive_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/shape/primitive_marker",
      rclcpp::QoS(1).reliable().transient_local());

  tof_beams_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/shape/tof_beams", rclcpp::SensorDataQoS());

  curvature_text_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/shape/curvature_text", rclcpp::SensorDataQoS());

  protuberance_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/shape/protuberance_marker",
      rclcpp::QoS(1).reliable().transient_local());

  // ── Service ────────────────────────────────────────────────────────────────
  clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "/shape/clear",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        ClearCallback(req, res);
      });

  // ── Timer (시각화 publish 주기) ────────────────────────────────────────────
  const auto viz_period = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / viz_rate_hz));
  viz_timer_ = create_wall_timer(viz_period, [this]() { VizTimerCallback(); }, cb_group_);

  last_estimate_pub_time_ = now();

  // ── 탐색 모션 초기화 (enable_exploration 파라미터에 따라) ──────────────────
  enable_exploration_ = get_parameter("enable_exploration").as_bool();
  if (enable_exploration_) {
    InitExploration();
    RCLCPP_INFO(node_log(), "ShapeEstimationNode configured. 탐색 모션 활성화");
  } else {
    RCLCPP_INFO(node_log(), "ShapeEstimationNode configured. 순수 추정 모드");
  }

  RCLCPP_INFO(node_log(),
              "설정: voxel_res=%.3fm, max_points=%d, expiry=%.1fs, "
              "min_fit_points=%d, pub_rate=%.1fHz, viz_rate=%.1fHz",
              cloud_config.voxel_resolution_m, cloud_config.max_points,
              cloud_config.expiry_duration_sec, min_points_for_fitting_,
              publish_rate_hz_, viz_rate_hz);
  return CallbackReturn::SUCCESS;
}

ShapeEstimationNode::CallbackReturn
ShapeEstimationNode::on_activate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_activate(state);
  state_ = State::kStopped;  // trigger 대기
  RCLCPP_INFO(node_log(), "ShapeEstimationNode activated");
  return CallbackReturn::SUCCESS;
}

ShapeEstimationNode::CallbackReturn
ShapeEstimationNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  // 탐색 중이면 중단
  if (action_active_) {
    motion_generator_.Abort();
    if (active_goal_handle_) {
      auto result = std::make_shared<ExploreShape::Result>();
      result->success = false;
      result->message = "Node deactivated";
      active_goal_handle_->abort(result);
    }
    StopExploration();
  }
  state_ = State::kStopped;
  if (explore_timer_) explore_timer_->cancel();
  if (delayed_start_timer_) delayed_start_timer_->cancel();
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(node_log(), "ShapeEstimationNode deactivated");
  return CallbackReturn::SUCCESS;
}

ShapeEstimationNode::CallbackReturn
ShapeEstimationNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
  // Reverse order of on_configure
  delayed_start_timer_.reset();
  explore_timer_.reset();
  explore_status_pub_.reset();
  sub_object_pose_.reset();
  sub_estop_.reset();
  sub_gui_position_.reset();
  pub_controller_gains_.reset();
  pub_robot_target_.reset();
  pub_controller_type_.reset();
  action_server_.reset();

  viz_timer_.reset();
  clear_srv_.reset();
  protuberance_marker_pub_.reset();
  curvature_text_pub_.reset();
  tof_beams_pub_.reset();
  primitive_marker_pub_.reset();
  point_cloud_pub_.reset();
  estimate_pub_.reset();
  trigger_sub_.reset();
  snapshot_sub_.reset();

  RCLCPP_INFO(node_log(), "ShapeEstimationNode cleaned up");
  return CallbackReturn::SUCCESS;
}

ShapeEstimationNode::CallbackReturn
ShapeEstimationNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
  }
  return on_cleanup(state);
}

ShapeEstimationNode::CallbackReturn
ShapeEstimationNode::on_error(const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_ERROR(node_log(), "ShapeEstimationNode error — attempting recovery");
  if (action_active_) {
    motion_generator_.Abort();
    StopExploration();
  }

  delayed_start_timer_.reset();
  explore_timer_.reset();
  explore_status_pub_.reset();
  sub_object_pose_.reset();
  sub_estop_.reset();
  sub_gui_position_.reset();
  pub_controller_gains_.reset();
  pub_robot_target_.reset();
  pub_controller_type_.reset();
  action_server_.reset();

  viz_timer_.reset();
  clear_srv_.reset();
  protuberance_marker_pub_.reset();
  curvature_text_pub_.reset();
  tof_beams_pub_.reset();
  primitive_marker_pub_.reset();
  point_cloud_pub_.reset();
  estimate_pub_.reset();
  trigger_sub_.reset();
  snapshot_sub_.reset();

  return CallbackReturn::SUCCESS;
}

void ShapeEstimationNode::DeclareParameters() {
  // 기존 파라미터
  declare_parameter("voxel_resolution", 0.002);
  declare_parameter("max_points", 2048);
  declare_parameter("point_expiry_sec", 5.0);
  declare_parameter("flat_curvature_threshold", 5.0);
  declare_parameter("curvature_uniformity_threshold", 2.0);
  declare_parameter("min_points_for_fitting", 10);
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("viz_rate_hz", 5.0);
  declare_parameter("frame_id", "base_link");

  // 돌출 구조 탐지 파라미터
  declare_parameter("protuberance.residual_threshold", -0.005);
  declare_parameter("protuberance.min_cluster_points", 3);
  declare_parameter("protuberance.cluster_radius", 0.015);
  declare_parameter("protuberance.gap_distance_jump", 0.020);
  declare_parameter("protuberance.min_gap_invalid_count", 2);
  declare_parameter("protuberance.curvature_jump_threshold", 15.0);
  declare_parameter("protuberance.gap_cluster_association_radius", 0.020);

  // 탐색 모션 on/off
  declare_parameter("enable_exploration", false);

  // 탐색 모션 파라미터
  declare_parameter("exploration.robot_namespace", "ur5e");
  declare_parameter("exploration.controller_name", "demo_task_controller");
  declare_parameter("exploration.controller_switch_delay_ms", 200);
  declare_parameter("exploration.explore_rate_hz", 10.0);

  // 탐색 게인 (16개 → 개별 파라미터로 선언)
  declare_parameter("exploration.exploration_gains.kp_translation",
                    std::vector<double>{10.0, 10.0, 10.0});
  declare_parameter("exploration.exploration_gains.kp_rotation",
                    std::vector<double>{5.0, 5.0, 5.0});
  declare_parameter("exploration.exploration_gains.damping", 0.01);
  declare_parameter("exploration.exploration_gains.null_kp", 0.0);
  declare_parameter("exploration.exploration_gains.enable_null_space", false);
  declare_parameter("exploration.exploration_gains.control_6dof", true);
  declare_parameter("exploration.exploration_gains.trajectory_speed", 0.05);
  declare_parameter("exploration.exploration_gains.trajectory_angular_speed", 0.3);
  declare_parameter("exploration.exploration_gains.hand_trajectory_speed", 0.0);
  declare_parameter("exploration.exploration_gains.max_traj_velocity", 0.10);
  declare_parameter("exploration.exploration_gains.max_traj_angular_velocity", 0.5);
  declare_parameter("exploration.exploration_gains.hand_max_traj_velocity", 0.0);

  // Phase 파라미터
  declare_parameter("exploration.approach_step_size", 0.005);
  declare_parameter("exploration.approach_timeout_sec", 5.0);
  declare_parameter("exploration.servo_target_distance", 0.030);
  declare_parameter("exploration.servo_step_gain", 0.5);
  declare_parameter("exploration.servo_max_step", 0.005);
  declare_parameter("exploration.servo_converge_tol", 0.003);
  declare_parameter("exploration.servo_timeout_sec", 3.0);
  declare_parameter("exploration.servo_min_valid_sensors", 3);
  declare_parameter("exploration.sweep_step_size", 0.003);
  declare_parameter("exploration.sweep_width", 0.06);
  declare_parameter("exploration.sweep_normal_gain", 0.5);
  declare_parameter("exploration.sweep_normal_max_step", 0.003);
  declare_parameter("exploration.tilt_amplitude_deg", 15.0);
  declare_parameter("exploration.tilt_steps", 10);
  declare_parameter("exploration.min_distance", 0.005);
  declare_parameter("exploration.max_step_size", 0.010);
  declare_parameter("exploration.confidence_threshold", 0.8);
  declare_parameter("exploration.min_points_for_success", 20);
  declare_parameter("exploration.max_total_time_sec", 10.0);
  declare_parameter("exploration.max_sweep_cycles", 3);

  // 센서 가중치 파라미터
  declare_parameter("exploration.finger_weights",
                    std::vector<double>{0.2, 0.4, 0.4});
  declare_parameter("exploration.min_classification_fingers", 1);
  declare_parameter("exploration.min_classification_coverage", 0.5);
}

// ═════════════════════════════════════════════════════════════════════════════
// 기존 콜백 (변경 없음)
// ═════════════════════════════════════════════════════════════════════════════

void ShapeEstimationNode::SnapshotCallback(
    rtc_msgs::msg::ToFSnapshot::SharedPtr msg) {
  if (state_ != State::kRunning && state_ != State::kSingleShot) {
    return;
  }

  // msg → 내부 구조체 변환 (센서 위치, 표면점, 곡률 계산 포함)
  latest_snapshot_ = ConvertFromMsg(*msg);

  // 유효 센서 수 확인
  int valid_count = 0;
  for (int i = 0; i < kTotalSensors; ++i) {
    if (latest_snapshot_.readings[static_cast<size_t>(i)].valid) {
      ++valid_count;
    }
  }
  if (valid_count == 0) {
    RCLCPP_WARN_THROTTLE(node_log(), *get_clock(),
                         ::rtc::shape::logging::kThrottleSlowMs,
                         "빈 snapshot 수신 (유효 센서 0개)");
  }
  has_snapshot_ = true;

  // ���인트 클라우드 누적
  voxel_cloud_.AddSnapshot(latest_snapshot_);

  // RemoveExpired throttle: 0.5초 간격 @500Hz
  if (++expire_counter_ >= kExpireInterval) {
    expire_counter_ = 0;
    voxel_cloud_.RemoveExpired(latest_snapshot_.timestamp_ns);
  }

  // Snapshot 시계열 저장 (돌출 구조 gap 분석용)
  snapshot_history_.Push(latest_snapshot_);

  RCLCPP_DEBUG_THROTTLE(node_log(), *get_clock(),
                        ::rtc::shape::logging::kThrottleSlowMs,
                        "SnapshotCallback: valid_sensors=%d, cloud_size=%d",
                        valid_count, voxel_cloud_.Size());

  // NOTE: 형상 추정(fitting) + 돌출 탐지 + estimate/시각화 publish는
  // VizTimerCallback (5-10Hz)에서 수행. 500Hz 콜백에서는 누적만 처리.

  // REMOVED: EstimateShape(), protuberance Detect(), estimate publish,
  // beam/curvature marker publish → moved to VizTimerCallback()

  // SingleShot 모드: 다음 VizTimerCallback에서 추정 후 Paused 전이
  if (state_ == State::kSingleShot) {
    state_ = State::kPaused;
    RCLCPP_INFO(node_log(), "SingleShot 완료 → PAUSED");
  }
}

ShapeEstimate ShapeEstimationNode::EstimateShape(
    const std::vector<PointWithNormal>& points) {
  // Fast classification (O(1))
  ShapeEstimate fast_result = fast_classifier_.Classify(
      latest_snapshot_.local_curvatures,
      latest_snapshot_.curvature_valid,
      latest_snapshot_.readings);

  // 충분한 포인트가 모이면 primitive fitting
  const auto n_points = static_cast<int>(points.size());
  if (n_points >= min_points_for_fitting_) {
    ShapeEstimate fitted_result = fitter_.FitBestPrimitive(points);

    // fast와 fitted 결과 결합: fitted 결과가 더 신뢰도가 높으면 사용
    if (fitted_result.type != ShapeType::kUnknown &&
        fitted_result.confidence >= fast_result.confidence) {
      RCLCPP_DEBUG_THROTTLE(node_log(), *get_clock(),
                            ::rtc::shape::logging::kThrottleFastMs,
                            "EstimateShape: fitted=%s(%.2f) 선택 (fast=%s(%.2f), points=%d)",
                            ShapeTypeToString(fitted_result.type).data(),
                            fitted_result.confidence,
                            ShapeTypeToString(fast_result.type).data(),
                            fast_result.confidence, n_points);
      return fitted_result;
    }
  }

  RCLCPP_DEBUG_THROTTLE(node_log(), *get_clock(),
                        ::rtc::shape::logging::kThrottleFastMs,
                        "EstimateShape: fast=%s(%.2f) 사용 (cloud=%d/%d)",
                        ShapeTypeToString(fast_result.type).data(),
                        fast_result.confidence,
                        n_points, min_points_for_fitting_);

  return fast_result;
}

void ShapeEstimationNode::TriggerCallback(std_msgs::msg::String::SharedPtr msg) {
  const auto& cmd = msg->data;

  if (cmd == "start") {
    voxel_cloud_.Clear();
    state_ = State::kRunning;
    RCLCPP_INFO(node_log(), "형상 추정 시작 (point cloud 초기화)");
  } else if (cmd == "stop") {
    state_ = State::kStopped;
    RCLCPP_INFO(node_log(), "형상 추정 중지");
  } else if (cmd == "pause") {
    state_ = State::kPaused;
    RCLCPP_INFO(node_log(), "형상 추정 일시 정지");
  } else if (cmd == "resume") {
    state_ = State::kRunning;
    RCLCPP_INFO(node_log(), "형상 추정 재개");
  } else if (cmd == "single") {
    state_ = State::kSingleShot;
    RCLCPP_INFO(node_log(), "SingleShot 모드 활성화");
  } else {
    RCLCPP_WARN(node_log(), "알 수 없는 trigger 명령: '%s'", cmd.c_str());
  }
}

void ShapeEstimationNode::ClearCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const int n = voxel_cloud_.Size();
  voxel_cloud_.Clear();
  snapshot_history_.Clear();
  latest_estimate_ = ShapeEstimate{};
  latest_protuberance_result_ = ProtuberanceResult{};
  has_snapshot_ = false;

  // DELETE 마커 publish
  primitive_marker_pub_->publish(viz::BuildDeleteAllMarkers());

  response->success = true;
  std::ostringstream oss;
  oss << "Point cloud cleared. " << n << " points removed.";
  response->message = oss.str();

  RCLCPP_INFO(node_log(), "%s", response->message.c_str());
}

void ShapeEstimationNode::VizTimerCallback() {
  const auto stamp = now();
  builtin_interfaces::msg::Time ros_stamp;
  ros_stamp.sec = static_cast<int32_t>(stamp.seconds());
  ros_stamp.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1'000'000'000LL);

  // ── 형상 추정 + 돌출 탐지 (SnapshotCallback에서 이동) ──────────────────────
  // GetPoints()를 1회만 호출하여 fitting, protuberance, visualization 모두에 재사용
  const auto points = voxel_cloud_.GetPoints();

  if (has_snapshot_) {
    latest_estimate_ = EstimateShape(points);

    // 돌출 구조 탐지
    if (latest_estimate_.type != ShapeType::kUnknown) {
      latest_protuberance_result_ = protuberance_detector_.Detect(
          latest_estimate_, points, snapshot_history_.Snapshots());
    } else {
      latest_protuberance_result_ = ProtuberanceResult{};
    }

    // Estimate publish
    std_msgs::msg::Header header;
    header.stamp.sec = ros_stamp.sec;
    header.stamp.nanosec = ros_stamp.nanosec;
    header.frame_id = frame_id_;
    estimate_pub_->publish(
        ToMsg(latest_estimate_, latest_snapshot_,
              latest_protuberance_result_, header));

    // 빔 + 곡률 시각화
    tof_beams_pub_->publish(
        viz::BuildBeamMarkers(latest_snapshot_, ros_stamp, frame_id_));
    curvature_text_pub_->publish(
        viz::BuildCurvatureTextMarkers(latest_snapshot_, ros_stamp, frame_id_));
  }

  // ── 시각화 publish ─────────────────────────────────────────────────────────
  // Point cloud publish
  point_cloud_pub_->publish(viz::BuildPointCloud2(points, ros_stamp, frame_id_));

  // Primitive marker publish
  primitive_marker_pub_->publish(
      viz::BuildPrimitiveMarkers(latest_estimate_, ros_stamp, frame_id_));

  // 돌출 구조 마커 publish
  protuberance_marker_pub_->publish(
      viz::BuildProtuberanceMarkers(
          latest_protuberance_result_, latest_estimate_, ros_stamp, frame_id_));
}

// ═════════════════════════════════════════════════════════════════════════════
// 탐색 모션 (enable_exploration: true일 때만 사용)
// ═════════════════════════════════════════════════════════════════════════════

void ShapeEstimationNode::InitExploration() {
  // 파라미터 로드
  robot_namespace_ = get_parameter("exploration.robot_namespace").as_string();
  controller_name_ = get_parameter("exploration.controller_name").as_string();
  controller_switch_delay_ms_ =
      static_cast<int>(get_parameter("exploration.controller_switch_delay_ms").as_int());

  // ExplorationConfig 로드
  auto config = LoadExplorationConfig();
  motion_generator_ = ExplorationMotionGenerator(config);

  // 게인 배열 로드
  exploration_gains_ = LoadExplorationGains();

  // ── Action Server ──────────────────────────────────────────────────────────
  action_server_ = rclcpp_action::create_server<ExploreShape>(
      this,
      "/shape/explore",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const ExploreShape::Goal> goal) {
        return HandleGoal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleExploreShape> goal_handle) {
        return HandleCancel(goal_handle);
      },
      [this](std::shared_ptr<GoalHandleExploreShape> goal_handle) {
        HandleAccepted(goal_handle);
      });

  // ── RT Controller 연동 publishers ──────────────────────────────────────────
  pub_controller_type_ = create_publisher<std_msgs::msg::String>(
      "/" + robot_namespace_ + "/controller_type",
      rclcpp::QoS(1).reliable());

  pub_robot_target_ = create_publisher<rtc_msgs::msg::RobotTarget>(
      "/" + robot_namespace_ + "/joint_goal",
      rclcpp::QoS(1).reliable());

  pub_controller_gains_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + robot_namespace_ + "/controller_gains",
      rclcpp::QoS(1).reliable());

  // ── 피드백 수신 subscribers ────────────────────────────────────────────────
  sub_gui_position_ = create_subscription<rtc_msgs::msg::GuiPosition>(
      "/" + robot_namespace_ + "/gui_position",
      rclcpp::SensorDataQoS(),
      [this](rtc_msgs::msg::GuiPosition::SharedPtr msg) {
        GuiPositionCallback(std::move(msg));
      });

  sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "/system/estop_status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        EstopCallback(std::move(msg));
      });

  sub_object_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object/pose_estimate",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ObjectPoseCallback(std::move(msg));
      });

  // ── 탐색 시각화 publisher ──────────────────────────────────────────────────
  explore_status_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/shape/explore_status", rclcpp::SensorDataQoS());

  // ── 탐색 루프 타이머 ──────────────────────────────────────────────────────
  const double explore_rate =
      get_parameter("exploration.explore_rate_hz").as_double();
  const auto explore_period = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / explore_rate));
  explore_timer_ = create_wall_timer(explore_period,
      [this]() { ExploreLoopCallback(); });
}

// ── Action Server 핸들러 ────────────────────────────────────────────────────

rclcpp_action::GoalResponse ShapeEstimationNode::HandleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExploreShape::Goal> /*goal*/) {
  if (action_active_) {
    RCLCPP_WARN(node_log(), "탐색이 이미 진행 중. 거부.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(node_log(), "ExploreShape 목표 수신");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ShapeEstimationNode::HandleCancel(
    const std::shared_ptr<GoalHandleExploreShape> /*goal_handle*/) {
  RCLCPP_INFO(node_log(), "ExploreShape 취소 요청");
  motion_generator_.Abort();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ShapeEstimationNode::HandleAccepted(
    std::shared_ptr<GoalHandleExploreShape> goal_handle) {
  active_goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();

  // Action goal에서 confidence/time override 적용
  if (goal->confidence_threshold > 0.0) {
    auto config = motion_generator_.GetConfig();
    config.confidence_threshold = goal->confidence_threshold;
    motion_generator_.SetConfig(config);
  }
  if (goal->max_time_sec > 0.0) {
    auto config = motion_generator_.GetConfig();
    config.max_total_time_sec = goal->max_time_sec;
    motion_generator_.SetConfig(config);
  }

  // 물체 위치 결정
  std::array<double, 3> object_pos;
  if (goal->use_current_object_pose && has_object_position_) {
    object_pos = latest_object_position_;
  } else {
    object_pos = {goal->object_position.x,
                  goal->object_position.y,
                  goal->object_position.z};
  }

  // 1) DemoTaskController로 전환
  auto switch_msg = std_msgs::msg::String();
  switch_msg.data = controller_name_;
  pub_controller_type_->publish(switch_msg);

  // 2) 탐색용 게인 설정
  auto gains_msg = std_msgs::msg::Float64MultiArray();
  gains_msg.data = exploration_gains_;
  pub_controller_gains_->publish(gains_msg);

  // 3) 컨트롤러 전환 대기 후 탐색 시작 (one-shot timer)
  // 멤버 변수로 타이머를 유지하여 lifetime 보장 + cancel 가능
  auto delay = std::chrono::milliseconds(controller_switch_delay_ms_);
  delayed_start_timer_ = create_wall_timer(delay, [this, object_pos]() {
    StartExploration(object_pos);
    // one-shot: 즉시 cancel
    delayed_start_timer_->cancel();
  });
}

// ── 탐색 시작/중지 ──────────────────────────────────────────────────────────

void ShapeEstimationNode::StartExploration(
    const std::array<double, 3>& object_position) {
  // Point cloud 초기화
  voxel_cloud_.Clear();
  state_ = State::kRunning;  // 형상 추정도 활성화
  action_active_ = true;

  // 현재 EE 위치에서 시작
  std::array<double, 6> current_pose = latest_gui_position_;
  if (!has_gui_position_) {
    RCLCPP_WARN(node_log(), "GuiPosition 미수신. 기본 위치로 시작.");
    current_pose = {0.4, 0.0, 0.3, 3.14, 0.0, 0.0};
  }

  motion_generator_.Start(current_pose, object_position);
  RCLCPP_INFO(node_log(), "탐색 시작: 물체=[%.3f, %.3f, %.3f]",
              object_position[0], object_position[1], object_position[2]);
}

void ShapeEstimationNode::StopExploration() {
  action_active_ = false;
  active_goal_handle_.reset();
}

// ── 탐색 루프 (10Hz) ────────────────────────────────────────────────────────

void ShapeEstimationNode::ExploreLoopCallback() {
  if (!action_active_) return;

  // E-STOP 검사
  if (estop_active_) {
    RCLCPP_WARN(node_log(), "E-STOP 활성 상태에서 탐색 중단");
    motion_generator_.Abort();
    SendActionResult(false, "E-STOP triggered");
    StopExploration();
    return;
  }

  // Goal cancellation 검사
  if (active_goal_handle_ && active_goal_handle_->is_canceling()) {
    motion_generator_.Abort();
    SendActionResult(false, "Cancelled by client");
    StopExploration();
    return;
  }

  // 현재 EE 자세
  std::array<double, 6> current_pose = latest_gui_position_;

  // 탐색 모션 step
  const double dt = 1.0 /
      get_parameter("exploration.explore_rate_hz").as_double();
  auto result = motion_generator_.Step(
      latest_snapshot_, latest_estimate_, current_pose, dt);

  // RobotTarget publish
  if (result.goal.valid) {
    rtc_msgs::msg::RobotTarget target;
    target.header.stamp = now();
    target.goal_type = "task";
    for (int i = 0; i < 6; ++i) {
      target.task_target[static_cast<size_t>(i)] = result.goal.pose[static_cast<size_t>(i)];
    }
    pub_robot_target_->publish(target);
  }

  // Action feedback
  PublishActionFeedback(result.phase, result.status_message);

  // 시각화
  {
    const auto stamp_now = now();
    builtin_interfaces::msg::Time ros_stamp;
    ros_stamp.sec = static_cast<int32_t>(stamp_now.seconds());
    ros_stamp.nanosec = static_cast<uint32_t>(
        stamp_now.nanoseconds() % 1'000'000'000LL);

    explore_status_pub_->publish(viz::BuildExploreStatusMarkers(
        result.phase, motion_generator_.Stats(), latest_estimate_,
        current_pose, ros_stamp, frame_id_));
    explore_status_pub_->publish(viz::BuildTargetGoalMarker(
        result.goal, ros_stamp, frame_id_));
  }

  // 종료 판정
  if (result.phase == ExplorePhase::kSucceeded) {
    SendActionResult(true, result.status_message);
    StopExploration();
  } else if (result.phase == ExplorePhase::kFailed ||
             result.phase == ExplorePhase::kAborted) {
    SendActionResult(false, result.status_message);
    StopExploration();
  }
}

// ── Action 결과/피드백 ──────────────────────────────────────────────────────

void ShapeEstimationNode::SendActionResult(
    bool success, const std::string& message) {
  if (!active_goal_handle_) return;

  auto result = std::make_shared<ExploreShape::Result>();
  result->success = success;
  result->message = message;

  // 형상 추정 결과 첨부
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;
  result->estimate = ToMsg(latest_estimate_, latest_snapshot_, header);

  const auto stats = motion_generator_.Stats();
  result->elapsed_sec = stats.elapsed_sec;
  result->total_snapshots_processed = stats.total_snapshots;
  result->sweep_cycles_completed = stats.sweep_cycles_completed;

  if (success) {
    active_goal_handle_->succeed(result);
    RCLCPP_INFO(node_log(), "탐색 성공: %s", message.c_str());
  } else if (active_goal_handle_->is_canceling()) {
    active_goal_handle_->canceled(result);
    RCLCPP_INFO(node_log(), "탐색 취소: %s", message.c_str());
  } else {
    active_goal_handle_->abort(result);
    RCLCPP_ERROR(node_log(), "탐색 실패: %s", message.c_str());
  }
}

void ShapeEstimationNode::PublishActionFeedback(
    ExplorePhase phase, const std::string& status_msg) {
  if (!active_goal_handle_) return;

  auto feedback = std::make_shared<ExploreShape::Feedback>();
  feedback->current_phase = static_cast<uint8_t>(phase);

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;
  feedback->current_estimate = ToMsg(latest_estimate_, latest_snapshot_, header);

  const auto stats = motion_generator_.Stats();
  feedback->elapsed_sec = stats.elapsed_sec;
  feedback->num_points_collected = static_cast<uint32_t>(voxel_cloud_.Size());
  feedback->status_message = status_msg;

  active_goal_handle_->publish_feedback(feedback);
}

// ── 피드백 수신 콜백 ────────────────────────────────────────────────────────

void ShapeEstimationNode::GuiPositionCallback(
    rtc_msgs::msg::GuiPosition::SharedPtr msg) {
  for (int i = 0; i < 6; ++i) {
    latest_gui_position_[static_cast<size_t>(i)] =
        msg->task_positions[static_cast<size_t>(i)];
  }
  has_gui_position_ = true;
}

void ShapeEstimationNode::EstopCallback(std_msgs::msg::Bool::SharedPtr msg) {
  estop_active_ = msg->data;
}

void ShapeEstimationNode::ObjectPoseCallback(
    geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  latest_object_position_ = {
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z};
  has_object_position_ = true;
}

// ── YAML 파라미터 로드 ──────────────────────────────────────────────────────

ExplorationConfig ShapeEstimationNode::LoadExplorationConfig() {
  ExplorationConfig config;
  config.approach_step_size = get_parameter("exploration.approach_step_size").as_double();
  config.approach_timeout_sec = get_parameter("exploration.approach_timeout_sec").as_double();
  config.servo_target_distance = get_parameter("exploration.servo_target_distance").as_double();
  config.servo_step_gain = get_parameter("exploration.servo_step_gain").as_double();
  config.servo_max_step = get_parameter("exploration.servo_max_step").as_double();
  config.servo_converge_tol = get_parameter("exploration.servo_converge_tol").as_double();
  config.servo_timeout_sec = get_parameter("exploration.servo_timeout_sec").as_double();
  config.servo_min_valid_sensors =
      static_cast<uint8_t>(get_parameter("exploration.servo_min_valid_sensors").as_int());
  config.sweep_step_size = get_parameter("exploration.sweep_step_size").as_double();
  config.sweep_width = get_parameter("exploration.sweep_width").as_double();
  config.sweep_normal_gain = get_parameter("exploration.sweep_normal_gain").as_double();
  config.sweep_normal_max_step = get_parameter("exploration.sweep_normal_max_step").as_double();
  config.tilt_amplitude_deg = get_parameter("exploration.tilt_amplitude_deg").as_double();
  config.tilt_steps =
      static_cast<uint32_t>(get_parameter("exploration.tilt_steps").as_int());
  config.min_distance = get_parameter("exploration.min_distance").as_double();
  config.max_step_size = get_parameter("exploration.max_step_size").as_double();
  config.confidence_threshold = get_parameter("exploration.confidence_threshold").as_double();
  config.min_points_for_success =
      static_cast<uint32_t>(get_parameter("exploration.min_points_for_success").as_int());
  config.max_total_time_sec = get_parameter("exploration.max_total_time_sec").as_double();
  config.max_sweep_cycles =
      static_cast<uint8_t>(get_parameter("exploration.max_sweep_cycles").as_int());

  // 센서 가중치
  const auto weights = get_parameter("exploration.finger_weights").as_double_array();
  if (weights.size() >= 3) {
    config.finger_weights = {weights[0], weights[1], weights[2]};
  }
  config.min_classification_fingers =
      static_cast<uint8_t>(get_parameter("exploration.min_classification_fingers").as_int());
  config.min_classification_coverage =
      get_parameter("exploration.min_classification_coverage").as_double();

  return config;
}

std::vector<double> ShapeEstimationNode::LoadExplorationGains() {
  // DemoTaskController 게인 레이아웃 (16개):
  // [kp_trans×3, kp_rot×3, damping, null_kp, enable_null_space(0/1),
  //  control_6dof(0/1), traj_speed, traj_angular_speed,
  //  hand_traj_speed, max_vel, max_angular_vel, hand_max_vel]
  std::vector<double> gains;
  gains.reserve(16);

  const auto kp_trans =
      get_parameter("exploration.exploration_gains.kp_translation").as_double_array();
  const auto kp_rot =
      get_parameter("exploration.exploration_gains.kp_rotation").as_double_array();

  for (const auto& v : kp_trans) gains.push_back(v);
  for (const auto& v : kp_rot) gains.push_back(v);

  gains.push_back(get_parameter("exploration.exploration_gains.damping").as_double());
  gains.push_back(get_parameter("exploration.exploration_gains.null_kp").as_double());
  gains.push_back(
      get_parameter("exploration.exploration_gains.enable_null_space").as_bool() ? 1.0 : 0.0);
  gains.push_back(
      get_parameter("exploration.exploration_gains.control_6dof").as_bool() ? 1.0 : 0.0);
  gains.push_back(
      get_parameter("exploration.exploration_gains.trajectory_speed").as_double());
  gains.push_back(
      get_parameter("exploration.exploration_gains.trajectory_angular_speed").as_double());
  gains.push_back(
      get_parameter("exploration.exploration_gains.hand_trajectory_speed").as_double());
  gains.push_back(
      get_parameter("exploration.exploration_gains.max_traj_velocity").as_double());
  gains.push_back(
      get_parameter("exploration.exploration_gains.max_traj_angular_velocity").as_double());
  gains.push_back(
      get_parameter("exploration.exploration_gains.hand_max_traj_velocity").as_double());

  return gains;
}

}  // namespace shape_estimation
