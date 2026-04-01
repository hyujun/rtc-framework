#include "shape_estimation/shape_estimation_node.hpp"

#include <sstream>

namespace shape_estimation {

ShapeEstimationNode::ShapeEstimationNode()
    : Node("shape_estimation_node") {
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

  frame_id_ = get_parameter("frame_id").as_string();
  min_points_for_fitting_ = static_cast<int>(get_parameter("min_points_for_fitting").as_int());
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  const double viz_rate_hz = get_parameter("viz_rate_hz").as_double();

  // Callback group: MutuallyExclusive로 voxel_cloud_ 접근 직렬화
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  // ── Subscribers ────────────────────────────────────────────────────────────
  snapshot_sub_ = create_subscription<shape_estimation_msgs::msg::ToFSnapshot>(
      "/tof/snapshot",
      rclcpp::SensorDataQoS().keep_last(5),
      [this](shape_estimation_msgs::msg::ToFSnapshot::SharedPtr msg) {
        SnapshotCallback(std::move(msg));
      },
      sub_options);

  trigger_sub_ = create_subscription<std_msgs::msg::String>(
      "/shape/trigger",
      rclcpp::QoS(5).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        TriggerCallback(std::move(msg));
      });

  // ── Publishers ─────────────────────────────────────────────────────────────
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

  RCLCPP_INFO(get_logger(), "ShapeEstimationNode 초기화 완료. 상태: STOPPED");
}

void ShapeEstimationNode::DeclareParameters() {
  declare_parameter("voxel_resolution", 0.002);
  declare_parameter("max_points", 2048);
  declare_parameter("point_expiry_sec", 5.0);
  declare_parameter("flat_curvature_threshold", 5.0);
  declare_parameter("curvature_uniformity_threshold", 2.0);
  declare_parameter("min_points_for_fitting", 10);
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("viz_rate_hz", 5.0);
  declare_parameter("frame_id", "base_link");
}

// ── Snapshot 콜백 ────────────────────────────────────────────────────────────

void ShapeEstimationNode::SnapshotCallback(
    shape_estimation_msgs::msg::ToFSnapshot::SharedPtr msg) {
  if (state_ != State::kRunning && state_ != State::kSingleShot) {
    return;
  }

  // msg → 내부 구조체 변환 (센서 위치, 표면점, 곡률 계산 포함)
  latest_snapshot_ = ConvertFromMsg(*msg);
  has_snapshot_ = true;

  // 포인트 클라우드 누적
  voxel_cloud_.AddSnapshot(latest_snapshot_);
  voxel_cloud_.RemoveExpired(latest_snapshot_.timestamp_ns);

  // 형상 추정
  latest_estimate_ = EstimateShape();

  // Estimate publish (rate limiting)
  const auto now_time = now();
  const double dt = (now_time - last_estimate_pub_time_).seconds();
  if (dt >= 1.0 / publish_rate_hz_) {
    std_msgs::msg::Header header;
    header.stamp = msg->stamp;
    header.frame_id = frame_id_;

    estimate_pub_->publish(ToMsg(latest_estimate_, latest_snapshot_, header));
    last_estimate_pub_time_ = now_time;
  }

  // 빔 + 곡률 시각화 (매 콜백)
  const auto stamp = msg->stamp;
  tof_beams_pub_->publish(viz::BuildBeamMarkers(latest_snapshot_, stamp, frame_id_));
  curvature_text_pub_->publish(
      viz::BuildCurvatureTextMarkers(latest_snapshot_, stamp, frame_id_));

  // SingleShot 모드: 1개 처리 후 Paused
  if (state_ == State::kSingleShot) {
    state_ = State::kPaused;
    RCLCPP_INFO(get_logger(), "SingleShot 완료 → PAUSED");
  }
}

// ── 형상 추정 ────────────────────────────────────────────────────────────────

ShapeEstimate ShapeEstimationNode::EstimateShape() {
  // Fast classification
  ShapeEstimate fast_result = fast_classifier_.Classify(
      latest_snapshot_.local_curvatures,
      latest_snapshot_.curvature_valid,
      latest_snapshot_.readings);

  // 충분한 포인트가 모이면 primitive fitting
  if (voxel_cloud_.Size() >= min_points_for_fitting_) {
    const auto points = voxel_cloud_.GetPoints();
    ShapeEstimate fitted_result = fitter_.FitBestPrimitive(points);

    // fast와 fitted 결과 결합: fitted 결과가 더 신뢰도가 높으면 사용
    if (fitted_result.type != ShapeType::kUnknown &&
        fitted_result.confidence >= fast_result.confidence) {
      return fitted_result;
    }
  }

  return fast_result;
}

// ── Trigger 콜백 ─────────────────────────────────────────────────────────────

void ShapeEstimationNode::TriggerCallback(std_msgs::msg::String::SharedPtr msg) {
  const auto& cmd = msg->data;

  if (cmd == "start") {
    voxel_cloud_.Clear();
    state_ = State::kRunning;
    RCLCPP_INFO(get_logger(), "형상 추정 시작 (point cloud 초기화)");
  } else if (cmd == "stop") {
    state_ = State::kStopped;
    RCLCPP_INFO(get_logger(), "형상 추정 중지");
  } else if (cmd == "pause") {
    state_ = State::kPaused;
    RCLCPP_INFO(get_logger(), "형상 추정 일시 정지");
  } else if (cmd == "resume") {
    state_ = State::kRunning;
    RCLCPP_INFO(get_logger(), "형상 추정 재개");
  } else if (cmd == "single") {
    state_ = State::kSingleShot;
    RCLCPP_INFO(get_logger(), "SingleShot 모드 활성화");
  } else {
    RCLCPP_WARN(get_logger(), "알 수 없는 trigger 명령: '%s'", cmd.c_str());
  }
}

// ── Clear 서비스 ─────────────────────────────────────────────────────────────

void ShapeEstimationNode::ClearCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const int n = voxel_cloud_.Size();
  voxel_cloud_.Clear();
  latest_estimate_ = ShapeEstimate{};
  has_snapshot_ = false;

  // DELETE 마커 publish
  primitive_marker_pub_->publish(viz::BuildDeleteAllMarkers());

  response->success = true;
  std::ostringstream oss;
  oss << "Point cloud cleared. " << n << " points removed.";
  response->message = oss.str();

  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

// ── 시각화 타이머 (5Hz) ──────────────────────────────────────────────────────

void ShapeEstimationNode::VizTimerCallback() {
  const auto stamp = now();
  builtin_interfaces::msg::Time ros_stamp;
  ros_stamp.sec = static_cast<int32_t>(stamp.seconds());
  ros_stamp.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1'000'000'000LL);

  // Point cloud publish
  const auto points = voxel_cloud_.GetPoints();
  point_cloud_pub_->publish(viz::BuildPointCloud2(points, ros_stamp, frame_id_));

  // Primitive marker publish
  primitive_marker_pub_->publish(
      viz::BuildPrimitiveMarkers(latest_estimate_, ros_stamp, frame_id_));
}

}  // namespace shape_estimation
