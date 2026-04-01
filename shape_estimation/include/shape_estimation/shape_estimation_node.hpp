#pragma once

#include "shape_estimation/fast_shape_classifier.hpp"
#include "shape_estimation/msg_conversions.hpp"
#include "shape_estimation/primitive_fitter.hpp"
#include "shape_estimation/rviz_markers.hpp"
#include "shape_estimation/tof_shape_types.hpp"
#include "shape_estimation/voxel_point_cloud.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <shape_estimation_msgs/msg/to_f_snapshot.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#pragma GCC diagnostic pop

namespace shape_estimation {

class ShapeEstimationNode : public rclcpp::Node {
 public:
  ShapeEstimationNode();

 private:
  // 상태 머신
  enum class State { kStopped, kRunning, kPaused, kSingleShot };

  // 콜백
  void SnapshotCallback(shape_estimation_msgs::msg::ToFSnapshot::SharedPtr msg);
  void TriggerCallback(std_msgs::msg::String::SharedPtr msg);
  void ClearCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void VizTimerCallback();

  // 형상 추정 실행
  ShapeEstimate EstimateShape();

  // 파라미터 선언
  void DeclareParameters();

  // ── 핵심 알고리즘 ──────────────────────────────────────────────────────────
  VoxelPointCloud voxel_cloud_;
  FastShapeClassifier fast_classifier_;
  PrimitiveFitter fitter_;

  // ── 상태 ────────────────────────────────────────────────────────────────────
  State state_{State::kStopped};
  ToFSnapshot latest_snapshot_{};
  ShapeEstimate latest_estimate_{};
  bool has_snapshot_{false};

  // ── ROS 인터페이스 ─────────────────────────────────────────────────────────
  // Subscribers
  rclcpp::Subscription<shape_estimation_msgs::msg::ToFSnapshot>::SharedPtr snapshot_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;

  // Publishers
  rclcpp::Publisher<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr estimate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr primitive_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tof_beams_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_text_pub_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // ── 파라미터 ────────────────────────────────────────────────────────────────
  std::string frame_id_{"base_link"};
  int min_points_for_fitting_{10};
  double publish_rate_hz_{10.0};

  // Publish rate limiting
  rclcpp::Time last_estimate_pub_time_;
};

}  // namespace shape_estimation
