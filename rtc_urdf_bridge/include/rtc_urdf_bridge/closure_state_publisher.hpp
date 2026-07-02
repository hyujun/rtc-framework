// ── closure_state_publisher: Extended-URDF 폐쇄 체인 시각화 노드 ────────────────
//    actuated JointState 스트림을 입력받아 measured actuated q 를 고정하고 passive q 를
//    closure 구속(ProjectPassiveToConstraint)으로 풀어 loop-consistent full q 를 만들어
//    RViz(robot_state_publisher)로 publish 한다. off-RT (콜백 추종, viz 도구).
#pragma once

#include "rtc_urdf_bridge/loop_projection.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rtc_urdf_bridge {

/// @brief actuated JointState → loop-consistent full JointState 재구성 노드.
///
/// configure(생성자): PinocchioModelBuilder(urdf + closure.yaml)로 full model + constraints +
///   actuated_joint_ids + q_ref(초기 passive seed) 일괄 획득. xacro 전처리 포함.
/// 입력 콜백: 이름→q-index 매핑으로 actuated 슬롯 갱신 → ProjectPassiveToConstraint(warm-start
///   = 직전 프레임 해) → 수렴 시 q_full_ 갱신, 전체 model 관절을 JointState 로 publish.
/// 미수렴/특이: 직전 해 hold + THROTTLE WARN (NaN publish 금지 — viz 는 loud 우선).
///
/// 로봇 비종속: 모든 robot identity 는 param(urdf/closure)에서 결정.
class ClosureStatePublisher : public rclcpp::Node {
 public:
  explicit ClosureStatePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /// closure model 빌드 + q_full_/캐시 초기화. 실패 시 std::runtime_error 전파.
  void BuildModel();

  /// 입력 actuated JointState 콜백: actuated 슬롯 갱신 → passive 사영 → publish.
  void OnJointState(const sensor_msgs::msg::JointState& msg);

  /// 현재 q_full_ 을 전체 model 관절 JointState 로 publish.
  void PublishState(const rclcpp::Time& stamp);

  // ── closure model (configure 시 1회 빌드) ───────────────────────────────────
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::RigidConstraintModel> constraints_;
  std::vector<pinocchio::JointIndex> actuated_joint_ids_;

  // loop-consistent full configuration (프레임 간 유지 = warm-start seed / hold 값).
  Eigen::VectorXd q_full_;

  // 입력 joint name → q index (nq==1 관절만). actuated 슬롯 갱신용.
  std::unordered_map<std::string, int> name_to_q_idx_;
  // 출력 관절 이름·q index (전체 model 의 nq==1 관절, universe 제외).
  std::vector<std::string> output_names_;
  std::vector<int> output_q_idx_;

  ProjectionOptions projection_opts_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

}  // namespace rtc_urdf_bridge
