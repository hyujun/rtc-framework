// file: src/nodes/move_opposition.cpp
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"

#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() {
  return ::rtc_bt::logging::ActionLogger("move_opposition");
}
}  // namespace

MoveOpposition::MoveOpposition(const std::string& name, const BT::NodeConfig& config,
                               std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList MoveOpposition::providedPorts() {
  return {
      BT::InputPort<std::string>("thumb_pose", "엄지 포즈 이름"),
      BT::InputPort<std::string>("target_finger", "대상 손가락 이름 (index/middle/ring)"),
      BT::InputPort<std::string>("target_pose", "대상 손가락 포즈 이름"),
      BT::InputPort<double>("hand_trajectory_speed", kDefaultHandTrajectorySpeed,
                            "Trajectory speed [rad/s]"),
      BT::InputPort<double>("tolerance", kDefaultHandConvergenceTol,
                            "Per-joint convergence tolerance [rad]"),
      BT::InputPort<double>("timeout_s", kDefaultHandConvergenceTimeout,
                            "Convergence timeout [s] (FAILURE upper bound)"),
  };
}

BT::NodeStatus MoveOpposition::onStart() {
  auto thumb_pose_name = getInput<std::string>("thumb_pose");
  if (!thumb_pose_name) {
    throw BT::RuntimeError("MoveOpposition: missing thumb_pose: ", thumb_pose_name.error());
  }

  auto target_finger = getInput<std::string>("target_finger");
  if (!target_finger) {
    throw BT::RuntimeError("MoveOpposition: missing target_finger: ", target_finger.error());
  }

  auto target_pose_name = getInput<std::string>("target_pose");
  if (!target_pose_name) {
    throw BT::RuntimeError("MoveOpposition: missing target_pose: ", target_pose_name.error());
  }

  const double speed =
      getInput<double>("hand_trajectory_speed").value_or(kDefaultHandTrajectorySpeed);
  const double max_vel = kDefaultHandMaxTrajVelocity;
  tolerance_ = getInput<double>("tolerance").value_or(kDefaultHandConvergenceTol);
  timeout_s_ = getInput<double>("timeout_s").value_or(kDefaultHandConvergenceTimeout);

  const auto& thumb_pose = bridge_->GetHandPose(thumb_pose_name.value());
  const auto& target_pose = bridge_->GetHandPose(target_pose_name.value());
  const auto target_indices = bridge_->GetFingerJointIndices(target_finger.value());
  if (target_indices.empty()) {
    throw BT::RuntimeError("MoveOpposition: no hand joints for finger '" + target_finger.value() +
                           "' (joint_states not received or unknown finger)");
  }

  // 현재 위치 읽기
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(bridge_->HandDof())) {
    current.resize(static_cast<std::size_t>(bridge_->HandDof()), 0.0);
  }

  // opposition 목표 전송 (비-target 손가락은 home으로 리셋). 반환된 full-DoF cmd를
  // duration 추정에 재사용 — 동일 조합을 raw-index로 재계산하던 OOB-취약 루프 제거.
  target_vec_ = ApplyOppositionTarget(*bridge_, thumb_pose, target_pose, target_indices);

  // 전체 10-DoF 기준 duration 추정 (비-target의 home 복귀 이동도 포함)
  duration_ = EstimateHandTrajectoryDuration(current, target_vec_, speed, max_vel);

  RCLCPP_INFO(logger(), "thumb=%s target=%s(%s) estimated_duration=%.3fs",
              thumb_pose_name.value().c_str(), target_finger.value().c_str(),
              target_pose_name.value().c_str(), duration_);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveOpposition::onRunning() {
  const double elapsed = ElapsedSeconds(start_time_);
  if (elapsed < duration_) {
    return BT::NodeStatus::RUNNING;  // trajectory still in progress
  }

  // D4 convergence gate: confirm the composed opposition target was reached.
  const double max_err = MaxHandJointError(bridge_->GetHandJointPositions(), target_vec_);
  if (max_err < tolerance_) {
    RCLCPP_INFO(logger(), "complete (max_err=%.4f elapsed=%.2fs)", max_err, elapsed);
    return BT::NodeStatus::SUCCESS;
  }
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(logger(), "timeout (%.1fs) — not converged (max_err=%.4f tol=%.4f)", timeout_s_,
                max_err, tolerance_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveOpposition::onHalted() {
  RCLCPP_INFO(logger(), "halted");
}

}  // namespace rtc_bt
