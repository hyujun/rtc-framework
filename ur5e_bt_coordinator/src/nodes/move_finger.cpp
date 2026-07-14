// file: src/nodes/move_finger.cpp
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"

#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() {
  return ::rtc_bt::logging::ActionLogger("move_finger");
}
}  // namespace

MoveFinger::MoveFinger(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList MoveFinger::providedPorts() {
  return {
      BT::InputPort<std::string>("finger_name", "손가락 이름 (thumb/index/middle/ring)"),
      BT::InputPort<std::string>("pose", "명명된 타겟 포즈"),
      BT::InputPort<double>("hand_trajectory_speed", kDefaultHandTrajectorySpeed,
                            "Trajectory speed [rad/s]"),
      BT::InputPort<double>("tolerance", kDefaultHandConvergenceTol,
                            "Per-joint convergence tolerance [rad]"),
      BT::InputPort<double>("timeout_s", kDefaultHandConvergenceTimeout,
                            "Convergence timeout [s] (FAILURE upper bound)"),
  };
}

BT::NodeStatus MoveFinger::onStart() {
  auto finger_name = getInput<std::string>("finger_name");
  if (!finger_name) {
    throw BT::RuntimeError("MoveFinger: missing finger_name: ", finger_name.error());
  }

  auto pose_name = getInput<std::string>("pose");
  if (!pose_name) {
    throw BT::RuntimeError("MoveFinger: missing pose: ", pose_name.error());
  }

  const double speed =
      getInput<double>("hand_trajectory_speed").value_or(kDefaultHandTrajectorySpeed);
  const double max_vel = kDefaultHandMaxTrajVelocity;
  tolerance_ = getInput<double>("tolerance").value_or(kDefaultHandConvergenceTol);
  timeout_s_ = getInput<double>("timeout_s").value_or(kDefaultHandConvergenceTimeout);

  target_pose_ = bridge_->GetHandPose(pose_name.value());
  joint_indices_ = bridge_->GetFingerJointIndices(finger_name.value());
  if (joint_indices_.empty()) {
    throw BT::RuntimeError("MoveFinger: no hand joints for finger '" + finger_name.value() +
                           "' (joint_states not received or unknown finger)");
  }

  // 현재 위치 읽기 → trajectory duration 추정
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(bridge_->HandDof())) {
    current.resize(static_cast<std::size_t>(bridge_->HandDof()), 0.0);
  }

  duration_ = EstimateHandTrajectoryDuration(current, target_pose_, joint_indices_, speed, max_vel);

  // 목표 전송
  ApplyPartialHandTarget(*bridge_, target_pose_, joint_indices_);

  RCLCPP_INFO(logger(), "finger=%s pose=%s estimated_duration=%.3fs (speed=%.2f)",
              finger_name.value().c_str(), pose_name.value().c_str(), duration_, speed);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveFinger::onRunning() {
  const double elapsed = ElapsedSeconds(start_time_);
  if (elapsed < duration_) {
    return BT::NodeStatus::RUNNING;  // trajectory still in progress
  }

  // D4 convergence gate: confirm the commanded finger joints reached the target.
  const double max_err =
      MaxHandJointError(bridge_->GetHandJointPositions(), target_pose_, joint_indices_);
  switch (ClassifyHandConvergence(max_err, elapsed, tolerance_, timeout_s_)) {
    case HandConvergence::kConverged:
      RCLCPP_INFO(logger(), "complete (max_err=%.4f elapsed=%.2fs)", max_err, elapsed);
      return BT::NodeStatus::SUCCESS;
    case HandConvergence::kTimedOut:
      RCLCPP_WARN(logger(), "timeout (%.1fs) — not converged (max_err=%.4f tol=%.4f)", timeout_s_,
                  max_err, tolerance_);
      return BT::NodeStatus::FAILURE;
    case HandConvergence::kRunning:
      break;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveFinger::onHalted() {
  RCLCPP_INFO(logger(), "halted");
}

}  // namespace rtc_bt
