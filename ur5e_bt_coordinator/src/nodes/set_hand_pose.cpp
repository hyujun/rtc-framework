// file: src/nodes/set_hand_pose.cpp
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"

#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() {
  return ::rtc_bt::logging::ActionLogger("set_hand_pose");
}
}  // namespace

SetHandPose::SetHandPose(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList SetHandPose::providedPorts() {
  return {
      BT::InputPort<std::string>("pose", "명명된 Hand 포즈 (예: home, full_flex)"),
      BT::InputPort<double>("hand_trajectory_speed", kDefaultHandTrajectorySpeed,
                            "Trajectory speed [rad/s]"),
      BT::InputPort<double>("tolerance", kDefaultHandConvergenceTol,
                            "Per-joint convergence tolerance [rad]"),
      BT::InputPort<double>("timeout_s", kDefaultHandConvergenceTimeout,
                            "Convergence timeout [s] (FAILURE upper bound)"),
  };
}

BT::NodeStatus SetHandPose::onStart() {
  auto pose_name = getInput<std::string>("pose");
  if (!pose_name) {
    throw BT::RuntimeError("SetHandPose: missing pose: ", pose_name.error());
  }

  const double speed =
      getInput<double>("hand_trajectory_speed").value_or(kDefaultHandTrajectorySpeed);
  const double max_vel = kDefaultHandMaxTrajVelocity;
  tolerance_ = getInput<double>("tolerance").value_or(kDefaultHandConvergenceTol);
  timeout_s_ = getInput<double>("timeout_s").value_or(kDefaultHandConvergenceTimeout);

  const auto& target = bridge_->GetHandPose(pose_name.value());
  target_vec_.assign(target.begin(), target.end());

  // 현재 위치 읽기 → trajectory duration 추정
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(bridge_->HandDof())) {
    current.resize(static_cast<std::size_t>(bridge_->HandDof()), 0.0);
  }

  duration_ = EstimateHandTrajectoryDuration(current, target_vec_, speed, max_vel);

  // 목표 전송
  bridge_->PublishHandTarget(target_vec_);

  RCLCPP_INFO(logger(), "pose=%s estimated_duration=%.3fs (speed=%.2f)", pose_name.value().c_str(),
              duration_, speed);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetHandPose::onRunning() {
  const double elapsed = ElapsedSeconds(start_time_);
  if (elapsed < duration_) {
    return BT::NodeStatus::RUNNING;  // trajectory still in progress
  }

  // D4 convergence gate: after the estimated trajectory duration, confirm the
  // hand actually reached the target before reporting SUCCESS.
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

void SetHandPose::onHalted() {
  RCLCPP_INFO(logger(), "halted");
}

}  // namespace rtc_bt
