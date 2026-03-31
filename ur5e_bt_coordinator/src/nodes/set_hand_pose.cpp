// file: src/nodes/set_hand_pose.cpp
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

SetHandPose::SetHandPose(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList SetHandPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose", "명명된 Hand 포즈 (예: home, full_flex)"),
    BT::InputPort<double>("hand_trajectory_speed", kDefaultHandTrajectorySpeed,
                           "Trajectory speed [rad/s]"),
    BT::InputPort<double>("hand_max_traj_velocity", kDefaultHandMaxTrajVelocity,
                           "Max trajectory velocity [rad/s]"),
  };
}

BT::NodeStatus SetHandPose::onStart()
{
  auto pose_name = getInput<std::string>("pose");
  if (!pose_name) {
    throw BT::RuntimeError("SetHandPose: missing pose: ", pose_name.error());
  }

  const double speed = getInput<double>("hand_trajectory_speed")
                           .value_or(kDefaultHandTrajectorySpeed);
  const double max_vel = getInput<double>("hand_max_traj_velocity")
                             .value_or(kDefaultHandMaxTrajVelocity);

  const auto& target = LookupOrThrow(kHandPoses, pose_name.value(), "SetHandPose");
  target_vec_.assign(target.begin(), target.end());

  // 현재 위치 읽기 → trajectory duration 추정
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(kHandDofCount)) {
    current.resize(kHandDofCount, 0.0);
  }

  duration_ = EstimateHandTrajectoryDuration(current, target_vec_, speed, max_vel);

  // 목표 전송
  bridge_->PublishHandTarget(target_vec_);

  RCLCPP_INFO(rclcpp::get_logger("bt"),
              "[SetHandPose] pose=%s estimated_duration=%.3fs (speed=%.2f)",
              pose_name.value().c_str(), duration_, speed);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetHandPose::onRunning()
{
  if (ElapsedSeconds(start_time_) >= duration_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SetHandPose::onHalted()
{
  RCLCPP_INFO(rclcpp::get_logger("bt"), "[SetHandPose] halted");
}

}  // namespace rtc_bt
