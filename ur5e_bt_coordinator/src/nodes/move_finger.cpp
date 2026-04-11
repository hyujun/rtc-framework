// file: src/nodes/move_finger.cpp
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("move_finger"); }
}  // namespace

MoveFinger::MoveFinger(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList MoveFinger::providedPorts()
{
  return {
    BT::InputPort<std::string>("finger_name", "손가락 이름 (thumb/index/middle/ring)"),
    BT::InputPort<std::string>("pose", "명명된 타겟 포즈"),
    BT::InputPort<double>("hand_trajectory_speed", kDefaultHandTrajectorySpeed,
                           "Trajectory speed [rad/s]"),
    BT::InputPort<std::vector<double>>("current_gains", "{current_gains}",
                                       "Cached gains from SwitchController"),
  };
}

BT::NodeStatus MoveFinger::onStart()
{
  auto finger_name = getInput<std::string>("finger_name");
  if (!finger_name) {
    throw BT::RuntimeError("MoveFinger: missing finger_name: ", finger_name.error());
  }

  auto pose_name = getInput<std::string>("pose");
  if (!pose_name) {
    throw BT::RuntimeError("MoveFinger: missing pose: ", pose_name.error());
  }

  const double speed = getInput<double>("hand_trajectory_speed")
                           .value_or(kDefaultHandTrajectorySpeed);
  auto cached = getInput<std::vector<double>>("current_gains");
  const double max_vel = (cached && !cached->empty())
      ? ExtractHandMaxTrajVelocity(cached.value())
      : kDefaultHandMaxTrajVelocity;

  target_pose_ = bridge_->GetHandPose(pose_name.value());
  joint_indices_ = LookupOrThrow(kFingerJointIndices, finger_name.value(), "MoveFinger");

  // 현재 위치 읽기 → trajectory duration 추정
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(kHandDofCount)) {
    current.resize(kHandDofCount, 0.0);
  }

  duration_ = EstimateHandTrajectoryDuration(current, target_pose_,
                                              joint_indices_, speed, max_vel);

  // 목표 전송
  ApplyPartialHandTarget(*bridge_, target_pose_, joint_indices_);

  RCLCPP_INFO(logger(),
              "finger=%s pose=%s estimated_duration=%.3fs (speed=%.2f)",
              finger_name.value().c_str(), pose_name.value().c_str(), duration_, speed);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveFinger::onRunning()
{
  if (ElapsedSeconds(start_time_) >= duration_) {
    RCLCPP_INFO(logger(), "complete (%.3fs)", duration_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveFinger::onHalted()
{
  RCLCPP_INFO(logger(), "halted");
}

}  // namespace rtc_bt
