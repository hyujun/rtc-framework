// file: src/nodes/move_opposition.cpp
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("move_opposition"); }
} // namespace

MoveOpposition::MoveOpposition(const std::string &name,
                               const BT::NodeConfig &config,
                               std::shared_ptr<BtRosBridge> bridge)
    : BT::StatefulActionNode(name, config), bridge_(std::move(bridge)) {}

BT::PortsList MoveOpposition::providedPorts() {
  return {
      BT::InputPort<std::string>("thumb_pose", "엄지 포즈 이름"),
      BT::InputPort<std::string>("target_finger",
                                 "대상 손가락 이름 (index/middle/ring)"),
      BT::InputPort<std::string>("target_pose", "대상 손가락 포즈 이름"),
      BT::InputPort<double>("hand_trajectory_speed",
                            kDefaultHandTrajectorySpeed,
                            "Trajectory speed [rad/s]"),
  };
}

BT::NodeStatus MoveOpposition::onStart() {
  auto thumb_pose_name = getInput<std::string>("thumb_pose");
  if (!thumb_pose_name) {
    throw BT::RuntimeError("MoveOpposition: missing thumb_pose: ",
                           thumb_pose_name.error());
  }

  auto target_finger = getInput<std::string>("target_finger");
  if (!target_finger) {
    throw BT::RuntimeError("MoveOpposition: missing target_finger: ",
                           target_finger.error());
  }

  auto target_pose_name = getInput<std::string>("target_pose");
  if (!target_pose_name) {
    throw BT::RuntimeError("MoveOpposition: missing target_pose: ",
                           target_pose_name.error());
  }

  const double speed = getInput<double>("hand_trajectory_speed")
                           .value_or(kDefaultHandTrajectorySpeed);
  const double max_vel = kDefaultHandMaxTrajVelocity;

  const auto &thumb_pose = bridge_->GetHandPose(thumb_pose_name.value());
  const auto &target_pose = bridge_->GetHandPose(target_pose_name.value());
  const auto &target_indices = LookupOrThrow(
      kFingerJointIndices, target_finger.value(), "MoveOpposition");

  // 현재 위치 읽기
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(kHandDofCount)) {
    current.resize(kHandDofCount, 0.0);
  }

  // opposition 목표 전송 (비-target 손가락은 home으로 리셋)
  ApplyOppositionTarget(*bridge_, thumb_pose, target_pose, target_indices);

  // 전체 10-DoF 기준 duration 추정 (비-target의 home 복귀 이동도 포함)
  const auto &home = bridge_->GetHandPose("home");
  std::vector<double> full_target(home.begin(), home.end());
  for (int idx : kFingerJointIndices.at("thumb")) {
    full_target[static_cast<std::size_t>(idx)] =
        thumb_pose[static_cast<std::size_t>(idx)];
  }
  for (int idx : target_indices) {
    full_target[static_cast<std::size_t>(idx)] =
        target_pose[static_cast<std::size_t>(idx)];
  }
  duration_ =
      EstimateHandTrajectoryDuration(current, full_target, speed, max_vel);

  RCLCPP_INFO(logger(), "thumb=%s target=%s(%s) estimated_duration=%.3fs",
              thumb_pose_name.value().c_str(), target_finger.value().c_str(),
              target_pose_name.value().c_str(), duration_);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveOpposition::onRunning() {
  if (ElapsedSeconds(start_time_) >= duration_) {
    RCLCPP_INFO(logger(), "complete (%.3fs)", duration_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveOpposition::onHalted() { RCLCPP_INFO(logger(), "halted"); }

} // namespace rtc_bt
