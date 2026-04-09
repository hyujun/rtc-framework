// file: src/nodes/flex_extend_finger.cpp
#include "ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

namespace rtc_bt {

FlexExtendFinger::FlexExtendFinger(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList FlexExtendFinger::providedPorts()
{
  return {
    BT::InputPort<std::string>("finger_name", "손가락 이름 (thumb/index/middle/ring)"),
    BT::InputPort<double>("hand_trajectory_speed", kDefaultHandTrajectorySpeed,
                           "Trajectory speed [rad/s]"),
    BT::InputPort<std::vector<double>>("current_gains", "{current_gains}",
                                       "Cached gains from SwitchController"),
  };
}

BT::NodeStatus FlexExtendFinger::onStart()
{
  auto finger_name = getInput<std::string>("finger_name");
  if (!finger_name) {
    throw BT::RuntimeError("FlexExtendFinger: missing finger_name: ", finger_name.error());
  }
  finger_name_ = finger_name.value();

  speed_ = getInput<double>("hand_trajectory_speed")
               .value_or(kDefaultHandTrajectorySpeed);
  auto cached = getInput<std::vector<double>>("current_gains");
  max_vel_ = (cached && !cached->empty())
      ? ExtractHandMaxTrajVelocity(cached.value())
      : kDefaultHandMaxTrajVelocity;

  // 포즈 lookup (bridge pose library 사용)
  const std::string flex_pose_name = finger_name_ + "_flex";
  flex_target_ = bridge_->GetHandPose(flex_pose_name);
  home_target_ = bridge_->GetHandPose("home");
  joint_indices_ = LookupOrThrow(kFingerJointIndices, finger_name_, "FlexExtendFinger");

  // 현재 위치 → flex duration 추정
  auto current = bridge_->GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(kHandDofCount)) {
    current.resize(kHandDofCount, 0.0);
  }

  flex_duration_ = EstimateHandTrajectoryDuration(
      current, flex_target_, joint_indices_, speed_, max_vel_);
  extend_duration_ = 0.01;  // extend phase 시작 시 재계산

  // Phase 1 (Flex): flex 타겟 전송
  ApplyPartialHandTarget(*bridge_, flex_target_, joint_indices_);

  RCLCPP_INFO(rclcpp::get_logger("bt"),
              "[FlexExtendFinger] finger=%s flex_duration=%.3fs (speed=%.2f)",
              finger_name_.c_str(), flex_duration_, speed_);

  phase_ = Phase::kFlex;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FlexExtendFinger::onRunning()
{
  const double elapsed = ElapsedSeconds(start_time_);

  // Phase 전환: FLEX → EXTEND
  if (phase_ == Phase::kFlex && elapsed >= flex_duration_) {
    // 현재 위치 다시 읽어서 extend duration 추정
    auto current = bridge_->GetHandJointPositions();
    if (current.size() < static_cast<std::size_t>(kHandDofCount)) {
      current.resize(kHandDofCount, 0.0);
    }

    extend_duration_ = EstimateHandTrajectoryDuration(
        current, home_target_, joint_indices_, speed_, max_vel_);

    ApplyPartialHandTarget(*bridge_, home_target_, joint_indices_);

    RCLCPP_INFO(rclcpp::get_logger("bt"),
                "[FlexExtendFinger] finger=%s extend phase (%.3fs)",
                finger_name_.c_str(), extend_duration_);

    phase_ = Phase::kExtend;
  }

  // 전체 완료
  if (phase_ == Phase::kExtend && elapsed >= flex_duration_ + extend_duration_) {
    RCLCPP_INFO(rclcpp::get_logger("bt"),
                "[FlexExtendFinger] complete finger=%s (total=%.3fs)",
                finger_name_.c_str(), flex_duration_ + extend_duration_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void FlexExtendFinger::onHalted()
{
  RCLCPP_INFO(rclcpp::get_logger("bt"), "[FlexExtendFinger] halted (finger=%s)",
              finger_name_.c_str());
}

}  // namespace rtc_bt
