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
    BT::InputPort<double>("duration", 1.0, "Flex+Extend 전체 시간 [s]"),
  };
}

BT::NodeStatus FlexExtendFinger::onStart()
{
  auto finger_name = getInput<std::string>("finger_name");
  if (!finger_name) {
    throw BT::RuntimeError("FlexExtendFinger: missing finger_name: ", finger_name.error());
  }
  finger_name_ = finger_name.value();

  duration_ = getInput<double>("duration").value_or(1.0);
  // 안전 clamp: 최소 duration 보장
  duration_ = std::max(duration_, kMinDuration);

  // Phase 1 (Flex): 해당 손가락을 flex 타겟으로 이동
  const std::string flex_pose_name = finger_name_ + "_flex";
  const auto& flex_pose = LookupOrThrow(kHandPoses, flex_pose_name, "FlexExtendFinger");
  const auto& joint_indices = LookupOrThrow(kFingerJointIndices, finger_name_, "FlexExtendFinger");

  ApplyPartialHandTarget(*bridge_, flex_pose, joint_indices);

  RCLCPP_INFO(rclcpp::get_logger("bt"),
              "[FlexExtendFinger] finger=%s duration=%.2fs (flex phase: %.2fs)",
              finger_name_.c_str(), duration_, duration_ / 2.0);

  phase_ = Phase::kFlex;
  extend_sent_ = false;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FlexExtendFinger::onRunning()
{
  double elapsed_s = ElapsedSeconds(start_time_);
  double half_duration = duration_ / 2.0;

  // Phase 전환: FLEX → EXTEND
  if (phase_ == Phase::kFlex && elapsed_s >= half_duration && !extend_sent_) {
    const auto& home_pose = LookupOrThrow(kHandPoses, std::string("home"), "FlexExtendFinger");
    const auto& joint_indices = LookupOrThrow(kFingerJointIndices, finger_name_, "FlexExtendFinger");

    ApplyPartialHandTarget(*bridge_, home_pose, joint_indices);

    RCLCPP_INFO(rclcpp::get_logger("bt"),
                "[FlexExtendFinger] finger=%s extend phase (%.2fs)",
                finger_name_.c_str(), half_duration);

    phase_ = Phase::kExtend;
    extend_sent_ = true;
  }

  // 전체 duration 완료
  if (elapsed_s >= duration_) {
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
