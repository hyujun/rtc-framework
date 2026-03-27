// file: src/nodes/move_finger.cpp
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sstream>

namespace rtc_bt {

MoveFinger::MoveFinger(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList MoveFinger::providedPorts()
{
  return {
    BT::InputPort<std::string>("finger_name", "손가락 이름 (thumb/index/middle/ring)"),
    BT::InputPort<std::string>("pose", "명명된 타겟 포즈"),
    BT::InputPort<double>("duration", 1.0, "Trajectory duration [s]"),
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

  duration_ = getInput<double>("duration").value_or(1.0);

  const auto& target_pose = LookupOrThrow(kHandPoses, pose_name.value(), "MoveFinger");
  const auto& joint_indices = LookupOrThrow(kFingerJointIndices, finger_name.value(), "MoveFinger");

  ApplyPartialHandTarget(*bridge_, target_pose, joint_indices);

  RCLCPP_INFO(rclcpp::get_logger("bt"),
              "[MoveFinger] finger=%s pose=%s duration=%.2fs",
              finger_name.value().c_str(), pose_name.value().c_str(), duration_);

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveFinger::onRunning()
{
  if (ElapsedSeconds(start_time_) >= duration_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveFinger::onHalted()
{
  RCLCPP_INFO(rclcpp::get_logger("bt"), "[MoveFinger] halted");
}

}  // namespace rtc_bt
