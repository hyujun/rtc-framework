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
    BT::InputPort<double>("duration", 1.0, "Trajectory duration [s]"),
  };
}

BT::NodeStatus SetHandPose::onStart()
{
  auto pose_name = getInput<std::string>("pose");
  if (!pose_name) {
    throw BT::RuntimeError("SetHandPose: missing pose: ", pose_name.error());
  }

  duration_ = getInput<double>("duration").value_or(1.0);

  const auto& target = LookupOrThrow(kHandPoses, pose_name.value(), "SetHandPose");

  std::vector<double> target_vec(target.begin(), target.end());
  bridge_->PublishHandTarget(target_vec);

  RCLCPP_INFO(rclcpp::get_logger("bt"),
              "[SetHandPose] pose=%s duration=%.2fs",
              pose_name.value().c_str(), duration_);

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
