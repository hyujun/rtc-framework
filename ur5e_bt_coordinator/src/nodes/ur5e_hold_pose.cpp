// file: src/nodes/ur5e_hold_pose.cpp
#include "ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

UR5eHoldPose::UR5eHoldPose(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList UR5eHoldPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose", "명명된 UR5e 포즈 (예: demo_pose)"),
  };
}

BT::NodeStatus UR5eHoldPose::onStart()
{
  auto pose_name = getInput<std::string>("pose");
  if (!pose_name) {
    throw BT::RuntimeError("UR5eHoldPose: missing pose: ", pose_name.error());
  }

  const auto& target = LookupOrThrow(kUR5ePoses, pose_name.value(), "UR5eHoldPose");

  std::vector<double> target_vec(target.begin(), target.end());
  bridge_->PublishArmJointTarget(target_vec);

  RCLCPP_INFO(rclcpp::get_logger("bt"),
              "[UR5eHoldPose] pose=%s", pose_name.value().c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UR5eHoldPose::onRunning()
{
  // 의도적으로 SUCCESS를 반환하지 않음 — Parallel 부모에 의해 halt될 때까지 자세 유지
  return BT::NodeStatus::RUNNING;
}

void UR5eHoldPose::onHalted()
{
  RCLCPP_INFO(rclcpp::get_logger("bt"), "[UR5eHoldPose] halted");
}

}  // namespace rtc_bt
