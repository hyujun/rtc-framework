#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

IsObjectDetected::IsObjectDetected(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList IsObjectDetected::providedPorts()
{
  return {
    BT::OutputPort<Pose6D>("pose"),
  };
}

BT::NodeStatus IsObjectDetected::tick()
{
  Pose6D pose;
  if (bridge_->GetObjectPose(pose)) {
    // Position only from /world_target_info; use current TCP orientation
    auto tcp = bridge_->GetTcpPose();
    pose.roll  = tcp.roll;
    pose.pitch = tcp.pitch;
    pose.yaw   = tcp.yaw;

    RCLCPP_INFO(logger(),
                "[IsObjectDetected] object at [%.3f, %.3f, %.3f] "
                "orient(tcp)=[%.3f, %.3f, %.3f]",
                pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_DEBUG(logger(), "[IsObjectDetected] no object detected");
  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
