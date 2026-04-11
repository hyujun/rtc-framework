#include "ur5e_bt_coordinator/condition_nodes/is_vision_target_ready.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

IsVisionTargetReady::IsVisionTargetReady(const std::string& name,
                                         const BT::NodeConfig& config,
                                         std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList IsVisionTargetReady::providedPorts()
{
  return {
    BT::OutputPort<Pose6D>("pose"),
  };
}

BT::NodeStatus IsVisionTargetReady::tick()
{
  Pose6D pose;
  if (bridge_->GetWorldTargetPose(pose)) {
    // Use current TCP orientation (position only from vision)
    auto tcp = bridge_->GetTcpPose();
    pose.roll  = tcp.roll;
    pose.pitch = tcp.pitch;
    pose.yaw   = tcp.yaw;

    RCLCPP_INFO(logger(),
                "[IsVisionTargetReady] target pos=[%.3f, %.3f, %.3f] "
                "orient(tcp)=[%.3f, %.3f, %.3f]",
                pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  RCLCPP_WARN_THROTTLE(
    logger(), steady_clock, 2000,
    "[IsVisionTargetReady] FAILURE: no valid target pose "
    "(waiting for /world_target_info)");
  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
