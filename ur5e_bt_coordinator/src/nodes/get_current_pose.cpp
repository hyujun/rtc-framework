#include "ur5e_bt_coordinator/action_nodes/get_current_pose.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("get_current_pose"); }
}  // namespace

GetCurrentPose::GetCurrentPose(const std::string& name, const BT::NodeConfig& config,
                               std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList GetCurrentPose::providedPorts()
{
  return {
    BT::OutputPort<Pose6D>("pose", "Current TCP pose [x,y,z,roll,pitch,yaw]"),
  };
}

BT::NodeStatus GetCurrentPose::tick()
{
  auto pose = bridge_->GetTcpPose();

  RCLCPP_DEBUG(logger(),
               "pose=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
               pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);

  setOutput("pose", pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
