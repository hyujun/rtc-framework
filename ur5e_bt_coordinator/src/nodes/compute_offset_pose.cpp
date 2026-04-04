#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

BT::PortsList ComputeOffsetPose::providedPorts()
{
  return {
    BT::InputPort<Pose6D>("input_pose"),
    BT::InputPort<double>("offset_x", 0.0, "X offset [m]"),
    BT::InputPort<double>("offset_y", 0.0, "Y offset [m]"),
    BT::InputPort<double>("offset_z", 0.0, "Z offset [m]"),
    BT::OutputPort<Pose6D>("output_pose"),
  };
}

BT::NodeStatus ComputeOffsetPose::tick()
{
  auto pose = getInput<Pose6D>("input_pose");
  if (!pose) {
    RCLCPP_ERROR(logger(), "[ComputeOffsetPose] missing input_pose port");
    throw BT::RuntimeError("ComputeOffsetPose: missing input_pose");
  }

  double dx = getInput<double>("offset_x").value_or(0.0);
  double dy = getInput<double>("offset_y").value_or(0.0);
  double dz = getInput<double>("offset_z").value_or(0.0);

  Pose6D result = pose.value();
  result.x += dx;
  result.y += dy;
  result.z += dz;

  RCLCPP_DEBUG(logger(),
               "[ComputeOffsetPose] offset=[%.3f, %.3f, %.3f] -> result=[%.3f, %.3f, %.3f]",
               dx, dy, dz, result.x, result.y, result.z);

  setOutput("output_pose", result);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
