#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"

namespace rtc_bt {

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
    throw BT::RuntimeError("ComputeOffsetPose: missing input_pose");
  }

  Pose6D result = pose.value();
  result.x += getInput<double>("offset_x").value_or(0.0);
  result.y += getInput<double>("offset_y").value_or(0.0);
  result.z += getInput<double>("offset_z").value_or(0.0);

  setOutput("output_pose", result);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
