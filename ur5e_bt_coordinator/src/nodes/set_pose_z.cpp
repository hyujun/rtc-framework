#include "ur5e_bt_coordinator/action_nodes/set_pose_z.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <limits>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

BT::PortsList SetPoseZ::providedPorts()
{
  return {
    BT::InputPort<Pose6D>("input_pose"),
    BT::InputPort<double>("z",
                          std::numeric_limits<double>::quiet_NaN(),
                          "Absolute Z [m]; NaN = disabled (pass-through)"),
    BT::OutputPort<Pose6D>("output_pose"),
  };
}

BT::NodeStatus SetPoseZ::tick()
{
  auto pose = getInput<Pose6D>("input_pose");
  if (!pose) {
    RCLCPP_ERROR(logger(), "[SetPoseZ] missing input_pose port");
    throw BT::RuntimeError("SetPoseZ: missing input_pose");
  }

  double z_override = getInput<double>("z").value_or(
      std::numeric_limits<double>::quiet_NaN());

  Pose6D result = pose.value();
  if (std::isnan(z_override)) {
    RCLCPP_DEBUG(logger(),
                 "[SetPoseZ] disabled (z=NaN), pass-through z=%.3f",
                 result.z);
  } else {
    RCLCPP_DEBUG(logger(),
                 "[SetPoseZ] override z: %.3f -> %.3f",
                 result.z, z_override);
    result.z = z_override;
  }

  setOutput("output_pose", result);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
