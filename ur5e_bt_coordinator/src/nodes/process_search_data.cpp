#include "ur5e_bt_coordinator/action_nodes/process_search_data.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("process_search_data"); }
}  // namespace

ProcessSearchData::ProcessSearchData(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList ProcessSearchData::providedPorts()
{
  return {
    BT::OutputPort<Pose6D>("output_pose",
                           "Task controller goal [x, y, z, roll, pitch, yaw]"),
  };
}

BT::NodeStatus ProcessSearchData::tick()
{
  // ══════════════════════════════════════════════════════════════════════════
  // Data Input
  // ══════════════════════════════════════════════════════════════════════════
  const auto& tof_buffer = bridge_->GetCollectedToFData();
  const auto  current_pose = bridge_->GetTcpPose();

  RCLCPP_INFO(logger(), "processing %zu ToF snapshots", tof_buffer.size());

  // Available per snapshot:
  //   tof_buffer[i].distances[2]  — index finger ToF A [m]
  //   tof_buffer[i].distances[3]  — index finger ToF B [m]
  //   tof_buffer[i].valid[2]      — index A validity
  //   tof_buffer[i].valid[3]      — index B validity
  //   tof_buffer[i].tip_poses[1]  — index fingertip SE3 (world frame)
  //   tof_buffer[i].stamp         — measurement timestamp

  // ══════════════════════════════════════════════════════════════════════════
  // Data Processing (TODO: implement)
  // ══════════════════════════════════════════════════════════════════════════

  double target_x = current_pose.x;
  double target_y = current_pose.y;
  double target_z = current_pose.z;

  RCLCPP_WARN(logger(),
              "processing logic not yet implemented — "
              "returning current TCP pose as fallback "
              "(x=%.3f, y=%.3f, z=%.3f)",
              target_x, target_y, target_z);

  // ══════════════════════════════════════════════════════════════════════════
  // Data Output
  // ══════════════════════════════════════════════════════════════════════════
  Pose6D result;
  result.x     = target_x;
  result.y     = target_y;
  result.z     = target_z;
  result.roll  = current_pose.roll;
  result.pitch = current_pose.pitch;
  result.yaw   = current_pose.yaw;

  setOutput("output_pose", result);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
