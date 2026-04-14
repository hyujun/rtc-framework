#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Process collected ToF data from a search move and output a target pose.
///
/// ── Data flow ──
///   Input  : bridge->GetCollectedToFData()
///            - std::vector<rtc_msgs::msg::ToFSnapshot>
///            - Each snapshot contains:
///              distances[6]  (thumb_A/B, index_A/B, middle_A/B) [m]
///              valid[6]      (per-sensor validity)
///              tip_poses[3]  (fingertip SE3 in world frame)
///              stamp         (measurement timestamp)
///            - Index finger ToF: distances[2] (index_A), distances[3] (index_B)
///
///   Output : output_pose (Pose6D)
///            - x, y, z  = computed from processing result
///            - roll, pitch, yaw = preserved from current TCP orientation
///
/// ── Processing logic ──
///   NOT YET IMPLEMENTED. The stub returns the current TCP pose and logs a
///   warning. When implementing, fill the section between the data-input and
///   data-output markers in the .cpp file.
///
/// Output ports:
///   - output_pose (Pose6D): task controller goal [x, y, z, roll, pitch, yaw]
class ProcessSearchData : public BT::SyncActionNode {
public:
  ProcessSearchData(const std::string& name, const BT::NodeConfig& config,
                    std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
