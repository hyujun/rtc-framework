#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

namespace rtc_bt {

TrackTrajectory::TrackTrajectory(const std::string& name, const BT::NodeConfig& config,
                                 std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList TrackTrajectory::providedPorts()
{
  return {
    BT::InputPort<std::vector<Pose6D>>("waypoints"),
    BT::InputPort<double>("position_tolerance", 0.01, "Per-waypoint tolerance [m]"),
    BT::InputPort<double>("timeout_s", 30.0, "Total timeout [s]"),
  };
}

BT::NodeStatus TrackTrajectory::onStart()
{
  auto wp = getInput<std::vector<Pose6D>>("waypoints");
  if (!wp || wp->empty()) {
    throw BT::RuntimeError("TrackTrajectory: missing or empty waypoints");
  }
  waypoints_ = wp.value();
  pos_tol_ = getInput<double>("position_tolerance").value_or(0.01);
  timeout_s_ = getInput<double>("timeout_s").value_or(30.0);
  current_idx_ = 0;
  start_time_ = std::chrono::steady_clock::now();

  bridge_->PublishArmTarget(waypoints_[0]);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackTrajectory::onRunning()
{
  if (ElapsedSeconds(start_time_) > timeout_s_) {
    return BT::NodeStatus::FAILURE;
  }

  auto current = bridge_->GetTcpPose();
  double err = current.PositionDistanceTo(waypoints_[current_idx_]);

  if (err < pos_tol_) {
    ++current_idx_;
    if (current_idx_ >= waypoints_.size()) {
      return BT::NodeStatus::SUCCESS;
    }
    bridge_->PublishArmTarget(waypoints_[current_idx_]);
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
