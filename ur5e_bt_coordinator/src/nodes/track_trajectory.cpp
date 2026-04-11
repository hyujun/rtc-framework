#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("track_trajectory"); }
}  // namespace

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
    RCLCPP_ERROR(logger(), "missing or empty waypoints");
    throw BT::RuntimeError("TrackTrajectory: missing or empty waypoints");
  }
  waypoints_ = wp.value();
  pos_tol_ = getInput<double>("position_tolerance").value_or(0.01);
  timeout_s_ = getInput<double>("timeout_s").value_or(30.0);
  current_idx_ = 0;
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(logger(), "%zu waypoints tol=%.4f timeout=%.1fs",
              waypoints_.size(), pos_tol_, timeout_s_);

  bridge_->PublishArmTarget(waypoints_[0]);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackTrajectory::onRunning()
{
  if (ElapsedSeconds(start_time_) > timeout_s_) {
    auto cur = bridge_->GetTcpPose();
    double err = current_idx_ < waypoints_.size()
        ? cur.PositionDistanceTo(waypoints_[current_idx_])
        : -1.0;
    RCLCPP_WARN(logger(),
                "timeout (%.1fs) at waypoint %zu/%zu (last_err=%.4f tol=%.4f)",
                timeout_s_, current_idx_, waypoints_.size(), err, pos_tol_);
    return BT::NodeStatus::FAILURE;
  }

  auto current = bridge_->GetTcpPose();
  double err = current.PositionDistanceTo(waypoints_[current_idx_]);

  if (err < pos_tol_) {
    ++current_idx_;
    if (current_idx_ >= waypoints_.size()) {
      RCLCPP_INFO(logger(), "all %zu waypoints reached (elapsed=%.2fs)",
                  waypoints_.size(), ElapsedSeconds(start_time_));
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_DEBUG(logger(), "waypoint %zu/%zu reached (err=%.4f)",
                 current_idx_, waypoints_.size(), err);
    bridge_->PublishArmTarget(waypoints_[current_idx_]);
  }

  return BT::NodeStatus::RUNNING;
}

void TrackTrajectory::onHalted()
{
  RCLCPP_INFO(logger(), "halted at waypoint %zu/%zu (elapsed=%.2fs)",
              current_idx_, waypoints_.size(), ElapsedSeconds(start_time_));
}

}  // namespace rtc_bt
