#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

BT::PortsList WaitDuration::providedPorts()
{
  return {
    BT::InputPort<double>("duration_s", 0.5, "Wait duration [s]"),
  };
}

BT::NodeStatus WaitDuration::onStart()
{
  duration_s_ = getInput<double>("duration_s").value_or(0.5);
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(logger(), "[WaitDuration] waiting %.3fs", duration_s_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDuration::onRunning()
{
  if (ElapsedSeconds(start_time_) >= duration_s_) {
    RCLCPP_DEBUG(logger(), "[WaitDuration] done (%.3fs)", duration_s_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
