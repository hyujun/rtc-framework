#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"

namespace rtc_bt {

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
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDuration::onRunning()
{
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (std::chrono::duration<double>(elapsed).count() >= duration_s_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
