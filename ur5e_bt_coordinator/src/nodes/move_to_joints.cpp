#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"

#include <algorithm>
#include <cmath>

namespace rtc_bt {

MoveToJoints::MoveToJoints(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList MoveToJoints::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("target"),
    BT::InputPort<double>("tolerance", 0.01, "Per-joint tolerance [rad]"),
    BT::InputPort<double>("timeout_s", 10.0, "Timeout [s]"),
  };
}

BT::NodeStatus MoveToJoints::onStart()
{
  auto target = getInput<std::vector<double>>("target");
  if (!target) {
    throw BT::RuntimeError("MoveToJoints: missing target: ", target.error());
  }
  target_ = target.value();
  tolerance_ = getInput<double>("tolerance").value_or(0.01);
  timeout_s_ = getInput<double>("timeout_s").value_or(10.0);
  start_time_ = std::chrono::steady_clock::now();

  bridge_->PublishArmJointTarget(target_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToJoints::onRunning()
{
  auto current = bridge_->GetArmJointPositions();
  if (current.size() >= target_.size()) {
    double max_err = 0.0;
    for (std::size_t i = 0; i < target_.size(); ++i) {
      max_err = std::max(max_err, std::abs(current[i] - target_[i]));
    }
    if (max_err < tolerance_) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (std::chrono::duration<double>(elapsed).count() > timeout_s_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
