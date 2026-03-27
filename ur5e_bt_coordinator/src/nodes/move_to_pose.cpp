#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

namespace rtc_bt {

MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList MoveToPose::providedPorts()
{
  return {
    BT::InputPort<Pose6D>("target"),
    BT::InputPort<double>("position_tolerance", 0.005, "Position tolerance [m]"),
    BT::InputPort<double>("orientation_tolerance", 0.05, "Orientation tolerance [rad]"),
    BT::InputPort<double>("timeout_s", 10.0, "Timeout [s]"),
  };
}

BT::NodeStatus MoveToPose::onStart()
{
  auto target = getInput<Pose6D>("target");
  if (!target) {
    throw BT::RuntimeError("MoveToPose: missing target: ", target.error());
  }
  target_ = target.value();
  pos_tol_ = getInput<double>("position_tolerance").value_or(0.005);
  rot_tol_ = getInput<double>("orientation_tolerance").value_or(0.05);
  timeout_s_ = getInput<double>("timeout_s").value_or(10.0);
  start_time_ = std::chrono::steady_clock::now();

  bridge_->PublishArmTarget(target_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPose::onRunning()
{
  auto current = bridge_->GetTcpPose();
  double pos_err = current.PositionDistanceTo(target_);
  double rot_err = current.OrientationDistanceTo(target_);

  if (pos_err < pos_tol_ && rot_err < rot_tol_) {
    return BT::NodeStatus::SUCCESS;
  }

  if (ElapsedSeconds(start_time_) > timeout_s_) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
