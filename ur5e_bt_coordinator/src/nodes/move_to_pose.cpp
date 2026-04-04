#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

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
    RCLCPP_ERROR(logger(), "[MoveToPose] missing target port: %s", target.error().c_str());
    throw BT::RuntimeError("MoveToPose: missing target: ", target.error());
  }
  target_ = target.value();
  pos_tol_ = getInput<double>("position_tolerance").value_or(0.005);
  rot_tol_ = getInput<double>("orientation_tolerance").value_or(0.05);
  timeout_s_ = getInput<double>("timeout_s").value_or(10.0);
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(logger(),
              "[MoveToPose] target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] "
              "pos_tol=%.4f rot_tol=%.4f timeout=%.1fs",
              target_.x, target_.y, target_.z,
              target_.roll, target_.pitch, target_.yaw,
              pos_tol_, rot_tol_, timeout_s_);

  bridge_->PublishArmTarget(target_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPose::onRunning()
{
  auto current = bridge_->GetTcpPose();
  double pos_err = current.PositionDistanceTo(target_);
  double rot_err = current.OrientationDistanceTo(target_);

  if (pos_err < pos_tol_ && rot_err < rot_tol_) {
    RCLCPP_INFO(logger(), "[MoveToPose] reached target (pos_err=%.4f rot_err=%.4f elapsed=%.2fs)",
                pos_err, rot_err, ElapsedSeconds(start_time_));
    return BT::NodeStatus::SUCCESS;
  }

  double elapsed = ElapsedSeconds(start_time_);
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(logger(),
                "[MoveToPose] timeout (%.1fs) pos_err=%.4f rot_err=%.4f",
                timeout_s_, pos_err, rot_err);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(logger(), "[MoveToPose] pos_err=%.4f rot_err=%.4f elapsed=%.2fs",
               pos_err, rot_err, elapsed);

  return BT::NodeStatus::RUNNING;
}

void MoveToPose::onHalted()
{
  RCLCPP_INFO(logger(), "[MoveToPose] halted (elapsed=%.2fs)", ElapsedSeconds(start_time_));
}

}  // namespace rtc_bt
