#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

MoveToJoints::MoveToJoints(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList MoveToJoints::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("target", "Joint target [rad] (ignored if pose_name set)"),
    BT::InputPort<std::string>("pose_name", "Named arm pose from poses.yaml (overrides target)"),
    BT::InputPort<double>("tolerance", 0.01, "Per-joint tolerance [rad]"),
    BT::InputPort<double>("timeout_s", 10.0, "Timeout [s]"),
  };
}

BT::NodeStatus MoveToJoints::onStart()
{
  auto pose_name = getInput<std::string>("pose_name");
  if (pose_name) {
    const auto& pose = bridge_->GetArmPose(pose_name.value());
    target_.assign(pose.begin(), pose.end());
  } else {
    auto target = getInput<std::vector<double>>("target");
    if (!target) {
      RCLCPP_ERROR(logger(), "[MoveToJoints] missing target or pose_name port: %s",
                   target.error().c_str());
      throw BT::RuntimeError("MoveToJoints: missing target or pose_name: ", target.error());
    }
    target_ = target.value();
  }
  tolerance_ = getInput<double>("tolerance").value_or(0.01);
  timeout_s_ = getInput<double>("timeout_s").value_or(10.0);
  start_time_ = std::chrono::steady_clock::now();

  // Format target for logging
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < target_.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << std::fixed << std::setprecision(3) << target_[i];
  }
  oss << "]";
  RCLCPP_INFO(logger(), "[MoveToJoints] target=%s tol=%.4f timeout=%.1fs",
              oss.str().c_str(), tolerance_, timeout_s_);

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
      RCLCPP_INFO(logger(), "[MoveToJoints] reached target (max_err=%.4f elapsed=%.2fs)",
                  max_err, ElapsedSeconds(start_time_));
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(logger(), "[MoveToJoints] max_err=%.4f elapsed=%.2fs",
                 max_err, ElapsedSeconds(start_time_));
  }

  if (ElapsedSeconds(start_time_) > timeout_s_) {
    RCLCPP_WARN(logger(), "[MoveToJoints] timeout (%.1fs)", timeout_s_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveToJoints::onHalted()
{
  RCLCPP_INFO(logger(), "[MoveToJoints] halted (elapsed=%.2fs)", ElapsedSeconds(start_time_));
}

}  // namespace rtc_bt
