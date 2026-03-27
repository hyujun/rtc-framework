#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <algorithm>
#include <cmath>

namespace rtc_bt {

GraspControl::GraspControl(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtRosBridge> bridge)
  : BT::StatefulActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList GraspControl::providedPorts()
{
  return {
    BT::InputPort<std::string>("mode", "close", "open|close|pinch|preset"),
    BT::InputPort<std::vector<double>>("target_positions"),
    BT::InputPort<double>("close_speed", 0.3, "Closing speed [rad/s]"),
    BT::InputPort<double>("max_position", 1.4, "Max motor position [rad]"),
    BT::InputPort<std::string>("pinch_motors", "0,1,2,3", "Motor indices for pinch"),
    BT::InputPort<double>("timeout_s", 8.0, "Timeout [s]"),
  };
}

BT::NodeStatus GraspControl::onStart()
{
  mode_ = getInput<std::string>("mode").value_or("close");
  close_speed_ = getInput<double>("close_speed").value_or(0.3);
  max_position_ = getInput<double>("max_position").value_or(1.4);
  timeout_s_ = getInput<double>("timeout_s").value_or(8.0);
  start_time_ = std::chrono::steady_clock::now();

  auto pinch_str = getInput<std::string>("pinch_motors").value_or("0,1,2,3");
  pinch_motor_indices_ = ParseCsvList<int>(pinch_str);

  if (mode_ == "open" || mode_ == "preset") {
    auto target = getInput<std::vector<double>>("target_positions");
    if (!target) {
      throw BT::RuntimeError("GraspControl: open/preset requires target_positions");
    }
    hand_target_ = target.value();
    bridge_->PublishHandTarget(hand_target_);
    return BT::NodeStatus::RUNNING;
  }

  // "close" or "pinch": start from current hand position
  hand_target_ = bridge_->GetHandJointPositions();
  if (hand_target_.size() < static_cast<std::size_t>(kHandDof)) {
    hand_target_.resize(kHandDof, 0.0);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GraspControl::onRunning()
{
  if (ElapsedSeconds(start_time_) > timeout_s_) {
    return BT::NodeStatus::FAILURE;
  }

  if (mode_ == "open" || mode_ == "preset") {
    // Check if hand reached target
    auto current = bridge_->GetHandJointPositions();
    if (current.size() >= hand_target_.size()) {
      double max_err = 0.0;
      for (std::size_t i = 0; i < hand_target_.size(); ++i) {
        max_err = std::max(max_err, std::abs(current[i] - hand_target_[i]));
      }
      if (max_err < 0.05) {
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  // "close" or "pinch": increment targets
  double increment = close_speed_ * tick_dt_;
  bool all_at_max = true;

  if (mode_ == "close") {
    for (std::size_t i = 0; i < hand_target_.size(); ++i) {
      hand_target_[i] = std::min(hand_target_[i] + increment, max_position_);
      if (hand_target_[i] < max_position_ - 0.01) all_at_max = false;
    }
  } else {  // pinch
    for (int idx : pinch_motor_indices_) {
      auto ui = static_cast<std::size_t>(idx);
      if (ui < hand_target_.size()) {
        hand_target_[ui] = std::min(hand_target_[ui] + increment, max_position_);
        if (hand_target_[ui] < max_position_ - 0.01) all_at_max = false;
      }
    }
  }

  bridge_->PublishHandTarget(hand_target_);

  // All motors at max without external stop → FAILURE (object not grasped)
  if (all_at_max) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace rtc_bt
