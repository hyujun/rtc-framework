#include "ur5e_bt_coordinator/condition_nodes/is_grasp_phase.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }

// GraspPhase enum values (matches rtc::grasp::GraspPhase)
const std::map<std::string, uint8_t> kPhaseMap = {
    {"idle",          0},
    {"approaching",   1},
    {"contact",       2},
    {"force_control", 3},
    {"holding",       4},
    {"releasing",     5},
};

const std::map<uint8_t, std::string> kPhaseNames = {
    {0, "Idle"},        {1, "Approaching"}, {2, "Contact"},
    {3, "ForceControl"},{4, "Holding"},     {5, "Releasing"},
};
}  // namespace

IsGraspPhase::IsGraspPhase(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<BtRosBridge> bridge)
  : BT::ConditionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList IsGraspPhase::providedPorts()
{
  return {
    BT::InputPort<std::string>("phase", "holding",
        "Target phase: idle|approaching|contact|force_control|holding|releasing"),
  };
}

BT::NodeStatus IsGraspPhase::tick()
{
  const auto target = getInput<std::string>("phase").value_or("holding");

  auto it = kPhaseMap.find(target);
  if (it == kPhaseMap.end()) {
    RCLCPP_ERROR(logger(),
                 "[IsGraspPhase] FAILURE: unknown phase '%s' "
                 "(valid: idle|approaching|contact|force_control|holding|releasing)",
                 target.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto gs = bridge_->GetGraspState();
  const bool match = (gs.grasp_phase == it->second);

  auto current_name = kPhaseNames.find(gs.grasp_phase);
  const char* current_str = (current_name != kPhaseNames.end())
                            ? current_name->second.c_str() : "Unknown";

  RCLCPP_DEBUG(logger(), "[IsGraspPhase] current=%s(%d) target=%s match=%s",
               current_str, gs.grasp_phase, target.c_str(),
               match ? "true" : "false");

  if (match) {
    RCLCPP_INFO(logger(), "[IsGraspPhase] phase '%s' reached", target.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // Throttle FAILURE log: IsGraspPhase is typically polled inside
  // RetryUntilSuccessful, so we don't want to flood the log every tick.
  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  RCLCPP_WARN_THROTTLE(
    logger(), steady_clock, 1000,
    "[IsGraspPhase] FAILURE: phase mismatch current=%s(%d) target=%s(%d). "
    "If this persists, check that grasp_controller_type='force_pi' is set "
    "in demo_shared.yaml and that the Force-PI state machine is actually "
    "running (expect controller log '[grasp:force_pi] phase X -> Y').",
    current_str, gs.grasp_phase, target.c_str(), it->second);

  return BT::NodeStatus::FAILURE;
}

}  // namespace rtc_bt
