#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return rclcpp::get_logger("bt"); }
}  // namespace

SetGains::SetGains(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList SetGains::providedPorts()
{
  return {
    BT::InputPort<std::string>("kp_translation", "", "e.g. \"5.0,5.0,5.0\""),
    BT::InputPort<std::string>("kp_rotation", "", "e.g. \"3.0,3.0,3.0\""),
    BT::InputPort<double>("damping", "Damping ratio (default 0.01)"),
    BT::InputPort<double>("null_kp", "Null-space stiffness (default 0.5)"),
    BT::InputPort<bool>("enable_null_space", "Enable null-space control"),
    BT::InputPort<bool>("control_6dof", "Enable 6-DoF control"),
    BT::InputPort<double>("trajectory_speed"),
    BT::InputPort<double>("trajectory_angular_speed"),
    BT::InputPort<double>("max_traj_velocity"),
    BT::InputPort<double>("max_traj_angular_velocity"),
    BT::InputPort<double>("hand_trajectory_speed"),
    BT::InputPort<double>("hand_max_traj_velocity"),
    BT::InputPort<std::vector<double>>("full_gains"),
  };
}

BT::NodeStatus SetGains::tick()
{
  // If full_gains provided, use it directly
  auto full = getInput<std::vector<double>>("full_gains");
  if (full && full->size() >= 10) {
    RCLCPP_INFO(logger(), "[SetGains] publishing full_gains (%zu values)", full->size());
    bridge_->PublishGains(full.value());
    return BT::NodeStatus::SUCCESS;
  }

  // Build partial gains: DemoTaskController layout (16 values)
  // Start with defaults that preserve current behavior
  // [kp_t×3, kp_r×3, damping, null_kp, en_null, en_6dof,
  //  traj_speed, traj_ang_speed, hand_traj_speed,
  //  max_traj_vel, max_traj_ang_vel, hand_max_traj_vel]
  // Defaults must match demo_task_controller.yaml to avoid silent gain resets
  std::vector<double> gains = {
    40.0, 40.0, 40.0,   // kp_translation
    20.0, 20.0, 20.0,   // kp_rotation
    0.01,                // damping
    0.5,                 // null_kp
    0.0,                 // enable_null_space
    1.0,                 // control_6dof
    0.1,                 // trajectory_speed
    0.78,                // trajectory_angular_speed
    3.14,                // hand_trajectory_speed
    0.5,                 // max_traj_velocity
    1.57,                // max_traj_angular_velocity
    6.28,                // hand_max_traj_velocity
  };

  auto kp_t = getInput<std::string>("kp_translation");
  if (kp_t && !kp_t->empty()) {
    auto vals = ParseCsvList<double>(kp_t.value());
    for (std::size_t i = 0; i < std::min(vals.size(), std::size_t{3}); ++i) {
      gains[i] = vals[i];
    }
  }

  auto kp_r = getInput<std::string>("kp_rotation");
  if (kp_r && !kp_r->empty()) {
    auto vals = ParseCsvList<double>(kp_r.value());
    for (std::size_t i = 0; i < std::min(vals.size(), std::size_t{3}); ++i) {
      gains[3 + i] = vals[i];
    }
  }

  auto damp = getInput<double>("damping");
  if (damp) gains[6] = damp.value();

  auto nkp = getInput<double>("null_kp");
  if (nkp) gains[7] = nkp.value();

  auto ens = getInput<bool>("enable_null_space");
  if (ens) gains[8] = ens.value() ? 1.0 : 0.0;

  auto c6d = getInput<bool>("control_6dof");
  if (c6d) gains[9] = c6d.value() ? 1.0 : 0.0;

  auto ts = getInput<double>("trajectory_speed");
  if (ts) gains[10] = ts.value();

  auto tas = getInput<double>("trajectory_angular_speed");
  if (tas) gains[11] = tas.value();

  auto hts = getInput<double>("hand_trajectory_speed");
  if (hts) gains[12] = hts.value();

  auto mtv = getInput<double>("max_traj_velocity");
  if (mtv) gains[13] = mtv.value();

  auto mtav = getInput<double>("max_traj_angular_velocity");
  if (mtav) gains[14] = mtav.value();

  auto hmtv = getInput<double>("hand_max_traj_velocity");
  if (hmtv) gains[15] = hmtv.value();

  RCLCPP_INFO(logger(),
              "[SetGains] kp_t=[%.1f,%.1f,%.1f] kp_r=[%.1f,%.1f,%.1f] "
              "traj_speed=%.2f hand_speed=%.2f",
              gains[0], gains[1], gains[2], gains[3], gains[4], gains[5],
              gains[10], gains[12]);
  bridge_->PublishGains(gains);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
