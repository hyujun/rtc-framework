#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/bt_utils.hpp"

namespace rtc_bt {

SetGains::SetGains(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList SetGains::providedPorts()
{
  return {
    BT::InputPort<std::string>("kp_translation", "", "e.g. \"5.0,5.0,5.0\""),
    BT::InputPort<std::string>("kp_rotation", "", "e.g. \"3.0,3.0,3.0\""),
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
    bridge_->PublishGains(full.value());
    return BT::NodeStatus::SUCCESS;
  }

  // Build partial gains: DemoTaskController layout (16 values)
  // Start with defaults that preserve current behavior
  // [kp_t×3, kp_r×3, damping, null_kp, en_null, en_6dof,
  //  traj_speed, traj_ang_speed, hand_traj_speed,
  //  max_traj_vel, max_traj_ang_vel, hand_max_traj_vel]
  std::vector<double> gains = {
    15.0, 15.0, 15.0,   // kp_translation
    5.0, 5.0, 5.0,      // kp_rotation
    0.01,                // damping
    0.5,                 // null_kp
    0.0,                 // enable_null_space
    1.0,                 // control_6dof
    0.1,                 // trajectory_speed
    0.5,                 // trajectory_angular_speed
    1.0,                 // hand_trajectory_speed
    0.5,                 // max_traj_velocity
    1.0,                 // max_traj_angular_velocity
    2.0,                 // hand_max_traj_velocity
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

  bridge_->PublishGains(gains);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
