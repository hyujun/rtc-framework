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
    BT::InputPort<int>("grasp_command", 0, "0=none, 1=grasp, 2=release (Force-PI)"),
    BT::InputPort<double>("grasp_target_force", 2.0, "Target grip force [N] (Force-PI)"),
    BT::InputPort<std::vector<double>>("current_gains",
                                       "Cached gains from SwitchController"),
  };
}

BT::NodeStatus SetGains::tick()
{
  // If full_gains provided, use it directly
  auto full = getInput<std::vector<double>>("full_gains");
  if (full && full->size() >= 4) {
    RCLCPP_INFO(logger(), "[SetGains] publishing full_gains (%zu values)", full->size());
    bridge_->PublishGains(full.value());
    return BT::NodeStatus::SUCCESS;
  }

  const auto active = bridge_->GetActiveController();
  const bool is_joint = (active == "DemoJointController" ||
                         active == "demo_joint_controller");

  if (is_joint) {
    return BuildDemoJointGains();
  }
  return BuildDemoTaskGains();
}

BT::NodeStatus SetGains::BuildDemoJointGains()
{
  // DemoJointController layout (7 + optional 2 values):
  // [robot_trajectory_speed, hand_trajectory_speed,
  //  robot_max_traj_velocity, hand_max_traj_velocity,
  //  grasp_contact_threshold, grasp_force_threshold,
  //  grasp_min_fingertips,
  //  (grasp_command, grasp_target_force)]   ← optional Force-PI
  constexpr std::size_t kBaseSize = 7;

  // Use cached gains as base if available and correctly sized
  auto cached = getInput<std::vector<double>>("current_gains");
  const bool use_cached = cached && cached->size() >= kBaseSize;

  std::vector<double> gains = use_cached
    ? std::vector<double>(cached->begin(),
                          cached->begin() + static_cast<std::ptrdiff_t>(kBaseSize))
    : std::vector<double>{
        0.1,    // [0] robot_trajectory_speed
        3.14,   // [1] hand_trajectory_speed
        0.5,    // [2] robot_max_traj_velocity
        6.28,   // [3] hand_max_traj_velocity
        0.5,    // [4] grasp_contact_threshold
        1.0,    // [5] grasp_force_threshold
        2.0,    // [6] grasp_min_fingertips
      };

  if (use_cached) {
    RCLCPP_INFO(logger(), "[SetGains] DemoJoint: using cached gains as base");
  }

  auto ts = getInput<double>("trajectory_speed");
  if (ts) gains[0] = ts.value();

  auto hts = getInput<double>("hand_trajectory_speed");
  if (hts) gains[1] = hts.value();

  auto mtv = getInput<double>("max_traj_velocity");
  if (mtv) gains[2] = mtv.value();

  auto hmtv = getInput<double>("hand_max_traj_velocity");
  if (hmtv) gains[3] = hmtv.value();

  // Force-PI grasp command (one-shot, only appended when non-zero)
  auto gcmd = getInput<int>("grasp_command");
  if (gcmd && gcmd.value() != 0) {
    auto gtf = getInput<double>("grasp_target_force");
    gains.push_back(static_cast<double>(gcmd.value()));
    gains.push_back(gtf.value_or(2.0));
    RCLCPP_INFO(logger(),
                "[SetGains] DemoJoint: grasp_command=%d target_force=%.2f",
                gcmd.value(), gains.back());
  }

  RCLCPP_INFO(logger(),
              "[SetGains] DemoJoint: robot_speed=%.2f hand_speed=%.2f "
              "max_vel=%.2f hand_max_vel=%.2f",
              gains[0], gains[1], gains[2], gains[3]);
  bridge_->PublishGains(gains);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetGains::BuildDemoTaskGains()
{
  // DemoTaskController layout (19 + optional 2 values):
  // [kp_t×3, kp_r×3, damping, null_kp, en_null, en_6dof,
  //  traj_speed, traj_ang_speed, hand_traj_speed,
  //  max_traj_vel, max_traj_ang_vel, hand_max_traj_vel,
  //  grasp_contact_threshold, grasp_force_threshold,
  //  grasp_min_fingertips,
  //  (grasp_command, grasp_target_force)]   ← optional Force-PI
  constexpr std::size_t kBaseSize = 19;

  // Use cached gains as base if available and correctly sized
  auto cached = getInput<std::vector<double>>("current_gains");
  const bool use_cached = cached && cached->size() >= kBaseSize;

  std::vector<double> gains = use_cached
    ? std::vector<double>(cached->begin(),
                          cached->begin() + static_cast<std::ptrdiff_t>(kBaseSize))
    : std::vector<double>{
        400.0, 400.0, 400.0,   // [0-2]  kp_translation
        200.0, 200.0, 200.0,   // [3-5]  kp_rotation
        0.01,                // [6]    damping
        0.5,                 // [7]    null_kp
        0.0,                 // [8]    enable_null_space
        1.0,                 // [9]    control_6dof
        0.1,                 // [10]   trajectory_speed
        0.78,                // [11]   trajectory_angular_speed
        3.14,                // [12]   hand_trajectory_speed
        0.5,                 // [13]   max_traj_velocity
        1.57,                // [14]   max_traj_angular_velocity
        6.28,                // [15]   hand_max_traj_velocity
        0.5,                 // [16]   grasp_contact_threshold
        1.0,                 // [17]   grasp_force_threshold
        2.0,                 // [18]   grasp_min_fingertips
      };

  if (use_cached) {
    RCLCPP_INFO(logger(), "[SetGains] DemoTask: using cached gains as base");
  }

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

  // Force-PI grasp command (one-shot, only appended when non-zero)
  auto gcmd = getInput<int>("grasp_command");
  if (gcmd && gcmd.value() != 0) {
    auto gtf = getInput<double>("grasp_target_force");
    gains.push_back(static_cast<double>(gcmd.value()));
    gains.push_back(gtf.value_or(2.0));
    RCLCPP_INFO(logger(),
                "[SetGains] DemoTask: grasp_command=%d target_force=%.2f",
                gcmd.value(), gains.back());
  }

  RCLCPP_INFO(logger(),
              "[SetGains] DemoTask: kp_t=[%.1f,%.1f,%.1f] kp_r=[%.1f,%.1f,%.1f] "
              "traj_speed=%.2f hand_speed=%.2f",
              gains[0], gains[1], gains[2], gains[3], gains[4], gains[5],
              gains[10], gains[12]);
  bridge_->PublishGains(gains);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
