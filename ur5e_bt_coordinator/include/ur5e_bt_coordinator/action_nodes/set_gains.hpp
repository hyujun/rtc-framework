#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Set runtime-tunable gains on the active controller via ROS 2 parameters,
/// and (optionally) issue a one-shot Force-PI grasp command via the active
/// controller's /<config_key>/grasp_command srv.
///
/// Each input port maps to a typed parameter on the active controller's
/// LifecycleNode; only ports explicitly set in the BT XML are pushed via
/// `set_parameters_atomically` — unspecified parameters keep whatever value
/// the controller currently holds (parameters are persistent state).
///
/// Active controller dispatch (parameter name selection):
///   demo_joint_controller: robot_trajectory_speed, hand_trajectory_speed,
///                          grasp_{contact,force}_threshold,
///                          grasp_min_fingertips
///   demo_task_controller:  kp_translation, kp_rotation, damping, null_kp,
///                          enable_null_space, control_6dof,
///                          trajectory_speed, trajectory_angular_speed,
///                          hand_trajectory_speed,
///                          grasp_{contact,force}_threshold,
///                          grasp_min_fingertips
///   demo_wbc_controller:   arm_trajectory_speed, hand_trajectory_speed,
///                          se3_weight, force_weight, posture_weight,
///                          mpc_enable, riccati_gain_scale
///
/// max_traj_velocity / max_traj_angular_velocity / hand_max_traj_velocity
/// are declared with read_only=true on the controller side and cannot be
/// set via this node — they are honoured only at startup via YAML.
class SetGains : public BT::SyncActionNode {
public:
  SetGains(const std::string &name, const BT::NodeConfig &config,
           std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

} // namespace rtc_bt
