#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Publish a gain update to the active controller.
///
/// Automatically detects the active controller and builds the appropriate
/// gain layout:
///
/// DemoTaskController (19 values + optional 2 for Force-PI):
///   [kp_translation×3, kp_rotation×3, damping, null_kp,
///    enable_null_space(0/1), control_6dof(0/1),
///    trajectory_speed, trajectory_angular_speed,
///    hand_trajectory_speed, max_traj_velocity,
///    max_traj_angular_velocity, hand_max_traj_velocity,
///    grasp_contact_threshold, grasp_force_threshold,
///    grasp_min_fingertips,
///    (grasp_command, grasp_target_force)]
///
/// DemoJointController (7 values + optional 2 for Force-PI):
///   [robot_trajectory_speed, hand_trajectory_speed,
///    robot_max_traj_velocity, hand_max_traj_velocity,
///    grasp_contact_threshold, grasp_force_threshold,
///    grasp_min_fingertips,
///    (grasp_command, grasp_target_force)]
///
/// NOTE: max_traj_velocity, max_traj_angular_velocity, hand_max_traj_velocity
/// are NOT configurable from BT. They are always taken from current_gains
/// (loaded by SwitchController) or from hard-coded defaults.
///
/// Input ports:
///   - kp_translation (string): "5.0,5.0,5.0" (DemoTask only, optional)
///   - kp_rotation (string): "3.0,3.0,3.0" (DemoTask only, optional)
///   - damping (double): damping ratio (DemoTask only, optional, default 0.01)
///   - null_kp (double): null-space stiffness (DemoTask only, optional, default 0.5)
///   - enable_null_space (bool): enable null-space control (DemoTask only, optional)
///   - control_6dof (bool): enable 6-DoF control (DemoTask only, optional)
///   - trajectory_speed (double): [m/s] (optional)
///   - trajectory_angular_speed (double): [rad/s] (DemoTask only, optional)
///   - hand_trajectory_speed (double): [rad/s] (optional)
///   - full_gains (vector<double>): complete gain array (optional, overrides all)
///   - grasp_command (int): 0=none, 1=grasp, 2=release (Force-PI, optional)
///   - grasp_target_force (double): target grip force [N] (Force-PI, default 2.0)
///   - current_gains (vector<double>): previously loaded gains from
///     SwitchController (optional). When present, these are used as the base
///     values instead of hard-coded defaults. Only ports explicitly set in
///     the BT XML will override the corresponding gain values.
class SetGains : public BT::SyncActionNode {
public:
  SetGains(const std::string& name, const BT::NodeConfig& config,
           std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  BT::NodeStatus BuildDemoJointGains();
  BT::NodeStatus BuildDemoTaskGains();

  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
