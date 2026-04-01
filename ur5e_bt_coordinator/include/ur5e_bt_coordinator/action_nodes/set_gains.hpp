#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// Publish a gain update to the active controller.
///
/// Gain layout (16 values for DemoTaskController):
///   [kp_translation×3, kp_rotation×3, damping, null_kp,
///    enable_null_space(0/1), control_6dof(0/1),
///    trajectory_speed, trajectory_angular_speed,
///    hand_trajectory_speed, max_traj_velocity,
///    max_traj_angular_velocity, hand_max_traj_velocity]
///
/// This node allows setting individual fields; unset fields keep current values.
///
/// Input ports:
///   - kp_translation (string): "5.0,5.0,5.0" (optional)
///   - kp_rotation (string): "3.0,3.0,3.0" (optional)
///   - damping (double): damping ratio (optional, default 0.01)
///   - null_kp (double): null-space stiffness (optional, default 0.5)
///   - enable_null_space (bool): enable null-space control (optional, default false)
///   - control_6dof (bool): enable 6-DoF control (optional, default true)
///   - trajectory_speed (double): [m/s] (optional)
///   - trajectory_angular_speed (double): [rad/s] (optional)
///   - max_traj_velocity (double): [m/s] (optional)
///   - max_traj_angular_velocity (double): [rad/s] (optional)
///   - hand_trajectory_speed (double): [rad/s] (optional)
///   - hand_max_traj_velocity (double): [rad/s] (optional)
///   - full_gains (vector<double>): complete 16-element array (optional, overrides all)
class SetGains : public BT::SyncActionNode {
public:
  SetGains(const std::string& name, const BT::NodeConfig& config,
           std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
