#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <rclcpp/parameter.hpp>

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

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
///
/// StatefulActionNode (async): the two service calls (set_parameters_atomically
/// on the controller, then the optional one-shot grasp_command srv) are fired
/// without blocking and polled across ticks. A blocking wait inside a tick
/// would deadlock the single-threaded executor that must dispatch the response.
/// The two stages run sequentially — parameters first, grasp only after they
/// commit — so a rejected gain set never fires a grasp against stale gains.
class SetGains : public BT::StatefulActionNode {
 public:
  SetGains(const std::string& name, const BT::NodeConfig& config,
           std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  /// Build the controller-specific parameter list from the set ports. Returns
  /// false (and logs) if the active controller is unknown. `is_*` reflect the
  /// active controller so onStart can also gate the grasp stage.
  bool BuildParams();

  std::shared_ptr<BtRosBridge> bridge_;

  enum class Stage { kParams, kGrasp, kDone };
  Stage stage_{Stage::kParams};
  bool stage_sent_{false};
  std::chrono::steady_clock::time_point stage_start_;

  std::vector<rclcpp::Parameter> params_;
  int grasp_command_{0};
  double grasp_force_{2.0};

  BtRosBridge::SetParametersFuture param_future_;
  BtRosBridge::GraspCommandFuture grasp_future_;
  std::string last_err_;
};

}  // namespace rtc_bt
