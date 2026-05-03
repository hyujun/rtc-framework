#include "integrated_bringup/controllers/demo_joint_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace integrated_bringup {

// ── Phase D: gain → ROS 2 parameter declaration & callback ────────────────
//
// Mirrors the DemoTaskController pattern (Phase B) for the joint-space demo.
// Tunable: robot_trajectory_speed, hand_trajectory_speed,
//          grasp_{contact,force}_threshold, grasp_min_fingertips.
// Read-only (D-2): robot_max_traj_velocity, hand_max_traj_velocity.
void DemoJointController::DeclareGainParameters() noexcept {
  if (!node_) {
    return;
  }

  auto g = gains_lock_.Load();

  auto declare_double = [&](const std::string& name, double default_val,
                            const std::string& description, bool read_only = false) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    d.read_only = read_only;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<double>(name, default_val, d);
    }
    return node_->get_parameter(name).as_double();
  };
  auto declare_int = [&](const std::string& name, int64_t default_val,
                         const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<int64_t>(name, default_val, d);
    }
    return node_->get_parameter(name).as_int();
  };

  g.robot_trajectory_speed =
      std::max(1e-6, declare_double("robot_trajectory_speed", g.robot_trajectory_speed,
                                    "Arm joint trajectory speed [rad/s]"));
  g.hand_trajectory_speed =
      std::max(1e-6, declare_double("hand_trajectory_speed", g.hand_trajectory_speed,
                                    "Hand motor trajectory speed [rad/s]"));

  g.robot_max_traj_velocity =
      declare_double("robot_max_traj_velocity", g.robot_max_traj_velocity,
                     "Max arm joint velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);
  g.hand_max_traj_velocity =
      declare_double("hand_max_traj_velocity", g.hand_max_traj_velocity,
                     "Max hand motor velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);

  g.grasp_contact_threshold = static_cast<float>(declare_double(
      "grasp_contact_threshold", g.grasp_contact_threshold, "Contact probability threshold [0,1]"));
  g.grasp_force_threshold = static_cast<float>(declare_double(
      "grasp_force_threshold", g.grasp_force_threshold, "Force magnitude threshold [N]"));
  g.grasp_min_fingertips =
      static_cast<int>(declare_int("grasp_min_fingertips", g.grasp_min_fingertips,
                                   "Min fingertips with contact for grasp detection"));

  gains_lock_.Store(g);
}

rcl_interfaces::msg::SetParametersResult DemoJointController::OnGainParametersSet(
    const std::vector<rclcpp::Parameter>& params) noexcept {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto g = gains_lock_.Load();
  bool gains_dirty = false;

  for (const auto& p : params) {
    const auto& name = p.get_name();
    try {
      if (name == "robot_trajectory_speed") {
        g.robot_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "hand_trajectory_speed") {
        g.hand_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "grasp_contact_threshold") {
        g.grasp_contact_threshold = static_cast<float>(p.as_double());
        gains_dirty = true;
      } else if (name == "grasp_force_threshold") {
        g.grasp_force_threshold = static_cast<float>(p.as_double());
        gains_dirty = true;
      } else if (name == "grasp_min_fingertips") {
        g.grasp_min_fingertips = static_cast<int>(p.as_int());
        gains_dirty = true;
      }
      // Unknown names: silently allowed (other callbacks may own them).
    } catch (const std::exception& e) {
      result.successful = false;
      result.reason = std::string("type error on '") + name + "': " + e.what();
      return result;
    }
  }

  if (gains_dirty) {
    gains_lock_.Store(g);
  }
  return result;
}

}  // namespace integrated_bringup
