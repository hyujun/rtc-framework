#include "ur5e_bringup/controllers/demo_task_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ur5e_bringup {

void DemoTaskController::DeclareGainParameters() noexcept {
  if (!node_) {
    return;
  }

  auto g = gains_lock_.Load();

  rcl_interfaces::msg::ParameterDescriptor desc_rw;
  rcl_interfaces::msg::ParameterDescriptor desc_ro;
  desc_ro.read_only = true;

  auto declare_double_array = [&](const std::string& name, std::vector<double> default_val,
                                  const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<std::vector<double>>(name, std::move(default_val), d);
    }
    return node_->get_parameter(name).as_double_array();
  };
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
  auto declare_bool = [&](const std::string& name, bool default_val,
                          const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<bool>(name, default_val, d);
    }
    return node_->get_parameter(name).as_bool();
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

  // CLIK gains
  const auto kp_t = declare_double_array(
      "kp_translation", std::vector<double>(g.kp_translation.begin(), g.kp_translation.end()),
      "Translation P gain (x, y, z) [1/s]");
  const auto kp_r = declare_double_array(
      "kp_rotation", std::vector<double>(g.kp_rotation.begin(), g.kp_rotation.end()),
      "Rotation P gain (rx, ry, rz) [1/s]");
  for (std::size_t i = 0; i < 3 && i < kp_t.size(); ++i) {
    g.kp_translation[i] = kp_t[i];
  }
  for (std::size_t i = 0; i < 3 && i < kp_r.size(); ++i) {
    g.kp_rotation[i] = kp_r[i];
  }

  g.damping =
      declare_double("damping", g.damping, "Damped pseudoinverse lambda (singularity stab.)");
  g.null_kp = declare_double("null_kp", g.null_kp, "Null-space joint-centering gain [1/s]");
  g.enable_null_space =
      declare_bool("enable_null_space", g.enable_null_space, "Enable null-space secondary task");
  g.control_6dof = declare_bool("control_6dof", g.control_6dof,
                                "false=3-DOF (position only), true=6-DOF (pose)");

  g.trajectory_speed = std::max(1e-6, declare_double("trajectory_speed", g.trajectory_speed,
                                                     "TCP translational trajectory speed [m/s]"));
  g.trajectory_angular_speed =
      std::max(1e-6, declare_double("trajectory_angular_speed", g.trajectory_angular_speed,
                                    "TCP rotational trajectory speed [rad/s]"));
  g.hand_trajectory_speed =
      std::max(1e-6, declare_double("hand_trajectory_speed", g.hand_trajectory_speed,
                                    "Hand motor trajectory speed [rad/s]"));

  // Read-only velocity caps (D-2)
  g.max_traj_velocity =
      declare_double("max_traj_velocity", g.max_traj_velocity,
                     "Max TCP translational velocity during trajectory [m/s] (read-only)",
                     /*read_only=*/true);
  g.max_traj_angular_velocity =
      declare_double("max_traj_angular_velocity", g.max_traj_angular_velocity,
                     "Max TCP angular velocity during trajectory [rad/s] "
                     "(read-only)",
                     /*read_only=*/true);
  g.hand_max_traj_velocity =
      declare_double("hand_max_traj_velocity", g.hand_max_traj_velocity,
                     "Max hand motor velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);

  // Grasp detection
  g.grasp_contact_threshold = static_cast<float>(declare_double(
      "grasp_contact_threshold", g.grasp_contact_threshold, "Contact probability threshold [0,1]"));
  g.grasp_force_threshold = static_cast<float>(declare_double(
      "grasp_force_threshold", g.grasp_force_threshold, "Force magnitude threshold [N]"));
  g.grasp_min_fingertips =
      static_cast<int>(declare_int("grasp_min_fingertips", g.grasp_min_fingertips,
                                   "Min fingertips with contact for grasp detection"));

  gains_lock_.Store(g);
}

rcl_interfaces::msg::SetParametersResult DemoTaskController::OnGainParametersSet(
    const std::vector<rclcpp::Parameter>& params) noexcept {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Snapshot → mutate → store. SeqLock makes the publish RT-safe.
  auto g = gains_lock_.Load();
  bool gains_dirty = false;

  for (const auto& p : params) {
    const auto& name = p.get_name();
    try {
      if (name == "kp_translation") {
        const auto v = p.as_double_array();
        if (v.size() != 3) {
          result.successful = false;
          result.reason = "kp_translation requires 3 values";
          return result;
        }
        for (std::size_t i = 0; i < 3; ++i) {
          g.kp_translation[i] = v[i];
        }
        gains_dirty = true;
      } else if (name == "kp_rotation") {
        const auto v = p.as_double_array();
        if (v.size() != 3) {
          result.successful = false;
          result.reason = "kp_rotation requires 3 values";
          return result;
        }
        for (std::size_t i = 0; i < 3; ++i) {
          g.kp_rotation[i] = v[i];
        }
        gains_dirty = true;
      } else if (name == "damping") {
        g.damping = p.as_double();
        gains_dirty = true;
      } else if (name == "null_kp") {
        g.null_kp = p.as_double();
        gains_dirty = true;
      } else if (name == "enable_null_space") {
        g.enable_null_space = p.as_bool();
        gains_dirty = true;
      } else if (name == "control_6dof") {
        g.control_6dof = p.as_bool();
        gains_dirty = true;
      } else if (name == "trajectory_speed") {
        g.trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "trajectory_angular_speed") {
        g.trajectory_angular_speed = std::max(1e-6, p.as_double());
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
      // Unknown parameter names are silently allowed — other callbacks
      // (CM topic param read-only validator, lifecycle) may own them.
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

}  // namespace ur5e_bringup
