#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"  // FloorMaxDamping, kMinSigma0
#include "rtc_controllers/gain_floor.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace integrated_bringup {

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
  auto declare_string_ro = [&](const std::string& name, const std::string& default_val,
                               const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    d.read_only = true;
    if (!node_->has_parameter(name)) {
      (void)node_->declare_parameter<std::string>(name, default_val, d);
    }
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

  // §6.5 DLS (#282). The retired `damping` is NOT declared — leaving it would
  // give `ros2 param set … damping` a parameter that accepts a value and
  // changes nothing, which is worse than the error a missing name produces.
  g.singularity_threshold =
      rtc::compliance::FloorSigma0(declare_double("singularity_threshold", g.singularity_threshold,
                                                  "σ₀: DLS damping engages below this σ_min(J)"));
  g.max_damping = rtc::compliance::FloorMaxDamping(
      declare_double("max_damping", g.max_damping, "λ_max: ceiling of the §6.5 DLS ramp"));
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

  // Display-only mirror of the hand grasp mode LoadConfig already resolved from
  // demo_shared.yaml. Unlike the read-only velocity caps above there is NO
  // reverse assignment (param -> grasp_hand_mode_): the parameter exists so a
  // client can *see* which mode is running, and a reverse path is the only way
  // the two could ever disagree. Changing the mode is a YAML + restart
  // operation; rclcpp rejects `ros2 param set` on a read_only parameter, so
  // OnGainParametersSet needs no case for this name.
  declare_string_ro("grasp_controller_type", GraspHandModeName(grasp_hand_mode_),
                    "Active hand grasp mode: 'contact_stop' | 'force_pi' | 'none' "
                    "(read-only; set via demo_shared.yaml)");

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
      } else if (name == "singularity_threshold") {
        // NUM-2 floor. ComputeControl applies it again on the tick, exactly like
        // λ_max below and for the identical reason — `set_gains()` writes the POD
        // straight into the SeqLock and never passes through here, so neither
        // half covers the other. σ₀ ≤ 0 does not narrow the shell:
        // AdaptiveDampingSquared short-circuits and returns λ² = 0 for EVERY
        // σ_min, i.e. it disarms §6.5 everywhere rather than making it stricter.
        g.singularity_threshold = rtc::compliance::FloorSigma0(p.as_double());
        gains_dirty = true;
      } else if (name == "max_damping") {
        // NUM-1 λ_max floor. ComputeControl applies it again on the tick —
        // set_gains() writes the POD straight into the SeqLock and never passes
        // through here, so neither half covers the other.
        g.max_damping = rtc::compliance::FloorMaxDamping(p.as_double());
        gains_dirty = true;
      } else if (name == "null_kp") {
        // NUM-6 (#277) — the same floor LoadConfig applies, here because this is
        // the ONLY surface that reaches this gain at runtime (`ros2 param set`
        // and the BT SetGains node both land here). A negative K_p drives the
        // null-space posture away from its target and (I − J⁺J) keeps that off
        // the Cartesian task, so nothing downstream would report it.
        g.null_kp = rtc::FloorNonNegativeGain(p.as_double());
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

}  // namespace integrated_bringup
