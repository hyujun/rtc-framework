#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace integrated_bringup {

void DemoWbcController::DeclareGainParameters() noexcept {
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
  auto declare_bool = [&](const std::string& name, bool default_val,
                          const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<bool>(name, default_val, d);
    }
    return node_->get_parameter(name).as_bool();
  };

  g.arm_trajectory_speed =
      std::max(1e-6, declare_double("arm_trajectory_speed", g.arm_trajectory_speed,
                                    "Arm joint trajectory speed [rad/s]"));
  g.hand_trajectory_speed =
      std::max(1e-6, declare_double("hand_trajectory_speed", g.hand_trajectory_speed,
                                    "Hand motor trajectory speed [rad/s]"));
  g.arm_max_traj_velocity =
      declare_double("arm_max_traj_velocity", g.arm_max_traj_velocity,
                     "Max arm joint velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);
  g.hand_max_traj_velocity =
      declare_double("hand_max_traj_velocity", g.hand_max_traj_velocity,
                     "Max hand motor velocity during trajectory [rad/s] (read-only)",
                     /*read_only=*/true);

  // TCP task-space trajectory (MPC-disabled SE3 ramp).
  g.tcp_trajectory_speed = std::max(
      1e-6, declare_double("tcp_trajectory_speed", g.tcp_trajectory_speed,
                           "TCP translational trajectory speed [m/s] (MPC-disabled SE3 ramp)"));
  g.tcp_trajectory_angular_speed =
      std::max(1e-6, declare_double("tcp_trajectory_angular_speed", g.tcp_trajectory_angular_speed,
                                    "TCP angular trajectory speed [rad/s] (MPC-disabled SE3 ramp)"));
  g.tcp_max_traj_velocity =
      declare_double("tcp_max_traj_velocity", g.tcp_max_traj_velocity,
                     "TCP translational velocity cap [m/s] (quintic peak = 1.875·d/T)");
  g.tcp_max_traj_angular_velocity =
      declare_double("tcp_max_traj_angular_velocity", g.tcp_max_traj_angular_velocity,
                     "TCP angular velocity cap [rad/s] (quintic peak = 1.875·d/T)");
  g.pi_rotation_margin = declare_double(
      "pi_rotation_margin", g.pi_rotation_margin,
      "π-rotation split margin [rad] — split into 2 segments when ang_dist > π - margin");

  g.se3_weight = declare_double("se3_weight", g.se3_weight, "TSID SE3Task weight (runtime tuning)");
  g.force_weight = declare_double("force_weight", g.force_weight, "TSID ForceTask weight");
  g.posture_weight = declare_double("posture_weight", g.posture_weight, "TSID PostureTask weight");

  // Grasp detection thresholds (mirror joint/task controllers). The contact
  // threshold is consulted only on sensor-A paths (has_native_contact_=true);
  // the force threshold + min_fingertips are common across sensor types.
  g.grasp_contact_threshold = static_cast<float>(declare_double(
      "grasp_contact_threshold", g.grasp_contact_threshold,
      "Native contact probability threshold [0,1] — sensor A path only"));
  g.grasp_force_threshold = static_cast<float>(declare_double(
      "grasp_force_threshold", g.grasp_force_threshold, "Force magnitude threshold [N]"));
  g.grasp_min_fingertips = static_cast<int>(
      declare_int("grasp_min_fingertips", g.grasp_min_fingertips,
                  "Min fingertips with contact for grasp detection"));

  // Stage C-2: low-level command source + CLIK runtime gains. command_source
  // is a string enum ('integrator' | 'clik') flippable live for A/B; the
  // structural CLIK config (damping_sq / v_limit) is YAML-only since it is
  // fixed at clik_.Init() time. CLIK gains take effect next RT tick (compute
  // forwards them to clik_).
  auto declare_string = [&](const std::string& name, const std::string& default_val,
                            const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<std::string>(name, default_val, d);
    }
    return node_->get_parameter(name).as_string();
  };
  const std::string src_default =
      (g.command_source == CommandSource::kClik) ? "clik" : "integrator";
  const std::string src = declare_string("command_source", src_default,
                                         "Low-level command source: 'integrator' | 'clik'");
  g.command_source = (src == "clik") ? CommandSource::kClik : CommandSource::kIntegrator;
  g.clik_kx_pos = declare_double("clik_kx_pos", g.clik_kx_pos, "CLIK TCP position task gain");
  g.clik_kx_rot = declare_double("clik_kx_rot", g.clik_kx_rot, "CLIK TCP rotation task gain");
  g.clik_ka = declare_double("clik_ka", g.clik_ka, "CLIK arm nullspace posture gain");
  g.clik_kh = declare_double("clik_kh", g.clik_kh, "CLIK hand posture gain");

  gains_lock_.Store(g);

  // MPC runtime gates: handed straight to mpc_manager_. mpc_enable is gated
  // by structural mpc_enabled_ (decided at LoadConfig from YAML — toggling
  // a controller without an MPC thread is a no-op).
  const bool mpc_enable =
      declare_bool("mpc_enable", mpc_manager_.Enabled(),
                   "Enable MPC output consumption (only effective when mpc_enabled_ "
                   "structural flag is true)");
  mpc_manager_.SetEnabled(mpc_enable && mpc_enabled_);

  const double rgs = declare_double("riccati_gain_scale", mpc_manager_.RiccatiGainScale(),
                                    "Riccati feedback gain scale [0,1]");
  mpc_manager_.SetRiccatiGainScale(rgs);
}


rcl_interfaces::msg::SetParametersResult DemoWbcController::OnGainParametersSet(
    const std::vector<rclcpp::Parameter>& params) noexcept {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto g = gains_lock_.Load();
  bool gains_dirty = false;

  for (const auto& p : params) {
    const auto& name = p.get_name();
    try {
      if (name == "arm_trajectory_speed") {
        g.arm_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "hand_trajectory_speed") {
        g.hand_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "tcp_trajectory_speed") {
        g.tcp_trajectory_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "tcp_trajectory_angular_speed") {
        g.tcp_trajectory_angular_speed = std::max(1e-6, p.as_double());
        gains_dirty = true;
      } else if (name == "tcp_max_traj_velocity") {
        g.tcp_max_traj_velocity = p.as_double();
        gains_dirty = true;
      } else if (name == "tcp_max_traj_angular_velocity") {
        g.tcp_max_traj_angular_velocity = p.as_double();
        gains_dirty = true;
      } else if (name == "pi_rotation_margin") {
        g.pi_rotation_margin = p.as_double();
        gains_dirty = true;
      } else if (name == "se3_weight") {
        g.se3_weight = p.as_double();
        gains_dirty = true;
      } else if (name == "force_weight") {
        g.force_weight = p.as_double();
        gains_dirty = true;
      } else if (name == "posture_weight") {
        g.posture_weight = p.as_double();
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
      } else if (name == "command_source") {
        const auto& src = p.as_string();
        if (src != "integrator" && src != "clik") {
          result.successful = false;
          result.reason = "command_source must be 'integrator' or 'clik'";
          return result;
        }
        g.command_source = (src == "clik") ? CommandSource::kClik : CommandSource::kIntegrator;
        gains_dirty = true;
      } else if (name == "clik_kx_pos") {
        g.clik_kx_pos = p.as_double();
        gains_dirty = true;
      } else if (name == "clik_kx_rot") {
        g.clik_kx_rot = p.as_double();
        gains_dirty = true;
      } else if (name == "clik_ka") {
        g.clik_ka = p.as_double();
        gains_dirty = true;
      } else if (name == "clik_kh") {
        g.clik_kh = p.as_double();
        gains_dirty = true;
      } else if (name == "mpc_enable") {
        mpc_manager_.SetEnabled(p.as_bool() && mpc_enabled_);
      } else if (name == "riccati_gain_scale") {
        mpc_manager_.SetRiccatiGainScale(p.as_double());
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
