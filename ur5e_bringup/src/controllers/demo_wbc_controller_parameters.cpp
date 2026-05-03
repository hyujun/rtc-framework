#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace ur5e_bringup {

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

  g.se3_weight = declare_double("se3_weight", g.se3_weight, "TSID SE3Task weight (runtime tuning)");
  g.force_weight = declare_double("force_weight", g.force_weight, "TSID ForceTask weight");
  g.posture_weight = declare_double("posture_weight", g.posture_weight, "TSID PostureTask weight");

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
      } else if (name == "se3_weight") {
        g.se3_weight = p.as_double();
        gains_dirty = true;
      } else if (name == "force_weight") {
        g.force_weight = p.as_double();
        gains_dirty = true;
      } else if (name == "posture_weight") {
        g.posture_weight = p.as_double();
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


}  // namespace ur5e_bringup
