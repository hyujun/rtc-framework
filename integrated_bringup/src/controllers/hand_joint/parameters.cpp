#include "integrated_bringup/controllers/hand_joint_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace integrated_bringup {

// Runtime-tunable: trajectory_speed, gravity_ff.enabled, gravity_ff.gain.
// Read-only: max_traj_velocity (a device rating, not a tuning knob).
//
// `gravity_ff.enabled` is the A/B switch of the sim gate (an operator flips it
// with `ros2 param set /hand_joint_controller/hand_joint_controller gravity_ff.enabled false`
// the hand holds a pose); the command type does not change with it, so the
// simulator's plant stays the same on both sides of the switch.
void HandJointController::DeclareGainParameters() noexcept {
  if (!node_) {
    return;
  }
  // noexcept: a declare that throws (a stored parameter of the wrong type,
  // set while the controller was cleaned up) is reported, and the YAML values
  // already in the SeqLock stay in force.
  try {
    DeclareGainParametersImpl();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "[hand_joint] parameter declaration failed (%s); keeping the YAML gains",
                 e.what());
  }
}

void HandJointController::DeclareGainParametersImpl() {
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

  const std::string key(kHandJointFfBlockKey);
  g.trajectory_speed =
      std::max(kHandJointMinSpeed, declare_double("trajectory_speed", g.trajectory_speed,
                                                  "Joint goal trajectory speed [rad/s]"));
  g.max_traj_velocity =
      declare_double("max_traj_velocity", g.max_traj_velocity,
                     "Peak joint velocity of a goal trajectory [rad/s] (read-only)",
                     /*read_only=*/true);
  g.ff_enabled =
      declare_bool(key + ".enabled", g.ff_enabled,
                   "Feedforward torque lane on/off (off: 0 N·m, command type unchanged)");
  // A stored out-of-range value (set while cleaned up) is clamped, not
  // honoured: this function is noexcept and cannot fail the transition.
  const double gain =
      declare_double(key + ".gain", g.ff_gain,
                     "Gravity feedforward gain, dimensionless [0, " +
                         std::to_string(rtc::params::kDynamicsFeedforwardGainMax) + "]");
  if (std::isfinite(gain)) {
    g.ff_gain = std::clamp(gain, 0.0, rtc::params::kDynamicsFeedforwardGainMax);
  }
  gains_lock_.Store(g);
}

rcl_interfaces::msg::SetParametersResult HandJointController::OnGainParametersSet(
    const std::vector<rclcpp::Parameter>& params) noexcept {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto g = gains_lock_.Load();
  bool dirty = false;
  const std::string key(kHandJointFfBlockKey);
  for (const auto& p : params) {
    const auto& name = p.get_name();
    try {
      if (name == "trajectory_speed") {
        const double v = p.as_double();
        if (!std::isfinite(v) || v <= 0.0 || v > g.max_traj_velocity) {
          result.successful = false;
          result.reason = "trajectory_speed must be finite, > 0 and <= max_traj_velocity (" +
                          std::to_string(g.max_traj_velocity) + " rad/s, read-only)";
          return result;
        }
        g.trajectory_speed = std::max(kHandJointMinSpeed, v);
        dirty = true;
      } else if (name == key + ".enabled") {
        g.ff_enabled = p.as_bool();
        dirty = true;
      } else if (name == key + ".gain") {
        const double v = p.as_double();
        if (!std::isfinite(v) || v < 0.0 || v > rtc::params::kDynamicsFeedforwardGainMax) {
          result.successful = false;
          result.reason = key + ".gain must be finite and within [0, " +
                          std::to_string(rtc::params::kDynamicsFeedforwardGainMax) + "]";
          return result;
        }
        g.ff_gain = v;
        dirty = true;
      }
      // Unknown names: silently allowed (other callbacks may own them).
    } catch (const std::exception& e) {
      result.successful = false;
      result.reason = std::string("type error on '") + name + "': " + e.what();
      return result;
    }
  }
  if (dirty) {
    gains_lock_.Store(g);
  }
  return result;
}

}  // namespace integrated_bringup
