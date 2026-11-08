#include "integrated_bringup/controllers/hand_joint_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace integrated_bringup {

// Runtime-tunable: trajectory_speed, gravity_ff.enabled, gravity_ff.gain, mode.
// Read-only: max_traj_velocity (a device rating, not a tuning knob).
//
// `mode` (tracking / damping / damping_feedforward) is accepted at any time —
// active or not, E-STOP or not (E-STOP dominates on the tick anyway); an
// off-whitelist string is refused with the three values in the reason.
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

  // One declare for every type: declared with its default when absent,
  // otherwise the stored value (set while the controller was cleaned up).
  auto declare = [&]<typename T>(const std::string& name, const T& default_val,
                                 const std::string& description, bool read_only = false) -> T {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    d.read_only = read_only;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<T>(name, default_val, d);
    }
    return node_->get_parameter(name).get_value<T>();
  };

  const std::string key(kHandJointFfBlockKey);
  g.trajectory_speed = std::max(kHandJointMinSpeed, declare("trajectory_speed", g.trajectory_speed,
                                                            "Joint goal trajectory speed [rad/s]"));
  g.max_traj_velocity =
      declare("max_traj_velocity", g.max_traj_velocity,
              "Peak joint velocity of a goal trajectory [rad/s] (read-only)", /*read_only=*/true);
  g.ff_enabled = declare(key + ".enabled", g.ff_enabled,
                         "Feedforward torque lane on/off (off: 0 N·m, command type unchanged)");
  // A stored out-of-range value (set while cleaned up) is clamped, not
  // honoured: this function is noexcept and cannot fail the transition.
  const double gain = declare(key + ".gain", g.ff_gain,
                              "Gravity feedforward gain, dimensionless [0, " +
                                  std::to_string(rtc::params::kDynamicsFeedforwardGainMax) + "]");
  if (std::isfinite(gain)) {
    g.ff_gain = std::clamp(gain, 0.0, rtc::params::kDynamicsFeedforwardGainMax);
  }
  // A stored off-whitelist (or wrong-typed) mode keeps the YAML mode; the
  // catch is local so the gains above are stored whatever happens here.
  bool mode_rejected = false;
  try {
    g.mode = ParseHandJointMode(declare(
        "mode", std::string(HandJointModeName(g.mode)),
        std::string("tracking | damping | damping_feedforward (what the two lanes carry)")));
  } catch (const std::exception& e) {
    mode_rejected = true;
    RCLCPP_ERROR(logger_, "[hand_joint] stored parameter 'mode' ignored (%s); keeping %s", e.what(),
                 HandJointModeName(g.mode).data());
  }
  gains_lock_.Store(g);
  if (mode_rejected) {
    // `ros2 param get mode` must not disagree with what the tick runs. The
    // on-set callback is not registered yet, so this write cannot be refused.
    // An undeclared `mode` (a wrong-typed stored value made declare throw)
    // is declared here with the mode in force.
    const std::string in_force(HandJointModeName(g.mode));
    if (node_->has_parameter("mode")) {
      (void)node_->set_parameter(rclcpp::Parameter("mode", in_force));
    } else {
      (void)node_->declare_parameter<std::string>("mode", in_force,
                                                  rcl_interfaces::msg::ParameterDescriptor{},
                                                  /*ignore_override=*/true);
    }
  }
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
      } else if (name == "mode") {
        const std::string text = p.as_string();
        try {
          g.mode = ParseHandJointMode(text);
        } catch (const std::runtime_error& e) {
          result.successful = false;
          result.reason = e.what();  // names the three accepted values
          return result;
        }
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
