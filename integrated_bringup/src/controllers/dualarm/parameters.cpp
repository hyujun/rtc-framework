// ── DemoDualArmController: runtime parameters (non-RT) ──────────────────────
//
// Writable: each task's two gains, each posture group's gain, the three
// trajectory speeds. Everything else — weights, caps, boxes, the acceleration
// constraint, which tasks and groups exist — is fixed at configure, because it
// sizes or shapes the QP.
//
// Two different bounds, on purpose:
//   - a gain is REFUSED outside [0, 1/dt]. Past 1/dt the discrete error
//     dynamics overshoot, and there is no value to floor it to that still does
//     what the operator asked;
//   - a speed is FLOORED at 1e-6 (NUM-4). It is a divisor, and a slow
//     trajectory is still the trajectory that was asked for.
// A non-finite value is refused in both cases, before either bound: std::max
// would turn a NaN speed into the floor and report success.

#include "integrated_bringup/controllers/demo_dualarm_controller.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace integrated_bringup {

namespace {

std::string TaskParam(const std::string& task, const char* field) {
  return "tasks." + task + "." + field;
}

std::string PostureParam(const std::string& group) {
  return "posture." + group + ".gain";
}

}  // namespace

void DemoDualArmController::DeclareParameters() {
  if (!node_) {
    return;
  }
  Gains gains = gains_lock_.Load();
  const double k_max = GainUpperBound();

  // Returns the parameter's value: the seed on a first configure, or what a
  // `ros2 param set` left while the controller was cleaned up (the callback
  // is gone then, so that value has passed no check and is judged below).
  const auto declare = [this](const std::string& name, double seed, const std::string& description,
                              bool read_only = false) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = read_only;
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<double>(name, seed, descriptor);
    }
    return node_->get_parameter(name).as_double();
  };
  // A refused value is also written back, so that the node does not go on
  // showing a number the controller is not running. (The set callback is not
  // registered yet: nothing judges this write but rclcpp.)
  const auto write_back = [this](const std::string& name, double seed) {
    if (!node_->set_parameter(rclcpp::Parameter(name, seed)).successful) {
      RCLCPP_WARN(logger_, "parameter '%s' could not be set back to the configured %g",
                  name.c_str(), seed);
    }
  };
  const auto gain_or_seed = [this, k_max, &write_back](const std::string& name, double value,
                                                       double seed) {
    if (std::isfinite(value) && value >= 0.0 && value <= k_max) {
      return value;
    }
    RCLCPP_WARN(logger_, "parameter '%s' = %g is outside [0, %g]; keeping the configured %g",
                name.c_str(), value, k_max, seed);
    write_back(name, seed);
    return seed;
  };
  const auto speed_or_seed = [this, &write_back](const std::string& name, double value,
                                                 double seed) {
    if (std::isfinite(value)) {
      return std::max(kDualArmMinSpeed, value);
    }
    RCLCPP_WARN(logger_, "parameter '%s' is not finite; keeping the configured %g", name.c_str(),
                seed);
    write_back(name, seed);
    return seed;
  };
  // A read-only parameter cannot be re-declared or set, so on a re-configure
  // it keeps what the first configure declared. Say so when the configuration
  // has moved: the tick runs the configured value, not the one the node shows.
  const auto declare_cap = [this, &declare](const std::string& name, double configured,
                                            const std::string& description) {
    const double shown = declare(name, configured, description, /*read_only=*/true);
    if (shown != configured) {
      RCLCPP_WARN(logger_,
                  "read-only parameter '%s' shows %g from an earlier configure; the controller "
                  "runs the configured %g",
                  name.c_str(), shown, configured);
    }
  };

  for (std::size_t k = 0; k < cfg_.tasks.size(); ++k) {
    const std::string& task = cfg_.tasks[k].name;
    const std::string lin = TaskParam(task, "gain_linear");
    const std::string ang = TaskParam(task, "gain_angular");
    gains.task_gain_linear[k] = gain_or_seed(
        lin, declare(lin, gains.task_gain_linear[k], "Position gain of the task [1/s], [0, 1/dt]"),
        cfg_.tasks[k].gain_linear);
    gains.task_gain_angular[k] = gain_or_seed(
        ang,
        declare(ang, gains.task_gain_angular[k], "Orientation gain of the task [1/s], [0, 1/dt]"),
        cfg_.tasks[k].gain_angular);
  }
  for (std::size_t g = 0; g < cfg_.posture_groups.size(); ++g) {
    const std::string name = PostureParam(cfg_.posture_groups[g].name);
    gains.posture_gain[g] = gain_or_seed(
        name, declare(name, gains.posture_gain[g], "Posture gain of the group [1/s], [0, 1/dt]"),
        cfg_.posture_groups[g].gain);
  }
  gains.linear_speed = speed_or_seed("trajectory.linear_speed",
                                     declare("trajectory.linear_speed", gains.linear_speed,
                                             "Task trajectory translation speed [m/s]"),
                                     cfg_.linear_speed);
  gains.angular_speed = speed_or_seed("trajectory.angular_speed",
                                      declare("trajectory.angular_speed", gains.angular_speed,
                                              "Task trajectory rotation speed [rad/s]"),
                                      cfg_.angular_speed);
  gains.hand_speed = speed_or_seed(
      "trajectory.hand_speed",
      declare("trajectory.hand_speed", gains.hand_speed, "Hand joint trajectory speed [rad/s]"),
      cfg_.hand_speed);

  declare_cap("trajectory.linear_speed_max", cfg_.linear_speed_max,
              "Peak translation speed a task trajectory may reach [m/s] (read-only)");
  declare_cap("trajectory.angular_speed_max", cfg_.angular_speed_max,
              "Peak rotation speed a task trajectory may reach [rad/s] (read-only)");
  declare_cap("trajectory.hand_speed_max", cfg_.hand_speed_max,
              "Peak joint speed a hand trajectory may reach [rad/s] (read-only)");

  gains_lock_.Store(gains);
}

rcl_interfaces::msg::SetParametersResult DemoDualArmController::OnParametersSet(
    const std::vector<rclcpp::Parameter>& params) noexcept {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  Gains gains = gains_lock_.Load();
  const double k_max = GainUpperBound();
  bool dirty = false;

  const auto refuse = [&result](const std::string& reason) {
    result.successful = false;
    result.reason = reason;
  };

  try {
    for (const auto& param : params) {
      const std::string& name = param.get_name();
      double* gain = nullptr;
      double* speed = nullptr;
      for (std::size_t k = 0; k < cfg_.tasks.size() && gain == nullptr; ++k) {
        if (name == TaskParam(cfg_.tasks[k].name, "gain_linear")) {
          gain = &gains.task_gain_linear[k];
        } else if (name == TaskParam(cfg_.tasks[k].name, "gain_angular")) {
          gain = &gains.task_gain_angular[k];
        }
      }
      for (std::size_t g = 0; g < cfg_.posture_groups.size() && gain == nullptr; ++g) {
        if (name == PostureParam(cfg_.posture_groups[g].name)) {
          gain = &gains.posture_gain[g];
        }
      }
      if (name == "trajectory.linear_speed") {
        speed = &gains.linear_speed;
      } else if (name == "trajectory.angular_speed") {
        speed = &gains.angular_speed;
      } else if (name == "trajectory.hand_speed") {
        speed = &gains.hand_speed;
      }
      if (gain == nullptr && speed == nullptr) {
        continue;  // not one of ours (read-only ones are refused by rclcpp)
      }
      const double value = param.as_double();
      if (!std::isfinite(value)) {
        refuse("'" + name + "' must be finite");
        return result;
      }
      if (gain != nullptr) {
        if (value < 0.0 || value > k_max) {
          refuse("'" + name + "' must be in [0, " + std::to_string(k_max) +
                 "] 1/s (1/dt: a larger gain overshoots in one tick)");
          return result;
        }
        *gain = value;
      } else {
        *speed = std::max(kDualArmMinSpeed, value);
      }
      dirty = true;
    }
  } catch (const std::exception& e) {
    refuse(std::string("type error: ") + e.what());
    return result;
  }
  // All-or-nothing: nothing is stored unless every parameter of the request
  // passed.
  if (dirty) {
    gains_lock_.Store(gains);
  }
  return result;
}

}  // namespace integrated_bringup
