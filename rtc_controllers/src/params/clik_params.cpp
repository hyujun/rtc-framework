#include "rtc_controllers/params/clik_params.hpp"

#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/gain_floor.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseClikParams(const YAML::Node& cfg, ClikParams& out, CommandType& command_type,
                     ClikRetiredKeys* retired) {
  if (retired) {
    *retired = ClikRetiredKeys{};
  }
  if (!cfg) {
    // See the header: the floor runs even with no node at all (NUM-6).
    out.null_kp = FloorNonNegativeGain(out.null_kp);
    return;
  }

  // CLIK gains — translation / rotation separated. Shape-checked, never silently
  // ignored (#302). This parser carried the `>= 3` variant, which reproduced the
  // #172 defect in its exact original shape: a 4-entry sequence parsed clean and
  // the extra entry was dropped without a word. The `< 3` and scalar cases were
  // worse still — the node was not read at all, so the DEFAULT gain ran under a
  // config file that named another value and `get_gains()` agreed with the
  // default. Converged onto the spelling the two compliance schemas already use.
  // No shipped config supplied a length other than 3, so no deployment changes
  // behaviour. No scalar broadcast (D5).
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr, const char* what) {
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 3) {
      throw std::runtime_error(std::string("clik: ") + what +
                               " must be a 3-entry sequence [x,y,z]");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      arr[i] = n[i].as<double>();
    }
  };
  load3(cfg["kp_translation"], out.kp_translation, "kp_translation");
  load3(cfg["kp_rotation"], out.kp_rotation, "kp_rotation");

  if (cfg["max_damping"]) {
    // Floor the damped-least-squares λ_max so a zero/negative value cannot
    // remove the singularity guard (NUM-1); a singular J Jᵀ would otherwise
    // yield a NaN joint-velocity command. Mirrors the OSC floor.
    out.max_damping = compliance::FloorMaxDamping(cfg["max_damping"].as<double>());
  }
  if (cfg["singularity_threshold"]) {
    // Floor σ₀ > 0 (NUM-2). AdaptiveDampingSquared short-circuits to λ² = 0 when
    // σ₀ ≤ 0, so a zero here removes the §6.5 ramp entirely instead of narrowing
    // it — and with λ² ≡ 0 a singular pose makes DifferentialIk report !ok, which
    // this law answers by holding at zero velocity every tick.
    out.singularity_threshold = compliance::FloorSigma0(cfg["singularity_threshold"].as<double>());
  }
  if (cfg["damping"] && retired) {
    // Retired in #236 S3b (#258). Reported rather than mapped onto max_damping:
    // a constant λ and the ceiling of a σ_min-adaptive ramp are not the same
    // quantity, so any mapping would be a guess.
    retired->damping = true;
  }

  if (cfg["null_kp"]) {
    out.null_kp = cfg["null_kp"].as<double>();
  }
  // Floored at 0 (#277): a negative posture gain drives q̇₀ away from the
  // null-space target instead of toward it, and N hides that from the task.
  // Unconditional rather than folded into the parse above — when the key is
  // ABSENT the value still arrives from the constructor or a previous
  // set_gains(). A binding must floor it again at the point of use (set_gains()
  // bypasses configure — NUM-1), as λ_max must through
  // compliance::FloorMaxDamping. Neither half has a tick-side caller today:
  // ClikParams has no consumer outside this parser and the tests.
  out.null_kp = FloorNonNegativeGain(out.null_kp);

  if (cfg["enable_null_space"]) {
    out.enable_null_space = cfg["enable_null_space"].as<bool>();
  }
  if (cfg["control_6dof"]) {
    out.control_6dof = cfg["control_6dof"].as<bool>();
  }

  if (cfg["trajectory_speed"]) {
    out.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["trajectory_angular_speed"]) {
    out.trajectory_angular_speed = std::max(1e-6, cfg["trajectory_angular_speed"].as<double>());
  }
  if (cfg["max_traj_velocity"]) {
    out.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    out.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

}  // namespace rtc::params
