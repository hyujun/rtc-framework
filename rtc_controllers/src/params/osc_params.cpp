#include "rtc_controllers/params/osc_params.hpp"

#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/joint/posture_law.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseOscParams(const YAML::Node& cfg, OscParams& out, CommandType& command_type,
                    OscRetiredKeys* retired) {
  if (retired) {
    *retired = OscRetiredKeys{};
  }
  if (!cfg) {
    // See the header: the floor runs even with no node at all (NUM-6).
    out.null_kp = joint::FloorPostureGain(out.null_kp);
    out.null_kd = joint::FloorPostureGain(out.null_kd);
    // The torque-only verdict runs here too. The YAML path below assigns it
    // UNCONDITIONALLY — the `command_type` key can only reject, never choose —
    // so leaving it alone on this branch would make "no config section" the one
    // way an OSC binding keeps a non-torque type for a law that emits N·m. It is
    // not a hypothetical default either: CommandType's first enumerator is
    // kPosition, so that is exactly what a value-initialised caller arrives
    // with, and the adapter could not reach this because its command_type_ was a
    // member fixed at kTorque (#298 S7c-2).
    command_type = CommandType::kTorque;
    return;
  }

  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr) {
    if (n && n.IsSequence() && n.size() == 3) {
      for (std::size_t i = 0; i < 3; ++i) {
        arr[i] = n[i].as<double>();
      }
    }
  };
  load3(cfg["kp_pos"], out.kp_pos);
  load3(cfg["kd_pos"], out.kd_pos);
  load3(cfg["kp_rot"], out.kp_rot);
  load3(cfg["kd_rot"], out.kd_rot);

  if (cfg["max_damping"]) {
    // Floor λ_max > 0 (NUM-1/NUM-4): λ² regularises Λ⁻¹ at kinematic
    // singularities. A zero/negative λ_max would remove that guard and admit NaN
    // torque.
    out.max_damping = std::max(compliance::kMinMaxDamping, cfg["max_damping"].as<double>());
  }
  if (cfg["singularity_threshold"]) {
    // Floor σ₀ > 0 (NUM-2). AdaptiveDampingSquared short-circuits to λ² = 0 when
    // σ₀ ≤ 0, so a zero here does not merely widen the shell — it removes the
    // §6.5 ramp entirely. The other three task-space controllers clamp this key
    // identically; "same names, same defaults" only holds if the VALIDATION
    // converges too.
    out.singularity_threshold =
        std::max(compliance::kMinSigma0, cfg["singularity_threshold"].as<double>());
  }
  if (cfg["damping"] && retired) {
    // Retired in #236 S2b. Reported rather than mapped onto max_damping: a
    // constant λ and the ceiling of a σ_min-adaptive ramp are not the same
    // quantity, so any mapping would be a guess.
    retired->damping = true;
  }

  // Dynamically-consistent null-space posture gains (only bite when nv > 6).
  if (cfg["null_kp"]) {
    out.null_kp = cfg["null_kp"].as<double>();
  }
  if (cfg["null_kd"]) {
    out.null_kd = cfg["null_kd"].as<double>();
  }
  // Both floored at 0 (#277): a negative Kp makes τ₀ = Kp·(q_safe − q) push away
  // from the configured posture and a negative Kd injects energy, and Nᵀ keeps
  // either from disturbing the Cartesian task — so the posture drifts silently.
  // Unconditional rather than folded into the two parses above: when a key is
  // ABSENT the value still arrives from the constructor or a previous
  // set_gains(). The law floors again at the point of use because set_gains()
  // bypasses configure entirely (NUM-1) — as max_damping does.
  out.null_kp = joint::FloorPostureGain(out.null_kp);
  out.null_kd = joint::FloorPostureGain(out.null_kd);

  if (cfg["estop_damping"]) {
    // D ≥ 0: the torque E-STOP hold (#184) subtracts D·q̇ to bleed kinetic
    // energy; a negative D would inject energy (destabilising a safety stop).
    out.estop_damping = std::max(0.0, cfg["estop_damping"].as<double>());
  }
  if (cfg["enable_gravity_compensation"]) {
    // Deprecated: torque OSC always compensates g(q)+C·v. Parsed for YAML
    // back-compat but has no effect on the control law.
    out.enable_gravity_compensation = cfg["enable_gravity_compensation"].as<bool>();
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

  // OSC is a torque controller (the operational-space law outputs N·m). Reject
  // any other command_type fail-fast instead of silently mislabelling the
  // output; position/velocity task control belongs to the CLIK law.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque") {
      throw std::runtime_error("osc: command_type must be 'torque' (got '" + s +
                               "'); use the CLIK law for position/velocity task control");
    }
  }
  command_type = CommandType::kTorque;
}

}  // namespace rtc::params
