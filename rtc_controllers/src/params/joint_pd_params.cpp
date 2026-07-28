#include "rtc_controllers/params/joint_pd_params.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseJointPdParams(const YAML::Node& cfg, int nv, JointPdParams& out,
                        CommandType& command_type) {
  if (!cfg) {
    return;
  }
  const auto nvu = static_cast<std::size_t>(nv);

  // `kp`/`kd` length must equal the model DOF — no silent truncation (#172).
  if (cfg["kp"]) {
    if (!cfg["kp"].IsSequence() || cfg["kp"].size() != nvu) {
      throw std::runtime_error("joint_pd: 'kp' must be a sequence of length nv=" +
                               std::to_string(nv));
    }
    for (std::size_t i = 0; i < nvu; ++i) {
      out.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["kd"]) {
    if (!cfg["kd"].IsSequence() || cfg["kd"].size() != nvu) {
      throw std::runtime_error("joint_pd: 'kd' must be a sequence of length nv=" +
                               std::to_string(nv));
    }
    for (std::size_t i = 0; i < nvu; ++i) {
      out.kd[i] = cfg["kd"][i].as<double>();
    }
  }
  if (cfg["enable_gravity_compensation"]) {
    out.enable_gravity_compensation = cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["enable_coriolis_compensation"]) {
    out.enable_coriolis_compensation = cfg["enable_coriolis_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    out.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
  // Dynamics compensation (g, C·v) produces N·m and is only valid on a torque
  // command. Reject the mixed-unit configuration fail-fast rather than silently
  // dropping the terms at runtime (issue #172). Evaluated after `command_type`
  // is resolved above so the rule sees the value that will be in force.
  if (command_type != CommandType::kTorque &&
      (out.enable_gravity_compensation || out.enable_coriolis_compensation)) {
    throw std::runtime_error(
        "joint_pd: enable_gravity_compensation / enable_coriolis_compensation require "
        "command_type: torque (dynamics compensation is N·m and cannot be added to a "
        "position/velocity command)");
  }
}

}  // namespace rtc::params
