#include "rtc_tsid/controller/solve_health.hpp"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc::tsid {
namespace {

int Saturating(int n) noexcept {
  return n < std::numeric_limits<int>::max() ? n + 1 : n;
}

SolveHealthConfig Parse(const YAML::Node& node) {
  SolveHealthConfig c;
  if (!node.IsDefined() || node.IsNull()) {
    return c;
  }
  if (!node.IsMap()) {
    throw std::invalid_argument("health: must be a map of keys");
  }
  for (auto it = node.begin(); it != node.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    bool known = false;
    for (const char* k :
         {"max_invalid_ticks", "recover_valid_ticks", "min_friction_margin", "min_cop_margin",
          "min_yaw_margin", "min_torque_margin", "min_joint_accel_margin", "max_eom_residual"}) {
      known = known || key == k;
    }
    if (!known) {
      throw std::invalid_argument("health: unknown key '" + key + "'");
    }
  }
  if (node["max_invalid_ticks"]) {
    c.max_invalid_ticks = node["max_invalid_ticks"].as<int>();
    if (c.max_invalid_ticks < 0) {
      throw std::invalid_argument("health.max_invalid_ticks must be >= 0 (omit it for off)");
    }
  }
  if (node["recover_valid_ticks"]) {
    c.recover_valid_ticks = node["recover_valid_ticks"].as<int>();
    if (c.recover_valid_ticks < 1) {
      throw std::invalid_argument("health.recover_valid_ticks must be >= 1");
    }
  }
  const auto floor = [&node](const char* key, double& out) {
    if (node[key]) {
      out = node[key].as<double>();
      if (!std::isfinite(out)) {
        throw std::invalid_argument(std::string("health.") + key + " must be finite");
      }
    }
  };
  floor("min_friction_margin", c.min_friction_margin);
  floor("min_cop_margin", c.min_cop_margin);
  floor("min_yaw_margin", c.min_yaw_margin);
  floor("min_torque_margin", c.min_torque_margin);
  floor("min_joint_accel_margin", c.min_joint_accel_margin);
  if (node["max_eom_residual"]) {
    c.max_eom_residual = node["max_eom_residual"].as<double>();
    if (!(std::isfinite(c.max_eom_residual) && c.max_eom_residual > 0.0)) {
      throw std::invalid_argument("health.max_eom_residual must be finite and > 0");
    }
  }
  return c;
}

}  // namespace

SolveHealthConfig ParseSolveHealthConfig(const YAML::Node& node) {
  try {
    return Parse(node);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("health: a value does not convert (") + e.what() + ")");
  }
}

bool SolveHealth::MarginsPass(const SolveHealthConfig& config, const WbcMargins& m) noexcept {
  // Written so that a NaN margin fails every comparison.
  return m.friction_n >= config.min_friction_margin && m.cop_nm >= config.min_cop_margin &&
         m.yaw_nm >= config.min_yaw_margin && m.torque_nm >= config.min_torque_margin &&
         m.joint_accel_box >= config.min_joint_accel_margin &&
         m.eom_residual_inf <= config.max_eom_residual;
}

SolveHealthState SolveHealth::Update(bool tau_valid, const WbcMargins& margins) noexcept {
  const bool good = tau_valid && MarginsPass(config_, margins);
  if (good) {
    good_run_ = Saturating(good_run_);
    // The bad ticks count until the state is Nominal again: a failure that
    // keeps coming back between short good stretches still reaches Fault.
    if (state_ != SolveHealthState::kNominal && good_run_ >= config_.recover_valid_ticks) {
      state_ = SolveHealthState::kNominal;
      bad_run_ = 0;
    }
  } else {
    good_run_ = 0;
    bad_run_ = Saturating(bad_run_);
    // In Fault the count is already past the limit (it clears only on
    // reaching Nominal), so a bad tick keeps it there.
    state_ = config_.max_invalid_ticks >= 0 && bad_run_ > config_.max_invalid_ticks
                 ? SolveHealthState::kFault
                 : SolveHealthState::kDegraded;
  }
  return state_;
}

}  // namespace rtc::tsid
