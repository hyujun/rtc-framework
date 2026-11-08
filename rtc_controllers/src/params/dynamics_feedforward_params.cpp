#include "rtc_controllers/params/dynamics_feedforward_params.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseDynamicsFeedforwardParams(const YAML::Node& block, std::string_view key,
                                    DynamicsFeedforwardParams& out) {
  const std::string k(key);
  // A default-constructed node (absent file) is Null-but-truthy and a missing
  // key is falsy; IsMap() refuses both the same way.
  if (!block || !block.IsMap()) {
    throw std::runtime_error(k + ": required block is missing (needs at least 'max_abs')");
  }
  if (block["enabled"]) {
    out.enabled = block["enabled"].as<bool>();
  }
  if (block["gain"]) {
    const double g = block["gain"].as<double>();
    if (!std::isfinite(g) || g < 0.0 || g > kDynamicsFeedforwardGainMax) {
      throw std::runtime_error(k + ".gain must be finite and within [0, " +
                               std::to_string(kDynamicsFeedforwardGainMax) + "]");
    }
    out.gravity_gain = g;
  }
  // Required: the bound is what keeps a placeholder device rating from being
  // the only thing between a model error and the motors.
  if (!block["max_abs"]) {
    throw std::runtime_error(k + ".max_abs is required (N·m, > 0) — there is no default bound");
  }
  const double m = block["max_abs"].as<double>();
  if (!std::isfinite(m) || !(m > 0.0)) {
    throw std::runtime_error(k + ".max_abs must be finite and > 0 (N·m)");
  }
  out.max_abs = m;
}

}  // namespace rtc::params
