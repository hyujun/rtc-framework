#include "rtc_controllers/params/reach_gate_params.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc::params {

namespace {

template <typename... Parts>
[[noreturn]] void Reject(Parts&&... parts) {
  std::string msg = "reach_gate: ";
  (msg.append(std::forward<Parts>(parts)), ...);
  throw std::invalid_argument(msg);
}

/// A present key must parse; only an ABSENT key takes the default. The default
/// is the trained constant, so defaulting a typo would look exactly like a
/// correct config.
template <typename T>
T ReadOptional(const YAML::Node& node, const char* key, T fallback) {
  const YAML::Node v = node[key];
  if (!v) {
    return fallback;
  }
  try {
    return v.as<T>();
  } catch (const YAML::Exception&) {
    Reject("'", key, "' is present but does not parse as a number");
  }
}

}  // namespace

ReachGateParams ParseReachGateParams(const YAML::Node& node) {
  if (!node || !node.IsMap()) {
    Reject("must be a map with a `tips:` list");
  }
  ReachGateParams out;

  const YAML::Node tips = node["tips"];
  if (!tips || !tips.IsSequence() || tips.size() < 2) {
    Reject(
        "`tips:` must list at least two tips (the opposing digit first, then the fingers it "
        "opposes)");
  }
  out.tips.reserve(tips.size());
  for (std::size_t i = 0; i < tips.size(); ++i) {
    const std::string at = "tips[" + std::to_string(i) + "]";
    const YAML::Node t = tips[i];
    if (!t || !t.IsMap()) {
      Reject(at, " must be a map {link, force_group, contact_obj}");
    }
    ReachGateTip tip;
    tip.link = t["link"].as<std::string>("");
    tip.force_group = t["force_group"].as<std::string>("");
    if (tip.link.empty()) {
      Reject(at, " is missing `link` (the body whose origin is the tip)");
    }
    if (tip.force_group.empty()) {
      Reject(at, " ('", tip.link, "') is missing `force_group` (the sensor group it reads)");
    }
    const YAML::Node c = t["contact_obj"];
    if (!c || !c.IsSequence() || c.size() != 3) {
      Reject(at, " ('", tip.link, "') needs `contact_obj: [x, y, z]` in the object frame [m]");
    }
    for (std::size_t k = 0; k < 3; ++k) {
      double v = std::nan("");
      try {
        v = c[k].as<double>();
      } catch (const YAML::Exception&) {
        v = std::nan("");
      }
      if (!std::isfinite(v)) {
        Reject(at, " ('", tip.link, "') contact_obj[", std::to_string(k), "] must be finite");
      }
      tip.contact_obj[k] = v;
    }
    for (const auto& prev : out.tips) {
      if (prev.link == tip.link) {
        Reject(at, " repeats link '", tip.link, "'");
      }
      if (prev.force_group == tip.force_group) {
        Reject(at, " repeats force_group '", tip.force_group,
               "' — two tips reading one lane would both press or both not");
      }
    }
    out.tips.push_back(std::move(tip));
  }

  out.tip_std = ReadOptional(node, "tip_std", out.tip_std);
  out.force_threshold = ReadOptional(node, "force_threshold", out.force_threshold);
  out.min_fingers = ReadOptional(node, "min_fingers", out.min_fingers);
  out.hold_on_steps = ReadOptional(node, "hold_on_steps", out.hold_on_steps);
  out.hold_off_steps = ReadOptional(node, "hold_off_steps", out.hold_off_steps);

  if (!(out.tip_std > 0.0) || !std::isfinite(out.tip_std)) {
    Reject("tip_std must be a finite value > 0 [m]");
  }
  if (!(out.force_threshold >= 0.0) || !std::isfinite(out.force_threshold)) {
    Reject("force_threshold must be a finite value >= 0 [N]");
  }
  const int others = static_cast<int>(out.tips.size()) - 1;
  if (out.min_fingers < 1 || out.min_fingers > others) {
    Reject("min_fingers must be in [1, ", std::to_string(others), "] (the tips after the first)");
  }
  if (out.hold_on_steps < 1 || out.hold_off_steps < 1) {
    Reject("hold_on_steps and hold_off_steps must both be >= 1 (they count policy steps)");
  }
  return out;
}

}  // namespace rtc::params
