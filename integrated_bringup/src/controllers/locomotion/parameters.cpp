// locomotion_controller's own YAML sections. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "rtc_controllers/params/yaml_read.hpp"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace integrated_bringup {

namespace {

constexpr const char* kContext = "locomotion_controller";

using rtc::params::FindMissingKey;
using rtc::params::FindUnknownKey;
using rtc::params::Spelling;

[[noreturn]] void Refuse(const std::string& where, const std::string& what) {
  throw std::invalid_argument(where + ": " + what);
}

void CheckKeys(const YAML::Node& map, const std::string& where,
               std::initializer_list<const char*> keys) {
  if (!map || !map.IsMap()) {
    Refuse(where, "expected a map, got " + Spelling(map));
  }
  if (const auto unknown = FindUnknownKey(map, keys)) {
    Refuse(where, "unknown key '" + *unknown + "'");
  }
  if (const auto missing = FindMissingKey(map, keys)) {
    Refuse(where, "required key '" + *missing + "' is missing");
  }
}

// A non-empty list of finite, non-negative numbers (a gain per joint).
std::vector<double> ReadGains(const YAML::Node& map, const char* key, const std::string& where) {
  const YAML::Node v = map[key];
  const std::string at = where + "." + key;
  if (!v.IsSequence() || v.size() == 0) {
    Refuse(at, "expected a non-empty list of gains, got " + Spelling(v));
  }
  std::vector<double> out;
  out.reserve(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) {
    double x = std::numeric_limits<double>::quiet_NaN();
    try {
      x = v[i].as<double>();
    } catch (const YAML::Exception&) {
    }
    if (!(std::isfinite(x) && x >= 0.0)) {
      Refuse(at + "[" + std::to_string(i) + "]",
             "expected a finite gain >= 0, got " + Spelling(v[i]));
    }
    out.push_back(x);
  }
  return out;
}

}  // namespace

LocomotionControllerConfig ParseLocomotionControllerConfig(const YAML::Node& cfg) {
  LocomotionControllerConfig out;
  const std::string root(kContext);
  // Presence first: a missing key subscripts to an invalid node, which every
  // later read would turn into a YAML exception instead of this message.
  if (!cfg.IsMap()) {
    Refuse(root, "expected a map, got " + Spelling(cfg));
  }
  if (const auto missing = FindMissingKey(cfg, {"command_type", "model", "estimator",
                                                "pd_feedforward", "fault", "whole_body_qp"})) {
    Refuse(root, "required key '" + *missing + "' is missing");
  }
  if (!cfg["command_type"] || !cfg["command_type"].IsScalar() ||
      cfg["command_type"].Scalar() != "pd_feedforward") {
    Refuse(root + ".command_type",
           "must be \"pd_feedforward\" — the output is a position target plus a feedforward "
           "torque, got " +
               Spelling(cfg["command_type"]));
  }
  out.model = locomotion::LoadHumanoidModelConfig(cfg["model"], root + ".model");
  out.estimator = locomotion::ParseEstimatorConfig(cfg["estimator"], root + ".estimator");

  const std::string pd = root + ".pd_feedforward";
  CheckKeys(cfg["pd_feedforward"], pd, {"kp", "kd"});
  out.pd_kp = ReadGains(cfg["pd_feedforward"], "kp", pd);
  out.pd_kd = ReadGains(cfg["pd_feedforward"], "kd", pd);

  const std::string fault = root + ".fault";
  CheckKeys(cfg["fault"], fault, {"max_source_bad_ticks", "damping_kd"});
  const YAML::Node n = cfg["fault"]["max_source_bad_ticks"];
  int ticks = 0;
  try {
    ticks = n.as<int>();
  } catch (const YAML::Exception&) {
    Refuse(fault + ".max_source_bad_ticks", "expected an integer, got " + Spelling(n));
  }
  if (ticks < 1) {
    Refuse(fault + ".max_source_bad_ticks", "must be >= 1, got " + std::to_string(ticks));
  }
  out.max_source_bad_ticks = ticks;
  out.damping_kd = ReadGains(cfg["fault"], "damping_kd", fault);

  if (!cfg["whole_body_qp"] || !cfg["whole_body_qp"].IsMap()) {
    Refuse(root + ".whole_body_qp", "expected a map, got " + Spelling(cfg["whole_body_qp"]));
  }
  out.whole_body_qp = YAML::Clone(cfg["whole_body_qp"]);
  return out;
}

}  // namespace integrated_bringup
