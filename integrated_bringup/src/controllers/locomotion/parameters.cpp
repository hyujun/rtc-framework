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

// A map whose keys are among `allowed`, with every one of `required` present.
void CheckKeys(const YAML::Node& map, const std::string& where,
               std::initializer_list<const char*> allowed,
               std::initializer_list<const char*> required) {
  if (!map || !map.IsMap()) {
    Refuse(where, "expected a map, got " + Spelling(map));
  }
  if (const auto unknown = FindUnknownKey(map, allowed)) {
    Refuse(where, "unknown key '" + *unknown + "'");
  }
  if (const auto missing = FindMissingKey(map, required)) {
    Refuse(where, "required key '" + *missing + "' is missing");
  }
}

void CheckKeys(const YAML::Node& map, const std::string& where,
               std::initializer_list<const char*> keys) {
  CheckKeys(map, where, keys, keys);
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

double ReadPositive(const YAML::Node& map, const char* key, const std::string& where,
                    bool zero_ok) {
  const YAML::Node v = map[key];
  double x = std::numeric_limits<double>::quiet_NaN();
  try {
    x = v.as<double>();
  } catch (const YAML::Exception&) {
  }
  if (!(std::isfinite(x) && (x > 0.0 || (zero_ok && x == 0.0)))) {
    Refuse(where + "." + key, std::string("expected a finite number ") +
                                  (zero_ok ? ">= 0" : "> 0") + ", got " + Spelling(v));
  }
  return x;
}

bool ReadBool(const YAML::Node& map, const char* key, const std::string& where) {
  const YAML::Node v = map[key];
  try {
    return v.as<bool>();
  } catch (const YAML::Exception&) {
  }
  Refuse(where + "." + key, "expected true or false, got " + Spelling(v));
}

// `mpc:` — the consumer's keys are required (an absent riccati.enabled would
// read as on in the manager); the factory's and the manager's own keys are
// passed through to them.
LocomotionMpcConfig ParseMpc(const YAML::Node& n, const std::string& where) {
  LocomotionMpcConfig out;
  CheckKeys(n, where,
            {"enabled", "period_nodes", "max_policy_age_s", "riccati", "sanity", "ocp_type",
             "solver", "kinodynamic", "max_stale_solutions"},
            {"enabled", "period_nodes", "max_policy_age_s", "riccati", "sanity"});
  out.present = true;
  out.enabled = ReadBool(n, "enabled", where);
  int nodes = 0;
  try {
    nodes = n["period_nodes"].as<int>();
  } catch (const YAML::Exception&) {
    Refuse(where + ".period_nodes", "expected an integer, got " + Spelling(n["period_nodes"]));
  }
  if (nodes < 1) {
    Refuse(where + ".period_nodes", "must be >= 1, got " + std::to_string(nodes));
  }
  out.period_nodes = nodes;
  out.max_policy_age_s = ReadPositive(n, "max_policy_age_s", where, /*zero_ok=*/true);
  const std::string ric = where + ".riccati";
  const YAML::Node r = n["riccati"];
  CheckKeys(r, ric, {"enabled", "gain_scale", "accel_only", "max_delta_x_norm", "max_delta_blocks"},
            {"enabled"});
  out.riccati_enabled = ReadBool(r, "enabled", ric);
  const std::string san = where + ".sanity";
  CheckKeys(n["sanity"], san, {"joint_delta_rad", "base_speed", "joint_accel", "wrench_scale"});
  out.sanity_joint_delta_rad = ReadPositive(n["sanity"], "joint_delta_rad", san, false);
  out.sanity_base_speed = ReadPositive(n["sanity"], "base_speed", san, false);
  out.sanity_joint_accel = ReadPositive(n["sanity"], "joint_accel", san, false);
  out.sanity_wrench_scale = ReadPositive(n["sanity"], "wrench_scale", san, false);
  out.node = YAML::Clone(n);
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
  if (cfg["mpc"]) {
    out.mpc = ParseMpc(cfg["mpc"], root + ".mpc");
  }
  // `velocity_command:` — the Twist topic, no default: a misspelt key or an
  // empty name must not fall back to a topic nobody publishes on.
  if (cfg["velocity_command"]) {
    const std::string where = root + ".velocity_command";
    CheckKeys(cfg["velocity_command"], where, {"topic"});
    const YAML::Node t = cfg["velocity_command"]["topic"];
    if (!t.IsScalar() || t.Scalar().empty()) {
      Refuse(where + ".topic", "expected a non-empty topic name, got " + Spelling(t));
    }
    out.velocity_command_topic = t.Scalar();
  }
  return out;
}

}  // namespace integrated_bringup
