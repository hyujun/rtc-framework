// locomotion_controller's own YAML sections. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "rtc_controllers/params/yaml_read.hpp"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace integrated_bringup {

namespace {

constexpr const char* kContext = "locomotion_controller";
constexpr double kHalfPi = std::numbers::pi / 2.0;

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
             "solver", "kinodynamic", "max_stale_solutions", "sync", "activation_height"},
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
  CheckKeys(n["sanity"], san, {"joint_delta_rad", "base_speed", "wrench_scale"});
  out.sanity_joint_delta_rad = ReadPositive(n["sanity"], "joint_delta_rad", san, false);
  out.sanity_base_speed = ReadPositive(n["sanity"], "base_speed", san, false);
  out.sanity_wrench_scale = ReadPositive(n["sanity"], "wrench_scale", san, false);
  // `activation_height:` (optional; absent = config).
  if (n["activation_height"]) {
    std::string source;
    try {
      source = n["activation_height"].as<std::string>();
    } catch (const YAML::Exception&) {
      Refuse(where + ".activation_height",
             "expected a string, got " + Spelling(n["activation_height"]));
    }
    if (source != "config" && source != "measured") {
      Refuse(where + ".activation_height", "must be config or measured, got '" + source + "'");
    }
    out.activation_height_measured = source == "measured";
  }
  // `sync:` (optional; absent = free_run). The tick counts (N integer, L in
  // 1..N) are checked where the control rate is known (BuildMpc).
  if (n["sync"]) {
    const std::string sy = where + ".sync";
    CheckKeys(n["sync"], sy, {"mode", "delivery_ticks", "wait_timeout_s"}, {"mode"});
    std::string mode;
    try {
      mode = n["sync"]["mode"].as<std::string>();
    } catch (const YAML::Exception&) {
      Refuse(sy + ".mode", "expected a string, got " + Spelling(n["sync"]["mode"]));
    }
    if (mode == "lockstep") {
      out.lockstep = true;
    } else if (mode != "free_run") {
      Refuse(sy + ".mode", "expected free_run or lockstep, got '" + mode + "'");
    }
    if (n["sync"]["delivery_ticks"]) {
      int ticks = 0;
      try {
        ticks = n["sync"]["delivery_ticks"].as<int>();
      } catch (const YAML::Exception&) {
        Refuse(sy + ".delivery_ticks",
               "expected an integer, got " + Spelling(n["sync"]["delivery_ticks"]));
      }
      if (ticks < 0) {
        Refuse(sy + ".delivery_ticks",
               "must be >= 0 (0 = one period), got " + std::to_string(ticks));
      }
      out.delivery_ticks = ticks;
    }
    if (n["sync"]["wait_timeout_s"]) {
      out.wait_timeout_s = ReadPositive(n["sync"], "wait_timeout_s", sy, /*zero_ok=*/false);
    }
  }
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
  CheckKeys(cfg["pd_feedforward"], pd, {"kp", "kd", "target", "target_time_constant_s"});
  const YAML::Node target = cfg["pd_feedforward"]["target"];
  if (!target.IsScalar() || (target.Scalar() != "plan" && target.Scalar() != "wbc")) {
    Refuse(pd + ".target", "expected \"plan\" or \"wbc\", got " + Spelling(target));
  }
  out.pd_target_wbc = target.Scalar() == "wbc";
  out.pd_kp = ReadGains(cfg["pd_feedforward"], "kp", pd);
  out.pd_kd = ReadGains(cfg["pd_feedforward"], "kd", pd);
  out.pd_target_time_constant_s =
      ReadPositive(cfg["pd_feedforward"], "target_time_constant_s", pd, /*zero_ok=*/true);

  const std::string fault = root + ".fault";
  CheckKeys(cfg["fault"], fault, {"max_source_bad_ticks", "damping_kd", "pose_guard"});
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
  // `fault.pose_guard` — the one tuned quantity of the guard (header): the
  // tilt from vertical a pose may have and still be latched, (0, π/2].
  const std::string guard = fault + ".pose_guard";
  CheckKeys(cfg["fault"]["pose_guard"], guard, {"base_tilt_rad"});
  const double tilt = ReadPositive(cfg["fault"]["pose_guard"], "base_tilt_rad", guard, false);
  if (!(tilt <= kHalfPi)) {
    Refuse(guard + ".base_tilt_rad", "must be at most pi/2 (a horizontal body), got " +
                                         Spelling(cfg["fault"]["pose_guard"]["base_tilt_rad"]));
  }
  out.pose_guard_base_tilt_rad = tilt;

  if (!cfg["whole_body_qp"] || !cfg["whole_body_qp"].IsMap()) {
    Refuse(root + ".whole_body_qp", "expected a map, got " + Spelling(cfg["whole_body_qp"]));
  }
  out.whole_body_qp = YAML::Clone(cfg["whole_body_qp"]);
  if (cfg["mpc"]) {
    out.mpc = ParseMpc(cfg["mpc"], root + ".mpc");
  }
  // `velocity_command:` — the topic, no default: a misspelt key or an empty
  // name must not fall back to a topic nobody publishes on. `type` picks the
  // message (twist when absent); `joy` maps a Joy (joy_command.hpp).
  if (cfg["velocity_command"]) {
    const std::string where = root + ".velocity_command";
    const YAML::Node vc = cfg["velocity_command"];
    CheckKeys(vc, where, {"topic", "type", "joy"}, {"topic"});
    const YAML::Node t = vc["topic"];
    if (!t.IsScalar() || t.Scalar().empty()) {
      Refuse(where + ".topic", "expected a non-empty topic name, got " + Spelling(t));
    }
    out.velocity_command_topic = t.Scalar();
    if (const YAML::Node type = vc["type"]) {
      if (!type.IsScalar() || (type.Scalar() != "twist" && type.Scalar() != "joy")) {
        Refuse(where + ".type", "expected twist or joy, got " + Spelling(type));
      }
      out.velocity_command_joy = type.Scalar() == "joy";
    }
    if (out.velocity_command_joy) {
      if (!vc["joy"]) {
        Refuse(where + ".joy", "required with type joy");
      }
      out.velocity_command_joy_map = locomotion::ParseJoyCommandMapping(vc["joy"], where + ".joy");
    } else if (vc["joy"]) {
      Refuse(where + ".joy", "only with type joy (the type is twist)");
    }
  }
  // `external_wrench:` — the simulator's applied-wrench report for the truth
  // log. Both keys required: an age bound nobody chose would decide which
  // ticks read as "nothing applied" and which as "unknown". An empty topic is
  // the lane switched off — the shipped value, there so that a launch overlay
  // (which can only replace a key the file has) can name one.
  if (cfg["external_wrench"]) {
    const std::string where = root + ".external_wrench";
    const YAML::Node ew = cfg["external_wrench"];
    CheckKeys(ew, where, {"topic", "max_age_ms"}, {"topic", "max_age_ms"});
    const YAML::Node t = ew["topic"];
    if (!t.IsScalar()) {
      Refuse(where + ".topic", "expected a topic name (empty = off), got " + Spelling(t));
    }
    out.external_wrench_topic = t.Scalar();
    double age = 0.0;
    try {
      age = ew["max_age_ms"].as<double>();
    } catch (const YAML::Exception&) {
      Refuse(where + ".max_age_ms", "expected a number, got " + Spelling(ew["max_age_ms"]));
    }
    if (!std::isfinite(age) || age <= 0.0) {
      Refuse(where + ".max_age_ms",
             "expected a finite value > 0, got " + Spelling(ew["max_age_ms"]));
    }
    out.external_wrench_max_age_ms = age;
  }
  return out;
}

}  // namespace integrated_bringup
