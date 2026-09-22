// Parser for `catching.planner.*` thread keys (S6). See planner_params.hpp.
#include "rtc_controllers/catching/planner_params.hpp"

#include "catching_yaml_read.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::catching {

namespace {

using params_detail::ReadSectionNode;
using params_detail::SectionKind;
using params_detail::Spelling;

[[noreturn]] void Reject(const std::string& what) {
  throw std::invalid_argument("ParsePlannerParams: " + what);
}

/// A finite number inside [lo, hi], or the fallback when absent. `TBD` is NOT
/// accepted: these keys have documented defaults (L3 §6), so a TBD here is a
/// profile that meant something and did not say what.
double ReadBounded(const YAML::Node& sec, const char* key, double fallback, double lo, double hi) {
  const YAML::Node v = sec[key];
  if (!v) {
    return fallback;
  }
  double d = 0.0;
  try {
    d = v.as<double>();
  } catch (const YAML::Exception&) {
    Reject(std::string("'planner.") + key + "' must be a number, got " + Spelling(v));
  }
  if (!std::isfinite(d) || d < lo || d > hi) {
    Reject(std::string("'planner.") + key + "' = " + Spelling(v) + " is outside [" +
           std::to_string(lo) + ", " + std::to_string(hi) + "] (L3 §6)");
  }
  return d;
}

}  // namespace

PlannerParams ParsePlannerParams(const YAML::Node& catching) {
  if (!catching || !catching.IsMap()) {
    Reject("must be given the `catching:` map");
  }
  PlannerParams out;
  const auto section = ReadSectionNode(catching, "planner");
  if (section.kind == SectionKind::kNotAMap) {
    Reject("'planner' must be a map");
  }
  if (section.kind == SectionKind::kAbsent) {
    return out;
  }
  const YAML::Node& planner = section.node;

  if (const YAML::Node v = planner["enabled"]; v) {
    try {
      out.enabled = v.as<bool>();
    } catch (const YAML::Exception&) {
      Reject("'planner.enabled' must be a bool, got " + Spelling(v));
    }
  }
  out.wake_timeout_s = ReadBounded(planner, "wake_timeout_s", out.wake_timeout_s,
                                   kPlannerWakeTimeoutMinS, kPlannerWakeTimeoutMaxS);
  out.budget_s =
      ReadBounded(planner, "budget_s", out.budget_s, kPlannerBudgetMinS, kPlannerBudgetMaxS);

  if (const YAML::Node pose = planner["wait_pose"]; pose) {
    if (!pose.IsSequence() || pose.size() == 0) {
      Reject("'planner.wait_pose' must be a non-empty sequence of joint angles, got " +
             Spelling(pose));
    }
    if (pose.size() > out.wait_pose.size()) {
      Reject("'planner.wait_pose' has " + std::to_string(pose.size()) + " entries; at most " +
             std::to_string(out.wait_pose.size()) + " (kMaxPlanNv)");
    }
    for (std::size_t i = 0; i < pose.size(); ++i) {
      double q = 0.0;
      try {
        q = pose[i].as<double>();
      } catch (const YAML::Exception&) {
        Reject("'planner.wait_pose[" + std::to_string(i) + "]' must be a number, got " +
               Spelling(pose[i]));
      }
      if (!std::isfinite(q)) {
        Reject("'planner.wait_pose[" + std::to_string(i) + "]' is not finite");
      }
      out.wait_pose[i] = q;
    }
    out.wait_pose_n = static_cast<std::int32_t>(pose.size());
  }
  return out;
}

}  // namespace rtc::catching
