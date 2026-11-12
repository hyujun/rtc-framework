#pragma once

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/contact/object_state_provider.hpp"
#include "rtc_tsid/core/constraint_base.hpp"
#include "rtc_tsid/core/formulation_base.hpp"
#include "rtc_tsid/core/task_base.hpp"
#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// YAML task / constraint dispatch — the one place a `type:` string becomes a
// rtc_tsid task or constraint (WD-58). Init time only: allocates and throws.
//
//   tasks:        {<key>: {type: posture | se3 | force | com | contact_consistency |
//                                object_wrench | internal_force | object_se3, …}}
//   constraints:  {<key>: {type: eom | contact_constraint | joint_limit |
//                                friction_cone | torque_limit, …}}
//
// Each entry is Init'd with its own map, wired to the context (contact
// manager, and for the object tasks the runtime contact manager, grasp cache,
// object frame and state provider), and added to the formulation in YAML
// order. With `control_period > 0` the constraints that integrate over one
// period (joint_limit, torque_limit) take it when their `dt` is missing and
// throw std::invalid_argument when it disagrees (#1019). An unknown type is
// skipped with an error message; the caller logs the messages (rtc_tsid has
// no logger). The report lists what was built so the caller can post-process
// (its own keys, pointers it keeps for diagnostics).
// ────────────────────────────────────────────────

struct TaskFactoryContext {
  const pinocchio::Model* model{nullptr};
  const RobotModelInfo* robot_info{nullptr};
  PinocchioCache* cache{nullptr};
  const ContactManagerConfig* contacts{nullptr};
  // The object tasks only (object_wrench · internal_force · object_se3).
  const ContactManager* contacts_runtime{nullptr};
  const GraspCache* grasp{nullptr};
  const ObjectFrame* object_frame{nullptr};
  const ObjectStateProvider* object_state{nullptr};
  double control_period{0.0};  // [s] > 0: the period-integrating constraints' dt
};

struct FactoryMessage {
  bool error{false};  // true: an entry was skipped; false: a warning
  std::string text;
};

struct BuiltEntry {
  std::string key;
  std::string type;
  YAML::Node config;
  TaskBase* task{nullptr};              // owned by the formulation
  ConstraintBase* constraint{nullptr};  // owned by the formulation
};

struct BuildReport {
  std::vector<BuiltEntry> built;
  std::vector<FactoryMessage> messages;

  // The entry built under `key`, or nullptr.
  [[nodiscard]] const BuiltEntry* Find(const std::string& key) const noexcept {
    for (const BuiltEntry& e : built) {
      if (e.key == key) {
        return &e;
      }
    }
    return nullptr;
  }
};

// Build every entry of `tasks` (a map) into `formulation`. A null or empty
// node builds nothing. Throws std::invalid_argument when the context lacks
// what an entry needs.
[[nodiscard]] BuildReport BuildTasks(const YAML::Node& tasks, const TaskFactoryContext& ctx,
                                     FormulationBase& formulation);

// Build every entry of `constraints` (a map) into `formulation` (see above).
[[nodiscard]] BuildReport BuildConstraints(const YAML::Node& constraints,
                                           const TaskFactoryContext& ctx,
                                           FormulationBase& formulation);

}  // namespace rtc::tsid
