#include "rtc_tsid/formulation/task_factory.hpp"

#include "rtc_tsid/constraints/contact_constraint.hpp"
#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/tasks/com_task.hpp"
#include "rtc_tsid/tasks/contact_consistency_task.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/internal_force_task.hpp"
#include "rtc_tsid/tasks/object_se3_task.hpp"
#include "rtc_tsid/tasks/object_wrench_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace rtc::tsid {
namespace {

void RequireCore(const TaskFactoryContext& ctx) {
  if (ctx.model == nullptr || ctx.robot_info == nullptr || ctx.cache == nullptr ||
      ctx.contacts == nullptr) {
    throw std::invalid_argument("task_factory: model, robot_info, cache and contacts are required");
  }
}

void RequireObjectContext(const TaskFactoryContext& ctx, const std::string& key) {
  if (ctx.contacts_runtime == nullptr || ctx.grasp == nullptr) {
    throw std::invalid_argument("task_factory: task '" + key +
                                "' needs the runtime contact manager and the grasp cache");
  }
}

}  // namespace

BuildReport BuildTasks(const YAML::Node& tasks, const TaskFactoryContext& ctx,
                       FormulationBase& formulation) {
  BuildReport report;
  if (!tasks || !tasks.IsMap()) {
    return report;
  }
  RequireCore(ctx);
  const auto& model = *ctx.model;
  const auto& info = *ctx.robot_info;
  auto& cache = *ctx.cache;
  for (auto it = tasks.begin(); it != tasks.end(); ++it) {
    const auto key = it->first.as<std::string>();
    // yaml-cpp iterator::operator->() returns a proxy holding a prvalue Node;
    // binding `it->second` by reference leaves a dangling handle once the
    // proxy temporary dies. Copy by value — Node is a lightweight handle.
    const YAML::Node cfg = it->second;
    const auto type = cfg["type"].as<std::string>("");
    std::unique_ptr<TaskBase> task;

    if (type == "posture") {
      auto t = std::make_unique<PostureTask>();
      t->Init(model, info, cache, cfg);
      task = std::move(t);
    } else if (type == "se3") {
      auto t = std::make_unique<SE3Task>();
      t->Init(model, info, cache, cfg);
      // SE3Task's identity comes from its inner `name:` field (default "se3"),
      // NOT the YAML map key. Phase presets and reference pushes look the task
      // up by the map key — if the two diverge the task is unreachable.
      const std::string resolved{t->Name()};
      if (resolved != key) {
        report.messages.push_back({false, "se3 task '" + key + "' resolves to name '" + resolved +
                                              "' — they must match; add `name: \"" + key +
                                              "\"` to the task block"});
      }
      task = std::move(t);
    } else if (type == "force") {
      auto t = std::make_unique<ForceTask>();
      t->Init(model, info, cache, cfg);
      t->SetContactManager(ctx.contacts);
      task = std::move(t);
    } else if (type == "com") {
      auto t = std::make_unique<CoMTask>();
      t->Init(model, info, cache, cfg);
      task = std::move(t);
    } else if (type == "contact_consistency") {
      auto t = std::make_unique<ContactConsistencyTask>();
      t->Init(model, info, cache, cfg);
      t->SetContactManager(ctx.contacts);
      task = std::move(t);
    } else if (type == "object_wrench") {
      RequireObjectContext(ctx, key);
      auto t = std::make_unique<ObjectWrenchTask>();
      t->Init(model, info, cache, cfg);
      t->SetContactManager(ctx.contacts);
      t->SetContactManagerRuntime(ctx.contacts_runtime);
      t->SetGraspCache(ctx.grasp);
      t->SetObjectFrame(ctx.object_frame);
      task = std::move(t);
    } else if (type == "internal_force") {
      RequireObjectContext(ctx, key);
      auto t = std::make_unique<InternalForceTask>();
      t->Init(model, info, cache, cfg);
      t->SetContactManager(ctx.contacts);
      t->SetContactManagerRuntime(ctx.contacts_runtime);
      t->SetGraspCache(ctx.grasp);
      task = std::move(t);
    } else if (type == "object_se3") {
      RequireObjectContext(ctx, key);
      auto t = std::make_unique<ObjectSE3Task>();
      t->Init(model, info, cache, cfg);
      t->SetContactManager(ctx.contacts);
      t->SetContactManagerRuntime(ctx.contacts_runtime);
      t->SetGraspCache(ctx.grasp);
      t->SetObjectStateProvider(ctx.object_state);
      task = std::move(t);
    } else {
      report.messages.push_back(
          {true, "unknown task type '" + type + "' for entry '" + key + "' — skipping"});
      continue;
    }
    TaskBase* raw = task.get();
    formulation.AddTask(std::move(task));
    report.built.push_back({key, type, cfg, raw, nullptr});
  }
  return report;
}

BuildReport BuildConstraints(const YAML::Node& constraints, const TaskFactoryContext& ctx,
                             FormulationBase& formulation) {
  BuildReport report;
  if (!constraints || !constraints.IsMap()) {
    return report;
  }
  RequireCore(ctx);
  const auto& model = *ctx.model;
  const auto& info = *ctx.robot_info;
  auto& cache = *ctx.cache;
  const double period = ctx.control_period;
  for (auto it = constraints.begin(); it != constraints.end(); ++it) {
    const auto key = it->first.as<std::string>();
    const YAML::Node cfg = it->second;  // by value (see BuildTasks)
    const auto type = cfg["type"].as<std::string>("");

    // A constraint that integrates over the control period: a missing `dt`
    // takes it, one that disagrees fails (#1019).
    const auto with_period = [&](const YAML::Node& node) {
      YAML::Node out = YAML::Clone(node);
      if (period > 0.0 && (!out["dt"] || out["dt"].IsNull())) {
        out["dt"] = period;
      }
      return out;
    };
    const auto check_period = [&](double dt) {
      if (period > 0.0 && std::abs(dt - period) > 1e-9 * period) {
        char msg[192];
        std::snprintf(msg, sizeof(msg),
                      "tsid.constraints.%s.dt %.9g s differs from the control period %.9g s "
                      "(1/control_rate); omit it to take the period",
                      key.c_str(), dt, period);
        throw std::invalid_argument(msg);
      }
    };

    std::unique_ptr<ConstraintBase> constraint;
    if (type == "eom") {
      auto c = std::make_unique<EomConstraint>();
      c->Init(model, info, cache, cfg);
      // The λ column offsets follow each contact's dimension (3 / 6).
      c->SetContactManager(ctx.contacts);
      constraint = std::move(c);
    } else if (type == "contact_constraint") {
      auto c = std::make_unique<ContactConstraint>();
      c->Init(model, info, cache, cfg);
      c->SetContactManager(ctx.contacts);
      constraint = std::move(c);
    } else if (type == "joint_limit") {
      auto c = std::make_unique<JointLimitConstraint>();
      c->Init(model, info, cache, with_period(cfg));
      check_period(c->Dt());
      constraint = std::move(c);
    } else if (type == "friction_cone") {
      // The face count is each contact's (contacts[*].n_faces / friction_faces).
      if (cfg["n_faces"]) {
        report.messages.push_back(
            {false,
             "friction_cone.n_faces is ignored — set the face count per contact via "
             "contacts[*].n_faces (or friction_faces)"});
      }
      auto c = std::make_unique<FrictionConeConstraint>();
      c->Init(model, info, cache, cfg);
      c->SetContactManager(ctx.contacts);
      constraint = std::move(c);
    } else if (type == "torque_limit") {
      auto c = std::make_unique<TorqueLimitConstraint>();
      c->Init(model, info, cache, with_period(cfg));
      check_period(c->Dt());
      c->SetContactManager(ctx.contacts);
      constraint = std::move(c);
    } else {
      report.messages.push_back(
          {true, "unknown constraint type '" + type + "' for entry '" + key + "' — skipping"});
      continue;
    }
    ConstraintBase* raw = constraint.get();
    formulation.AddConstraint(std::move(constraint));
    report.built.push_back({key, type, cfg, nullptr, raw});
  }
  return report;
}

}  // namespace rtc::tsid
