#include "rtc_tsid/locomotion/locomotion_formulation.hpp"

#include "rtc_tsid/formulation/task_factory.hpp"

#include <cmath>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>

namespace rtc::tsid {
namespace {

void RefuseUnknownKeys(const YAML::Node& map, std::initializer_list<const char*> allowed,
                       const std::string& where, const char* also_allowed = nullptr) {
  if (!map) {
    return;
  }
  if (!map.IsMap()) {
    throw std::invalid_argument(where + ": must be a map");
  }
  for (auto it = map.begin(); it != map.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    bool known = also_allowed != nullptr && key == also_allowed;
    for (const char* a : allowed) {
      known = known || key == a;
    }
    if (!known) {
      throw std::invalid_argument(where + ": unknown key '" + key + "'");
    }
  }
}

// Per contact (contact_overrides.<name>); `contact` also takes t_ramp, which
// is one value for the whole contact set.
constexpr std::initializer_list<const char*> kContactKeys = {
    "friction_coeff",           "friction_faces",   "patch_half_length_x", "patch_half_length_y",
    "torsional_friction_coeff", "normal_force_min", "normal_force_max"};

// `entry` (a task / constraint map) as a task_factory entry of `type`. The
// keys the builder sets itself are refused rather than silently overwritten.
YAML::Node Entry(const YAML::Node& entry, const char* type, const std::string& where) {
  if (entry && !entry.IsMap()) {
    throw std::invalid_argument(where + ": must be a map");
  }
  for (const char* reserved : {"type", "name", "frame", "base_frame"}) {
    if (entry && entry[reserved]) {
      throw std::invalid_argument(where + "." + reserved + ": set by the builder, not the YAML");
    }
  }
  YAML::Node out = entry ? YAML::Clone(entry) : YAML::Node(YAML::NodeType::Map);
  out["type"] = type;
  return out;
}

}  // namespace

std::vector<pinocchio::FrameIndex> LocomotionFormulationBuilder::ContactFrameIds(
    const pinocchio::Model& model, const LocomotionModelSpec& spec) {
  std::vector<pinocchio::FrameIndex> ids;
  for (const auto& c : spec.contacts) {
    if (!model.existFrame(c.frame)) {
      throw std::invalid_argument("whole_body_qp: contact frame '" + c.frame +
                                  "' is not in the model");
    }
    ids.push_back(model.getFrameId(c.frame));
  }
  return ids;
}

void LocomotionFormulationBuilder::Init(const pinocchio::Model& model,
                                        const RobotModelInfo& robot_info, PinocchioCache& cache,
                                        const LocomotionModelSpec& spec, const YAML::Node& wbq,
                                        double control_period) {
  if (!wbq || !wbq.IsMap()) {
    throw std::invalid_argument("whole_body_qp: missing or not a map");
  }
  RefuseUnknownKeys(
      wbq, {"formulation_type", "solver", "contact", "contact_overrides", "tasks", "constraints"},
      "whole_body_qp");
  if (!(std::isfinite(control_period) && control_period > 0.0)) {
    throw std::invalid_argument("whole_body_qp: control_period must be finite and > 0");
  }
  // A re-Init replaces the formulation: drop every pointer into the old one first.
  feet_.clear();
  torso_ = nullptr;
  posture_ = nullptr;
  force_ = nullptr;
  com_ = nullptr;
  contact_rows_ = nullptr;
  if (spec.contacts.empty()) {
    throw std::invalid_argument("whole_body_qp: the model spec has no contact");
  }
  if (cache.registration_locked) {
    throw std::invalid_argument(
        "whole_body_qp: the PinocchioCache is already locked (Init the builder before its first "
        "Update)");
  }
  const std::vector<pinocchio::FrameIndex> contact_ids = ContactFrameIds(model, spec);
  if (cache.contact_frames.size() != contact_ids.size()) {
    throw std::invalid_argument("whole_body_qp: the cache's contact frames are not the spec's");
  }
  for (std::size_t i = 0; i < contact_ids.size(); ++i) {
    if (cache.contact_frames[i].frame_id != contact_ids[i]) {
      throw std::invalid_argument("whole_body_qp: the cache's contact frames are not the spec's");
    }
  }
  if (!model.existFrame(spec.torso_frame)) {
    throw std::invalid_argument("whole_body_qp: torso frame '" + spec.torso_frame +
                                "' is not in the model");
  }

  // ── Contacts: the spec's set with the YAML parameters ──
  const YAML::Node common = wbq["contact"];
  RefuseUnknownKeys(common, kContactKeys, "whole_body_qp.contact", "t_ramp");
  const YAML::Node overrides = wbq["contact_overrides"];
  if (overrides && !overrides.IsMap()) {
    throw std::invalid_argument("whole_body_qp.contact_overrides: must be a map");
  }
  std::set<std::string> names;
  for (const auto& c : spec.contacts) {
    if (c.dim != 3 && c.dim != 6) {
      throw std::invalid_argument("whole_body_qp: contact '" + c.name + "' dim must be 3 or 6");
    }
    if (!names.insert(c.name).second) {
      throw std::invalid_argument("whole_body_qp: contact name '" + c.name + "' used twice");
    }
  }
  if (overrides) {
    for (auto it = overrides.begin(); it != overrides.end(); ++it) {
      const std::string name = it->first.as<std::string>();
      if (names.count(name) == 0) {
        throw std::invalid_argument("whole_body_qp.contact_overrides: '" + name +
                                    "' is not a contact");
      }
      RefuseUnknownKeys(it->second, kContactKeys, "whole_body_qp.contact_overrides." + name);
    }
  }
  YAML::Node tsid(YAML::NodeType::Map);
  const std::string formulation = wbq["formulation_type"].as<std::string>("wqp");
  if (formulation != "wqp" && formulation != "hqp") {
    throw std::invalid_argument("whole_body_qp.formulation_type: wqp or hqp");
  }
  tsid["formulation_type"] = formulation;
  if (wbq["solver"]) {
    if (formulation == "wqp") {
      tsid["wqp"]["solver"] = YAML::Clone(wbq["solver"]);
    } else {
      tsid["hqp"]["solver_per_level"] = YAML::Clone(wbq["solver"]);
    }
  }
  t_ramp_ = common && common["t_ramp"] ? common["t_ramp"].as<double>() : 0.0;
  if (!(std::isfinite(t_ramp_) && t_ramp_ >= 0.0)) {
    throw std::invalid_argument("whole_body_qp.contact.t_ramp must be finite and >= 0");
  }
  for (const auto& c : spec.contacts) {
    YAML::Node e(YAML::NodeType::Map);
    e["name"] = c.name;
    e["frame"] = c.frame;
    e["type"] = c.dim == 6 ? "surface" : "point";
    for (const YAML::Node& src : {common, overrides ? overrides[c.name] : YAML::Node()}) {
      if (!src) {
        continue;
      }
      for (auto it = src.begin(); it != src.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        if (key != "t_ramp") {
          e[key] = YAML::Clone(it->second);
        }
      }
    }
    tsid["contacts"].push_back(e);
  }
  controller_.Init(model, robot_info, tsid);
  contacts_ = ContactManagerConfig{};
  contacts_.Load(tsid, model);

  // ── Tasks and constraints through the dispatch ──
  const YAML::Node tasks = wbq["tasks"];
  RefuseUnknownKeys(tasks, {"posture", "feet", "torso", "force", "com"}, "whole_body_qp.tasks");
  const YAML::Node constraints = wbq["constraints"];
  RefuseUnknownKeys(constraints, {"eom", "contact", "friction_cone", "joint_limit", "torque_limit"},
                    "whole_body_qp.constraints");
  if (!tasks || !tasks["posture"] || !tasks["feet"] || !tasks["torso"] || !tasks["force"]) {
    throw std::invalid_argument("whole_body_qp.tasks: posture, feet, torso and force are required");
  }
  YAML::Node task_entries(YAML::NodeType::Map);
  task_entries["posture"] = Entry(tasks["posture"], "posture", "whole_body_qp.tasks.posture");
  std::vector<std::string> foot_keys;
  for (const auto& c : spec.contacts) {
    const std::string key = "foot_" + c.name;
    YAML::Node e = Entry(tasks["feet"], "se3", "whole_body_qp.tasks.feet");
    e["name"] = key;
    e["frame"] = c.frame;
    e["base_frame"] = "universe";
    task_entries[key] = e;
    foot_keys.push_back(key);
  }
  {
    YAML::Node e = Entry(tasks["torso"], "se3", "whole_body_qp.tasks.torso");
    e["name"] = "torso";
    e["frame"] = spec.torso_frame;
    e["base_frame"] = "universe";
    task_entries["torso"] = e;
  }
  task_entries["force"] = Entry(tasks["force"], "force", "whole_body_qp.tasks.force");
  if (tasks["com"]) {
    task_entries["com"] = Entry(tasks["com"], "com", "whole_body_qp.tasks.com");
  }
  YAML::Node constraint_entries(YAML::NodeType::Map);
  if (constraints) {
    const std::pair<const char*, const char*> kTypes[] = {{"eom", "eom"},
                                                          {"contact", "contact_constraint"},
                                                          {"friction_cone", "friction_cone"},
                                                          {"joint_limit", "joint_limit"},
                                                          {"torque_limit", "torque_limit"}};
    for (const auto& [key, type] : kTypes) {
      if (constraints[key]) {
        constraint_entries[key] =
            Entry(constraints[key], type, std::string("whole_body_qp.constraints.") + key);
      }
    }
  }
  if (!constraint_entries["eom"] || !constraint_entries["contact"]) {
    // Without the dynamics rows a, λ and τ are not tied to the model.
    throw std::invalid_argument("whole_body_qp.constraints: eom and contact are required");
  }

  TaskFactoryContext ctx;
  ctx.model = &model;
  ctx.robot_info = &robot_info;
  ctx.cache = &cache;
  ctx.contacts = &contacts_;
  ctx.control_period = control_period;
  auto& f = controller_.Formulation();
  const BuildReport built_tasks = BuildTasks(task_entries, ctx, f);
  const BuildReport built_constraints = BuildConstraints(constraint_entries, ctx, f);
  for (const BuildReport* r : {&built_tasks, &built_constraints}) {
    for (const auto& m : r->messages) {
      throw std::invalid_argument("whole_body_qp: " + m.text);  // nothing here may be skipped
    }
  }

  // Every entry above was built (a message would have thrown): the casts are
  // to the types the entries were built as.
  for (const std::string& key : foot_keys) {
    feet_.push_back(static_cast<SE3Task*>(built_tasks.Find(key)->task));
  }
  torso_ = static_cast<SE3Task*>(built_tasks.Find("torso")->task);
  posture_ = static_cast<PostureTask*>(built_tasks.Find("posture")->task);
  force_ = static_cast<ForceTask*>(built_tasks.Find("force")->task);
  if (const BuiltEntry* com = built_tasks.Find("com")) {
    com_ = static_cast<CoMTask*>(com->task);
  }
  contact_rows_ = static_cast<ContactConstraint*>(built_constraints.Find("contact")->constraint);

  std::vector<pinocchio::FrameIndex> frames = contact_ids;
  frames.push_back(model.getFrameId(spec.torso_frame));
  kinematics_.Init(model, robot_info, frames, com_ != nullptr);
}

ContactState LocomotionFormulationBuilder::MakeContactState() const {
  ContactState s;
  s.Init(contacts_.max_contacts);
  s.SeedNormals(contacts_);
  for (auto& c : s.contacts) {
    c.active = true;
    c.activation = 1.0;
    c.activation_target = 1.0;
    c.t_ramp_sec = t_ramp_;
  }
  s.RecomputeActive(contacts_);
  return s;
}

bool LocomotionFormulationBuilder::UpdateReferences(const ControlReference& ref,
                                                    const ContactState& contacts) noexcept {
  // Each foot task works while its contact does not (WD-59).
  for (std::size_t i = 0; i < feet_.size() && i < contacts.contacts.size(); ++i) {
    feet_[i]->SetActive(!contacts.contacts[i].active);
  }
  // The force task's rows follow the contacts BEFORE the solve reads them (#1040).
  force_->UpdateResidualDim(contacts);
  if (!kinematics_.Update(ref.q_des, ref.v_des, ref.a_des)) {
    return false;
  }
  for (std::size_t i = 0; i < feet_.size(); ++i) {
    feet_[i]->SetSe3Reference(kinematics_.Placement(i), kinematics_.Twist(i),
                              kinematics_.Acceleration(i));
  }
  const std::size_t t = feet_.size();
  torso_->SetSe3Reference(kinematics_.Placement(t), kinematics_.Twist(t),
                          kinematics_.Acceleration(t));
  if (com_ != nullptr) {
    com_->SetComReference(kinematics_.Com(), kinematics_.ComVelocity(),
                          kinematics_.ComAcceleration());
  }
  return true;
}

const CommandOutput& LocomotionFormulationBuilder::Compute(const ControlState& state,
                                                           const ControlReference& ref,
                                                           const PinocchioCache& cache,
                                                           const ContactState& contacts) noexcept {
  if (!UpdateReferences(ref, contacts)) {
    // The frame tasks would track a stale (or never set) reference: no solve.
    return controller_.SkipTick();
  }
  return controller_.Compute(state, ref, cache, contacts);
}

}  // namespace rtc::tsid
