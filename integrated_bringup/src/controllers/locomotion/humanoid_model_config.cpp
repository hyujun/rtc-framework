// `locomotion_controller.model.*` parser and checks. See humanoid_model_config.hpp.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"

#include "rtc_controllers/params/yaml_read.hpp"
#include "rtc_mpc/model/wb_layout.hpp"           // kMaxContacts
#include "rtc_mpc/types/mpc_solution_types.hpp"  // kMaxContactVars
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>

namespace integrated_bringup::locomotion {

namespace {

using rtc::params::FindMissingKey;
using rtc::params::FindUnknownKey;
using rtc::params::ReadEnum;
using rtc::params::Spelling;

[[noreturn]] void Refuse(const std::string& where, const std::string& what) {
  throw std::invalid_argument(where + ": " + what);
}

// Unknown keys and missing keys are different mistakes (a misspelling shows as both):
// the unknown one is reported first and worded apart.
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

std::string ReadNameNode(const YAML::Node& v, const std::string& where) {
  if (!v.IsScalar() || v.Scalar().empty()) {
    Refuse(where, "expected a non-empty name, got " + Spelling(v));
  }
  return v.Scalar();
}

std::string ReadName(const YAML::Node& map, const char* key, const std::string& where) {
  return ReadNameNode(map[key], where + "." + key);
}

YAML::Node ReadList(const YAML::Node& map, const char* key, const std::string& where) {
  const YAML::Node v = map[key];
  if (!v.IsSequence()) {
    Refuse(where + "." + key, "expected a list, got " + Spelling(v));
  }
  return v;
}

std::string Item(const std::string& where, const char* key, std::size_t i) {
  return where + "." + key + "[" + std::to_string(i) + "]";
}

// One name space and one frame space for contacts and end-effectors: a frame that is
// both "the hand" and "a contact" is two roles bound to one point — refused here,
// added deliberately later as its own contact entry if a braced hand needs it.
void Claim(std::set<std::string>& used, const std::string& value, const std::string& where,
           const char* what) {
  if (!used.insert(value).second) {
    Refuse(where, std::string(what) + " '" + value + "' is used twice");
  }
}

}  // namespace

HumanoidModelConfig LoadHumanoidModelConfig(const YAML::Node& model, const std::string& context) {
  CheckKeys(model, context,
            {"body_group", "base_frame", "torso_frame", "end_effectors", "contacts", "hand_groups",
             "imu"});
  HumanoidModelConfig cfg;
  cfg.body_group = ReadName(model, "body_group", context);
  cfg.base_frame = ReadName(model, "base_frame", context);
  cfg.torso_frame = ReadName(model, "torso_frame", context);

  std::set<std::string> names;
  std::set<std::string> frames;

  const YAML::Node ees = ReadList(model, "end_effectors", context);
  if (ees.size() == 0 || ees.size() > kMaxEndEffectors) {
    Refuse(context + ".end_effectors", "expected 1 to " + std::to_string(kMaxEndEffectors) +
                                           " entries, got " + std::to_string(ees.size()));
  }
  for (std::size_t i = 0; i < ees.size(); ++i) {
    const std::string where = Item(context, "end_effectors", i);
    const YAML::Node e = ees[i];
    CheckKeys(e, where, {"name", "frame"});
    NamedFrame ee{ReadName(e, "name", where), ReadName(e, "frame", where)};
    Claim(names, ee.name, where + ".name", "name");
    Claim(frames, ee.frame, where + ".frame", "frame");
    cfg.end_effectors.push_back(std::move(ee));
  }

  const YAML::Node cs = ReadList(model, "contacts", context);
  const auto max_contacts = static_cast<std::size_t>(rtc::mpc::kMaxContacts);
  if (cs.size() == 0 || cs.size() > max_contacts) {
    Refuse(context + ".contacts", "expected 1 to " + std::to_string(max_contacts) +
                                      " entries, got " + std::to_string(cs.size()));
  }
  int contact_vars = 0;
  for (std::size_t i = 0; i < cs.size(); ++i) {
    const std::string where = Item(context, "contacts", i);
    const YAML::Node c = cs[i];
    CheckKeys(c, where, {"name", "frame", "dim"});
    ContactSpec contact{ReadName(c, "name", where), ReadName(c, "frame", where), 0};
    const YAML::Node dim = c["dim"];
    const auto dim_index = ReadEnum(dim, {"3", "6"});
    if (!dim_index) {
      Refuse(where + ".dim", "expected 3 (point) or 6 (surface), got " + Spelling(dim));
    }
    contact.dim = *dim_index == 0 ? 3 : 6;
    contact_vars += contact.dim;
    Claim(names, contact.name, where + ".name", "name");
    Claim(frames, contact.frame, where + ".frame", "frame");
    cfg.contacts.push_back(std::move(contact));
  }
  if (contact_vars > rtc::mpc::kMaxContactVars) {
    Refuse(context + ".contacts", "the contacts need " + std::to_string(contact_vars) +
                                      " force variables, more than the " +
                                      std::to_string(rtc::mpc::kMaxContactVars) + " rtc_mpc holds");
  }
  if (frames.count(cfg.torso_frame) != 0) {
    Refuse(context + ".torso_frame",
           "frame '" + cfg.torso_frame + "' is also a contact or end-effector frame");
  }

  const YAML::Node hands = ReadList(model, "hand_groups", context);
  std::set<std::string> groups{cfg.body_group};
  for (std::size_t i = 0; i < hands.size(); ++i) {
    const std::string where = Item(context, "hand_groups", i);
    std::string group = ReadNameNode(hands[i], where);
    Claim(groups, group, where, "device group");
    cfg.hand_groups.push_back(std::move(group));
  }

  const std::string imu_where = context + ".imu";
  const YAML::Node imu = model["imu"];
  CheckKeys(imu, imu_where, {"sites", "primary_site"});
  const YAML::Node sites = ReadList(imu, "sites", imu_where);
  const auto max_sites = static_cast<std::size_t>(rtc::kMaxImuSites);
  if (sites.size() == 0 || sites.size() > max_sites) {
    Refuse(imu_where + ".sites", "expected 1 to " + std::to_string(max_sites) + " entries, got " +
                                     std::to_string(sites.size()));
  }
  std::set<std::string> labels;
  for (std::size_t i = 0; i < sites.size(); ++i) {
    const std::string where = Item(imu_where, "sites", i);
    std::string label = ReadNameNode(sites[i], where);
    Claim(labels, label, where, "IMU site");
    cfg.imu.sites.push_back(std::move(label));
  }
  cfg.imu.primary_site = ReadName(imu, "primary_site", imu_where);
  if (labels.count(cfg.imu.primary_site) == 0) {
    Refuse(imu_where + ".primary_site",
           "'" + cfg.imu.primary_site + "' is not one of model.imu.sites");
  }
  return cfg;
}

void ValidateAgainstModel(const HumanoidModelConfig& cfg, const pinocchio::Model& model,
                          const std::string& context) {
  const auto need_frame = [&](const std::string& frame, const std::string& where) {
    if (!model.existFrame(frame)) {
      Refuse(where, "frame '" + frame + "' is not in the model");
    }
  };
  need_frame(cfg.base_frame, context + ".base_frame");
  need_frame(cfg.torso_frame, context + ".torso_frame");
  for (std::size_t i = 0; i < cfg.end_effectors.size(); ++i) {
    need_frame(cfg.end_effectors[i].frame, Item(context, "end_effectors", i) + ".frame");
  }
  for (std::size_t i = 0; i < cfg.contacts.size(); ++i) {
    const std::string where = Item(context, "contacts", i) + ".frame";
    need_frame(cfg.contacts[i].frame, where);
    if (cfg.contacts[i].frame == cfg.base_frame) {
      Refuse(where, "a contact cannot be the base frame '" + cfg.base_frame + "'");
    }
  }
  if (rtc_urdf_bridge::IsFreeFlyerRoot(model) && cfg.base_frame != "universe") {
    Refuse(context + ".base_frame",
           "the model has a floating base, so the base frame must be "
           "'universe' (got '" +
               cfg.base_frame + "')");
  }
}

std::vector<std::string> JointOrder(const pinocchio::Model& model) {
  std::vector<std::string> order;
  const auto first = static_cast<std::size_t>(rtc_urdf_bridge::IsFreeFlyerRoot(model) ? 2 : 1);
  for (std::size_t j = first; j < model.names.size(); ++j) {
    order.push_back(model.names[j]);
  }
  return order;
}

std::optional<JointOrderMismatch> CompareJointOrder(const std::vector<std::string>& expected,
                                                    const std::vector<std::string>& actual) {
  const std::size_t n = std::max(expected.size(), actual.size());
  for (std::size_t i = 0; i < n; ++i) {
    const std::string e = i < expected.size() ? expected[i] : std::string("<end>");
    const std::string a = i < actual.size() ? actual[i] : std::string("<end>");
    if (e != a) {
      return JointOrderMismatch{i, e, a};
    }
  }
  return std::nullopt;
}

std::string Describe(const JointOrderMismatch& mismatch) {
  return "index " + std::to_string(mismatch.index) + ": expected '" + mismatch.expected +
         "', actual '" + mismatch.actual + "'";
}

void ValidateDevices(const HumanoidModelConfig& cfg,
                     const std::vector<std::string>& body_joint_order, const DeviceLookup& lookup,
                     const std::string& context) {
  // Every group the config names must be a device with a roster: a missing roster is
  // refused, never taken as the model order (AP-CTRL-5).
  const auto roster = [&](const std::string& group,
                          const std::string& where) -> const std::vector<std::string>& {
    const rtc::DeviceNameConfig* device = lookup ? lookup(group) : nullptr;
    if (device == nullptr) {
      Refuse(where, "no device group '" + group + "'");
    }
    if (device->joint_state_names.empty()) {
      Refuse(where, "device group '" + group + "' has no joint_state_names");
    }
    return device->joint_state_names;
  };
  const auto& body = roster(cfg.body_group, context + ".body_group");
  if (const auto mismatch = CompareJointOrder(body_joint_order, body)) {
    Refuse(context + ".body_group", "joint_state_names of '" + cfg.body_group +
                                        "' is not the model's joint order (" + Describe(*mismatch) +
                                        ")");
  }
  for (std::size_t i = 0; i < cfg.hand_groups.size(); ++i) {
    (void)roster(cfg.hand_groups[i], Item(context, "hand_groups", i));
  }
}

YAML::Node ToRobotModelHandlerYaml(const HumanoidModelConfig& cfg) {
  if (cfg.end_effectors.empty()) {
    throw std::invalid_argument(
        "ToRobotModelHandlerYaml: no end-effector (the handler requires one) — "
        "build the config with LoadHumanoidModelConfig");
  }
  YAML::Node out;
  YAML::Node end_effectors(YAML::NodeType::Sequence);
  for (const auto& ee : cfg.end_effectors) {
    end_effectors.push_back(ee.frame);
  }
  out["end_effector_frames"] = end_effectors;
  out["base_frame"] = cfg.base_frame;
  YAML::Node contacts(YAML::NodeType::Sequence);
  for (const auto& c : cfg.contacts) {
    YAML::Node entry;
    entry["name"] = c.frame;  // the handler's `name` is the frame name
    entry["dim"] = c.dim;
    contacts.push_back(entry);
  }
  out["contact_frames"] = contacts;
  return out;
}

}  // namespace integrated_bringup::locomotion
