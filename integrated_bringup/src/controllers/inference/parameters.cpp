// ── DemoInferenceController: YAML schema ─────────────────────────────────────
// LoadConfig is Pass 1 of the 3-pass bring-up: `topic_config_` exists but the
// device rosters do not yet, so anything that has to be checked against a
// device's joint count or limits waits for OnDeviceConfigsSet / on_configure.
// What lands here is everything that is self-contained in the YAML.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {

/// Read a required sequence of doubles. Empty and absent are distinguished:
/// an absent posture is a missing key (configure fails), while an empty one is
/// a key the operator emptied — both are refused, but the message differs so
/// the fix is obvious.
std::vector<double> ParsePosture(const YAML::Node& node, const char* where) {
  if (!node) {
    throw std::invalid_argument(std::string("demo_inference_controller: ") + where +
                                " is required");
  }
  if (!node.IsSequence() || node.size() == 0) {
    throw std::invalid_argument(std::string("demo_inference_controller: ") + where +
                                " must be a non-empty sequence of joint positions [rad]");
  }
  std::vector<double> out;
  out.reserve(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    out.push_back(node[i].as<double>());
  }
  return out;
}

/// Refuse the frame keys this binding stopped reading, naming the migration.
///
/// Without this an old config would fail as "palm.position has unknown id" —
/// true and useless — or, worse, configure: `base_pose_in_world` would simply be
/// ignored while the policy went on receiving poses in whatever frame the new
/// keys default to. The rotation it carried is now read from the URDF.
void RejectLegacyFrameKeys(const YAML::Node& inf) {
  if (inf["palm"]) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.palm is gone. Observe the palm as a link feature "
        "(`link.<frame>.position` / `link.<frame>.orientation_xyzw`) and name the frame every "
        "pose is expressed in with `inference.policy_frame: <urdf frame>`");
  }
  if (inf["base_pose_in_world"]) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.base_pose_in_world is gone. Poses are expressed in "
        "`inference.policy_frame` (a URDF frame), and the object lane names the URDF frame its "
        "messages are quoted in with `object_pose.source_frame_link` — the transform between the "
        "two is read from the model instead of typed in");
  }
  if (const YAML::Node obj = inf["object_pose"]; obj && obj.IsMap() && obj["reference_frame"]) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.object_pose.reference_frame is gone. The object "
        "pose is delivered in `inference.policy_frame`; declare which URDF frame the incoming "
        "messages are in with `object_pose.source_frame_link`");
  }
}

/// A required, non-empty string key.
std::string RequireString(const YAML::Node& node, const std::string& where, const char* why) {
  if (!node || !node.IsScalar()) {
    throw std::invalid_argument("demo_inference_controller: " + where + " is required (" + why +
                                ")");
  }
  auto s = node.as<std::string>("");
  if (s.empty()) {
    throw std::invalid_argument("demo_inference_controller: " + where + " is empty");
  }
  return s;
}

}  // namespace

void DemoInferenceController::LoadConfig(const YAML::Node& cfg) {
  // Base pass first: it owns `topics:` and `command_type`.
  RTControllerInterface::LoadConfig(cfg);

  if (!cfg || !cfg.IsMap()) {
    throw std::invalid_argument("demo_inference_controller: config node is missing or not a map");
  }

  if (const auto ct = cfg["command_type"]) {
    const auto s = ct.as<std::string>("position");
    if (s != "position") {
      // The policy emits joint POSITIONS. Accepting "torque" here would parse
      // fine and then reinterpret radians as newton-metres at the actuator
      // boundary, which is exactly the class of failure this controller's
      // validation exists to prevent.
      throw std::invalid_argument(
          "demo_inference_controller: command_type must be \"position\" (got \"" + s + "\")");
    }
  }
  command_type_ = CommandType::kPosition;

  joint_limit_margin_ = cfg["joint_limit_margin"].as<double>(0.0);
  if (!(joint_limit_margin_ >= 0.0)) {
    throw std::invalid_argument(
        "demo_inference_controller: joint_limit_margin must be >= 0 (a NaN also lands here)");
  }

  const YAML::Node inf = cfg["inference"];
  if (!inf || !inf.IsMap()) {
    throw std::invalid_argument("demo_inference_controller: an `inference:` map is required");
  }
  RejectLegacyFrameKeys(inf);

  // Kept as written. `${VAR}` / `~` are expanded in on_configure, where an unset
  // variable can still become a hold mode instead of a thrown LoadConfig.
  model_path_raw_ = inf["model_path"].as<std::string>("");
  optimized_model_path_ = inf["optimized_model_path"].as<std::string>("");
  intra_op_threads_ = inf["intra_op_threads"].as<int>(1);
  if (intra_op_threads_ < 1) {
    throw std::invalid_argument("demo_inference_controller: intra_op_threads must be >= 1");
  }
  allow_missing_model_ = inf["allow_missing_model"].as<bool>(false);

  // #511 D-3. Read here (Pass 1) because it is self-contained; whether it does
  // anything depends on the schema declaring a recurrent link, and an inert key
  // on a feed-forward policy is harmless. A NaN is refused rather than silently
  // read as "never reset", which is what every comparison against it would mean.
  reset_after_hold_sec_ = inf["reset_after_hold_sec"].as<double>(0.1);
  if (std::isnan(reset_after_hold_sec_)) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.reset_after_hold_sec must be a number (0 resets "
        "after any hold, negative never resets outside activation)");
  }

  // `hand_posture`, `joint_convention`, `reach_gate` and the frame keys are NOT
  // read here: each is checked against the device rosters or the feature list,
  // and those only exist in Pass 3 (#511 D-9).

  // ── `logs:` — the sibling controllers' schema, a closed set of types ──────
  // An unknown type is refused rather than skipped: a log the operator asked
  // for and silently did not get is found out only when the session directory
  // is opened, after the run it was meant to record.
  parsed_log_entries_.clear();
  if (const YAML::Node logs = cfg["logs"]) {
    if (!logs.IsSequence()) {
      throw std::invalid_argument("demo_inference_controller: `logs` must be a sequence");
    }
    for (const auto& entry : logs) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::invalid_argument(
            "demo_inference_controller: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      e.instance = entry["instance"].as<std::string>("");
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != kInferenceDiagLogMsgType) {
        throw std::invalid_argument(
            "demo_inference_controller: unknown msg_type in `logs`: " + e.msg_type +
            " (accepted: rtc_msgs/DeviceStateLog, " + std::string(kInferenceDiagLogMsgType) + ")");
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

bool DemoInferenceController::ExpandModelPath(const std::string& raw, std::string& out,
                                              std::string& missing) {
  out.clear();
  missing.clear();
  std::size_t i = 0;
  // A leading `~` is the user's home, and only at the start: a `~` anywhere
  // else is a character of a file name.
  if (!raw.empty() && raw[0] == '~' && (raw.size() == 1 || raw[1] == '/')) {
    const char* home = std::getenv("HOME");
    if (home == nullptr || *home == '\0') {
      missing = "HOME";
      return false;
    }
    out = home;
    i = 1;
  }
  while (i < raw.size()) {
    if (raw[i] == '$' && i + 1 < raw.size() && raw[i + 1] == '{') {
      const auto close = raw.find('}', i + 2);
      if (close == std::string::npos) {
        throw std::invalid_argument("demo_inference_controller: inference.model_path '" + raw +
                                    "' has an unterminated `${`");
      }
      const std::string name = raw.substr(i + 2, close - (i + 2));
      const bool valid_name =
          !name.empty() &&
          (std::isalpha(static_cast<unsigned char>(name[0])) != 0 || name[0] == '_') &&
          std::all_of(name.begin(), name.end(), [](char c) {
            return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
          });
      if (!valid_name) {
        throw std::invalid_argument("demo_inference_controller: inference.model_path '" + raw +
                                    "' references `${" + name +
                                    "}`, which is not a valid environment variable name");
      }
      const char* value = std::getenv(name.c_str());
      if (value == nullptr || *value == '\0') {
        missing = name;
        return false;
      }
      out += value;
      i = close + 1;
      continue;
    }
    out += raw[i];
    ++i;
  }
  return true;
}

// ── Pass 3: the half of the schema that needs the device rosters ────────────
//
// Split out of LoadConfig rather than deferred inside it, because the split is
// the bring-up contract and not an implementation detail: feature WIDTHS are
// device facts ("arm.position" is as wide as the primary group's joint roster),
// and those facts do not exist until the CM has injected device configs.
// Running this from on_configure is what makes a width disagreement between the
// policy and the robot a configure failure that names both numbers.
void DemoInferenceController::ApplyIoSchema(const YAML::Node& cfg) {
  const YAML::Node inf = cfg["inference"];
  if (!inf || !inf.IsMap()) {
    throw std::invalid_argument("demo_inference_controller: an `inference:` map is required");
  }

  // The rosters must fit the fixed capacities the tick indexes. OnDeviceConfigsSet
  // clamps the DOF it keeps, and a feature as wide as the UNclamped roster would
  // then be packed with a tail the tick never wrote.
  if (const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
      arm_cfg != nullptr &&
      arm_cfg->joint_state_names.size() > static_cast<std::size_t>(kMaxArmDof)) {
    throw std::invalid_argument("demo_inference_controller: the primary device declares " +
                                std::to_string(arm_cfg->joint_state_names.size()) +
                                " joints, over the fixed capacity of " +
                                std::to_string(kMaxArmDof));
  }
  if (const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName()); hand_cfg != nullptr) {
    if (hand_cfg->joint_state_names.size() > static_cast<std::size_t>(kMaxHandDof)) {
      throw std::invalid_argument("demo_inference_controller: the secondary device declares " +
                                  std::to_string(hand_cfg->joint_state_names.size()) +
                                  " joints, over the fixed capacity of " +
                                  std::to_string(kMaxHandDof));
    }
    if (hand_cfg->sensor_names.size() > static_cast<std::size_t>(kMaxFingertips)) {
      throw std::invalid_argument("demo_inference_controller: the secondary device declares " +
                                  std::to_string(hand_cfg->sensor_names.size()) +
                                  " sensor groups, over the fixed capacity of " +
                                  std::to_string(kMaxFingertips));
    }
  }

  io_ = rtc::params::ParsePolicyIoParams(
      inf,
      [this](std::string_view id) {
        ResolvedFeature r;
        return ResolveFeature(id, r) ? r.width : 0;
      },
      [this](std::string_view id) {
        ResolvedFeature r;
        return ResolveFeature(id, r) ? r.rows : std::vector<std::string>{};
      });

  if (io_.inputs.size() > static_cast<std::size_t>(kMaxInputTensors)) {
    throw std::invalid_argument(
        "demo_inference_controller: the schema declares " + std::to_string(io_.inputs.size()) +
        " input tensors, over the fixed capacity of " + std::to_string(kMaxInputTensors));
  }

  // ── Feature table, FLATTENED across tensors ───────────────────────────────
  // Each feature keeps its segment, and the segment carries its own tensor
  // index, so the tick walks one flat list instead of nesting two loops. Links
  // are interned by name so a link observed twice (a position row and an
  // orientation row) is computed once per step.
  feature_kinds_.clear();
  feature_args_.clear();
  flat_segments_.clear();
  links_.clear();
  bool wants_links = false;
  bool wants_reach = false;
  wants_object_ = false;
  for (const auto& tensor : io_.inputs) {
    for (std::size_t f = 0; f < tensor.features.size(); ++f) {
      const auto& id = tensor.features[f];
      ResolvedFeature r;
      if (!ResolveFeature(id, r)) {
        // Unreachable through ParsePolicyIoParams (its size resolver is this
        // same function) — kept so a future resolver split cannot drift.
        throw std::invalid_argument("demo_inference_controller: feature '" + id +
                                    "' has a size but no extractor (binding bug)");
      }
      int arg = -1;
      switch (r.kind) {
        case PolicyFeature::kGroupForceNorm:
          arg = r.group;
          break;
        case PolicyFeature::kLinkPosition:
        case PolicyFeature::kLinkOrientationXyzw:
          arg = InternLink(r.link);
          wants_links = true;
          break;
        case PolicyFeature::kReachPhase:
          wants_reach = true;
          break;
        case PolicyFeature::kObjectPosition:
        case PolicyFeature::kObjectOrientationXyzw:
          wants_object_ = true;
          break;
        default:
          break;
      }
      feature_kinds_.push_back(r.kind);
      feature_args_.push_back(arg);
      flat_segments_.push_back(tensor.segments[f]);
    }
  }

  // ── Recurrent seeds ───────────────────────────────────────────────────────
  // Restricted to device lanes and the object pose: a seed is extracted BEFORE
  // the observation is packed, and a link pose or the reach gate only exists
  // once that packing has computed it.
  seeds_.clear();
  for (const auto& link : io_.recurrent_links) {
    const auto& spec = io_.inputs[static_cast<std::size_t>(link.input_tensor)];
    RecurrentSeed seed;
    seed.tensor = link.input_tensor;
    if (!spec.seed_feature.empty()) {
      ResolvedFeature r;
      if (!ResolveFeature(spec.seed_feature, r)) {
        throw std::invalid_argument("demo_inference_controller: recurrent input '" + spec.name +
                                    "' is seeded from unknown id '" + spec.seed_feature + "'");
      }
      if (r.kind == PolicyFeature::kLinkPosition || r.kind == PolicyFeature::kLinkOrientationXyzw ||
          r.kind == PolicyFeature::kReachPhase) {
        throw std::invalid_argument(
            "demo_inference_controller: recurrent input '" + spec.name + "' is seeded from '" +
            spec.seed_feature +
            "', which is computed while the observation is packed — after the seed is taken. "
            "Seed from a joint lane, a force lane or the object pose");
      }
      seed.has_seed = true;
      seed.kind = r.kind;
      seed.arg = (r.kind == PolicyFeature::kGroupForceNorm) ? r.group : -1;
      seed.width = r.width;
    }
    seeds_.push_back(seed);
  }

  // ── Joint convention ──────────────────────────────────────────────────────
  // `q_policy = sign · q_device + offset`. A training asset can define a joint's
  // axis opposite to this robot's URDF, which is invisible to every range check:
  // the flipped value is still inside the limits, and the hand simply closes by
  // opening. Each key must be a joint of one of the two devices — a typo here
  // would otherwise leave that joint at identity and look exactly like a joint
  // that needs none.
  arm_sign_.fill(1.0);
  arm_offset_.fill(0.0);
  hand_sign_.fill(1.0);
  hand_offset_.fill(0.0);
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();
  if (const YAML::Node conv = inf["joint_convention"]) {
    if (!conv.IsMap()) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.joint_convention must be a map of "
          "<joint>: {sign: ±1, offset: <rad>}");
    }
    const auto* arm_cfg = GetDeviceNameConfig(primary);
    const auto* hand_cfg = secondary.empty() ? nullptr : GetDeviceNameConfig(secondary);
    for (auto it = conv.begin(); it != conv.end(); ++it) {
      const auto joint = it->first.as<std::string>("");
      const std::string at = "inference.joint_convention." + joint;
      const YAML::Node entry = it->second;
      if (!entry.IsMap()) {
        throw std::invalid_argument("demo_inference_controller: " + at +
                                    " must be a map {sign: ±1, offset: <rad>}");
      }
      for (auto k = entry.begin(); k != entry.end(); ++k) {
        const auto key = k->first.as<std::string>("");
        if (key != "sign" && key != "offset") {
          throw std::invalid_argument("demo_inference_controller: " + at + " has unknown key '" +
                                      key + "' (this binding knows `sign` and `offset`)");
        }
      }
      const double sign = entry["sign"] ? entry["sign"].as<double>(0.0) : 1.0;
      if (sign != 1.0 && sign != -1.0) {
        // A scale is not a convention: it would change the unit of the lane,
        // and nothing downstream could tell.
        throw std::invalid_argument("demo_inference_controller: " + at +
                                    ".sign must be exactly 1 or -1");
      }
      const double offset =
          entry["offset"] ? entry["offset"].as<double>(std::numeric_limits<double>::quiet_NaN())
                          : 0.0;
      if (!std::isfinite(offset)) {
        throw std::invalid_argument("demo_inference_controller: " + at +
                                    ".offset must be a finite number [rad]");
      }
      bool found = false;
      const auto place = [&](const rtc::DeviceNameConfig* dev, auto& signs, auto& offsets,
                             int dof) {
        if (dev == nullptr) {
          return;
        }
        const auto& names = dev->joint_state_names;
        const auto pos = std::find(names.begin(), names.end(), joint);
        if (pos == names.end()) {
          return;
        }
        const auto idx = static_cast<std::size_t>(pos - names.begin());
        if (idx < static_cast<std::size_t>(dof)) {
          signs[idx] = sign;
          offsets[idx] = offset;
          found = true;
        }
      };
      place(arm_cfg, arm_sign_, arm_offset_, arm_dof_);
      place(hand_cfg, hand_sign_, hand_offset_, hand_dof_);
      if (!found) {
        throw std::invalid_argument("demo_inference_controller: " + at + " names joint '" + joint +
                                    "', which neither device ('" + primary + "', '" + secondary +
                                    "') declares");
      }
    }
  }

  // ── Reach gate ────────────────────────────────────────────────────────────
  // Both directions are refused: `reach.phase` without a gate has no formula,
  // and a gate that no feature reads is a block of trained constants that looks
  // configured and does nothing.
  reach_enabled_ = false;
  reach_tips_ = 0;
  reach_state_ = {};
  reach_state_pending_ = {};
  reach_pending_ = false;
  const YAML::Node gate = inf["reach_gate"];
  if (wants_reach && !gate) {
    throw std::invalid_argument(
        "demo_inference_controller: the feature `reach.phase` is declared, so "
        "inference.reach_gate is required (tips, force groups, contact points)");
  }
  if (gate && !wants_reach) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.reach_gate is declared but no input feature reads "
        "`reach.phase` — the gate would be computed for nothing");
  }
  if (wants_reach) {
    reach_ = rtc::params::ParseReachGateParams(gate);
    if (reach_.tips.size() > static_cast<std::size_t>(kMaxReachTips)) {
      throw std::invalid_argument(
          "demo_inference_controller: reach_gate declares " + std::to_string(reach_.tips.size()) +
          " tips, over the fixed capacity of " + std::to_string(kMaxReachTips));
    }
    const auto* hand_cfg = secondary.empty() ? nullptr : GetDeviceNameConfig(secondary);
    if (hand_cfg == nullptr) {
      throw std::invalid_argument(
          "demo_inference_controller: the reach gate reads fingertip forces from the hand's "
          "sensor groups, and there is no hand device config");
    }
    for (std::size_t i = 0; i < reach_.tips.size(); ++i) {
      const auto& tip = reach_.tips[i];
      const auto& groups = hand_cfg->sensor_names;
      const auto g = std::find(groups.begin(), groups.end(), tip.force_group);
      if (g == groups.end()) {
        throw std::invalid_argument("demo_inference_controller: reach_gate.tips[" +
                                    std::to_string(i) + "] reads force group '" + tip.force_group +
                                    "', which device '" + secondary +
                                    "' does not list in sensor_names");
      }
      reach_force_group_[i] = static_cast<int>(g - groups.begin());
      reach_link_slot_[i] = InternLink(tip.link);
      for (std::size_t c = 0; c < 3; ++c) {
        reach_contacts_[(3 * i) + c] = tip.contact_obj[c];
      }
    }
    reach_tips_ = static_cast<int>(reach_.tips.size());
    reach_enabled_ = true;
    // The gate measures tips against the object, so it needs the object lane
    // even when no input observes the object directly.
    wants_object_ = true;
    wants_links = true;
  }

  // ── Frames ────────────────────────────────────────────────────────────────
  // Required only when a pose is observed. Demanding it from a policy that
  // observes joints and forces alone would be a configure failure with nothing
  // wrong.
  policy_frame_.clear();
  if (wants_links || wants_object_) {
    policy_frame_ = RequireString(inf["policy_frame"], "inference.policy_frame",
                                  "a pose is observed, and every pose is expressed in this URDF "
                                  "frame — the one the policy was trained in");
  }

  if (wants_object_) {
    const YAML::Node obj = inf["object_pose"];
    if (!obj || !obj.IsMap()) {
      throw std::invalid_argument(
          "demo_inference_controller: the object pose is observed (an object feature or the "
          "reach gate), so inference.object_pose is required (topic + frame match)");
    }
    object_topic_ =
        RequireString(obj["topic"], "inference.object_pose.topic", "the TFMessage topic");
    object_frame_match_ = obj["frame_match"].as<std::string>("");
    if (object_frame_match_.empty()) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.frame_match is empty — an empty "
          "match would accept every body in the message");
    }
    const auto mode = obj["match_mode"].as<std::string>("prefix");
    if (mode == "prefix") {
      object_match_prefix_ = true;
    } else if (mode == "exact") {
      object_match_prefix_ = false;
    } else {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.match_mode must be \"prefix\" or "
          "\"exact\" (got \"" +
          mode + "\")");
    }
    object_source_frame_id_ = obj["source_frame_id"].as<std::string>("world");
    if (object_source_frame_id_.empty()) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.source_frame_id is empty — it is what "
          "turns \"the incoming poses are in world\" from an assumption into a check");
    }
    object_source_frame_link_ =
        RequireString(obj["source_frame_link"], "inference.object_pose.source_frame_link",
                      "the URDF frame the incoming messages are quoted in, so the model can supply "
                      "the transform into policy_frame");
    object_timeout_sec_ = obj["timeout_sec"].as<double>(0.2);
    if (!(object_timeout_sec_ > 0.0)) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.timeout_sec must be > 0. It is "
          "configuration and not a constant because the sim republishes every tick (~500 Hz) "
          "while a real perception stack runs at 10-30 Hz");
    }
  }

  // ── Fixed-capacity check ──────────────────────────────────────────────────
  // Per tensor, because the capacities bound the buffers the tick indexes and
  // each tensor is its own buffer. The sum across tensors is deliberately NOT
  // the quantity checked: two 400-element inputs are two 400-element buffers.
  for (const auto& tensor : io_.inputs) {
    if (tensor.Numel() > static_cast<std::size_t>(kMaxInputElements)) {
      throw std::invalid_argument("demo_inference_controller: input tensor '" + tensor.name +
                                  "' has " + std::to_string(tensor.Numel()) +
                                  " elements, over the fixed capacity of " +
                                  std::to_string(kMaxInputElements));
    }
  }
  for (const auto& tensor : io_.outputs) {
    if (tensor.Numel() > static_cast<std::size_t>(kMaxOutputElements)) {
      throw std::invalid_argument("demo_inference_controller: output tensor '" + tensor.name +
                                  "' has " + std::to_string(tensor.Numel()) +
                                  " elements, over the fixed capacity of " +
                                  std::to_string(kMaxOutputElements));
    }
  }

  // ── Output roles ──────────────────────────────────────────────────────────
  // DECLARED, not inferred from the entry's spelling. The suffix loop this
  // replaced had no `break`, so two devices declaring the same role left the
  // last one driving both — see PolicyOutputRole's declaration for why that
  // failure is invisible downstream (#511 B-2, D-4).
  //
  // The device half is matched against the group rosters the same way input
  // features are, so `banana.joint_target` is a configure failure naming both
  // groups instead of a command quietly bound to the arm.
  arm_target_idx_ = -1;
  hand_command_idx_ = -1;
  arm_out_idx_.clear();
  hand_out_idx_.clear();
  for (std::size_t i = 0; i < io_.output_features.size(); ++i) {
    const auto& spec = io_.output_features[i];
    const std::string at =
        "output_features[" + std::to_string(i) + "] (" + spec.device + "/" + spec.role + "): ";

    const bool is_primary = !primary.empty() && spec.device == primary;
    const bool is_secondary = !secondary.empty() && spec.device == secondary;
    if (!is_primary && !is_secondary) {
      // D-5 middle ground: the YAML is already shaped for N groups, but this
      // binding drives two. A third group is refused rather than ignored.
      throw std::invalid_argument(at + "device '" + spec.device +
                                  "' is not one of this controller's device groups ('" + primary +
                                  "', '" + secondary + "')");
    }

    PolicyOutputRole role{};
    if (spec.role == "joint_target") {
      role = PolicyOutputRole::kJointTarget;
    } else if (spec.role == "posture_scalar") {
      role = PolicyOutputRole::kPostureScalar;
    } else {
      throw std::invalid_argument(at + "unknown role '" + spec.role +
                                  "' (this binding knows \"joint_target\" and "
                                  "\"posture_scalar\")");
    }

    // A head that names its elements is gathered BY NAME into the device's
    // joint order (A3). The width check below still applies, so a named head
    // must name exactly the device's joints — each once (the parser refuses a
    // repeated name) and nothing else.
    const auto& tensor = io_.outputs[static_cast<std::size_t>(spec.slice.tensor)];
    std::vector<int> gather;
    if (!tensor.element_names.empty() && role == PolicyOutputRole::kJointTarget) {
      const auto* dev = GetDeviceNameConfig(spec.device);
      if (dev == nullptr) {
        throw std::invalid_argument(at + "device '" + spec.device + "' has no joint roster");
      }
      gather = rtc::params::ResolveNamedIndices(tensor.element_names, dev->joint_state_names,
                                                "output tensor '" + tensor.name + "'");
    }

    const int count = spec.slice.count;
    if (is_primary) {
      if (role != PolicyOutputRole::kJointTarget) {
        throw std::invalid_argument(at +
                                    "the posture scalar interpolates the HAND postures; the "
                                    "primary group can only take \"joint_target\"");
      }
      if (count != arm_dof_) {
        throw std::invalid_argument(at + "slices " + std::to_string(count) +
                                    " elements but device '" + spec.device + "' declares " +
                                    std::to_string(arm_dof_) + " joints");
      }
      arm_target_idx_ = static_cast<int>(i);
      arm_out_idx_ = std::move(gather);
      continue;
    }

    if (hand_command_idx_ >= 0) {
      // The schema already refuses the SAME role twice on one device; this is
      // the other shape of the same mistake — two different roles aiming at one
      // device, i.e. two commands for the same joints with no defensible order.
      throw std::invalid_argument(at + "device '" + spec.device +
                                  "' is already driven by output_features[" +
                                  std::to_string(hand_command_idx_) + "]");
    }
    const int want = (role == PolicyOutputRole::kJointTarget) ? hand_dof_ : 1;
    if (count != want) {
      throw std::invalid_argument(at + "slices " + std::to_string(count) + " elements but " +
                                  spec.role + " on '" + spec.device + "' needs exactly " +
                                  std::to_string(want));
    }
    hand_command_idx_ = static_cast<int>(i);
    hand_role_ = role;
    hand_out_idx_ = std::move(gather);
  }
  if (arm_target_idx_ < 0) {
    throw std::invalid_argument(
        "demo_inference_controller: output_features must declare { device: \"" + primary +
        "\", role: \"joint_target\" } — without it the arm receives nothing and the "
        "controller can only hold");
  }
  if (hand_command_idx_ < 0) {
    throw std::invalid_argument(
        "demo_inference_controller: output_features must declare a role for device '" + secondary +
        "' (\"joint_target\" for direct joint commands, \"posture_scalar\" to interpolate "
        "hand_posture)");
  }

  // ── Hand postures (Pass 3, and only when a scalar asks for them) ──────────
  // D-9: requiring the block unconditionally made a policy that commands the
  // hand joints directly fail configure over two lists it would never read.
  if (hand_role_ == PolicyOutputRole::kPostureScalar) {
    const YAML::Node posture = inf["hand_posture"];
    if (!posture || !posture.IsMap()) {
      throw std::invalid_argument(
          "demo_inference_controller: a posture_scalar role is declared, so "
          "inference.hand_posture must define `open` and `close`");
    }
    posture_open_ = ParsePosture(posture["open"], "inference.hand_posture.open");
    posture_close_ = ParsePosture(posture["close"], "inference.hand_posture.close");
    if (posture_open_.size() != posture_close_.size()) {
      throw std::invalid_argument("demo_inference_controller: hand_posture.open has " +
                                  std::to_string(posture_open_.size()) +
                                  " joints but hand_posture.close has " +
                                  std::to_string(posture_close_.size()));
    }
    if (posture_open_.size() > static_cast<std::size_t>(kMaxHandDof)) {
      throw std::invalid_argument("demo_inference_controller: hand_posture exceeds kMaxHandDof");
    }
  } else {
    posture_open_.clear();
    posture_close_.clear();
  }
}

}  // namespace integrated_bringup
