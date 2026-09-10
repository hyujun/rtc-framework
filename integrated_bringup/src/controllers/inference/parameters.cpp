// ── DemoInferenceController: YAML schema ─────────────────────────────────────
// LoadConfig is Pass 1 of the 3-pass bring-up: `topic_config_` exists but the
// device rosters do not yet, so anything that has to be checked against a
// device's joint count or limits waits for OnDeviceConfigsSet / on_configure.
// What lands here is everything that is self-contained in the YAML.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_math/se3/so3.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <stdexcept>
#include <string>
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

/// "base" | "world", defaulting to world (the frame the policy's object pose
/// is quoted in). Anything else is a typo and is refused rather than silently
/// falling back — the two frames differ by 180 deg about z on this robot, and
/// the wrong one produces poses that look entirely reasonable.
PoseFrame ParsePoseFrame(const YAML::Node& node, const char* where) {
  const auto s = node ? node.as<std::string>("world") : std::string("world");
  if (s == "world") {
    return PoseFrame::kWorld;
  }
  if (s == "base") {
    return PoseFrame::kBase;
  }
  throw std::invalid_argument(std::string("demo_inference_controller: ") + where +
                              " must be \"world\" or \"base\" (got \"" + s + "\")");
}

/// The static `world → base` transform. Identity when the key is absent, which
/// is the right default for a robot mounted at the world origin; this one is
/// not, and says so in its config rather than in this file.
pinocchio::SE3 ParseBasePoseInWorld(const YAML::Node& node) {
  if (!node) {
    return pinocchio::SE3::Identity();
  }
  if (!node.IsMap()) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.base_pose_in_world must be a map with `position` "
        "and `rpy`");
  }
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  if (const auto pn = node["position"]) {
    if (!pn.IsSequence() || pn.size() != 3) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.base_pose_in_world.position must be [x, y, z]");
    }
    for (int i = 0; i < 3; ++i) {
      p[i] = pn[static_cast<std::size_t>(i)].as<double>();
    }
  }
  if (const auto rn = node["rpy"]) {
    if (!rn.IsSequence() || rn.size() != 3) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.base_pose_in_world.rpy must be [r, p, y] in rad "
          "(ZYX Euler, the repo boundary convention)");
    }
    for (int i = 0; i < 3; ++i) {
      rpy[i] = rn[static_cast<std::size_t>(i)].as<double>();
    }
  }
  if (!p.allFinite() || !rpy.allFinite()) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.base_pose_in_world carries a non-finite value");
  }
  return pinocchio::SE3(rtc::math::se3::RpyToRotationZyx(rpy), p);
}

}  // namespace

void DemoInferenceController::LoadConfig(const YAML::Node& cfg) {
  // Base pass first: it owns `topics:` (this controller declares none, which is
  // legal) and `command_type`.
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

  model_path_ = inf["model_path"].as<std::string>("");
  optimized_model_path_ = inf["optimized_model_path"].as<std::string>("");
  intra_op_threads_ = inf["intra_op_threads"].as<int>(1);
  if (intra_op_threads_ < 1) {
    throw std::invalid_argument("demo_inference_controller: intra_op_threads must be >= 1");
  }
  allow_missing_model_ = inf["allow_missing_model"].as<bool>(false);

  // `hand_posture` is NOT read here. Whether it is required at all depends on
  // whether a `posture_scalar` role is declared, and roles are only resolvable
  // once the device rosters exist — so it moved to Pass 3 (#511 D-9).
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

  io_ = rtc::params::ParsePolicyIoParams(inf,
                                         [this](std::string_view id) { return FeatureSize(id); });

  if (io_.inputs.size() > static_cast<std::size_t>(kMaxInputTensors)) {
    throw std::invalid_argument(
        "demo_inference_controller: the schema declares " + std::to_string(io_.inputs.size()) +
        " input tensors, over the fixed capacity of " + std::to_string(kMaxInputTensors));
  }

  // Feature kinds, FLATTENED across tensors. `FeatureFromId` cannot tell arm
  // from hand on its own (both spell "<group>.position"), so the group name
  // decides here, where it is known — one pass over the ids the parser already
  // accepted. Each feature keeps its segment, and the segment carries its own
  // tensor index, so the tick walks one flat list instead of nesting two loops.
  const auto primary = GetPrimaryDeviceName();
  feature_kinds_.clear();
  flat_segments_.clear();
  for (const auto& tensor : io_.inputs) {
    for (std::size_t f = 0; f < tensor.features.size(); ++f) {
      const auto& id = tensor.features[f];
      PolicyFeature kind{};
      if (!FeatureFromId(id, kind)) {
        // Unreachable through ParsePolicyIoParams (FeatureSize already refused
        // any id this switch does not know) — kept so the two tables cannot
        // drift apart silently if one gains an entry the other does not.
        throw std::invalid_argument("demo_inference_controller: feature '" + id +
                                    "' has a size but no extractor (binding bug)");
      }
      if (kind == PolicyFeature::kArmPosition) {
        const auto dot = id.rfind('.');
        const auto group = id.substr(0, dot);
        kind = (group == primary) ? PolicyFeature::kArmPosition : PolicyFeature::kHandPosition;
      }
      feature_kinds_.push_back(kind);
      flat_segments_.push_back(tensor.segments[f]);
    }
  }

  // ── Pose-feature configuration ────────────────────────────────────────────
  // Required only when a pose feature asks for it. Demanding `palm:` from a
  // policy that observes joints and forces alone would be a configure failure
  // with nothing wrong.
  const bool wants_palm =
      std::any_of(feature_kinds_.begin(), feature_kinds_.end(), [](PolicyFeature k) {
        return k == PolicyFeature::kPalmPosition || k == PolicyFeature::kPalmOrientationXyzw;
      });
  const bool wants_object =
      std::any_of(feature_kinds_.begin(), feature_kinds_.end(), [](PolicyFeature k) {
        return k == PolicyFeature::kObjectPosition || k == PolicyFeature::kObjectOrientationXyzw;
      });

  if (wants_palm) {
    // No default link name. A wrong one is caught here (frame lookup fails at
    // configure), but a DEFAULT that happens to exist on some other robot would
    // resolve and silently feed the policy the wrong body — the original
    // design's default was `hand_base_link`, a leap-hand link that does not
    // exist on this hand at all.
    const YAML::Node palm = inf["palm"];
    if (!palm || !palm.IsMap() || !palm["link"]) {
      throw std::invalid_argument(
          "demo_inference_controller: a palm pose feature is declared, so inference.palm.link is "
          "required (the link whose pose the policy observes)");
    }
    palm_link_ = palm["link"].as<std::string>("");
    if (palm_link_.empty()) {
      throw std::invalid_argument("demo_inference_controller: inference.palm.link is empty");
    }
    palm_frame_ = ParsePoseFrame(palm["reference_frame"], "inference.palm.reference_frame");
  }

  if (wants_object) {
    const YAML::Node obj = inf["object_pose"];
    if (!obj || !obj.IsMap()) {
      throw std::invalid_argument(
          "demo_inference_controller: an object pose feature is declared, so "
          "inference.object_pose is required (topic + frame match)");
    }
    object_topic_ = obj["topic"].as<std::string>("");
    if (object_topic_.empty()) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.topic is empty");
    }
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
    object_frame_ = ParsePoseFrame(obj["reference_frame"], "inference.object_pose.reference_frame");
    object_source_frame_id_ = obj["source_frame_id"].as<std::string>("world");
    if (object_source_frame_id_.empty()) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.source_frame_id is empty — it is what "
          "turns \"the incoming poses are in world\" from an assumption into a check");
    }
    object_timeout_sec_ = obj["timeout_sec"].as<double>(0.2);
    if (!(object_timeout_sec_ > 0.0)) {
      throw std::invalid_argument(
          "demo_inference_controller: inference.object_pose.timeout_sec must be > 0. It is "
          "configuration and not a constant because the sim republishes every tick (~500 Hz) "
          "while a real perception stack runs at 10-30 Hz");
    }
  }

  // ── What `world` means ────────────────────────────────────────────────────
  world_from_base_ = ParseBasePoseInWorld(inf["base_pose_in_world"]);

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
  const auto secondary = GetSecondaryDeviceName();
  arm_target_idx_ = -1;
  hand_command_idx_ = -1;
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
