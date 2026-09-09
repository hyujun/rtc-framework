// ── DemoInferenceController: YAML schema ─────────────────────────────────────
// LoadConfig is Pass 1 of the 3-pass bring-up: `topic_config_` exists but the
// device rosters do not yet, so anything that has to be checked against a
// device's joint count or limits waits for OnDeviceConfigsSet / on_configure.
// What lands here is everything that is self-contained in the YAML.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"

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

  // ── I/O schema is NOT parsed here ──────────────────────────────────────────
  // Resolving a feature id to its width needs the device rosters, and this is
  // Pass 1: `PreConfigure` calls LoadConfig, and the CM only injects device
  // configs afterwards (`SetDeviceNameConfigs`, then `on_configure`). Parsing
  // here would resolve every id against an empty roster, size each feature 0,
  // and then reject the config for a sum mismatch — a real error reported as
  // the wrong one. `ApplyIoSchema` runs from on_configure instead.

  // ── Hand postures ─────────────────────────────────────────────────────────
  // Length is cross-checked against the hand device's roster in
  // OnDeviceConfigsSet — here we only know they must agree with each other.
  const YAML::Node posture = inf["hand_posture"];
  if (!posture || !posture.IsMap()) {
    throw std::invalid_argument(
        "demo_inference_controller: inference.hand_posture must define `open` and `close`");
  }
  posture_open_ = ParsePosture(posture["open"], "inference.hand_posture.open");
  posture_close_ = ParsePosture(posture["close"], "inference.hand_posture.close");
  if (posture_open_.size() != posture_close_.size()) {
    throw std::invalid_argument(
        "demo_inference_controller: hand_posture.open has " + std::to_string(posture_open_.size()) +
        " joints but hand_posture.close has " + std::to_string(posture_close_.size()));
  }
  if (posture_open_.size() > static_cast<std::size_t>(kMaxHandDof)) {
    throw std::invalid_argument("demo_inference_controller: hand_posture exceeds kMaxHandDof");
  }
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

  // Feature kinds. `FeatureFromId` cannot tell arm from hand on its own (both
  // spell "<group>.position"), so the group name decides here, where it is
  // known — one pass over the ids the parser already accepted.
  const auto primary = GetPrimaryDeviceName();
  feature_kinds_.clear();
  feature_kinds_.reserve(io_.input_features.size());
  for (const auto& id : io_.input_features) {
    PolicyFeature kind{};
    if (!FeatureFromId(id, kind)) {
      // Unreachable through ParsePolicyIoParams (FeatureSize already refused any
      // id this switch does not know) — kept so the two tables cannot drift
      // apart silently if one gains an entry the other does not.
      throw std::invalid_argument("demo_inference_controller: feature '" + id +
                                  "' has a size but no extractor (binding bug)");
    }
    if (kind == PolicyFeature::kArmPosition) {
      const auto dot = id.rfind('.');
      const auto group = id.substr(0, dot);
      kind = (group == primary) ? PolicyFeature::kArmPosition : PolicyFeature::kHandPosition;
    }
    feature_kinds_.push_back(kind);
  }

  // ── Fixed-capacity check ──────────────────────────────────────────────────
  if (io_.InputNumel() > static_cast<std::size_t>(kMaxInputElements)) {
    throw std::invalid_argument("demo_inference_controller: input tensor has " +
                                std::to_string(io_.InputNumel()) +
                                " elements, over the fixed capacity of " +
                                std::to_string(kMaxInputElements));
  }
  for (const auto& shape : io_.output_shapes) {
    std::size_t numel = 1;
    for (const auto dim : shape) {
      numel *= static_cast<std::size_t>(dim);
    }
    if (numel > static_cast<std::size_t>(kMaxOutputElements)) {
      throw std::invalid_argument(
          "demo_inference_controller: an output head has " + std::to_string(numel) +
          " elements, over the fixed capacity of " + std::to_string(kMaxOutputElements));
    }
  }

  // ── Output roles ──────────────────────────────────────────────────────────
  // Matched by NAME, not by position. The slices are already positional against
  // the model's heads; making the binding depend on YAML list order as well
  // would mean a harmless reordering of the two entries silently swapped arm
  // and hand, and both would still be finite and in range.
  arm_output_idx_ = -1;
  hand_output_idx_ = -1;
  for (std::size_t i = 0; i < io_.output_names.size(); ++i) {
    const auto& n = io_.output_names[i];
    const auto dot = n.rfind('.');
    const auto field = (dot == std::string::npos) ? n : n.substr(dot + 1);
    if (field == "target_position") {
      arm_output_idx_ = static_cast<int>(i);
    } else if (field == "posture_scalar") {
      hand_output_idx_ = static_cast<int>(i);
    }
  }
  if (arm_output_idx_ < 0 || hand_output_idx_ < 0) {
    throw std::invalid_argument(
        "demo_inference_controller: output_features must name both "
        "\"<arm group>.target_position\" and \"<hand group>.posture_scalar\"");
  }
  if (io_.output_slices[static_cast<std::size_t>(hand_output_idx_)].count != 1) {
    throw std::invalid_argument(
        "demo_inference_controller: the hand posture scalar must slice exactly 1 element");
  }
  if (io_.output_slices[static_cast<std::size_t>(arm_output_idx_)].count != arm_dof_) {
    throw std::invalid_argument(
        "demo_inference_controller: the arm target slices " +
        std::to_string(io_.output_slices[static_cast<std::size_t>(arm_output_idx_)].count) +
        " elements but the arm device declares " + std::to_string(arm_dof_) + " joints");
  }
}

}  // namespace integrated_bringup
