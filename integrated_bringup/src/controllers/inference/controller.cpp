// ── DemoInferenceController: construction, feature table, device wiring ──────

#include "integrated_bringup/controllers/demo_inference_controller.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace integrated_bringup {

DemoInferenceController::DemoInferenceController(std::string_view urdf_path,
                                                 std::unique_ptr<rtc::InferenceEngine> engine)
    : engine_(std::move(engine)), urdf_path_(urdf_path) {}

void DemoInferenceController::SetDeviceTarget(int /*device_idx*/,
                                              std::span<const double> /*target*/) noexcept {
  // The goal is dropped, but not quietly — see the declaration.
  external_target_seen_.store(true, std::memory_order_relaxed);
}

// ── Feature table ────────────────────────────────────────────────────────────
//
// Device-scoped ids are spelled with the DEVICE GROUP NAME as their prefix so a
// config reads like the robot it configures ("ur5e.position"), but the binding
// compares the prefix to the group names the config actually declared.
// Hardcoding "ur5e" here would make this controller ur5e-only for no benefit;
// accepting any prefix would let "ur5f.position" resolve silently.
//
//   <group>.position | <group>.velocity    joint lane, policy convention
//   <hand>.fingertip_force_norm            ‖f‖ of every sensor group
//   <hand>.<sensor>.force_norm             ‖f‖ of one sensor group
//   link.<frame>.position | .orientation_xyzw   pose in `policy_frame`
//   object.position | object.orientation_xyzw   tracked object, `policy_frame`
//   reach.phase                            reach gate scalar
//
// A frame name therefore cannot contain a dot; URDF link names do not.
bool DemoInferenceController::ResolveFeature(std::string_view id, ResolvedFeature& out) const {
  out = ResolvedFeature{};
  const auto last_dot = id.rfind('.');
  if (last_dot == std::string_view::npos || last_dot == 0 || last_dot + 1 >= id.size()) {
    return false;
  }
  const std::string_view head = id.substr(0, last_dot);
  const std::string_view field = id.substr(last_dot + 1);

  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // Device groups are consulted FIRST and a miss falls through: "position" is
  // spelled the same for a device roster and for a pose feature
  // ("object.position"), so an early return here would make every pose feature
  // an unknown id. Widths come from the loaded rosters, never from a literal —
  // that is what keeps a 6-DoF arm from being baked in (ARCH-1) and what makes
  // a width disagreement between the policy and the robot fail loudly.
  if (field == "position" || field == "velocity") {
    const bool position = (field == "position");
    const bool is_primary = !primary.empty() && head == primary;
    const bool is_secondary = !secondary.empty() && head == secondary;
    if (is_primary || is_secondary) {
      const auto* cfg = GetDeviceNameConfig(is_primary ? primary : secondary);
      if (cfg == nullptr) {
        return false;
      }
      if (is_primary) {
        out.kind = position ? PolicyFeature::kArmPosition : PolicyFeature::kArmVelocity;
      } else {
        out.kind = position ? PolicyFeature::kHandPosition : PolicyFeature::kHandVelocity;
      }
      out.width = static_cast<int>(cfg->joint_state_names.size());
      out.rows = cfg->joint_state_names;
      return out.width > 0;
    }
  }

  if (field == "fingertip_force_norm") {
    if (secondary.empty() || head != secondary) {
      return false;
    }
    const auto* cfg = GetDeviceNameConfig(secondary);
    out.kind = PolicyFeature::kFingertipForceNorm;
    out.width = cfg ? static_cast<int>(cfg->sensor_names.size()) : 0;
    return out.width > 0;
  }

  if (field == "force_norm") {
    // "<hand>.<sensor>": only the hand carries an inference force lane.
    const auto dot = head.find('.');
    if (dot == std::string_view::npos || secondary.empty() || head.substr(0, dot) != secondary) {
      return false;
    }
    const auto* cfg = GetDeviceNameConfig(secondary);
    if (cfg == nullptr) {
      return false;
    }
    const std::string_view sensor = head.substr(dot + 1);
    const auto it = std::find(cfg->sensor_names.begin(), cfg->sensor_names.end(), sensor);
    if (it == cfg->sensor_names.end()) {
      return false;
    }
    out.kind = PolicyFeature::kGroupForceNorm;
    out.group = static_cast<int>(it - cfg->sensor_names.begin());
    out.width = 1;
    return true;
  }

  // Pose widths are the shape of a pose, not a robot fact, so they are
  // constants here.
  constexpr std::string_view kLinkPrefix = "link.";
  if (head.substr(0, kLinkPrefix.size()) == kLinkPrefix && head.size() > kLinkPrefix.size()) {
    const std::string_view frame = head.substr(kLinkPrefix.size());
    if (frame.find('.') != std::string_view::npos) {
      return false;
    }
    if (field == "position") {
      out.kind = PolicyFeature::kLinkPosition;
      out.width = 3;
    } else if (field == "orientation_xyzw") {
      out.kind = PolicyFeature::kLinkOrientationXyzw;
      out.width = 4;
    } else {
      return false;
    }
    out.link = std::string(frame);
    // One row per link: the export names its body table by link, and a row of
    // it is the whole xyz (or xyzw) of one body.
    out.rows = {out.link};
    return true;
  }

  if (head == "object") {
    if (field == "position") {
      out.kind = PolicyFeature::kObjectPosition;
      out.width = 3;
      return true;
    }
    if (field == "orientation_xyzw") {
      out.kind = PolicyFeature::kObjectOrientationXyzw;
      out.width = 4;
      return true;
    }
    return false;
  }

  if (head == "reach" && field == "phase") {
    out.kind = PolicyFeature::kReachPhase;
    out.width = 1;
    return true;
  }

  return false;
}

int DemoInferenceController::InternLink(const std::string& name) {
  for (std::size_t i = 0; i < links_.size(); ++i) {
    if (links_[i].name == name) {
      return static_cast<int>(i);
    }
  }
  if (links_.size() >= static_cast<std::size_t>(kMaxLinks)) {
    throw std::invalid_argument("demo_inference_controller: more than " +
                                std::to_string(kMaxLinks) +
                                " distinct links are observed (fixed capacity)");
  }
  links_.push_back(LinkSlot{name, -1, -1});
  return static_cast<int>(links_.size()) - 1;
}

void DemoInferenceController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  const auto* arm_cfg = GetDeviceNameConfig(primary);
  const auto* hand_cfg = secondary.empty() ? nullptr : GetDeviceNameConfig(secondary);

  arm_dof_ = arm_cfg ? static_cast<int>(arm_cfg->joint_state_names.size()) : 0;
  hand_dof_ = hand_cfg ? static_cast<int>(hand_cfg->joint_state_names.size()) : 0;
  num_fingertips_ = hand_cfg ? static_cast<int>(hand_cfg->sensor_names.size()) : 0;

  arm_dof_ = std::min(arm_dof_, kMaxArmDof);
  hand_dof_ = std::min(hand_dof_, kMaxHandDof);
  num_fingertips_ = std::min(num_fingertips_, kMaxFingertips);
  fingertip_stride_ = (hand_cfg != nullptr && hand_cfg->sensor_layout.has_value())
                          ? hand_cfg->sensor_layout->inference_values_per_group
                          : 0;

  // Per-device bounds for the §7.3 command tail. Missing entries fall back to
  // the same generous defaults the sibling bindings use, so the tail always has
  // a band to work with even on a config that omits limits.
  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             -6.2832, 6.2832, 2.0);
}

}  // namespace integrated_bringup
