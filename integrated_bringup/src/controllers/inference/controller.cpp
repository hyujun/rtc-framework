// ── DemoInferenceController: construction, feature table, device wiring ──────

#include "integrated_bringup/controllers/demo_inference_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
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
// The ids are spelled with the DEVICE GROUP NAME as their prefix so a config
// reads like the robot it configures ("ur5e.position"), but the binding matches
// on the suffix and compares the prefix to the group names the config actually
// declared. Hardcoding "ur5e" here would make this controller ur5e-only for no
// benefit; accepting any prefix would let "ur5f.position" resolve silently.
namespace {

/// Split "group.field" into its two halves. Returns false when there is no dot.
bool SplitFeatureId(std::string_view id, std::string_view& group, std::string_view& field) {
  const auto dot = id.rfind('.');
  if (dot == std::string_view::npos || dot == 0 || dot + 1 >= id.size()) {
    return false;
  }
  group = id.substr(0, dot);
  field = id.substr(dot + 1);
  return true;
}

}  // namespace

int DemoInferenceController::FeatureSize(std::string_view id) const {
  std::string_view group;
  std::string_view field;
  if (!SplitFeatureId(id, group, field)) {
    return 0;
  }

  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // Widths come from the loaded rosters, never from a literal: that is what
  // keeps a 6-DoF arm from being baked in (ARCH-1) and what makes a config
  // whose feature width disagrees with its device roster fail loudly.
  if (field == "position") {
    if (!primary.empty() && group == primary) {
      const auto* cfg = GetDeviceNameConfig(primary);
      return cfg ? static_cast<int>(cfg->joint_state_names.size()) : 0;
    }
    if (!secondary.empty() && group == secondary) {
      const auto* cfg = GetDeviceNameConfig(secondary);
      return cfg ? static_cast<int>(cfg->joint_state_names.size()) : 0;
    }
    return 0;
  }

  if (field == "fingertip_force_norm") {
    if (secondary.empty() || group != secondary) {
      return 0;
    }
    const auto* cfg = GetDeviceNameConfig(secondary);
    return cfg ? static_cast<int>(cfg->sensor_names.size()) : 0;
  }

  return 0;
}

bool DemoInferenceController::FeatureFromId(std::string_view id, PolicyFeature& out) {
  std::string_view group;
  std::string_view field;
  if (!SplitFeatureId(id, group, field)) {
    return false;
  }
  // The group half was already validated by FeatureSize; only the field decides
  // which extractor runs. The arm/hand split is by group position, resolved in
  // LoadConfig's caller order, so it is re-derived here from the same names.
  if (field == "fingertip_force_norm") {
    out = PolicyFeature::kFingertipForceNorm;
    return true;
  }
  if (field != "position") {
    return false;
  }
  out = PolicyFeature::kArmPosition;  // corrected below by OnDeviceConfigsSet order
  return true;
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

  // Per-device bounds for the §7.3 command tail. Missing entries fall back to
  // the same generous defaults the sibling bindings use, so the tail always has
  // a band to work with even on a config that omits limits.
  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             -6.2832, 6.2832, 2.0);
}

}  // namespace integrated_bringup
