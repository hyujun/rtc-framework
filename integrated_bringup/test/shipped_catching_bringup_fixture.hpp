// ── A catching controller brought up from a SHIPPED profile (sim axis) ───────
//
// The CM's bring-up order for a controller of a shipped robot profile, read
// from the shipped files rather than restated: the device configs of the
// groups the controller's `topics:` claims (tagged as the simulator), the
// profile's system model as the CM builds it (`_base.yaml` + `sim.yaml`
// merged, as the sim launch layers them), one builder per profile per process. Shared by
// test_demo_catching_controller (the shipped-profile suites) and the allocation gate's second robot
// (test_demo_catching_alloc_armed).
#pragma once

#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/support/profile_params.hpp"
#include "p1b_surrogate_models.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

/// The shipped `control_rate` both catching profiles run at.
inline constexpr double kShippedProfileControlRateHz = 500.0;

/// The directory of the shipped robot profiles: the source tree where the
/// target defines it, else the installed share.
inline std::string ShippedProfilesDir() {
#ifdef RTC_DEMO_SHARED_CONFIG_DIR
  return std::string(RTC_DEMO_SHARED_CONFIG_DIR);
#else
  return ament_index_cpp::get_package_share_directory("integrated_bringup") + "/config";
#endif
}

using integrated_bringup::MergeParamsInto;  // the profile files' layering (support/)

/// The RT node's parameters of a shipped sim bring-up: `_base.yaml` then
/// `sim.yaml`, merged as the launch hands them to the CM. A profile may split
/// a block across the two — ur5e_p1b keeps `urdf` in `_base.yaml` and its
/// `urdf.extra_frames.scene_world` in `sim.yaml` (#779) — so taking the first
/// file that has a block would drop the other half.
inline YAML::Node ShippedSimParams(const std::string& profile) {
  const std::string base = ShippedProfilesDir() + "/" + profile + "/";
  std::vector<std::string> files;
  for (const char* file : {"_base.yaml", "sim.yaml"}) {
    if (std::filesystem::exists(base + file)) {
      files.push_back(base + file);
    }
  }
  return LoadProfileParams(files);
}

/// Device configs built FROM the shipped YAML for the two groups the
/// controller's own `topics:` block claims, tagged as the simulator.
///
/// Read rather than restated: a copy of the joint names here would pass while
/// the shipped config said something else, which is the whole failure mode the
/// test below exists to catch.
inline std::map<std::string, rtc::DeviceNameConfig> ShippedSimConfigs(const std::string& profile,
                                                                      const YAML::Node& node) {
  std::map<std::string, rtc::DeviceNameConfig> configs;
  const YAML::Node params = ShippedSimParams(profile);
  const YAML::Node devices = params["devices"];
  if (!devices || !devices.IsMap()) {
    return configs;
  }
  for (auto it = node["topics"].begin(); it != node["topics"].end(); ++it) {
    const auto group = it->first.as<std::string>();
    const YAML::Node dev = devices[group];
    if (!dev || !dev["joint_state_names"]) {
      continue;
    }
    rtc::DeviceNameConfig cfg;
    cfg.device_name = group;
    cfg.joint_state_names = dev["joint_state_names"].as<std::vector<std::string>>();
    if (dev["motor_state_names"]) {
      cfg.motor_state_names = dev["motor_state_names"].as<std::vector<std::string>>();
    }
    if (const YAML::Node limits = dev["joint_limits"]; limits) {
      // Every field the CM carries: the shipped `mpc` law and the
      // dynamic CLIK read the torque and velocity ratings, not only the box.
      const auto per_joint = [&limits](const char* key) {
        return limits[key] ? limits[key].as<std::vector<double>>() : std::vector<double>{};
      };
      rtc::DeviceJointLimits jl;
      jl.max_velocity = per_joint("max_velocity");
      jl.max_torque = per_joint("max_torque");
      jl.position_lower = limits["position_lower"].as<std::vector<double>>();
      jl.position_upper = limits["position_upper"].as<std::vector<double>>();
      cfg.joint_limits = jl;
    }
    rtc::DeviceBackendBinding backend;
    backend.type = kCatchingSimBackendType;
    cfg.backend = backend;
    configs[group] = std::move(cfg);
  }
  return configs;
}

/// The profile's system model as the CM builds it: every entry of the shipped
/// `urdf:` block, read from the file, except the model files of a profile whose
/// model is not in this repo: those are its test model's (TestModelFiles). The
/// shipped DECEL law is `mpc`, which parks without a model, so a bring-up
/// test of the shipped file that left the model out would configure a
/// controller production never builds.
inline rtc_urdf_bridge::ModelConfig ShippedModelConfig(const std::string& profile) {
  const YAML::Node params = ShippedSimParams(profile);
  const YAML::Node urdf = params["urdf"];
  // Throws rather than ASSERT_: the callers would go on with no model.
  if (!urdf || !urdf.IsMap()) {
    throw std::runtime_error(profile + ": no shipped urdf block");
  }
  const bool extended = urdf["extended"].as<bool>(false);
  const ModelFiles files =
      TestModelFiles(profile, urdf["package"].as<std::string>("robot_descriptions"),
                     urdf["path"].as<std::string>(),
                     extended ? urdf["closure_path"].as<std::string>() : std::string());
  const std::string share = ament_index_cpp::get_package_share_directory("robot_descriptions");
  rtc_urdf_bridge::ModelConfig cfg = ModelConfigFromUrdfParams(
      urdf, share + "/" + files.urdf_rel, extended ? share + "/" + files.closure_rel : "");
  return cfg;
}

/// One builder per profile per process — the xacro → Pinocchio parse is the
/// expensive part, and production shares one builder the same way.
inline std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> ShippedModelBuilder(
    const std::string& profile) {
  static std::map<std::string, std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder>> builders;
  auto& builder = builders[profile];
  if (!builder) {
    builder = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(ShippedModelConfig(profile));
  }
  return builder;
}

/// The CM's bring-up order for a controller of the shipped profile: model,
/// builder, rate, devices (iiwa7_leap_test_fixture.hpp). on_configure is the
/// caller's. `tweak_model` edits the shipped model config first (a scene
/// other than the shipped one); the builder is then built for that config
/// alone instead of shared.
inline void BringUpShipped(
    integrated_bringup::DemoCatchingController& ctrl, const std::string& profile,
    const std::map<std::string, rtc::DeviceNameConfig>& configs,
    const std::function<void(rtc_urdf_bridge::ModelConfig&)>& tweak_model = nullptr) {
  rtc_urdf_bridge::ModelConfig cfg = ShippedModelConfig(profile);
  if (tweak_model) {
    tweak_model(cfg);
    ctrl.SetSharedModelBuilder(std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(cfg));
  } else {
    ctrl.SetSharedModelBuilder(ShippedModelBuilder(profile));
  }
  ctrl.SetSystemModelConfig(cfg);
  ctrl.SetControlRate(kShippedProfileControlRateHz);
  ctrl.SetDeviceNameConfigs(configs);
}

}  // namespace integrated_bringup::testfx
