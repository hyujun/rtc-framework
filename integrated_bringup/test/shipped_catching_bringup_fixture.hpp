// ── A catching controller brought up from a SHIPPED profile (sim axis) ───────
//
// The CM's bring-up order for a controller of a shipped robot profile, read
// from the shipped files rather than restated: the device configs of the
// groups the controller's `topics:` claims (tagged as the simulator), the
// profile's system model as the CM builds it, one builder per profile per
// process. Shared by test_demo_catching_controller (the shipped-profile
// suites) and the allocation gate's second robot (test_demo_catching_alloc_s7).
#pragma once

#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
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

/// Device configs built FROM the shipped YAML for the two groups the
/// controller's own `topics:` block claims, tagged as the simulator.
///
/// Read rather than restated: a copy of the joint names here would pass while
/// the shipped config said something else, which is the whole failure mode the
/// test below exists to catch.
inline std::map<std::string, rtc::DeviceNameConfig> ShippedSimConfigs(const std::string& profile,
                                                                      const YAML::Node& node) {
  std::map<std::string, rtc::DeviceNameConfig> configs;
  const std::string base = ShippedProfilesDir() + "/" + profile + "/";
  YAML::Node devices;
  for (const char* candidate : {"_base.yaml", "sim.yaml"}) {
    if (!std::filesystem::exists(base + candidate)) {
      continue;
    }
    const YAML::Node root = YAML::LoadFile(base + candidate);
    const YAML::Node d = root["/**"]["ros__parameters"]["devices"];
    if (d && d.IsMap()) {
      devices = d;
      break;
    }
  }
  if (!devices) {
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
      // Every field the CM carries: the shipped `mpc` law (MD-89) and the
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
/// `urdf:` block, read from the file. Only the package is swapped, for the
/// vendored `robot_descriptions` copy at the same relative path — the one
/// model CI can acquire (ur5e_p1b_test_fixture.hpp has why). Since MD-89 the
/// shipped DECEL law is `mpc`, which parks without a model, so a bring-up
/// test of the shipped file that left the model out would configure a
/// controller production never builds.
inline rtc_urdf_bridge::ModelConfig ShippedModelConfig(const std::string& profile) {
  const std::string base = ShippedProfilesDir() + "/" + profile + "/";
  YAML::Node urdf;
  for (const char* candidate : {"_base.yaml", "sim.yaml"}) {
    if (!std::filesystem::exists(base + candidate)) {
      continue;
    }
    const YAML::Node u = YAML::LoadFile(base + candidate)["/**"]["ros__parameters"]["urdf"];
    if (u && u.IsMap()) {
      urdf = u;
      break;
    }
  }
  // Throws rather than ASSERT_: the callers would go on with no model.
  if (!urdf) {
    throw std::runtime_error(profile + ": no shipped urdf block");
  }
  const std::string share = ament_index_cpp::get_package_share_directory("robot_descriptions");
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = share + "/" + urdf["path"].as<std::string>();
  cfg.root_joint_type = urdf["root_joint_type"].as<std::string>("fixed");
  if (urdf["extended"].as<bool>(false)) {
    cfg.closure_yaml_path = share + "/" + urdf["closure_path"].as<std::string>();
  }
  // Document order: controllers take the FIRST sub-model as the arm.
  for (auto it = urdf["sub_models"].begin(); it != urdf["sub_models"].end(); ++it) {
    cfg.sub_models.push_back({it->first.as<std::string>(),
                              it->second["root_link"].as<std::string>(),
                              it->second["tip_link"].as<std::string>()});
  }
  for (auto it = urdf["tree_models"].begin(); it != urdf["tree_models"].end(); ++it) {
    cfg.tree_models.push_back({it->first.as<std::string>(),
                               it->second["root_link"].as<std::string>(),
                               it->second["tip_links"].as<std::vector<std::string>>()});
  }
  for (auto it = urdf["extra_frames"].begin(); it != urdf["extra_frames"].end(); ++it) {
    rtc_urdf_bridge::ExtraFrameConfig frame;
    frame.name = it->first.as<std::string>();
    frame.parent = it->second["parent"].as<std::string>();
    const auto xyz = it->second["xyz"].as<std::vector<double>>();
    const auto rpy = it->second["rpy"].as<std::vector<double>>(std::vector<double>(3, 0.0));
    frame.xyz = Eigen::Vector3d(xyz.at(0), xyz.at(1), xyz.at(2));
    frame.rpy = Eigen::Vector3d(rpy.at(0), rpy.at(1), rpy.at(2));
    frame.provisional = it->second["provisional"].as<bool>(true);
    cfg.extra_frames.push_back(frame);
  }
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
/// caller's.
inline void BringUpShipped(integrated_bringup::DemoCatchingController& ctrl,
                           const std::string& profile,
                           const std::map<std::string, rtc::DeviceNameConfig>& configs) {
  ctrl.SetSystemModelConfig(ShippedModelConfig(profile));
  ctrl.SetSharedModelBuilder(ShippedModelBuilder(profile));
  ctrl.SetControlRate(kShippedProfileControlRateHz);
  ctrl.SetDeviceNameConfigs(configs);
}

}  // namespace integrated_bringup::testfx
