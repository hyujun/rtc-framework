// ── iiwa7_leap URDF-backed controller test fixture helpers ───────────────────
//
// Shared by test_demo_task_controller / test_demo_joint_controller_urdf: the
// demo controllers gate their model-dependent paths (combined-model cache,
// hand fingertip FK, virtual TCP, E-STOP FK) behind a real Pinocchio model, so
// these tests replay the CM bring-up sequence by hand on the iiwa7_leap serial
// profile (same topology as config/iiwa7_leap sim.yaml and
// test_task_arm_cache_equivalence):
//
//   SetSystemModelConfig → SetSharedModelBuilder → SetControlRate →
//   LoadConfig(yaml) → SetDeviceNameConfigs (triggers OnDeviceConfigsSet)
//
// The builder is a process-wide singleton — the xacro→Pinocchio parse is the
// expensive part, and production shares one builder across controllers the
// same way (SetSharedModelBuilder).
//
// No ROS node / DDS is involved anywhere: deterministic, no ROS_DOMAIN_ID
// isolation needed.
#pragma once

#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <map>
#include <memory>
#include <string>

namespace integrated_bringup::testfx {

inline constexpr int kArmDof = 7;
inline constexpr int kHandDof = 16;
inline constexpr double kDt = 0.002;

// iiwa7 rest pose used as the simulated measured state (non-trivial FK).
inline constexpr std::array<double, kArmDof> kArmHome = {0.0, 0.7, 0.0, -1.4, 0.0, 0.7, 0.0};

// Model topology — mirrors config/iiwa7_leap sim.yaml `urdf:` section.
inline rtc_urdf_bridge::ModelConfig MakeIiwa7LeapModelConfig() {
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = ament_index_cpp::get_package_share_directory("robot_descriptions") +
                  "/robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro";
  cfg.root_joint_type = "fixed";
  cfg.sub_models.push_back({"iiwa7", "link_0", "ee_link"});
  cfg.tree_models.push_back(
      {"leap", "base", {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}});
  cfg.tree_models.push_back(
      {"wbc", "link_0", {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}});
  return cfg;
}

inline const rtc_urdf_bridge::ModelConfig& SharedIiwa7LeapModelConfig() {
  static const rtc_urdf_bridge::ModelConfig cfg = MakeIiwa7LeapModelConfig();
  return cfg;
}

inline std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> SharedIiwa7LeapBuilder() {
  static auto builder =
      std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(SharedIiwa7LeapModelConfig());
  return builder;
}

// Device configs as CM would resolve them (urdf root/tip auto-resolved from
// sub_models/tree_models; leap sensor capability flags = mujoco wrench lane).
inline std::map<std::string, rtc::DeviceNameConfig> MakeIiwa7LeapDeviceConfigs() {
  std::map<std::string, rtc::DeviceNameConfig> configs;

  rtc::DeviceNameConfig arm;
  arm.device_name = "iiwa7";
  arm.joint_state_names = {"A1", "A2", "A3", "A4", "A5", "A6", "A7"};
  rtc::DeviceUrdfConfig arm_urdf;
  arm_urdf.package = "robot_descriptions";
  arm_urdf.path = "robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro";
  arm_urdf.root_link = "link_0";
  arm_urdf.tip_link = "ee_link";
  arm.urdf = arm_urdf;
  rtc::DeviceJointLimits arm_limits;
  arm_limits.max_velocity = {1.71, 1.71, 1.74, 2.27, 2.44, 3.14, 3.14};
  arm_limits.position_lower = {-2.9671, -2.0944, -2.9671, -2.0944, -2.9671, -2.0944, -3.0543};
  arm_limits.position_upper = {2.9671, 2.0944, 2.9671, 2.0944, 2.9671, 2.0944, 3.0543};
  arm.joint_limits = arm_limits;
  configs["iiwa7"] = std::move(arm);

  rtc::DeviceNameConfig hand;
  hand.device_name = "leap";
  // Controller-facing order (thumb first) — same as the shipped config.
  hand.joint_state_names = {"12", "13", "14", "15", "0", "1", "2",  "3",
                            "4",  "5",  "6",  "7",  "8", "9", "10", "11"};
  rtc::DeviceUrdfConfig hand_urdf;
  hand_urdf.root_link = "base";
  hand.urdf = hand_urdf;
  rtc::DeviceSensorLayout layout;
  layout.inference_values_per_group = 7;
  layout.has_native_contact = false;
  layout.has_native_displacement = false;
  hand.sensor_layout = layout;
  configs["leap"] = std::move(hand);

  return configs;
}

inline rtc::ControllerState MakeIiwa7LeapState() {
  rtc::ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = kArmDof;
  dev0.valid = true;
  for (int i = 0; i < kArmDof; ++i) {
    dev0.positions[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }

  auto& dev1 = state.devices[1];
  dev1.num_channels = kHandDof;
  dev1.valid = true;

  return state;
}

// Write a fingertip force (Fz) into the inference lane (mujoco wrench shape:
// slot 0 = native contact (zero-fill), slots 1..3 = F).
inline void SetFingertipForce(rtc::ControllerState& s, int f, float fz, float contact = 0.0f) {
  auto& dev1 = s.devices[1];
  dev1.inference_enable[static_cast<std::size_t>(f)] = true;
  const int base = f * static_cast<int>(kHandInferenceValuesPerFingertipCapacity);
  dev1.inference_data[static_cast<std::size_t>(base)] = contact;
  dev1.inference_data[static_cast<std::size_t>(base + 1)] = 0.0f;
  dev1.inference_data[static_cast<std::size_t>(base + 2)] = 0.0f;
  dev1.inference_data[static_cast<std::size_t>(base + 3)] = fz;
}

}  // namespace integrated_bringup::testfx
