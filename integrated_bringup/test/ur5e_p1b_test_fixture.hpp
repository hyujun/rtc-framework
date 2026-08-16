// ── ur5e_p1b URDF-backed controller test fixture helpers ─────────────────────
//
// The closed-chain sibling of iiwa7_leap_test_fixture.hpp. Same shape and same
// reason (the demo controllers gate their model-dependent paths behind a real
// Pinocchio model, so tests replay the CM bring-up sequence by hand), but this
// profile is what iiwa7_leap cannot be: `urdf.extended: true` with a genuine
// 5-loop closure sidecar, which is the only configuration where the closed-chain
// projection paths (reduced dynamics provider, fingertip FK) are active at all.
//
// The two headers are data, not logic — the bring-up sequence itself lives in
// each test file — so they are siblings rather than one parameterised header:
//   SetSystemModelConfig → SetSharedModelBuilder → SetControlRate →
//   LoadConfig(yaml) → SetDeviceNameConfigs (triggers OnDeviceConfigsSet)
//
// The model is vendored in-repo at robot_descriptions/robots/ur5e_p1b/ (#457), so
// these paths run in CI. It used to resolve `hand_description`, a separate project
// present only on developer machines, which made every case below an invisible red
// there and then an explicit skip (#454) — see that robot's README.md for the
// vendoring rationale and for what tracking upstream costs.
//
// Every value below mirrors config/ur5e_p1b/_base.yaml (`urdf:` + `devices:`) except
// the package: the shipped profile still names `hand_description` because it drives
// a real, separate robot, while this fixture needs a model CI can acquire. The
// kinematics, closure sidecar and joint names are the same model either way.
// The builder is a process-wide singleton for the same reason as the iiwa7 one:
// the xacro→Pinocchio parse dominates, and production shares one builder.
#pragma once

#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

inline constexpr int kUr5eArmDof = 6;
inline constexpr int kP1bHandDof = 10;
inline constexpr double kUr5eDt = 0.002;

// A non-singular UR5e posture — away from the zero configuration so fingertip
// FK is a non-trivial function of the arm joints (a zero pose would let a wrong
// frame chain pass by symmetry, the vacuity trap §3.3 of compliance-conventions).
inline constexpr std::array<double, kUr5eArmDof> kUr5eHome = {0.0, -1.2, 1.4, -1.7, -1.5, 0.0};

inline rtc_urdf_bridge::ModelConfig MakeUr5eP1bModelConfig() {
  rtc_urdf_bridge::ModelConfig cfg;
  const std::string share = ament_index_cpp::get_package_share_directory("robot_descriptions");
  cfg.urdf_path = share + "/robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro";
  cfg.root_joint_type = "fixed";
  // The closure sidecar is what makes GetActuatedModel() non-null and the
  // reduced-dynamics provider configurable — without it this fixture would be
  // just another serial robot and every closed-chain assertion would be vacuous.
  cfg.closure_yaml_path = share + "/robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml";
  cfg.sub_models.push_back({"ur5e", "base", "tool0"});
  cfg.tree_models.push_back({"p1b",
                             "base_adapter",
                             {"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket",
                              "l_ring_tip_bracket"}});
  cfg.tree_models.push_back({"wbc",
                             "base",
                             {"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket",
                              "l_ring_tip_bracket"}});
  return cfg;
}

inline const rtc_urdf_bridge::ModelConfig& SharedUr5eP1bModelConfig() {
  static const rtc_urdf_bridge::ModelConfig cfg = MakeUr5eP1bModelConfig();
  return cfg;
}

inline std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> SharedUr5eP1bBuilder() {
  static auto builder =
      std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(SharedUr5eP1bModelConfig());
  return builder;
}

// Device configs as CM would resolve them. The joint name lists are the q_a
// bridge source for the closed-chain hand FK, so they must match the URDF joint
// names exactly — a typo shows up as an inactive wrapper, not as a failure.
inline std::map<std::string, rtc::DeviceNameConfig> MakeUr5eP1bDeviceConfigs() {
  std::map<std::string, rtc::DeviceNameConfig> configs;

  rtc::DeviceNameConfig arm;
  arm.device_name = "ur5e";
  arm.joint_state_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  rtc::DeviceUrdfConfig arm_urdf;
  arm_urdf.package = "robot_descriptions";
  arm_urdf.path = "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro";
  arm_urdf.root_link = "base";
  arm_urdf.tip_link = "tool0";
  arm.urdf = arm_urdf;
  rtc::DeviceJointLimits arm_limits;
  arm_limits.max_velocity = {2.0, 2.0, 3.0, 3.0, 3.0, 3.0};
  arm_limits.max_acceleration = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  arm_limits.max_torque = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};
  arm_limits.position_lower = {-6.28, -6.28, -3.14, -6.28, -6.28, -6.28};
  arm_limits.position_upper = {6.28, 6.28, 3.14, 6.28, 6.28, 6.28};
  arm.joint_limits = arm_limits;
  configs["ur5e"] = std::move(arm);

  rtc::DeviceNameConfig hand;
  hand.device_name = "p1b";
  hand.joint_state_names = {"thumb_cmc_aa_joint", "thumb_cmc_fe_joint",  "thumb_mcp_joint",
                            "thumb_dip_fe_joint", "index_mcp_aa_joint",  "index_mcp_fe_joint",
                            "index_dip_fe_joint", "middle_mcp_fe_joint", "middle_dip_fe_joint",
                            "ring_mcp_fe_joint"};
  hand.sensor_names = {"thumb", "index", "middle", "ring"};
  rtc::DeviceUrdfConfig hand_urdf;
  hand_urdf.root_link = "base_adapter";
  hand.urdf = hand_urdf;
  configs["p1b"] = std::move(hand);

  return configs;
}

// Fully-reported state: both devices valid, full width, no holes.
inline rtc::ControllerState MakeUr5eP1bState() {
  rtc::ControllerState state{};
  state.num_devices = 2;
  state.dt = kUr5eDt;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = kUr5eArmDof;
  dev0.valid = true;
  dev0.hole_mask = 0;
  for (int i = 0; i < kUr5eArmDof; ++i) {
    dev0.positions[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
  }

  auto& dev1 = state.devices[1];
  dev1.num_channels = kP1bHandDof;
  dev1.valid = true;
  dev1.hole_mask = 0;
  // Slightly flexed fingers: a zero hand configuration sits at the closure
  // reference, where a stale q and a fresh q would be indistinguishable.
  for (int i = 0; i < kP1bHandDof; ++i) {
    dev1.positions[static_cast<std::size_t>(i)] = 0.05 + 0.01 * static_cast<double>(i);
  }

  return state;
}

}  // namespace integrated_bringup::testfx
