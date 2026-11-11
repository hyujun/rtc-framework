// ── HandJointController test rigs ───────────────────────────────────────────
//
// Two hands this repository owns, neither a real one:
//   - CLOSED: robot_descriptions/robots/p1b_surrogate on its own — the
//     synthetic p1b-shaped hand (ten actuated joints, five four-bar loops),
//     processed from its xacro with the hand-only sidecar
//     p1b_surrogate.closure.yaml. The controller's gravity term is then the
//     reduced g_a of the closed chain, which is what the shipped p2 profile
//     exercises on the (private) proto2 model;
//   - PLAIN: rtc_urdf_bridge/test/urdf/tree_hand.urdf — a ten-joint tree
//     with no loops, listed by the device in an order that is NOT the model's,
//     so the plain path's joint-order install is load-bearing.
//
// Shared by test_hand_joint_controller.cpp (behaviour) and
// test_hand_joint_alloc.cpp (RT-1), which cannot be one binary: the latter
// defines the global allocation gates.
#pragma once

#include "integrated_bringup/controllers/hand_joint_controller.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "test_urdf_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hand_joint_rig {

namespace rub = rtc_urdf_bridge;
using integrated_bringup::HandJointController;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr double kDt = 0.002;
constexpr const char* kGroup = "hand";

// ── CLOSED rig: the p1b surrogate hand on its own ───────────────────────────
// Device order = the shipped ur5e_p1b hand roster (thumb 4, index 3, middle 2,
// ring 1); the sidecar's independent-joint order is the model's, which differs
// from this, so the name bridge is exercised.
const std::vector<std::string> kClosedJoints = {
    "thumb_cmc_aa_joint",  "thumb_cmc_fe_joint", "thumb_mcp_joint",    "thumb_dip_fe_joint",
    "index_mcp_aa_joint",  "index_mcp_fe_joint", "index_dip_fe_joint", "middle_mcp_fe_joint",
    "middle_dip_fe_joint", "ring_mcp_fe_joint"};
const std::vector<std::string> kClosedTips = {"l_thumb_tip_bracket", "l_index_tip_bracket",
                                              "l_middle_tip_bracket", "l_ring_tip_bracket"};
constexpr const char* kClosedRoot = "base_adapter";
// A pose off the loops' reference (q = 0): every actuated joint flexed by a
// quarter radian, inside the range the surrogate assembles in (its README).
// Off zero on every joint so a dropped or swapped term changes the torque.
constexpr double kClosedQ = 0.25;

// ── PLAIN rig: the loop-free tree hand ──────────────────────────────────────
// Device order deliberately not the URDF's (fingers listed middle, thumb,
// ring, index, and each finger's joints reversed).
const std::vector<std::string> kPlainJoints = {
    "middle_joint_2", "middle_joint_1", "thumb_joint_3", "thumb_joint_2", "thumb_joint_1",
    "ring_joint_2",   "ring_joint_1",   "index_joint_3", "index_joint_2", "index_joint_1"};
const std::vector<std::string> kPlainTips = {"thumb_tip", "index_tip", "middle_tip", "ring_tip"};
constexpr const char* kPlainRoot = "palm_link";
const std::vector<double> kPlainQ = {0.30, -0.40, 0.50, 0.20,  -0.25,
                                     0.35, -0.15, 0.45, -0.30, 0.10};

inline std::string ClosedUrdf() {
  return ament_index_cpp::get_package_share_directory("robot_descriptions") +
         "/robots/p1b_surrogate/urdf/p1b_surrogate.urdf.xacro";
}

inline std::string ClosedClosure() {
  return ament_index_cpp::get_package_share_directory("robot_descriptions") +
         "/robots/p1b_surrogate/urdf/p1b_surrogate.closure.yaml";
}

inline rub::ModelConfig ClosedModelConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = ClosedUrdf();
  cfg.closure_yaml_path = ClosedClosure();
  cfg.root_joint_type = "fixed";
  cfg.tree_models.push_back({kGroup, kClosedRoot, kClosedTips});
  return cfg;
}

inline rub::ModelConfig PlainModelConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = rtc::test::TestUrdfPath("tree_hand.urdf");
  cfg.root_joint_type = "fixed";
  cfg.tree_models.push_back({kGroup, kPlainRoot, kPlainTips});
  return cfg;
}

/// The controller YAML. `max_abs` is the feedforward bound (N·m);
/// `inertia_gain` is written only when non-zero (absent = the default 0).
inline std::string Yaml(double max_abs = 10.0, bool enabled = true, double gain = 1.0,
                        const char* command_type = "pd_feedforward", double inertia_gain = 0.0) {
  const std::string inertia =
      inertia_gain != 0.0 ? "\n  inertia_gain: " + std::to_string(inertia_gain) : std::string();
  return std::string("trajectory_speed: 2.0\nmax_traj_velocity: 4.0\ncommand_type: \"") +
         command_type + "\"\ndynamics_feedforward:\n  enabled: " + (enabled ? "true" : "false") +
         "\n  gravity_gain: " + std::to_string(gain) + inertia +
         "\n  max_abs: " + std::to_string(max_abs) +
         R"(
topics:
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
    publish:
      - topic: "transforms"
        role: "robot_transforms"
logs:
  - msg_type: rtc_msgs/DeviceStateLog
    instance: hand_state
  - msg_type: integrated_bringup/JointFeedforwardLog
    instance: joint_ff
)";
}

inline rtc::DeviceNameConfig Device(const std::vector<std::string>& joints, double max_torque) {
  rtc::DeviceNameConfig dev;
  dev.device_name = kGroup;
  dev.joint_state_names = joints;
  rtc::DeviceJointLimits limits;
  limits.max_velocity.assign(joints.size(), 4.0);
  limits.max_torque.assign(joints.size(), max_torque);
  limits.position_lower.assign(joints.size(), -3.0);
  limits.position_upper.assign(joints.size(), 3.0);
  dev.joint_limits = limits;
  return dev;
}

struct Rig {
  rub::ModelConfig model;
  std::shared_ptr<rub::PinocchioModelBuilder> builder;
  std::map<std::string, rtc::DeviceNameConfig> devices;
  std::vector<std::string> joints;
  std::vector<double> q;
  std::string yaml;
};

inline Rig ClosedRig(const std::string& yaml = Yaml(), double max_torque = 3.0) {
  static const rub::ModelConfig cfg = ClosedModelConfig();
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);
  Rig rig;
  rig.model = cfg;
  rig.builder = builder;
  rig.devices[kGroup] = Device(kClosedJoints, max_torque);
  rig.joints = kClosedJoints;
  rig.q.assign(kClosedJoints.size(), kClosedQ);
  rig.yaml = yaml;
  return rig;
}

inline Rig PlainRig(const std::string& yaml = Yaml(), double max_torque = 3.0) {
  static const rub::ModelConfig cfg = PlainModelConfig();
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);
  Rig rig;
  rig.model = cfg;
  rig.builder = builder;
  rig.devices[kGroup] = Device(kPlainJoints, max_torque);
  rig.joints = kPlainJoints;
  rig.q = kPlainQ;
  rig.yaml = yaml;
  return rig;
}

/// A controller with what the controller manager injects before any config is
/// read, then the config and the device configs (no node involved).
inline std::unique_ptr<HandJointController> BringUp(const Rig& rig) {
  auto ctrl = std::make_unique<HandJointController>("");
  ctrl->SetSystemModelConfig(rig.model);
  ctrl->SetSharedModelBuilder(rig.builder);
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(YAML::Load(rig.yaml));
  ctrl->SetDeviceNameConfigs(rig.devices);
  return ctrl;
}

inline ControllerState MakeState(const std::vector<double>& q, bool valid = true,
                                 std::uint64_t iteration = 1) {
  ControllerState state{};
  state.num_devices = 1;
  state.dt = kDt;
  state.iteration = iteration;
  state.t_relative_s = static_cast<double>(iteration) * kDt;
  auto& dev = state.devices[0];
  dev.num_channels = static_cast<int>(q.size());
  dev.valid = valid;
  dev.hole_mask = 0;
  for (std::size_t i = 0; i < q.size(); ++i) {
    dev.positions[i] = q[i];
  }
  return state;
}

/// Tick with the same measurement until the model term is fresh and the
/// projection is neither held nor singular (the walk-in after a seed jump),
/// or `max_ticks` have passed. Returns the last output.
inline ControllerOutput TickUntilFresh(HandJointController& ctrl, const std::vector<double>& q,
                                       int max_ticks = 500) {
  ControllerOutput out{};
  for (int t = 1; t <= max_ticks; ++t) {
    out = ctrl.Compute(MakeState(q, true, static_cast<std::uint64_t>(t)));
    const auto& tick = ctrl.LastTick();
    if (tick.model_fresh && !tick.held && !tick.singular && tick.seeded) {
      return out;
    }
  }
  return out;
}

}  // namespace hand_joint_rig
