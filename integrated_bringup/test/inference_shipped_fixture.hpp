// ── The shipped inference config on the ur5e_p1b model: shared test data ─────
//
// Policy constants, device configs and the training pregrasp state for the
// suites that drive DemoInferenceController with the SHIPPED controller YAML on
// the real model: test_demo_inference_urdf.cpp (fake engine, CI) and
// test_demo_inference_real_model.cpp (the policy itself, local only). One copy
// so that "the pregrasp" is the same state in both — the real-model suite's
// first action is only comparable with the urdf suite's oracle if they start
// from the same joints.
//
// The constants are the export's ONNX initializers (policy frame, policy joint
// convention) and the training asset's joint signs, stated here independently
// of the shipped YAML so that the YAML is what gets checked.
#pragma once

#include "ur5e_p1b_test_fixture.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

// ── Policy constants (ONNX initializers, policy frame / policy convention) ──
inline constexpr std::array<double, 6> kC18 = {-0.011404, -1.148533, 1.720851,
                                               -2.153859, -2.254593, 2.771491};
// C20 in the gripper head's order (index → ring → thumb → middle).
inline const std::vector<std::string> kGripperOrder = {
    "index_mcp_aa_joint",  "index_mcp_fe_joint", "index_dip_fe_joint", "ring_mcp_fe_joint",
    "thumb_cmc_aa_joint",  "thumb_cmc_fe_joint", "thumb_mcp_joint",    "thumb_dip_fe_joint",
    "middle_mcp_fe_joint", "middle_dip_fe_joint"};
inline constexpr std::array<double, 10> kC20 = {0.064476,  0.557459, 0.440156, 0.45242,  -1.426097,
                                                -0.429778, 0.049478, 0.609149, 0.529181, 0.511366};
// The training asset's joint signs (q_policy = s · q_device).
inline const std::map<std::string, double> kPolicySign = {
    {"thumb_cmc_aa_joint", -1.0}, {"thumb_cmc_fe_joint", 1.0},   {"thumb_mcp_joint", 1.0},
    {"thumb_dip_fe_joint", -1.0}, {"index_mcp_aa_joint", -1.0},  {"index_mcp_fe_joint", -1.0},
    {"index_dip_fe_joint", -1.0}, {"middle_mcp_fe_joint", -1.0}, {"middle_dip_fe_joint", -1.0},
    {"ring_mcp_fe_joint", -1.0}};

/// Floats per sensor group on the hand's inference lane, as _base.yaml ships it.
inline constexpr int kInferenceStride = 7;

/// Element count of every tensor a YAML `inputs:` / `outputs:` list declares.
inline std::vector<std::size_t> Numels(const YAML::Node& tensors) {
  std::vector<std::size_t> out;
  for (const auto& t : tensors) {
    std::size_t n = 1;
    for (const auto& d : t["shape"]) {
      n *= d.as<std::size_t>();
    }
    out.push_back(n);
  }
  return out;
}

inline std::size_t IndexOfName(const std::vector<std::string>& names, const std::string& name) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) {
      return i;
    }
  }
  ADD_FAILURE() << "no element named '" << name << "'";
  return names.size();
}

/// The fixture's device configs plus what the p1b hand needs here and the
/// fixture does not carry: the inference lane stride (force features) and the
/// hand's position band (the command tail), both as _base.yaml ships them.
inline std::map<std::string, rtc::DeviceNameConfig> MakeInferenceDeviceConfigs() {
  auto devices = MakeUr5eP1bDeviceConfigs();
  auto& hand = devices.at("p1b");
  rtc::DeviceSensorLayout layout;
  layout.inference_values_per_group = kInferenceStride;
  hand.sensor_layout = layout;
  rtc::DeviceJointLimits lim;
  lim.max_velocity.assign(kP1bHandDof, 10.384);
  lim.position_lower = {0.0,        -1.5707963, -1.5707963, -1.5707963, -0.34906585,
                        -1.5707963, -1.5707963, -1.5707963, -1.5707963, -1.5707963};
  lim.position_upper = {2.356194487, 1.5707963, 1.5707963, 0.0, 0.523598776,
                        0.0,         0.0,       0.0,       0.0, 0.0};
  hand.joint_limits = lim;
  return devices;
}

/// Arm at C18, hand at s ⊙ C20 in DEVICE order — the training pregrasp.
inline rtc::ControllerState MakePregraspState() {
  auto state = MakeUr5eP1bState();
  for (std::size_t i = 0; i < kC18.size(); ++i) {
    state.devices[0].positions[i] = kC18[i];
  }
  const auto devices = MakeUr5eP1bDeviceConfigs();
  const auto& hand_names = devices.at("p1b").joint_state_names;
  for (std::size_t j = 0; j < hand_names.size(); ++j) {
    const auto k = IndexOfName(kGripperOrder, hand_names[j]);
    state.devices[1].positions[j] = kPolicySign.at(hand_names[j]) * kC20[k];
  }
  return state;
}

/// The object as the sim publishes it: `world` → `pool_pole_object`.
inline tf2_msgs::msg::TFMessage MakeObjectTf(const Eigen::Vector3d& p_world,
                                             const Eigen::Quaterniond& q_world) {
  tf2_msgs::msg::TFMessage msg;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world";
  tf.child_frame_id = "pool_pole_object";
  tf.transform.translation.x = p_world.x();
  tf.transform.translation.y = p_world.y();
  tf.transform.translation.z = p_world.z();
  tf.transform.rotation.x = q_world.x();
  tf.transform.rotation.y = q_world.y();
  tf.transform.rotation.z = q_world.z();
  tf.transform.rotation.w = q_world.w();
  msg.transforms.push_back(tf);
  return msg;
}

/// A pose given in the POLICY frame (URDF `base_link`), expressed in `world`
/// (= URDF `base`, a half turn about z away).
inline Eigen::Vector3d WorldFromPolicyFrame(const Eigen::Vector3d& p_pf) {
  return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * p_pf;
}

inline Eigen::Quaterniond WorldFromPolicyFrame(const Eigen::Quaterniond& q_pf) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) * q_pf;
}

}  // namespace integrated_bringup::testfx
