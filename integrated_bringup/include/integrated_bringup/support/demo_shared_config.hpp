#ifndef UR5E_BRINGUP_SUPPORT_DEMO_SHARED_CONFIG_HPP_
#define UR5E_BRINGUP_SUPPORT_DEMO_SHARED_CONFIG_HPP_

#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <memory>
#include <string>

namespace integrated_bringup {

// Parameters that DemoJointController and DemoTaskController share.
// Defaults live in config/ur5e_hand/controllers/demo_shared.yaml; per-controller YAMLs
// may override individual keys via a second ApplyDemoSharedConfig() pass.
struct DemoSharedConfig {
  VirtualTcpConfig vtcp{};
  float grasp_contact_threshold{0.5f};
  float grasp_force_threshold{1.0f};
  int grasp_min_fingertips{2};

  std::string grasp_controller_type{"contact_stop"};

  // Hand joint topology (10-DoF UR5e hand, 3-finger × 3-joint per finger).
  // Indices into device 1 (hand) joint arrays.
  // hand_finger_joint_map[finger][i] = motor index for {MCP_AA, MCP_FE,
  // DIP_FE}.
  std::array<std::array<int, 3>, 3> hand_finger_joint_map{{{{0, 1, 2}}, {{3, 4, 5}}, {{6, 7, 8}}}};
  // Per-finger MCP_FE index used by the contact_stop release gate.
  int hand_idx_thumb_cmc_fe{1};
  int hand_idx_index_mcp_fe{4};
  int hand_idx_middle_mcp_fe{7};

  // Force-PI grasp parameters; only consumed when grasp_controller_type ==
  // "force_pi".
  rtc::grasp::GraspParams force_pi_params{};
  std::array<rtc::grasp::FingerConfig, rtc::grasp::kNumGraspFingers> force_pi_fingers{};
  bool has_force_pi_block{false};
};

// Overlay any keys present in `node` onto `cfg`. Missing keys are left
// untouched, so callers can stack defaults -> shared file -> per-controller
// overrides.
void ApplyDemoSharedConfig(const YAML::Node& node, DemoSharedConfig& cfg);

// Load demo_shared.yaml from the integrated_bringup package share dir.
// On error (file missing / parse failure), `cfg` is left unchanged.
//
// `config_variant` mirrors the rt_controller_node `config_variant` ROS
// parameter (e.g. "ur5e_hand", "iiwa7_leap").  Empty → legacy flat layout
// (config/controllers/demo_shared.yaml); non-empty → variant path
// (config/<config_variant>/controllers/demo_shared.yaml).  Per-controller call
// sites forward the value from their LifecycleNode's `config_variant` param.
void LoadDemoSharedYamlFile(DemoSharedConfig& cfg, const std::string& config_variant = "");

// Build (or reset) the GraspController based on `cfg`. Resets to nullptr unless
// grasp_controller_type == "force_pi" and a force-pi block was provided.
void BuildGraspController(const DemoSharedConfig& cfg, double control_rate_hz,
                          std::unique_ptr<rtc::grasp::GraspController>& grasp_controller);

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_DEMO_SHARED_CONFIG_HPP_
