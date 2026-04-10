#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_SHARED_CONFIG_HPP_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_SHARED_CONFIG_HPP_

#include "ur5e_bringup/controllers/virtual_tcp.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <memory>
#include <string>

namespace ur5e_bringup {

// Parameters that DemoJointController and DemoTaskController share.
// Defaults live in config/controllers/demo_shared.yaml; per-controller YAMLs
// may override individual keys via a second ApplyDemoSharedConfig() pass.
struct DemoSharedConfig {
  VirtualTcpConfig vtcp{};
  float grasp_contact_threshold{0.5f};
  float grasp_force_threshold{1.0f};
  int   grasp_min_fingertips{2};

  std::string grasp_controller_type{"contact_stop"};

  // Force-PI grasp parameters; only consumed when grasp_controller_type == "force_pi".
  rtc::grasp::GraspParams force_pi_params{};
  std::array<rtc::grasp::FingerConfig, rtc::grasp::kNumGraspFingers> force_pi_fingers{};
  bool has_force_pi_block{false};
};

// Overlay any keys present in `node` onto `cfg`. Missing keys are left untouched,
// so callers can stack defaults -> shared file -> per-controller overrides.
void ApplyDemoSharedConfig(const YAML::Node & node, DemoSharedConfig & cfg);

// Load demo_shared.yaml from the ur5e_bringup package share dir.
// On error (file missing / parse failure), `cfg` is left unchanged.
void LoadDemoSharedYamlFile(DemoSharedConfig & cfg);

// Build (or reset) the GraspController based on `cfg`. Resets to nullptr unless
// grasp_controller_type == "force_pi" and a force-pi block was provided.
void BuildGraspController(
    const DemoSharedConfig & cfg,
    double control_rate_hz,
    std::unique_ptr<rtc::grasp::GraspController> & grasp_controller);

}  // namespace ur5e_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_SHARED_CONFIG_HPP_
