// ── test_fixture.hpp ─────────────────────────────────────────────────────────
// Shared helpers for MJCF-fixture-based tests.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_TEST_FIXTURE_HPP_
#define RTC_MUJOCO_SIM_TEST_FIXTURE_HPP_

#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <string>

namespace rtc::test {

#ifndef MINIMAL_MJCF_PATH
#error "MINIMAL_MJCF_PATH must be defined by CMake"
#endif

inline MuJoCoSimulator::Config MakeMinimalConfig() {
  MuJoCoSimulator::Config cfg;
  cfg.model_path          = MINIMAL_MJCF_PATH;
  cfg.enable_viewer       = false;
  cfg.sync_timeout_ms     = 10.0;
  cfg.max_rtf             = 0.0;
  cfg.n_substeps          = 1;
  cfg.viewer_refresh_rate = 60.0;
  cfg.use_yaml_servo_gains = false;

  JointGroupConfig g;
  g.name                 = "arm";
  g.command_joint_names  = {"j1", "j2"};
  g.state_joint_names    = {"j1", "j2"};
  g.command_topic        = "/arm/cmd";
  g.state_topic          = "/arm/state";
  g.is_robot             = true;
  cfg.groups.push_back(g);

  return cfg;
}

}  // namespace rtc::test

#endif  // RTC_MUJOCO_SIM_TEST_FIXTURE_HPP_
