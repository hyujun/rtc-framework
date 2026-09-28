// ── test_fixture.hpp ─────────────────────────────────────────────────────────
// Shared helpers for MJCF-fixture-based tests.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_TEST_FIXTURE_HPP_
#define RTC_MUJOCO_SIM_TEST_FIXTURE_HPP_

#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "sim_config_fixture.hpp"

#include <string>

namespace rtc::test {

#ifndef MINIMAL_MJCF_PATH
#error "MINIMAL_MJCF_PATH must be defined by CMake"
#endif

inline MuJoCoSimulator::Config MakeMinimalConfig() {
  auto cfg = HeadlessConfig(MINIMAL_MJCF_PATH, 10.0);
  cfg.groups.push_back(RobotGroup("arm", {"j1", "j2"}));
  return cfg;
}

/// Two robot groups over the minimal scene's two joints — the shape every
/// shipped arm+hand scene has, and the one a single-group fixture cannot tell
/// "wait for the primary" from "wait for every group" with (issue #566).
/// A long sync timeout so a step that is waiting is visibly not stepping.
inline MuJoCoSimulator::Config MakeTwoGroupConfig() {
  auto cfg = HeadlessConfig(MINIMAL_MJCF_PATH, 2000.0);
  cfg.groups.push_back(RobotGroup("arm", {"j1"}));
  cfg.groups.push_back(RobotGroup("hand", {"j2"}));
  return cfg;
}

}  // namespace rtc::test

#endif  // RTC_MUJOCO_SIM_TEST_FIXTURE_HPP_
