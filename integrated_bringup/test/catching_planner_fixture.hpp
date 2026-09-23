#pragma once

// ── Fixtures for the S6-A planner tests ─────────────────────────────────────
//
// A model-free catching profile that CONFIGURES in sim (every consumed key
// resolved, simulator backends) with the planner switchable, plus `/proc`
// helpers for finding threads by name. The profile is the same shape as
// test_demo_catching_controller.cpp's MinimalYaml; it is restated here rather
// than shared because that suite's copy is pinned by its own cases, and a
// planner key added there would change what they configure.

#include "integrated_bringup/controllers/demo_catching_controller.hpp"

#include <sched.h>
#include <sys/types.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace integrated_bringup::testfx {

inline std::string PlannerMinimalYaml(bool planner_enabled, bool oracle_enabled = false) {
  return std::string(R"(
command_type: "position"
diagnostic:
  hand_step: false
  oracle_plan:
    enabled: )") +
         (oracle_enabled ? "true" : "false") + R"(
    p_c: [0.5, 0.1, 0.4]
catching:
  io:
    expected_frame: "world"
    n_min: 7
    t_stale: 0.2
    future_tol: 0.01
    horizon_min: 0.3
    track:
      eval_offset: 0.05
  prediction:
    dt_expected: 0.05
  sim:
    io:
      future_tol: 0.2
  reference:
    provisional: true
    omega: 10.0
    zeta: 1.0
    v_max: 2.0
    a_max: 15.0
  joint_cmd:
    K_p: 20.0
    K_a: 8.0
    K_n: 1.0
    w_task: 1.0
    w_a: 0.5
    w_arm: 0.01
    w_smooth: 0.001
    damping_sq: 0.0001
    qp:
      max_iter: 20
    lag:
      T_arm: 0.0
  supervisor:
    track_err_abort: 0.3
    n_qp: 3
  planner:
    enabled: )" +
         (planner_enabled ? "true" : "false") + R"(
    wake_timeout_s: 0.02
    budget_s: 0.02
    wait_pose: [0.0, -1.0, 1.0, -1.5, -1.5, 0.0]
    # The S6-B decision values: without them an enabled planner parks. This
    # fixture has no system model, so the search stays the stub either way.
    sub_model: "arm_catch"
    freeze: {T_freeze: 0.36}
    hand: {d_eff: 0.28, r_cap: 0.024}
    workspace: {catch_box: {min: [-2.0, -2.0, -2.0], max: [2.0, 2.0, 2.0]}}
  robot:
    arm:
      limit_margin: 0.05
    hand:
      provisional: true
      rho_eps: 0.02
      q_open:  [0.0, 0.0, 0.0, 0.0]
      q_pre:   [0.1, 0.1, 0.1, 0.1]
      q_close: [0.6, 0.6, 0.6, 0.6]
      caging_mask: [true, true, true, true]
      eta_close: 0.9
      T_close_e2e: 0.28
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
)";
}

inline std::map<std::string, rtc::DeviceNameConfig> PlannerSimDevices() {
  std::map<std::string, rtc::DeviceNameConfig> configs;
  rtc::DeviceBackendBinding sim;
  sim.type = integrated_bringup::kCatchingSimBackendType;

  rtc::DeviceNameConfig arm;
  arm.device_name = "arm";
  arm.joint_state_names = {"a0", "a1", "a2", "a3", "a4", "a5"};
  arm.backend = sim;
  configs["arm"] = arm;

  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  hand.joint_state_names = {"h0", "h1", "h2", "h3"};
  rtc::DeviceJointLimits limits;
  limits.position_lower = {-1.0, -1.0, -1.0, -1.0};
  limits.position_upper = {1.0, 1.0, 1.0, 1.0};
  hand.joint_limits = limits;
  hand.backend = sim;
  configs["hand"] = hand;
  return configs;
}

// ── /proc thread helpers (Linux) ────────────────────────────────────────────

/// TIDs of this process whose `comm` is exactly `name`.
inline std::vector<pid_t> ThreadsNamed(const std::string& name) {
  std::vector<pid_t> out;
  for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) {
    std::ifstream comm(entry.path() / "comm");
    std::string value;
    std::getline(comm, value);
    if (value == name) {
      out.push_back(static_cast<pid_t>(std::stol(entry.path().filename().string())));
    }
  }
  return out;
}

/// Wait until exactly `count` threads carry `name` (the loop names itself on
/// entry, a moment after Start returns). Returns the final list.
inline std::vector<pid_t> WaitForThreadsNamed(
    const std::string& name, std::size_t count,
    std::chrono::milliseconds limit = std::chrono::milliseconds(2000)) {
  const auto deadline = std::chrono::steady_clock::now() + limit;
  auto tids = ThreadsNamed(name);
  while (tids.size() != count && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    tids = ThreadsNamed(name);
  }
  return tids;
}

/// The logical CPUs `tid` may run on, from sched_getaffinity.
inline std::vector<int> AllowedCpus(pid_t tid) {
  cpu_set_t set;
  CPU_ZERO(&set);
  std::vector<int> out;
  if (sched_getaffinity(tid, sizeof(set), &set) != 0) {
    return out;
  }
  for (int c = 0; c < CPU_SETSIZE; ++c) {
    if (CPU_ISSET(c, &set)) {
      out.push_back(c);
    }
  }
  return out;
}

}  // namespace integrated_bringup::testfx
