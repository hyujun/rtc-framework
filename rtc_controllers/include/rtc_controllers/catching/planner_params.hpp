// ── Planner thread parameters (dynamic_catching S6, L3 §6) ──────────────────
//
// The `catching.planner.*` keys the planner THREAD reads. Sibling of
// `catching_params.hpp` (the G0-C validator subset) and
// `catch_pose_ik_params.hpp` (`planner.ik.*` / `planner.catchability.*`), not
// part of either: those two describe WHAT a catch pose is, this one describes
// how the thread that searches for one runs. Non-RT (`on_configure`).
//
// The keys grow with the steps that consume them — S6-A reads the four below;
// the candidate grid, switching, scoring and freeze keys join in S6-B/S6-C in
// the commit that first reads them, never before (a parsed key nothing reads is
// a key nobody notices is wrong).
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"  // kMaxPlanNv

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>

namespace rtc::catching {

/// Range bounds (L3 §6). Exposed so the tests and the doc table cite the same
/// numbers the parser enforces.
inline constexpr double kPlannerWakeTimeoutMinS = 0.005;
inline constexpr double kPlannerWakeTimeoutMaxS = 0.5;
inline constexpr double kPlannerBudgetMinS = 0.001;
inline constexpr double kPlannerBudgetMaxS = 0.05;

struct PlannerParams {
  /// `planner.enabled` — spawn the planner thread at activation (S6-A).
  /// Default false: a profile that says nothing gets no thread.
  bool enabled{false};

  /// `planner.wake_timeout_s` [s] — the longest the thread sleeps without a
  /// new trajectory (decision H). It doubles as the `PeriodicRtThread`
  /// period, which the base requires to be positive.
  double wake_timeout_s{0.05};

  /// `planner.budget_s` [s] — one cycle's compute budget (R-2: 0.020, with a
  /// candidate pre-filter; L3 §6).
  double budget_s{0.020};

  /// `planner.wait_pose` [rad] — the IK seed and wait pose, ARM joint order
  /// (decision L, provisional). `wait_pose_n` entries are meaningful; 0 means
  /// the key is absent. The binding checks `wait_pose_n` against the arm's
  /// width at configure — this parser does not know the robot.
  std::array<double, kMaxPlanNv> wait_pose{};
  std::int32_t wait_pose_n{0};
};

/// Parse `planner.*` from the `catching:` tree root (the same node
/// ParseCatchingParams takes). An absent `planner:` section yields the
/// defaults above. Throws `std::invalid_argument` (and only that) on: a
/// `planner` that is not a map; `enabled` that is not a bool; a
/// `wake_timeout_s` / `budget_s` that is not a finite number or lies outside
/// [kPlanner*Min, kPlanner*Max]; a `wait_pose` that is not a sequence of
/// finite numbers, is empty, or is longer than `kMaxPlanNv`.
[[nodiscard]] PlannerParams ParsePlannerParams(const YAML::Node& catching);

}  // namespace rtc::catching
