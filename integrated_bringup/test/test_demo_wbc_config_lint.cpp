// ─────────────────────────────────────────────────────────────────────────────
// Shipped demo_wbc_controller.yaml lint (C-0.2)
//
// SE3Task's runtime identity comes from its inner `name:` field (default "se3"),
// NOT the YAML map key. Every lookup that drives the SE3 task — phase_presets,
// the per-tick reference push GetTask("se3_tcp"), and se3_task_active_in_phase —
// keys on the map key. If a task block omits `name:` (so it resolves to "se3")
// while its map key is something else, GetTask returns null: the task is never
// deactivated by a preset and never receives a reference, so it tracks identity
// and fights every phase.
//
// This is a config invariant, so it is checked by parsing the shipped YAML
// directly (no model / lifecycle needed). RTC_WBC_CONFIG_DIR is injected by
// CMake and points at integrated_bringup/config.
// ─────────────────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace {

#ifndef RTC_WBC_CONFIG_DIR
#error "RTC_WBC_CONFIG_DIR must be defined by CMake"
#endif

struct ConfigCase {
  std::string robot;
  std::string path;
};

std::vector<ConfigCase> ShippedConfigs() {
  const std::string dir = RTC_WBC_CONFIG_DIR;
  return {
      {"ur5e_p1a", dir + "/ur5e_p1a/controllers/demo_wbc_controller.yaml"},
      {"iiwa7_leap", dir + "/iiwa7_leap/controllers/demo_wbc_controller.yaml"},
  };
}

// Resolve a task block's effective SE3Task identity, mirroring SE3Task::Init:
// the `name:` field if present, otherwise the default "se3".
std::string ResolvedName(const YAML::Node& task_cfg) {
  if (task_cfg["name"]) {
    return task_cfg["name"].as<std::string>();
  }
  return "se3";
}

// For every `type: se3` entry under demo_wbc_controller.tsid.tasks, the resolved
// name must equal the map key, or GetTask(key) returns null at runtime.
TEST(DemoWbcConfigLint, Se3TaskNameMatchesMapKey) {
  for (const auto& cfg : ShippedConfigs()) {
    YAML::Node root = YAML::LoadFile(cfg.path);
    ASSERT_TRUE(root["demo_wbc_controller"]) << cfg.robot << ": missing top-level key";
    const YAML::Node tasks = root["demo_wbc_controller"]["tsid"]["tasks"];
    ASSERT_TRUE(tasks && tasks.IsMap()) << cfg.robot << ": tsid.tasks missing or not a map";

    bool saw_se3 = false;
    for (auto it = tasks.begin(); it != tasks.end(); ++it) {
      const auto key = it->first.as<std::string>();
      const YAML::Node task_cfg = it->second;
      if (task_cfg["type"] && task_cfg["type"].as<std::string>() == "se3") {
        saw_se3 = true;
        EXPECT_EQ(ResolvedName(task_cfg), key)
            << cfg.robot << ": se3 task '" << key << "' resolves to name '"
            << ResolvedName(task_cfg) << "'. Add `name: \"" << key << "\"` to the block.";
      }
    }
    EXPECT_TRUE(saw_se3) << cfg.robot << ": expected at least one se3 task in tsid.tasks";
  }
}

// ── #340: the shipped `arm_dof` migration, pinned ────────────────────────────
//
// `arm_dof` became a required key with no fallback, so a shipped YAML that
// misses it does not degrade — the controller throws at configure. Nothing else
// in the suite reads the shipped controller files (every controller fixture is
// an inline YAML string), so without this the 9-file migration is asserted only
// by whoever ran the migration.
//
// Scope is all three controllers, not just wbc — this file is where shipped-YAML
// parsing already lives, and forking a second lint harness for the same corpus
// would be the duplication P5 exists to prevent. The robot list is explicit for
// the same reason ShippedConfigs() is: no filesystem walk in a unit test.
std::vector<ConfigCase> AllShippedControllerConfigs() {
  const std::string dir = RTC_WBC_CONFIG_DIR;
  std::vector<ConfigCase> out;
  for (const char* robot : {"ur5e_p1a", "ur5e_p1b", "iiwa7_leap"}) {
    for (const char* ctrl :
         {"demo_joint_controller", "demo_task_controller", "demo_wbc_controller"}) {
      out.push_back(
          {std::string(robot) + "/" + ctrl, dir + "/" + robot + "/controllers/" + ctrl + ".yaml"});
    }
  }
  return out;
}

TEST(DemoControllerConfigLint, ArmDofIsPresentAndMatchesSafePositionLength) {
  const auto configs = AllShippedControllerConfigs();
  ASSERT_EQ(configs.size(), 9U) << "the shipped controller corpus changed — update this list";

  for (const auto& cfg : configs) {
    YAML::Node root = YAML::LoadFile(cfg.path);
    const auto key = cfg.robot.substr(cfg.robot.find('/') + 1);
    const YAML::Node ctrl = root[key];
    ASSERT_TRUE(ctrl) << cfg.robot << ": missing top-level key '" << key << "'";

    ASSERT_TRUE(ctrl["arm_dof"]) << cfg.robot
                                 << ": required key 'arm_dof' is missing — this config throws at "
                                    "configure (#340)";
    const int arm_dof = ctrl["arm_dof"].as<int>();
    EXPECT_GT(arm_dof, 0) << cfg.robot;

    const YAML::Node safe = ctrl["estop"]["arm_safe_position"];
    ASSERT_TRUE(safe && safe.IsSequence()) << cfg.robot << ": estop.arm_safe_position missing";
    EXPECT_EQ(static_cast<int>(safe.size()), arm_dof)
        << cfg.robot << ": estop.arm_safe_position length " << safe.size() << " != arm_dof "
        << arm_dof << " — LoadConfig throws on this";
  }
}

// Cross-check: every task key referenced in a phase_preset must be defined under
// tsid.tasks (by map key). Catches preset typos that would silently no-op.
TEST(DemoWbcConfigLint, PhasePresetTaskKeysExist) {
  for (const auto& cfg : ShippedConfigs()) {
    YAML::Node root = YAML::LoadFile(cfg.path);
    const YAML::Node tsid = root["demo_wbc_controller"]["tsid"];
    ASSERT_TRUE(tsid) << cfg.robot << ": missing tsid block";
    const YAML::Node tasks = tsid["tasks"];
    const YAML::Node presets = tsid["phase_presets"];
    if (!presets || !presets.IsMap()) {
      continue;  // presets are optional
    }
    for (auto pit = presets.begin(); pit != presets.end(); ++pit) {
      const auto phase = pit->first.as<std::string>();
      const YAML::Node ptasks = pit->second["tasks"];
      if (!ptasks || !ptasks.IsMap()) {
        continue;
      }
      for (auto tit = ptasks.begin(); tit != ptasks.end(); ++tit) {
        const auto task_key = tit->first.as<std::string>();
        EXPECT_TRUE(static_cast<bool>(tasks[task_key]))
            << cfg.robot << ": phase '" << phase << "' references undefined task '" << task_key
            << "'";
      }
    }
  }
}

// Stage C-2: when a `clik` block is present (CLIK-QP is the position backbone)
// its structural keys must be well-formed (damping_sq > 0). A typo here would
// silently fall back to defaults at runtime.
TEST(DemoWbcConfigLint, ClikBlockWellFormed) {
  for (const auto& cfg : ShippedConfigs()) {
    YAML::Node root = YAML::LoadFile(cfg.path);
    const YAML::Node wbc = root["demo_wbc_controller"];
    ASSERT_TRUE(wbc) << cfg.robot << ": missing top-level key";

    const YAML::Node clik = wbc["clik"];
    if (clik && clik.IsMap()) {
      ASSERT_TRUE(clik["damping_sq"]) << cfg.robot << ": clik block missing damping_sq";
      EXPECT_GT(clik["damping_sq"].as<double>(), 0.0)
          << cfg.robot << ": clik.damping_sq must be > 0 (ClikReferenceGenerator::Init contract)";
      if (clik["v_limit"]) {
        EXPECT_NO_THROW((void)clik["v_limit"].as<double>()) << cfg.robot;
      }
    }
  }
}

}  // namespace
