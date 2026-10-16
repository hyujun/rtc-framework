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

#include <cmath>
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
      {"ur5e_p1b", dir + "/ur5e_p1b/controllers/demo_wbc_controller.yaml"},
      {"iiwa7_leap", dir + "/iiwa7_leap/controllers/demo_wbc_controller.yaml"},
  };
}

// The arm's root_link / tip_link: the first urdf.sub_models entry of the
// profile's system config (_base.yaml when the profile has one, else sim.yaml)
// — the entry the controllers take as the arm.
struct ArmLinks {
  std::string root;
  std::string tip;
};

ArmLinks ShippedArmLinks(const std::string& robot) {
  const std::string dir = std::string(RTC_WBC_CONFIG_DIR) + "/" + robot + "/";
  for (const char* name : {"_base.yaml", "sim.yaml"}) {
    YAML::Node root;
    try {
      root = YAML::LoadFile(dir + name);
    } catch (const YAML::BadFile&) {
      continue;
    }
    const YAML::Node sub_models = root["/**"]["ros__parameters"]["urdf"]["sub_models"];
    if (sub_models && sub_models.IsMap() && sub_models.size() > 0) {
      const YAML::Node arm = sub_models.begin()->second;
      return {arm["root_link"].as<std::string>(), arm["tip_link"].as<std::string>()};
    }
  }
  return {};
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
// misses it does not degrade — the controller throws at configure. Almost every
// controller fixture in this package is an inline YAML string, so without this
// the migration is asserted only by whoever ran it. (test_compliance_task_equivalence
// also reads the shipped files, but it compares two of them to each other rather
// than checking either against the schema.)
//
// Scope is every shipped controller, not just wbc — this file is where shipped-YAML
// parsing already lives, and forking a second lint harness for the same corpus
// would be the duplication P5 exists to prevent. The robot list is explicit for
// the same reason ShippedConfigs() is: no filesystem walk in a unit test.
std::vector<ConfigCase> AllShippedControllerConfigs() {
  const std::string dir = RTC_WBC_CONFIG_DIR;
  std::vector<ConfigCase> out;
  for (const char* robot : {"ur5e_p1a", "ur5e_p1b", "iiwa7_leap"}) {
    for (const char* ctrl : {"demo_joint_controller", "demo_task_controller", "demo_wbc_controller",
                             "demo_compliance_controller"}) {
      out.push_back(
          {std::string(robot) + "/" + ctrl, dir + "/" + robot + "/controllers/" + ctrl + ".yaml"});
    }
  }
  return out;
}

TEST(DemoControllerConfigLint, ArmDofIsPresentAndMatchesSafePositionLength) {
  const auto configs = AllShippedControllerConfigs();
  // 12 = 3 profiles x 4 controllers. This number is not a fact about the tree —
  // it is the tripwire that makes adding a controller stop here: the loop above
  // names its controllers by hand, so a new one is simply absent from the corpus
  // and its shipped YAMLs go unlinted while this file stays green (#469 S2 added
  // the fourth and found exactly that).
  ASSERT_EQ(configs.size(), 12U) << "the shipped controller corpus changed — update this list";

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

      // Every scalar Init() gates on finiteness (#316 D-9). A non-finite value
      // here is boot-fatal — Init throws, clik_enabled_ goes false, and DEC-1 ⓐ
      // turns that into on_configure FAILURE — so the shipped YAML needs a CI
      // signal of its own rather than being caught on the robot. `.nan` /
      // `.inf` are legal YAML scalars, so parsing succeeds and only this check
      // separates them.
      for (const char* key :
           {"damping_sq", "w_task", "w_arm", "w_hand", "v_limit", "anchor_drift_max"}) {
        if (clik[key]) {
          EXPECT_TRUE(std::isfinite(clik[key].as<double>()))
              << cfg.robot << ": clik." << key
              << " must be finite (ClikReferenceGenerator::Init contract)";
        }
      }
    }
  }
}

}  // namespace

// F3 (#779): the device's urdf.root_link / tip_link are the SSoT of the arm's
// base and EE frames. The WBC's SE3 tasks and the MPC models name those frames
// again in their own YAML; a copy that drifts from the device makes the
// controller track one frame while goals are converted into another. The F-2
// gate catches a base_frame mismatch at configure; this catches both frames on
// every shipped profile without a model.
TEST(DemoWbcConfigLint, ArmFramesMatchTheDeviceRootAndTip) {
  for (const auto& cfg : ShippedConfigs()) {
    const ArmLinks arm = ShippedArmLinks(cfg.robot);
    ASSERT_FALSE(arm.root.empty()) << cfg.robot << ": no urdf.sub_models";
    ASSERT_FALSE(arm.tip.empty()) << cfg.robot << ": no urdf.sub_models";

    const YAML::Node tasks = YAML::LoadFile(cfg.path)["demo_wbc_controller"]["tsid"]["tasks"];
    int se3_tasks = 0;
    for (const auto& kv : tasks) {
      const YAML::Node task = kv.second;
      if (!task["type"] || task["type"].as<std::string>() != "se3") {
        continue;
      }
      ++se3_tasks;
      const auto key = kv.first.as<std::string>();
      EXPECT_EQ(task["frame"].as<std::string>(), arm.tip) << cfg.robot << " tsid.tasks." << key;
      EXPECT_EQ(task["base_frame"].as<std::string>(), arm.root)
          << cfg.robot << " tsid.tasks." << key;
    }
    EXPECT_GT(se3_tasks, 0) << cfg.robot << ": no se3 task — nothing was checked";

    const std::string mpc_dir =
        std::string(RTC_WBC_CONFIG_DIR) + "/" + cfg.robot + "/controllers/mpc/";
    for (const char* ocp : {"contact_light.yaml", "contact_rich.yaml"}) {
      const YAML::Node model = YAML::LoadFile(mpc_dir + ocp)["mpc"]["model"];
      ASSERT_TRUE(model) << cfg.robot << " mpc/" << ocp << ": no mpc.model";
      EXPECT_EQ(model["end_effector_frame"].as<std::string>(), arm.tip)
          << cfg.robot << " mpc/" << ocp;
      if (model["base_frame"]) {
        EXPECT_EQ(model["base_frame"].as<std::string>(), arm.root) << cfg.robot << " mpc/" << ocp;
      }
    }
  }
}
