// Tier 2 — full-pipeline configuration tests for RtControllerNode.
//
// Drives the real on_configure bring-up with a registry controller whose YAML
// fixture (config/test_fixtures/controllers/rtc_cm_cfg_test.yaml, selected via
// config_variant=test_fixtures) declares an "arm" device group. Covers the
// paths test_on_configure.cpp (group-less controller) cannot reach:
//   - params.cpp: config_variant YAML path, ApplyControllerParamOverrides +
//     SetYamlScalarFromParam (all scalar/array kinds) and the warning for an
//     override whose path the YAML lacks (issue #574), matched device-timeout
//     parsing, initial_controller found-branch, logging block and the refused
//     log_dir override, urdf.closure_path explicit override, and what a
//     refused configure must tear down before it returns (#935)
//   - device_config.cpp: ParseSubModels/ParseTreeModels, LoadDeviceNameConfigs
//     (names / limits / safe_position / backend / sensor_layout blocks + URDF
//     joint-name validation and joint-limit merge)
//
// URDF cases use the repo-vendored panda model via the installed
// robot_descriptions share dir (ament runtime lookup, ARCH-5 compliant —
// configure-time fixture, same as test_on_configure.cpp).

#include "rclcpp_suite_fixture.hpp"
#include "rt_cm_log_sink.hpp"
#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;

RTC_REGISTER_CONTROLLER(rtc_cm_cfg_test, "", "rtc_controller_manager",
                        std::make_unique<PipelineTestController>())
RTC_REGISTER_DEVICE_BACKEND(cm_pipe_backend, std::make_unique<PipelineStubBackend>())

rclcpp_lifecycle::State StateUnconfigured() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                 "unconfigured");
}

rclcpp_lifecycle::State StateInactive() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

class CmConfigPipelineTest : public RclcppSuiteTest {
 protected:
  void SetUp() override {
    PipelineTestController::ResetCaptured();
    LogSink::Install();
  }

  void TearDown() override { LogSink::Restore(); }

  static std::vector<std::string> OverrideWarnings(const std::vector<std::string>& needles) {
    std::vector<std::string> all{"[param-override]"};
    all.insert(all.end(), needles.begin(), needles.end());
    return LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, all);
  }

  // Node with the test_fixtures config variant active and low-side-effect
  // defaults. Individual tests declare devices.* / urdf.* parameters on top.
  static std::shared_ptr<RtControllerNode> MakeNode() {
    auto node = std::make_shared<RtControllerNode>("test_cm_config_pipeline_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", 200.0);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    return node;
  }

  // devices.arm.* baseline: two panda joints + a complete backend binding.
  static void DeclareArmDevice(RtControllerNode& node) {
    node.declare_parameter("devices.arm.joint_state_names",
                           std::vector<std::string>{"panda_joint1", "panda_joint2"});
    node.declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
    node.declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
    node.declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));
  }

  static const DeviceNameConfig& ArmConfig(const RtControllerNode& node) {
    const auto& cfgs = ControllerLifecycleTestAccess::GetDeviceNameConfigs(node);
    const auto it = cfgs.find("arm");
    EXPECT_NE(it, cfgs.end());
    return it->second;
  }
};

// ── Param overrides: YAML value ← ROS parameter, all scalar/array kinds ──────

TEST_F(CmConfigPipelineTest, ParamOverridesReachControllerYaml) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.gains.kp", 9.5);
  node->declare_parameter("rtc_cm_cfg_test.gains.label", std::string("overridden"));
  node->declare_parameter("rtc_cm_cfg_test.gains.enabled", true);
  node->declare_parameter("rtc_cm_cfg_test.gains.count", 7);
  node->declare_parameter("rtc_cm_cfg_test.gains.vals", std::vector<double>{1.5, 2.5});
  node->declare_parameter("rtc_cm_cfg_test.gains.tags", std::vector<std::string>{"a", "b"});
  node->declare_parameter("rtc_cm_cfg_test.gains.ids", std::vector<int64_t>{3, 4});
  node->declare_parameter("rtc_cm_cfg_test.gains.flags", std::vector<bool>{true, false});
  node->declare_parameter("rtc_cm_cfg_test.deep.a.b", 4.25);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_DOUBLE_EQ(9.5, PipelineTestController::captured_kp);
  EXPECT_EQ("overridden", PipelineTestController::captured_label);
  EXPECT_TRUE(PipelineTestController::captured_enabled);
  EXPECT_EQ(7, PipelineTestController::captured_count);
  EXPECT_EQ((std::vector<double>{1.5, 2.5}), PipelineTestController::captured_vals);
  EXPECT_EQ((std::vector<std::string>{"a", "b"}), PipelineTestController::captured_tags);
  EXPECT_EQ((std::vector<int64_t>{3, 4}), PipelineTestController::captured_ids);
  EXPECT_EQ((std::vector<bool>{true, false}), PipelineTestController::captured_flags);
  EXPECT_DOUBLE_EQ(4.25, PipelineTestController::captured_deep_b);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Param overrides naming a path the YAML lacks (issue #574) ───────────────
//
// The write creates whatever path it is handed, so a key one level off used to
// be written, never read, and the run silently used the shipped value. The
// configure now warns about such a path — and still writes it, because a
// controller may read a key with a built-in default that its YAML omits (this
// fixture's gains.vals is one: absent from the YAML, read by LoadConfig).

TEST_F(CmConfigPipelineTest, AnOverrideOfAPathTheYamlLacksWarnsAndIsStillWritten) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.gains.kp", 9.5);   // in the YAML
  node->declare_parameter("rtc_cm_cfg_test.deep.a.b", 4.25);  // in the YAML, 3 levels
  node->declare_parameter("rtc_cm_cfg_test.gains.vals", std::vector<double>{1.5, 2.5});  // not

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  // Exactly one warning, for the absent path, naming the part that does exist.
  // The presence check sits beside the absence checks on purpose: on its own,
  // "no warning" would also pass with the sink capturing nothing.
  EXPECT_EQ(1U, OverrideWarnings({"'rtc_cm_cfg_test.gains.vals'", "prefix: 'gains'"}).size());
  EXPECT_EQ(1U, OverrideWarnings({}).size()) << "an override of a present path was warned about";

  EXPECT_EQ((std::vector<double>{1.5, 2.5}), PipelineTestController::captured_vals);
  EXPECT_DOUBLE_EQ(9.5, PipelineTestController::captured_kp);
  EXPECT_DOUBLE_EQ(4.25, PipelineTestController::captured_deep_b);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, AShallowOverrideNamesTheKeyItProbablyMeant) {
  // `a.b` for the YAML's `deep.a.b`: the mistake #574 is about. The value never
  // reaches the controller — it keeps the YAML's 0.0 — so the warning is the
  // operator's only signal, and it has to point at the real key.
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.a.b", 4.25);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(1U, OverrideWarnings({"'rtc_cm_cfg_test.a.b'", "prefix: none", "'deep.a.b'"}).size());
  EXPECT_DOUBLE_EQ(0.0, PipelineTestController::captured_deep_b);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, ANodeAnEarlierOverrideCreatedIsNotAnExistingPrefix) {
  // Two siblings under a block the YAML does not have. Overrides are visited in
  // name order, so newblock.x is written before newblock.y is judged; a check
  // made between writes would see the `newblock` map x created and report it
  // as an existing prefix of y. Both are judged on the tree as loaded.
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.newblock.x", 1.0);
  node->declare_parameter("rtc_cm_cfg_test.newblock.y", 2.0);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(1U, OverrideWarnings({"'rtc_cm_cfg_test.newblock.x'", "prefix: none"}).size());
  EXPECT_EQ(1U, OverrideWarnings({"'rtc_cm_cfg_test.newblock.y'", "prefix: none"}).size());

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// An override that cannot be written without breaking the YAML's structure is
// a bad override, and a bad override refuses the configure (#196 D1). Each of
// these used to fail differently: through a scalar it threw a bare
// BadSubscript right after a WARN that called it harmless, through a sequence
// it turned the list into a map, onto a map it replaced every key under it,
// and an override beside its own parent key broke whichever came second.

class CmOverrideRefusalTest : public CmConfigPipelineTest {
 protected:
  static std::vector<std::string> RefusalErrors(const std::vector<std::string>& needles) {
    std::vector<std::string> all{"[param-override]", "cannot be written"};
    all.insert(all.end(), needles.begin(), needles.end());
    return LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, all);
  }
};

TEST_F(CmOverrideRefusalTest, AnOverrideThroughAScalarRefusesAndSaysWhy) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.gains.kp.x", 1.0);

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U, RefusalErrors({"'rtc_cm_cfg_test.gains.kp.x'", "scalar 'gains.kp'"}).size());
  // Not also announced as "written anyway" — the refusal above is non-empty,
  // so this absence is not a dead sink.
  EXPECT_TRUE(OverrideWarnings({"'rtc_cm_cfg_test.gains.kp.x'"}).empty());
}

TEST_F(CmOverrideRefusalTest, AnOverrideThroughASequenceRefuses) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.topics.arm.subscribe.extra", 1.0);

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U, RefusalErrors({"'rtc_cm_cfg_test.topics.arm.subscribe.extra'",
                               "sequence 'topics.arm.subscribe'"})
                    .size());
}

TEST_F(CmOverrideRefusalTest, AnOverrideNamingAMapRefuses) {
  // `gains` for `gains.kp`: the shallow mistake in the other direction. Written,
  // the value would replace kp, label, enabled and count at once.
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.gains", 2.0);

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U, RefusalErrors({"'rtc_cm_cfg_test.gains'", "names the map 'gains'"}).size());
}

TEST_F(CmOverrideRefusalTest, AnOverrideBesideItsOwnParentKeyRefuses) {
  // `newblock-x` sorts between `newblock` and `newblock.x` ('-' < '.'), so the
  // pair is not adjacent in name order and a neighbour-only check misses it.
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.newblock", 1.0);
  node->declare_parameter("rtc_cm_cfg_test.newblock-x", 0.5);
  node->declare_parameter("rtc_cm_cfg_test.newblock.x", 2.0);

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U,
            RefusalErrors({"'rtc_cm_cfg_test.newblock' and 'rtc_cm_cfg_test.newblock.x'"}).size());
}

// Issue #350's profile reaches each controller's OWN node. The launches set it
// on the CM node, and controller LifecycleNodes ignore the process's global
// arguments, so without an explicit forward every profile-keyed activation
// gate read its default — found by dynamic_catching S6 R-1.
TEST_F(CmConfigPipelineTest, LayoutProfileReachesTheControllersOwnNode) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("rt_layout_profile", std::string("mpc_off"));
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ("mpc_off", PipelineTestController::captured_layout_profile);
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, AnUnsetLayoutProfileIsNotInventedOnTheControllerNode) {
  // A bring-up that predates the profile keeps the controller's own default:
  // the CM forwards what the launch set and nothing else.
  auto node = MakeNode();
  DeclareArmDevice(*node);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ("<absent>", PipelineTestController::captured_layout_profile);
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, YamlDefaultsSurviveWithoutOverrides) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  // Values come straight from the fixture YAML.
  EXPECT_DOUBLE_EQ(1.0, PipelineTestController::captured_kp);
  EXPECT_EQ("yaml_default", PipelineTestController::captured_label);
  EXPECT_FALSE(PipelineTestController::captured_enabled);
  EXPECT_EQ(1, PipelineTestController::captured_count);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── `include:` fragments (controller_config_loader) ──────────────────────────
//
// config/test_include/ splits the controller's tree over a main file and two
// fragments. The loader's own rules are pinned in test_controller_config_loader;
// what is pinned here is the bring-up around it — the controller is configured
// from the composed tree, an override reaches a key that lives in a fragment,
// and a broken include refuses the configure.

TEST_F(CmConfigPipelineTest, IncludeFragmentsReachTheControllerAsOneTree) {
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("config_variant", std::string("test_include")));
  DeclareArmDevice(*node);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  // Main file.
  EXPECT_DOUBLE_EQ(1.0, PipelineTestController::captured_kp);
  EXPECT_EQ("from_main", PipelineTestController::captured_label);
  // First fragment — leaves of a map the main file also sets.
  EXPECT_TRUE(PipelineTestController::captured_enabled);
  EXPECT_EQ(3, PipelineTestController::captured_count);
  EXPECT_EQ((std::vector<double>{0.5, 1.0e-4}), PipelineTestController::captured_vals);
  // Second fragment.
  EXPECT_EQ((std::vector<std::string>{"x", "y"}), PipelineTestController::captured_tags);
  EXPECT_DOUBLE_EQ(2.5, PipelineTestController::captured_deep_b);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, ParamOverridesReachAFragmentsKeys) {
  // Overrides are applied to the composed node. Applied to the main file's
  // node before the merge they would either be lost or collide with the
  // fragment's leaf as a duplicate.
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("config_variant", std::string("test_include")));
  DeclareArmDevice(*node);
  node->declare_parameter("rtc_cm_cfg_test.gains.count", 9);  // parts/gains_extra.yaml
  node->declare_parameter("rtc_cm_cfg_test.deep.a.b", 7.5);   // parts/deep.yaml
  node->declare_parameter("rtc_cm_cfg_test.gains.kp", 4.0);   // main file

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(9, PipelineTestController::captured_count);
  EXPECT_DOUBLE_EQ(7.5, PipelineTestController::captured_deep_b);
  EXPECT_DOUBLE_EQ(4.0, PipelineTestController::captured_kp);
  // Untouched leaves keep their file's value.
  EXPECT_TRUE(PipelineTestController::captured_enabled);
  EXPECT_EQ("from_main", PipelineTestController::captured_label);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, AMissingFragmentRefusesConfigure) {
  // This controller tolerates a missing config FILE (defaults). A main file
  // that is there but names a fragment that is not is a different thing: the
  // operator configured the controller and part of that config is gone.
  auto node = MakeNode();
  node->set_parameter(
      rclcpp::Parameter("config_variant", std::string("test_include_missing_fragment")));
  DeclareArmDevice(*node);

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
}

// The control rate must be visible to the controller's FIRST LoadConfig, i.e. the
// one PreConfigure drives. Controllers validate rate-dependent YAML there — filter
// cutoffs against Nyquist, for instance — and handing the rate over only in Pass 2
// made those checks run against GetDefaultDt()'s fallback, rejecting cutoffs that
// are perfectly legal at the configured rate (kMaxControlRateHz is 5000). The
// fixture rate (200 Hz ⇒ 5 ms) differs from the fallback (kDefaultControlRateHz,
// 2 ms), which is what makes the two orderings distinguishable at all.
TEST_F(CmConfigPipelineTest, ControlRateIsVisibleToPreConfigureLoadConfig) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  ASSERT_GT(PipelineTestController::captured_dt_at_load_config, 0.0)
      << "LoadConfig never ran — the observation is vacuous";
  EXPECT_DOUBLE_EQ(1.0 / 200.0, PipelineTestController::captured_dt_at_load_config)
      << "PreConfigure's LoadConfig saw the fallback dt, not the configured rate";

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Device-name config parsing (names / backend / layout / limits) ──────────

TEST_F(CmConfigPipelineTest, DeviceConfigFullParse) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("devices.arm.motor_state_names", std::vector<std::string>{"m1"});
  node->declare_parameter("devices.arm.sensor_names", std::vector<std::string>{"s1", "s2"});
  node->declare_parameter("devices.arm.backend.motor_topic", std::string("/arm/motor"));
  node->declare_parameter("devices.arm.backend.sensor_topic", std::string("/arm/sensor"));
  node->declare_parameter("devices.arm.safe_position", std::vector<double>{0.0, 0.5});
  node->declare_parameter("devices.arm.joint_limits.max_velocity", std::vector<double>{1.0, 1.0});
  node->declare_parameter("devices.arm.sensor_layout.primary_count_per_group", 4);
  node->declare_parameter("devices.arm.sensor_layout.secondary_count_per_group", 2);
  node->declare_parameter("devices.arm.sensor_layout.inference_values_per_group", 3);
  node->declare_parameter("devices.arm.sensor_layout.has_native_contact", true);
  node->declare_parameter("device_timeout_names", std::vector<std::string>{"arm"});
  node->declare_parameter("device_timeout_values", std::vector<double>{150.0});
  node->declare_parameter("initial_controller", std::string("rtc_cm_cfg_test"));

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  const auto& cfg = ArmConfig(*node);
  EXPECT_EQ((std::vector<std::string>{"panda_joint1", "panda_joint2"}), cfg.joint_state_names);
  // joint_command_names omitted → defaults to joint_state_names.
  EXPECT_EQ(cfg.joint_state_names, cfg.joint_command_names);
  EXPECT_EQ((std::vector<std::string>{"m1"}), cfg.motor_state_names);
  EXPECT_EQ((std::vector<std::string>{"s1", "s2"}), cfg.sensor_names);
  EXPECT_EQ((std::vector<double>{0.0, 0.5}), cfg.safe_position);

  ASSERT_TRUE(cfg.backend.has_value());
  EXPECT_EQ("cm_pipe_backend", cfg.backend->type);
  EXPECT_EQ("/arm/state", cfg.backend->state_topic);
  EXPECT_EQ("/arm/cmd", cfg.backend->command_topic);
  EXPECT_EQ("/arm/motor", cfg.backend->motor_topic);
  EXPECT_EQ("/arm/sensor", cfg.backend->sensor_topic);

  ASSERT_TRUE(cfg.sensor_layout.has_value());
  EXPECT_EQ(4, cfg.sensor_layout->primary_count_per_group);
  EXPECT_EQ(2, cfg.sensor_layout->secondary_count_per_group);
  // values_per_group omitted → defaults to primary + secondary.
  EXPECT_EQ(6, cfg.sensor_layout->values_per_group);
  EXPECT_EQ(3, cfg.sensor_layout->inference_values_per_group);
  EXPECT_TRUE(cfg.sensor_layout->has_native_contact);
  EXPECT_FALSE(cfg.sensor_layout->has_native_displacement);

  ASSERT_TRUE(cfg.joint_limits.has_value());
  EXPECT_EQ((std::vector<double>{1.0, 1.0}), cfg.joint_limits->max_velocity);

  // Slot-indexed sensor-layout cache populated for the "arm" slot.
  const auto slot_layout = ControllerLifecycleTestAccess::GetSlotSensorLayout(*node, 0);
  ASSERT_TRUE(slot_layout.has_value());
  EXPECT_EQ(6, slot_layout->values_per_group);

  // Matched device timeout (group + backend.state_topic both present).
  EXPECT_EQ(1u, ControllerLifecycleTestAccess::GetDeviceTimeoutCount(*node));

  // initial_controller resolves to the registered controller (found branch).
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetActiveIdx(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, MalformedDeviceBlocksAreDroppedNotFatal) {
  auto node = MakeNode();
  node->declare_parameter("devices.arm.joint_state_names",
                          std::vector<std::string>{"panda_joint1", "panda_joint2"});
  // Size mismatches → cleared with ERROR log.
  node->declare_parameter("devices.arm.joint_limits.max_velocity",
                          std::vector<double>{1.0, 2.0, 3.0});
  node->declare_parameter("devices.arm.safe_position", std::vector<double>{0.1});
  // Wrong parameter type → ParameterTypeException branch → cleared.
  node->declare_parameter("devices.arm.motor_state_names", 1.25);
  node->declare_parameter("devices.arm.sensor_names", true);
  // Backend block missing state/command topics → binding ignored.
  node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  const auto& cfg = ArmConfig(*node);
  EXPECT_TRUE(cfg.safe_position.empty());
  EXPECT_TRUE(cfg.motor_state_names.empty());
  EXPECT_TRUE(cfg.sensor_names.empty());
  EXPECT_FALSE(cfg.backend.has_value());
  // The mismatched max_velocity array is cleared; no other YAML limit arrays
  // and no URDF are configured, so the limits block stays all-empty.
  if (cfg.joint_limits.has_value()) {
    EXPECT_TRUE(cfg.joint_limits->max_velocity.empty());
  }

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── System model topology (sub_models / tree_models) + URDF validation ───────

TEST_F(CmConfigPipelineTest, SubModelResolvesLinksAndMergesUrdfLimits) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.sub_models.arm.root_link", std::string("panda_link0"));
  node->declare_parameter("urdf.sub_models.arm.tip_link", std::string("panda_link8"));
  // Incomplete sub-model (no tip_link) → skipped by ParseSubModels.
  node->declare_parameter("urdf.sub_models.bad.root_link", std::string("panda_link0"));
  // Loose YAML limits → merged with (tightened to) the URDF bounds.
  node->declare_parameter("devices.arm.joint_limits.max_velocity",
                          std::vector<double>{100.0, 100.0});
  node->declare_parameter("devices.arm.joint_limits.position_lower",
                          std::vector<double>{-100.0, -100.0});
  node->declare_parameter("devices.arm.joint_limits.position_upper",
                          std::vector<double>{100.0, 100.0});

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  const auto& model_cfg = ControllerLifecycleTestAccess::GetSystemModelConfig(*node);
  ASSERT_EQ(1u, model_cfg.sub_models.size());  // "bad" skipped
  EXPECT_EQ("arm", model_cfg.sub_models[0].name);

  const auto& cfg = ArmConfig(*node);
  ASSERT_TRUE(cfg.urdf.has_value());
  // root/tip auto-resolved from the sub_model whose name matches the group.
  EXPECT_EQ("panda_link0", cfg.urdf->root_link);
  EXPECT_EQ("panda_link8", cfg.urdf->tip_link);

  ASSERT_TRUE(cfg.joint_limits.has_value());
  const auto& lim = *cfg.joint_limits;
  ASSERT_EQ(2u, lim.max_velocity.size());
  // Panda URDF bounds are far tighter than ±100 — merge must have shrunk them.
  EXPECT_LT(lim.max_velocity[0], 100.0);
  EXPECT_GT(lim.position_lower[0], -100.0);
  EXPECT_LT(lim.position_upper[0], 100.0);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── urdf.extra_frames (dynamic_catching S2.3a) ──────────────────────────────

TEST_F(CmConfigPipelineTest, ExtraFramesParseAndSurviveReconfigure) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.extra_frames.catch_frame.parent", std::string("panda_hand"));
  node->declare_parameter("urdf.extra_frames.catch_frame.xyz", std::vector<double>{0.0, 0.0, 0.1});
  node->declare_parameter("urdf.extra_frames.catch_frame.rpy",
                          std::vector<double>{3.141592653589793, 0.0, 0.0});
  // Integer lists are accepted (YAML `[0, 0, 0]` arrives as an integer array).
  node->declare_parameter("urdf.extra_frames.marker.parent", std::string("panda_link0"));
  node->declare_parameter("urdf.extra_frames.marker.xyz", std::vector<int64_t>{1, 0, 0});
  node->declare_parameter("urdf.extra_frames.marker.rpy", std::vector<int64_t>{0, 0, 0});
  node->declare_parameter("urdf.extra_frames.marker.provisional", false);

  for (int round = 0; round < 2; ++round) {  // configure → cleanup → configure
    ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured())) << round;
    const auto& model_cfg = ControllerLifecycleTestAccess::GetSystemModelConfig(*node);
    ASSERT_EQ(2u, model_cfg.extra_frames.size()) << round;
    for (const auto& ef : model_cfg.extra_frames) {
      if (ef.name == "catch_frame") {
        EXPECT_EQ("panda_hand", ef.parent);
        EXPECT_DOUBLE_EQ(0.1, ef.xyz.z());
        EXPECT_TRUE(ef.provisional);
      } else {
        EXPECT_EQ("marker", ef.name);
        EXPECT_DOUBLE_EQ(1.0, ef.xyz.x());
        EXPECT_FALSE(ef.provisional);
      }
    }
    EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive())) << round;
  }
}

TEST_F(CmConfigPipelineTest, ExtraFrameWithMissingParentRefusesConfigure) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.extra_frames.catch_frame.parent", std::string("no_such_link"));
  node->declare_parameter("urdf.extra_frames.catch_frame.xyz", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("urdf.extra_frames.catch_frame.rpy", std::vector<double>{0.0, 0.0, 0.0});
  EXPECT_NE(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
}

TEST_F(CmConfigPipelineTest, IncompleteExtraFrameRefusesConfigure) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.extra_frames.catch_frame.parent", std::string("panda_hand"));
  node->declare_parameter("urdf.extra_frames.catch_frame.xyz", std::vector<double>{0.0, 0.0});
  node->declare_parameter("urdf.extra_frames.catch_frame.rpy", std::vector<double>{0.0, 0.0, 0.0});
  EXPECT_NE(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
}

TEST_F(CmConfigPipelineTest, TreeModelResolvesRootAndUrdfOnlyLimits) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.tree_models.arm.root_link", std::string("panda_link0"));
  node->declare_parameter("urdf.tree_models.arm.tip_links",
                          std::vector<std::string>{"panda_link8", "panda_hand"});

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  const auto& model_cfg = ControllerLifecycleTestAccess::GetSystemModelConfig(*node);
  ASSERT_EQ(1u, model_cfg.tree_models.size());
  EXPECT_EQ((std::vector<std::string>{"panda_link8", "panda_hand"}),
            model_cfg.tree_models[0].tip_links);

  const auto& cfg = ArmConfig(*node);
  ASSERT_TRUE(cfg.urdf.has_value());
  // tree_model resolves root only (multiple tips → tip_link left empty).
  EXPECT_EQ("panda_link0", cfg.urdf->root_link);
  EXPECT_TRUE(cfg.urdf->tip_link.empty());

  // No YAML limits → created straight from the URDF.
  ASSERT_TRUE(cfg.joint_limits.has_value());
  ASSERT_EQ(2u, cfg.joint_limits->max_velocity.size());
  EXPECT_GT(cfg.joint_limits->max_velocity[0], 0.0);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, UrdfJointMismatchesAreNonFatal) {
  auto node = MakeNode();
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  // One unknown joint (not-found ERROR branch) + swapped order relative to the
  // URDF (order-mismatch WARN branch exercised in the next test).
  node->declare_parameter("devices.arm.joint_state_names",
                          std::vector<std::string>{"panda_joint1", "no_such_joint"});
  node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
  node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
  node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));
  // root/tip link frame checks: one valid, one unknown (WARN branch).
  node->declare_parameter("devices.arm.urdf.root_link", std::string("panda_link0"));
  node->declare_parameter("devices.arm.urdf.tip_link", std::string("no_such_link"));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(CmConfigPipelineTest, UrdfJointOrderMismatchWarnsButSucceeds) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  // YAML order reversed vs URDF → strictly-increasing index check fails.
  node->set_parameter(rclcpp::Parameter("devices.arm.joint_state_names",
                                        std::vector<std::string>{"panda_joint2", "panda_joint1"}));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Extended-URDF closure_path explicit override ─────────────────────────────

TEST_F(CmConfigPipelineTest, ClosurePathOverrideBindsWhenFileExists) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.extended", true);
  // Explicit override resolved against the URDF's package share dir. Any
  // existing file exercises the exists→bind branch (content is consumed later
  // by PinocchioModelBuilder, whose failure path is non-fatal by design).
  node->declare_parameter("urdf.closure_path", std::string("robots/panda/urdf/panda.urdf"));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_FALSE(ControllerLifecycleTestAccess::GetResolvedClosureYamlPath(*node).empty());
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Timing CSVs across a reconfigure (#932) ─────────────────────────────────
//
// The loggers are members, so every configure re-opens the same instances. A
// second Open() on a still-open stream used to fail and poison it: the WARN
// below fired and every timing row after the first configure was lost while
// IsOpen() still said true. The session comes from the binary's
// RTC_SESSION_DIR (rtc_base_isolate_test_sessions() in CMakeLists).

std::vector<std::string> TimingOpenFailures() {
  return LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {"Failed to open"});
}

std::shared_ptr<RtControllerNode> MakeTimingLogNode() {
  auto node = std::make_shared<RtControllerNode>("test_cm_config_pipeline_node");
  node->declare_parameter("enable_logging", true);
  node->declare_parameter("enable_timing_log", true);
  node->declare_parameter("enable_device_log", false);
  node->declare_parameter("control_rate", 200.0);
  node->declare_parameter("config_variant", std::string("test_fixtures"));
  return node;
}

// {header lines, data rows} of a timing CSV; {0, 0} when it does not exist.
std::pair<int, int> CsvLineCounts(const std::filesystem::path& path) {
  std::pair<int, int> counts{0, 0};
  std::ifstream in(path);
  for (std::string line; std::getline(in, line);) {
    ++(line.rfind("t_wall_ns,", 0) == 0 ? counts.first : counts.second);
  }
  return counts;
}

TEST_F(CmConfigPipelineTest, TimingCsvsReopenAcrossReconfigure) {
  const char* session = std::getenv("RTC_SESSION_DIR");
  ASSERT_NE(nullptr, session) << "this binary must isolate its session dir";
  const auto cm_csv = std::filesystem::path(session) / "timing" / "cm_timing_log.csv";
  // The session is shared by the whole binary; another case may have made the
  // file already.
  const int rows_before = CsvLineCounts(cm_csv).second;

  auto node = MakeTimingLogNode();
  DeclareArmDevice(*node);
  for (int cycle = 0; cycle < 2; ++cycle) {
    ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()))
        << "cycle " << cycle;
    EXPECT_TRUE(ControllerLifecycleTestAccess::CmTimingLogOpen(*node)) << "cycle " << cycle;
    EXPECT_TRUE(ControllerLifecycleTestAccess::RtCallbackTimingLogOpen(*node)) << "cycle " << cycle;
    // Queued, not drained: cleanup must write it before it closes the file.
    ASSERT_TRUE(ControllerLifecycleTestAccess::PushCmTimingSample(*node));
    ASSERT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive())) << "cycle " << cycle;
    EXPECT_FALSE(ControllerLifecycleTestAccess::CmTimingLogOpen(*node)) << "cycle " << cycle;
    EXPECT_FALSE(ControllerLifecycleTestAccess::RtCallbackTimingLogOpen(*node))
        << "cycle " << cycle;
  }
  EXPECT_TRUE(TimingOpenFailures().empty());

  const auto [headers, rows] = CsvLineCounts(cm_csv);
  EXPECT_EQ(1, headers);
  EXPECT_EQ(rows_before + 2, rows) << "one row per configure cycle";
}

// Every test binary of this package writes its session into a private temp
// dir. CM's enable_logging defaults to true, and a configure then writes
// timing CSVs into the resolved session and runs retention on its parent —
// without RTC_SESSION_DIR, the workspace's real logging_data. This pins the
// isolation itself, with the node left at its logging defaults.
TEST_F(CmConfigPipelineTest, ConfigureWritesIntoThisBinarysPrivateSession) {
  const char* raw = std::getenv("RTC_SESSION_DIR");
  ASSERT_NE(nullptr, raw) << "RTC_SESSION_DIR is not set — the session would be the real one";
  const std::filesystem::path session{raw};
  const auto temp = std::filesystem::temp_directory_path();
  ASSERT_EQ(0U, session.string().rfind(temp.string(), 0)) << session << " is not under " << temp;
  // Retention runs on the session's parent: it must be a root of our own, not
  // the shared temp dir.
  EXPECT_EQ("logging_data", session.parent_path().filename());
  EXPECT_NE(temp, session.parent_path());

  auto node = std::make_shared<RtControllerNode>("test_cm_config_pipeline_node");
  node->declare_parameter("control_rate", 200.0);
  node->declare_parameter("config_variant", std::string("test_fixtures"));
  node->declare_parameter("enable_device_log", false);
  DeclareArmDevice(*node);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_TRUE(std::filesystem::exists(session / "timing" / "cm_timing_log.csv"));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// A configure refused after the logging setup returns FAILURE, and FAILURE
// skips on_cleanup — the retry must still open both CSVs.
TEST_F(CmConfigPipelineTest, TimingCsvsReopenAfterAConfigureRefusedPastLogging) {
  auto node = MakeTimingLogNode();
  DeclareArmDevice(*node);
  node->declare_parameter("use_sim_time_sync", true);
  node->declare_parameter("sim_sync_tick_devices", std::vector<std::string>{"no_such_group"});

  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  node->set_parameter(rclcpp::Parameter("sim_sync_tick_devices", std::vector<std::string>{"arm"}));
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_TRUE(ControllerLifecycleTestAccess::CmTimingLogOpen(*node));
  EXPECT_TRUE(ControllerLifecycleTestAccess::RtCallbackTimingLogOpen(*node));
  EXPECT_TRUE(TimingOpenFailures().empty());
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── A refused configure cleans up what it built (#935) ──────────────────────
//
// FAILURE parks CM in Unconfigured and skips on_cleanup, so the refusal itself
// is the last point at which the controllers it created can be torn down.
// Past Pass 3 each one holds what its on_configure allocated; dropping it
// without on_cleanup leaves that to the destructor, and the device liveness
// entries the attempt registered used to carry into the retry.
TEST_F(CmConfigPipelineTest, AConfigureRefusedPastPass3CleansUpItsControllers) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  node->declare_parameter("use_sim_time_sync", true);
  node->declare_parameter("sim_sync_tick_devices", std::vector<std::string>{"no_such_group"});

  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  ASSERT_EQ(1, PipelineTestController::instances_created.load());
  EXPECT_EQ(1, PipelineTestController::cleanup_count.load())
      << "the refused attempt's controller was dropped without on_cleanup";
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetDeviceTimeoutCount(*node));

  node->set_parameter(rclcpp::Parameter("sim_sync_tick_devices", std::vector<std::string>{"arm"}));
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U, ControllerLifecycleTestAccess::GetDeviceTimeoutCount(*node))
      << "one liveness entry per device group, not one per configure attempt";
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// The Pass-3 checkpoint itself: a controller whose on_configure failed is the
// one most likely to hold half of what it allocated.
TEST_F(CmConfigPipelineTest, AFailedControllerConfigureCleansUpTheController) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  PipelineTestController::fail_on_configure.store(true);

  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  ASSERT_EQ(1, PipelineTestController::instances_created.load());
  EXPECT_EQ(1, PipelineTestController::cleanup_count.load());
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetDeviceTimeoutCount(*node));
}

// ── Configure after the dedicated executors took the node (#935) ───────────
//
// RtControllerMain hands the node's callback groups and the controller nodes
// to its dedicated executors once, after the first activation. A configure
// after that builds new groups and new controller nodes none of them will ever
// serve (and shutdown then fails to detach the controller nodes), so it is
// refused, and the operator is told to restart instead.
TEST_F(CmConfigPipelineTest, ConfigureAfterTheDedicatedExecutorsAttachedIsRefused) {
  auto node = MakeNode();
  DeclareArmDevice(*node);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  node->MarkDedicatedExecutorsAttached();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
  ASSERT_EQ(1, PipelineTestController::instances_created.load());

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1, PipelineTestController::instances_created.load())
      << "the refused configure built a controller";
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  EXPECT_EQ(1U, LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, {"Restart the process"}).size());
}

// ── A leftover log_dir override (#933) ──────────────────────────────────────
//
// log_dir used to send CM's logs to an explicit dir instead of the session.
// It is gone; an override still carrying it is refused, not silently ignored,
// so the operator learns the logs now follow RTC_SESSION_DIR. The accepted
// counterpart is ConfigureWritesIntoThisBinarysPrivateSession — a statically
// typed parameter cannot be undeclared, so this node cannot be retried.
TEST_F(CmConfigPipelineTest, ALogDirOverrideRefusesConfigureAndWritesNothingThere) {
  const char* session = std::getenv("RTC_SESSION_DIR");
  ASSERT_NE(nullptr, session) << "this binary must isolate its session dir";
  // The binary's own temp root, so a regression that honours log_dir writes
  // nowhere shared.
  const auto root = std::filesystem::path(session).parent_path().parent_path();
  const auto temp = std::filesystem::temp_directory_path();
  ASSERT_EQ(0U, root.string().rfind(temp.string(), 0)) << root << " is not under " << temp;
  ASSERT_NE(temp, root);
  const auto log_dir = root / "log_dir_override";
  std::filesystem::remove_all(log_dir);

  auto node = MakeTimingLogNode();
  DeclareArmDevice(*node);
  node->declare_parameter("log_dir", log_dir.string());

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U,
            LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, {"'log_dir'", "RTC_SESSION_DIR"}).size());
  EXPECT_FALSE(std::filesystem::exists(log_dir));
}

// ── Device config beyond the RT path's fixed capacity (issue #196 §2) ────────

TEST_F(CmConfigPipelineTest, DeviceDeclaringMoreJointsThanCapacityRefusesConfigure) {
  // Target ingress reorders into a kMaxDeviceChannels-wide buffer indexed by
  // position in joint_state_names, so a longer list cannot be represented.
  // This used to be accepted, leaving the reorder to write past the buffer.
  auto node = MakeNode();
  std::vector<std::string> too_many;
  too_many.reserve(static_cast<std::size_t>(kMaxDeviceChannels) + 1);
  for (int i = 0; i <= kMaxDeviceChannels; ++i) {
    too_many.push_back("j" + std::to_string(i));
  }
  node->declare_parameter("devices.arm.joint_state_names", too_many);
  node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
  node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
  node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
}

TEST_F(CmConfigPipelineTest, DeviceAtExactlyCapacityConfigures) {
  // The bound is inclusive — exactly kMaxDeviceChannels joints still fit.
  auto node = MakeNode();
  std::vector<std::string> exact;
  exact.reserve(static_cast<std::size_t>(kMaxDeviceChannels));
  for (int i = 0; i < kMaxDeviceChannels; ++i) {
    exact.push_back("j" + std::to_string(i));
  }
  node->declare_parameter("devices.arm.joint_state_names", exact);
  node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
  node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
  node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── sensor_layout strides beyond the sensor lane's capacity (issue #885) ────
//
// The backend sensor lane writes group g at g * values_per_group into a
// kMaxSensorChannels-wide buffer and at g * inference_values_per_group into a
// kMaxInferenceValues-wide one, for up to kMaxSensorGroups groups — so
// values_per_group <= 16 and inference_values_per_group <= 8. These used to be
// narrowed to int and accepted unchecked.

struct SensorLayoutParams {
  int64_t primary;
  int64_t secondary;
  std::optional<int64_t> values_per_group;  // nullopt → omitted (primary + secondary)
  int64_t inference_values_per_group;
};

void DeclareSensorLayout(RtControllerNode& node, const SensorLayoutParams& p) {
  node.declare_parameter("devices.arm.sensor_layout.primary_count_per_group", p.primary);
  node.declare_parameter("devices.arm.sensor_layout.secondary_count_per_group", p.secondary);
  if (p.values_per_group) {
    node.declare_parameter("devices.arm.sensor_layout.values_per_group", *p.values_per_group);
  }
  node.declare_parameter("devices.arm.sensor_layout.inference_values_per_group",
                         p.inference_values_per_group);
}

TEST_F(CmConfigPipelineTest, SensorLayoutOutOfRangeRefusesConfigure) {
  const std::vector<std::pair<std::string, SensorLayoutParams>> cases = {
      {"negative primary", {-1, 3, 11, 7}},
      {"primary + secondary over the stride", {8, 3, 10, 7}},
      {"stride 17 over 128 / 8", {8, 3, 17, 7}},
      {"inference stride 9 over 64 / 8", {8, 3, 11, 9}},
      {"negative inference stride", {8, 3, 11, -1}},
      // 2^32 + 11 narrows to 11 through int — it must be refused, not wrapped.
      {"stride that wraps to 11", {8, 3, (int64_t{1} << 32) + 11, 7}},
      {"omitted stride defaulting to 17", {9, 8, std::nullopt, 7}},
  };
  for (const auto& [label, params] : cases) {
    SCOPED_TRACE(label);
    LogSink::Install();  // clears the previous case's lines
    auto node = MakeNode();
    DeclareArmDevice(*node);
    DeclareSensorLayout(*node, params);

    EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
    EXPECT_EQ(1U,
              LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, {"[arm] sensor_layout", "out of range"})
                  .size());
  }
}

TEST_F(CmConfigPipelineTest, SensorLayoutAtCapacityConfigures) {
  const std::vector<std::pair<std::string, SensorLayoutParams>> cases = {
      // Both bounds are inclusive.
      {"stride 16 and inference stride 8", {8, 3, 16, 8}},
      // The shipped fingertip hand layout (8 barometer + 3 ToF, 7 inference).
      {"shipped hand layout", {8, 3, 11, 7}},
  };
  for (const auto& [label, params] : cases) {
    SCOPED_TRACE(label);
    auto node = MakeNode();
    DeclareArmDevice(*node);
    DeclareSensorLayout(*node, params);

    ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
    const auto& layout = ArmConfig(*node).sensor_layout;
    ASSERT_TRUE(layout.has_value());
    EXPECT_EQ(params.primary, layout->primary_count_per_group);
    EXPECT_EQ(params.secondary, layout->secondary_count_per_group);
    EXPECT_EQ(*params.values_per_group, layout->values_per_group);
    EXPECT_EQ(params.inference_values_per_group, layout->inference_values_per_group);
    EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
  }
}

}  // namespace
}  // namespace rtc
