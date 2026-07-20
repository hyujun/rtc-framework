// Tier 2 — full-pipeline configuration tests for RtControllerNode.
//
// Drives the real on_configure bring-up with a registry controller whose YAML
// fixture (config/test_fixtures/controllers/rtc_cm_cfg_test.yaml, selected via
// config_variant=test_fixtures) declares an "arm" device group. Covers the
// paths test_on_configure.cpp (group-less controller) cannot reach:
//   - params.cpp: config_variant YAML path, ApplyControllerParamOverrides +
//     SetYamlScalarFromParam (all scalar/array kinds), matched device-timeout
//     parsing, initial_controller found-branch, logging block with log_dir,
//     urdf.closure_path explicit override
//   - device_config.cpp: ParseSubModels/ParseTreeModels, LoadDeviceNameConfigs
//     (names / limits / safe_position / backend / sensor_layout blocks + URDF
//     joint-name validation and joint-limit merge)
//
// URDF cases use the repo-vendored panda model via the installed
// robot_descriptions share dir (ament runtime lookup, ARCH-5 compliant —
// configure-time fixture, same as test_on_configure.cpp).

#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
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

class CmConfigPipelineTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override { PipelineTestController::ResetCaptured(); }

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

// ── Logging block (log_dir + timing CSV) ─────────────────────────────────────

TEST_F(CmConfigPipelineTest, LoggingSetupCreatesTimingCsvInLogDir) {
  const auto log_dir =
      std::filesystem::temp_directory_path() / "rtc_cm_cfg_test_logs" / "session_a";
  std::filesystem::remove_all(log_dir.parent_path());

  auto node = std::make_shared<RtControllerNode>("test_cm_config_pipeline_node");
  node->declare_parameter("enable_logging", true);
  node->declare_parameter("enable_timing_log", true);
  node->declare_parameter("enable_device_log", false);
  node->declare_parameter("control_rate", 200.0);
  node->declare_parameter("config_variant", std::string("test_fixtures"));
  node->declare_parameter("log_dir", log_dir.string());
  node->declare_parameter("max_log_sessions", 2);
  DeclareArmDevice(*node);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_TRUE(std::filesystem::exists(log_dir));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));

  std::filesystem::remove_all(log_dir.parent_path());
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

}  // namespace
}  // namespace rtc
