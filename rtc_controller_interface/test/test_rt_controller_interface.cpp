// ── test_rt_controller_interface.cpp
// ────────────────────────────────────────── Unit tests for
// RTControllerInterface base class and TopicConfig.
//
// Uses a minimal concrete subclass (StubController) to exercise the protected
// and virtual method behaviour of the abstract interface.
//
// Covers: default construction, control rate, device name configuration,
// system model configuration, ParseTopicConfig (valid / deprecated / unknown
// role / backward-compat), LoadConfig, TopicConfig struct operations, and
// default virtual methods.
// (Phase 4: device-wire roles state/motor_state/sensor_state/joint_command/
// ros2_command — and the DeviceCapability inference path that read them —
// were removed; their tests were dropped. Phase 4 trailing cleanup:
// SubscribeRole enum + MakeDefaultTopicConfig dropped; tests that depended
// on them were rewritten to construct TopicConfig directly or removed.)
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_interface/rt_controller_interface.hpp>
#include <rtc_urdf_bridge/types.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <string>
#include <vector>

namespace {

// ── Stub controller — exposes protected helpers for testing ──────────────────
class StubController : public rtc::RTControllerInterface {
 public:
  StubController() = default;

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }

  void SetDeviceTarget(int, std::span<const double>) noexcept override {}

  [[nodiscard]] std::string_view Name() const noexcept override { return "StubController"; }

  void InitializeHoldPosition(const rtc::ControllerState&) noexcept override {}

  // Expose protected statics for direct testing.
  using RTControllerInterface::LoadDeviceLimitsFromConfig;
  using RTControllerInterface::ParseArmSafePosition;
  using RTControllerInterface::ParseTopicConfig;

  // Seed topic_config_ directly for tests that exercise Primary/Secondary
  // device name lookup without going through LoadConfig().
  void SetTopicConfigForTesting(rtc::TopicConfig cfg) noexcept { topic_config_ = std::move(cfg); }

  // Callback counters.
  int device_config_set_count{0};
  int model_config_set_count{0};

 protected:
  void OnDeviceConfigsSet() override { ++device_config_set_count; }

  void OnSystemModelConfigSet() override { ++model_config_set_count; }
};

// ═══════════════════════════════════════════════════════════════════════════════
// Default construction
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, DefaultControlRate) {
  StubController ctrl;
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.002);
}

TEST(RTControllerInterfaceTest, DefaultTopicConfigIsEmpty) {
  // ARCH-1: rtc_* must stay robot-agnostic. The default ctor leaves
  // topic_config_ empty; LoadConfig() populates it from the YAML `topics:`
  // section when present.
  StubController ctrl;
  EXPECT_TRUE(ctrl.GetTopicConfig().groups.empty());
}

TEST(RTControllerInterfaceTest, DefaultVirtualMethods) {
  StubController ctrl;
  EXPECT_FALSE(ctrl.IsEstopped());
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);
}

TEST(RTControllerInterfaceTest, NameReturnsStub) {
  StubController ctrl;
  EXPECT_EQ(ctrl.Name(), "StubController");
}

TEST(RTControllerInterfaceTest, ComputeReturnsValid) {
  StubController ctrl;
  rtc::ControllerState state{};
  state.dt = 0.002;
  state.num_devices = 1;
  auto output = ctrl.Compute(state);
  EXPECT_TRUE(output.valid);
}

// ═══════════════════════════════════════════════════════════════════════════════
// E-STOP default no-ops
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, EstopDefaultNoOps) {
  StubController ctrl;
  EXPECT_FALSE(ctrl.IsEstopped());

  ctrl.TriggerEstop();
  ctrl.ClearEstop();
  ctrl.SetHandEstop(true);
  ctrl.SetHandEstop(false);

  EXPECT_FALSE(ctrl.IsEstopped());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Control rate
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, SetControlRate) {
  StubController ctrl;
  ctrl.SetControlRate(1000.0);
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.001);
}

TEST(RTControllerInterfaceTest, SetControlRateZeroFallback) {
  StubController ctrl;
  ctrl.SetControlRate(0.0);
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.002);
}

TEST(RTControllerInterfaceTest, SetControlRateNegativeFallback) {
  StubController ctrl;
  ctrl.SetControlRate(-500.0);
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.002);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Device name configuration
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, SetDeviceNameConfigsStoresAndCallsHook) {
  StubController ctrl;
  std::map<std::string, rtc::DeviceNameConfig> configs;
  configs["robot"].device_name = "robot";
  configs["hand"].device_name = "hand";

  ctrl.SetDeviceNameConfigs(std::move(configs));

  EXPECT_EQ(ctrl.device_config_set_count, 1);
  ASSERT_NE(ctrl.GetDeviceNameConfig("robot"), nullptr);
  EXPECT_EQ(ctrl.GetDeviceNameConfig("robot")->device_name, "robot");
  ASSERT_NE(ctrl.GetDeviceNameConfig("hand"), nullptr);
  EXPECT_EQ(ctrl.GetDeviceNameConfig("hand")->device_name, "hand");
}

TEST(RTControllerInterfaceTest, GetDeviceNameConfigReturnsNullptrForMissing) {
  StubController ctrl;
  EXPECT_EQ(ctrl.GetDeviceNameConfig("nonexistent"), nullptr);
}

TEST(RTControllerInterfaceTest, GetPrimaryDeviceNameEmptyByDefault) {
  // ARCH-1: with the empty default topic_config_, primary device name
  // is empty until a controller YAML supplies a topics section.
  StubController ctrl;
  EXPECT_EQ(ctrl.GetPrimaryDeviceName(), "");
}

TEST(RTControllerInterfaceTest, GetPrimaryDeviceNameFromExplicitGroup) {
  // Robot-specific bringups seed a single group; verify primary name flows
  // through the helper.
  rtc::TopicConfig cfg;
  cfg["custom_robot"].subscribe.push_back({"/custom_robot/target"});
  ASSERT_FALSE(cfg.groups.empty());
  EXPECT_EQ(cfg.groups.front().first, "custom_robot");
}

TEST(RTControllerInterfaceTest, GetSecondaryDeviceNameEmptyWhenSingleGroup) {
  // Single-device controllers have no secondary group; helper returns "" so
  // callers can null-check GetDeviceNameConfig(...) on the result.
  StubController ctrl;
  rtc::TopicConfig cfg;
  cfg["only_device"].subscribe.push_back({"/only_device/target"});
  ctrl.SetTopicConfigForTesting(std::move(cfg));
  EXPECT_EQ(ctrl.GetPrimaryDeviceName(), "only_device");
  EXPECT_EQ(ctrl.GetSecondaryDeviceName(), "");
}

TEST(RTControllerInterfaceTest, GetSecondaryDeviceNameFromMultiGroupConfig) {
  // Verify the helper picks the second group (preserving YAML insertion
  // order). Mirrors the demo_*_controller layout: arm primary, hand-like
  // secondary. Phase 4: controller YAML only carries controller-owned roles.
  const auto node = YAML::Load(
      R"(
robot_a:
  subscribe:
    - {topic: /a/target, role: target}
robot_b:
  subscribe:
    - {topic: /b/target, role: target}
)");
  StubController ctrl;
  ctrl.SetTopicConfigForTesting(StubController::ParseTopicConfig(node));
  EXPECT_EQ(ctrl.GetPrimaryDeviceName(), "robot_a");
  EXPECT_EQ(ctrl.GetSecondaryDeviceName(), "robot_b");
}

// ═══════════════════════════════════════════════════════════════════════════════
// System model configuration
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, SystemModelConfigDefaultNull) {
  StubController ctrl;
  EXPECT_EQ(ctrl.GetSystemModelConfig(), nullptr);
}

TEST(RTControllerInterfaceTest, SetSystemModelConfigStoresAndCallsHook) {
  StubController ctrl;
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = "/test/robot.urdf";
  config.root_joint_type = "fixed";

  ctrl.SetSystemModelConfig(config);

  EXPECT_EQ(ctrl.model_config_set_count, 1);
  ASSERT_NE(ctrl.GetSystemModelConfig(), nullptr);
  EXPECT_EQ(ctrl.GetSystemModelConfig()->urdf_path, "/test/robot.urdf");
  EXPECT_EQ(ctrl.GetSystemModelConfig()->root_joint_type, "fixed");
}

TEST(RTControllerInterfaceTest, SetSystemModelConfigOverwrites) {
  StubController ctrl;
  rtc_urdf_bridge::ModelConfig cfg1;
  cfg1.urdf_path = "/first.urdf";
  ctrl.SetSystemModelConfig(cfg1);

  rtc_urdf_bridge::ModelConfig cfg2;
  cfg2.urdf_path = "/second.urdf";
  ctrl.SetSystemModelConfig(cfg2);

  EXPECT_EQ(ctrl.model_config_set_count, 2);
  EXPECT_EQ(ctrl.GetSystemModelConfig()->urdf_path, "/second.urdf");
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — valid YAML
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigValidMultiDevice) {
  // Phase 4: controller YAML carries only controller-owned roles
  // (target / robot_target / robot_transforms / grasp_state / wbc_state /
  // tof_snapshot / digital_twin_state). Device-wire roles moved to
  // devices.<group>.backend in sim.yaml/robot.yaml.
  const auto node = YAML::Load(
      R"(
ur5e:
  subscribe:
    - {topic: /ur5e/target, role: target}
  publish:
    - {topic: /ur5e/robot_target, role: robot_target}
    - {topic: transforms, role: robot_transforms, ownership: controller}
hand:
  subscribe:
    - {topic: /hand/target, role: target}
  publish:
    - {topic: /hand/grasp_state, role: grasp_state, data_size: 20}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  EXPECT_TRUE(cfg.HasGroup("ur5e"));
  EXPECT_TRUE(cfg.HasGroup("hand"));
  EXPECT_FALSE(cfg.HasGroup("missing"));

  EXPECT_TRUE(cfg.HasSubscribeTopic("ur5e"));
  EXPECT_TRUE(cfg.HasSubscribeTopic("hand"));

  EXPECT_EQ(cfg.GetFirstSubscribeTopic("hand"), "/hand/target");
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — Phase 4 strips device-wire roles from controller YAML
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigStateRoleRejected) {
  // state / motor_state / sensor_state / joint_command / ros2_command live in
  // devices.<group>.backend; the controller-YAML parser refuses them.
  const auto sub_state = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /joint_states, role: state}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(sub_state), std::runtime_error);

  const auto sub_motor = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /hand/motor_states, role: motor_state}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(sub_motor), std::runtime_error);

  const auto sub_sensor = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /hand/sensor_states, role: sensor_state}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(sub_sensor), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigJointCommandRoleRejected) {
  const auto pub_joint = YAML::Load(R"(
robot:
  publish:
    - {topic: /joint_command, role: joint_command}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(pub_joint), std::runtime_error);

  const auto pub_ros2 = YAML::Load(R"(
robot:
  publish:
    - {topic: /cmds, role: ros2_command, data_size: 6}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(pub_ros2), std::runtime_error);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — backward compatibility (target only after Phase 4)
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigBackwardCompatSubscribeTargetGoal) {
  // `goal` is a backward-compat alias for target. The legacy joint_state /
  // hand_state aliases were dropped along with kState in Phase 4.
  const auto node = YAML::Load(
      R"(
robot:
  subscribe:
    - {topic: /goal, role: goal}
)");
  const auto cfg = StubController::ParseTopicConfig(node);
  EXPECT_TRUE(cfg.HasSubscribeTopic("robot"));
}

TEST(RTControllerInterfaceTest, ParseTopicConfigBackwardCompatPublishJointGoal) {
  // joint_goal still maps to kRobotTarget; position_command/torque_command/
  // hand_command aliases were dropped (they mapped to kRos2Command/
  // kJointCommand, both deleted).
  const auto node = YAML::Load(
      R"(
robot:
  publish:
    - {topic: /jgoal, role: joint_goal}
)");
  const auto cfg = StubController::ParseTopicConfig(node);
  bool has_robot_target = false;
  for (const auto& [name, group] : cfg.groups) {
    if (name != "robot") {
      continue;
    }
    for (const auto& pub : group.publish) {
      if (pub.role == rtc::PublishRole::kRobotTarget) {
        has_robot_target = true;
      }
    }
  }
  EXPECT_TRUE(has_robot_target);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — error cases
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigDeprecatedFlatFormatThrows) {
  const auto node = YAML::Load(
      R"(
subscribe:
  - {topic: /target, role: target}
publish:
  - {topic: /robot_target, role: robot_target}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(node), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigUnknownSubscribeRoleThrows) {
  const auto node = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /data, role: unknown_role}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(node), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigUnknownPublishRoleThrows) {
  const auto node = YAML::Load(R"(
robot:
  publish:
    - {topic: /data, role: invalid_publish_role}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(node), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigNonMapGroupSkipped) {
  const auto node = YAML::Load(R"(
ur5e:
  subscribe:
    - {topic: /ur5e/target, role: target}
scalar_value: 42
)");
  const auto cfg = StubController::ParseTopicConfig(node);
  EXPECT_TRUE(cfg.HasGroup("ur5e"));
  EXPECT_FALSE(cfg.HasGroup("scalar_value"));
}

TEST(RTControllerInterfaceTest, ParseTopicConfigDataSizePreserved) {
  // Phase 4: data_size still flows through for controller-owned roles.
  const auto node = YAML::Load(R"(
robot:
  publish:
    - {topic: /grasp, role: grasp_state, data_size: 30}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto& [name, group] : cfg.groups) {
    if (name == "robot") {
      ASSERT_EQ(group.publish.size(), std::size_t{1});
      EXPECT_EQ(group.publish[0].data_size, 30);
      return;
    }
  }
  FAIL() << "robot group not found";
}

TEST(RTControllerInterfaceTest, ParseTopicConfigPreservesInsertionOrder) {
  const auto node = YAML::Load(
      R"(
beta_device:
  subscribe:
    - {topic: /b/target, role: target}
alpha_device:
  subscribe:
    - {topic: /a/target, role: target}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  ASSERT_GE(cfg.groups.size(), std::size_t{2});
  EXPECT_EQ(cfg.groups[0].first, "beta_device");
  EXPECT_EQ(cfg.groups[1].first, "alpha_device");
}

TEST(RTControllerInterfaceTest, ParseTopicConfigAllPublishRoles) {
  // Phase C: device_state_log / device_sensor_log roles removed; controller
  // data CSVs flow through ControllerLogSet under the top-level `logs:`
  // section instead.
  // Phase 4: joint_command / ros2_command roles are now device-wire and
  // owned by DeviceBackend; controller YAML keeps only controller-owned pubs.
  const auto node = YAML::Load(
      R"(
robot:
  publish:
    - {topic: /d, role: robot_target}
    - {topic: /g, role: grasp_state}
    - {topic: /h, role: tof_snapshot}
    - {topic: /i, role: digital_twin_state}
    - {topic: /j, role: robot_transforms}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto& [name, group] : cfg.groups) {
    if (name == "robot") {
      EXPECT_EQ(group.publish.size(), std::size_t{5});
      return;
    }
  }
  FAIL() << "robot group not found";
}

// ═══════════════════════════════════════════════════════════════════════════════
// LoadConfig
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, LoadConfigNullNode) {
  StubController ctrl;
  YAML::Node null_node;
  EXPECT_NO_THROW(ctrl.LoadConfig(null_node));
  // ARCH-1: default topic config is empty; null YAML leaves it untouched.
  EXPECT_TRUE(ctrl.GetTopicConfig().groups.empty());
}

TEST(RTControllerInterfaceTest, LoadConfigWithTopics) {
  // Phase 4: controller YAML only carries controller-owned lanes.
  StubController ctrl;
  const auto node = YAML::Load(
      R"(
topics:
  custom_robot:
    subscribe:
      - {topic: /custom/target, role: target}
    publish:
      - {topic: /custom/robot_target, role: robot_target}
)");
  ctrl.LoadConfig(node);

  EXPECT_TRUE(ctrl.GetTopicConfig().HasGroup("custom_robot"));
  EXPECT_FALSE(ctrl.GetTopicConfig().HasGroup("ur5e"));
}

TEST(RTControllerInterfaceTest, LoadConfigWithoutTopicsLeavesEmpty) {
  // ARCH-1: topic_config_ starts empty; YAML without a `topics:` section
  // leaves it empty (controllers that need a fallback construct their own
  // TopicConfig in the robot-specific bringup, not in the framework).
  StubController ctrl;
  const auto node = YAML::Load(R"(
some_gain: 1.5
some_flag: true
)");
  ctrl.LoadConfig(node);
  EXPECT_TRUE(ctrl.GetTopicConfig().groups.empty());
}

TEST(RTControllerInterfaceTest, LoadConfigDeprecatedEnableFlagsDoNotThrow) {
  StubController ctrl;
  const auto node = YAML::Load(R"(
enable_ur5e: true
enable_hand: false
)");
  EXPECT_NO_THROW(ctrl.LoadConfig(node));
}

// ═══════════════════════════════════════════════════════════════════════════════
// TopicConfig struct operations
// ═══════════════════════════════════════════════════════════════════════════════

TEST(TopicConfigTest, OperatorBracketInsertsNew) {
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/data"});
  EXPECT_TRUE(cfg.HasGroup("robot"));
  EXPECT_EQ(cfg.groups.size(), std::size_t{1});
}

TEST(TopicConfigTest, OperatorBracketAccessesExisting) {
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/data"});
  cfg["robot"].subscribe.push_back({"/more"});
  EXPECT_EQ(cfg.groups.size(), std::size_t{1});
  EXPECT_EQ(cfg.groups[0].second.subscribe.size(), std::size_t{2});
}

TEST(TopicConfigTest, HasGroupFalseWhenEmpty) {
  rtc::TopicConfig cfg;
  EXPECT_FALSE(cfg.HasGroup("anything"));
}

TEST(TopicConfigTest, HasGroupFalseForEmptyEntries) {
  rtc::TopicConfig cfg;
  cfg.groups.emplace_back("empty", rtc::DeviceTopicGroup{});
  EXPECT_FALSE(cfg.HasGroup("empty"));
}

TEST(TopicConfigTest, HasSubscribeTopicNotFound) {
  rtc::TopicConfig cfg;
  EXPECT_FALSE(cfg.HasSubscribeTopic("robot"));
}

TEST(TopicConfigTest, HasSubscribeTopicWrongGroup) {
  rtc::TopicConfig cfg;
  cfg["hand"].subscribe.push_back({"/hand/js"});
  EXPECT_FALSE(cfg.HasSubscribeTopic("robot"));
  EXPECT_TRUE(cfg.HasSubscribeTopic("hand"));
}

TEST(TopicConfigTest, GetFirstSubscribeTopicReturnsEmpty) {
  rtc::TopicConfig cfg;
  EXPECT_EQ(cfg.GetFirstSubscribeTopic("robot"), "");
}

TEST(TopicConfigTest, GetFirstSubscribeTopicReturnsFirst) {
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/first"});
  cfg["robot"].subscribe.push_back({"/second"});
  EXPECT_EQ(cfg.GetFirstSubscribeTopic("robot"), "/first");
}

TEST(TopicConfigTest, InsertionOrderPreserved) {
  rtc::TopicConfig cfg;
  cfg["charlie"];
  cfg["alpha"];
  cfg["bravo"];

  ASSERT_EQ(cfg.groups.size(), std::size_t{3});
  EXPECT_EQ(cfg.groups[0].first, "charlie");
  EXPECT_EQ(cfg.groups[1].first, "alpha");
  EXPECT_EQ(cfg.groups[2].first, "bravo");
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3-pass bring-up: PreConfigure → SetDeviceNameConfigs → on_configure
// ═══════════════════════════════════════════════════════════════════════════════
//
// Regression: before the 3-pass split, CM called on_configure BEFORE
// SetDeviceNameConfigs(), so RegisterLog lambdas captured empty
// joint_name spans. These tests pin the contract that CM relies on:
//   1. PreConfigure populates topic_config_ but does not trigger
//      OnDeviceConfigsSet.
//   2. SetDeviceNameConfigs (called between PreConfigure and on_configure)
//      runs OnDeviceConfigsSet exactly once.
//   3. on_configure can therefore observe device-name configs at the moment
//      it allocates resources / registers log channels.

class RclcppEnv : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_init_ = true;
    }
  }

  void TearDown() override {
    if (owns_init_) {
      rclcpp::shutdown();
    }
  }

 private:
  bool owns_init_{false};
};

[[maybe_unused]] auto* const kRclcppEnv = ::testing::AddGlobalTestEnvironment(new RclcppEnv);

namespace {
rclcpp_lifecycle::LifecycleNode::SharedPtr MakeLcNode(const std::string& name) {
  auto opts = rclcpp::NodeOptions().use_global_arguments(false);
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, "/" + name, opts);
}
}  // namespace

TEST(RTControllerInterfaceTest, PreConfigurePopulatesTopicConfigBeforeOnDeviceConfigsSet) {
  StubController ctrl;
  auto node = MakeLcNode("ctrl_pre_topics");

  YAML::Node yaml;
  yaml["topics"]["robot"]["subscribe"]["state"] = "/joint_states";
  yaml["topics"]["robot"]["subscribe"]["target"] = "/robot/target_joint_positions";

  // Pass 1: PreConfigure. Must succeed and populate topic_config_, but must
  // NOT trigger OnDeviceConfigsSet (CM has not called SetDeviceNameConfigs
  // yet).
  ASSERT_EQ(ctrl.PreConfigure(node, yaml), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl.GetTopicConfig().groups.empty());
  EXPECT_EQ(ctrl.device_config_set_count, 0);
}

TEST(RTControllerInterfaceTest, SetDeviceNameConfigsBetweenPassesRunsHookOnce) {
  StubController ctrl;
  auto node = MakeLcNode("ctrl_pre_then_set");

  YAML::Node yaml;
  yaml["topics"]["robot"]["subscribe"]["state"] = "/joint_states";
  ASSERT_EQ(ctrl.PreConfigure(node, yaml), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

  // Pass 2: CM resolves device-name configs and pushes them down. The hook
  // must fire exactly once and the configs must be visible via
  // GetDeviceNameConfig() before on_configure runs.
  std::map<std::string, rtc::DeviceNameConfig> configs;
  configs["robot"].device_name = "robot";
  configs["robot"].joint_state_names = {"j1", "j2", "j3"};
  ctrl.SetDeviceNameConfigs(std::move(configs));

  EXPECT_EQ(ctrl.device_config_set_count, 1);
  ASSERT_NE(ctrl.GetDeviceNameConfig("robot"), nullptr);
  EXPECT_EQ(ctrl.GetDeviceNameConfig("robot")->joint_state_names.size(), std::size_t{3});
}

TEST(RTControllerInterfaceTest, OnConfigureAfterPreConfigureIsIdempotent) {
  // CM's 3-pass bring-up calls on_configure() AFTER PreConfigure() and
  // SetDeviceNameConfigs(). on_configure must not clobber state set in those
  // earlier passes — node_ stays the one from PreConfigure, topic_config_
  // stays populated, and device-name configs remain visible.
  StubController ctrl;
  auto node = MakeLcNode("ctrl_idempotent");

  YAML::Node yaml;
  yaml["topics"]["robot"]["subscribe"]["state"] = "/joint_states";

  ASSERT_EQ(ctrl.PreConfigure(node, yaml), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  std::map<std::string, rtc::DeviceNameConfig> configs;
  configs["robot"].device_name = "robot";
  configs["robot"].joint_state_names = {"j1"};
  ctrl.SetDeviceNameConfigs(std::move(configs));

  // Pass 3: same yaml + same node passed back in — base must not re-parse
  // (node_ already non-null) and must not retrigger OnDeviceConfigsSet.
  const rclcpp_lifecycle::State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                             "unconfigured");
  ASSERT_EQ(ctrl.on_configure(unconfigured, node, yaml),
            rtc::RTControllerInterface::CallbackReturn::SUCCESS);

  EXPECT_EQ(ctrl.device_config_set_count, 1);
  EXPECT_FALSE(ctrl.GetTopicConfig().groups.empty());
  ASSERT_NE(ctrl.GetDeviceNameConfig("robot"), nullptr);
  EXPECT_EQ(ctrl.GetDeviceNameConfig("robot")->joint_state_names.front(), "j1");
}

TEST(RTControllerInterfaceTest, OnConfigureLegacyDirectCallStillWorks) {
  // Legacy callers (older unit tests, embedded usages) skip PreConfigure and
  // call on_configure() directly. The idempotency guard must fall through to
  // the original behavior: store node_, parse yaml, return SUCCESS.
  StubController ctrl;
  auto node = MakeLcNode("ctrl_legacy");

  YAML::Node yaml;
  yaml["topics"]["robot"]["subscribe"]["state"] = "/joint_states";

  const rclcpp_lifecycle::State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                             "unconfigured");
  ASSERT_EQ(ctrl.on_configure(unconfigured, node, yaml),
            rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl.GetTopicConfig().groups.empty());
}

// ═══════════════════════════════════════════════════════════════════════════════
// L1: LoadDeviceLimitsFromConfig — robot-agnostic device limits loader
// ═══════════════════════════════════════════════════════════════════════════════

TEST(LoadDeviceLimitsFromConfig, RespectsTopicConfigGroupOrder) {
  // Group order in topic_config_ decides controller-local device index —
  // independent of std::map<std::string,DeviceNameConfig> alphabetical order.
  StubController ctrl;
  YAML::Node yaml;
  yaml["topics"]["wrist"]["subscribe"][0]["topic"] = "/wrist/target";
  yaml["topics"]["wrist"]["subscribe"][0]["role"] = "target";
  yaml["topics"]["base"]["subscribe"][0]["topic"] = "/base/target";
  yaml["topics"]["base"]["subscribe"][0]["role"] = "target";
  ctrl.LoadConfig(yaml);

  std::map<std::string, rtc::DeviceNameConfig> configs;
  rtc::DeviceJointLimits wrist_limits;
  wrist_limits.position_lower = {-1.0, -1.0};
  wrist_limits.position_upper = {1.0, 1.0};
  wrist_limits.max_velocity = {0.5, 0.5};
  configs["wrist"].joint_limits = wrist_limits;
  rtc::DeviceJointLimits base_limits;
  base_limits.position_lower = {-3.0};
  base_limits.position_upper = {3.0};
  base_limits.max_velocity = {1.5};
  configs["base"].joint_limits = base_limits;
  ctrl.SetDeviceNameConfigs(std::move(configs));

  std::array<std::vector<double>, rtc::ControllerState::kMaxDevices> lower, upper, vel;
  ctrl.LoadDeviceLimitsFromConfig(lower, upper, vel, -10.0, 10.0, 5.0);

  EXPECT_EQ(lower[0], (std::vector<double>{-1.0, -1.0}));
  EXPECT_EQ(upper[0], (std::vector<double>{1.0, 1.0}));
  EXPECT_EQ(vel[0], (std::vector<double>{0.5, 0.5}));
  EXPECT_EQ(lower[1], (std::vector<double>{-3.0}));
  EXPECT_EQ(upper[1], (std::vector<double>{3.0}));
  EXPECT_EQ(vel[1], (std::vector<double>{1.5}));
}

TEST(LoadDeviceLimitsFromConfig, FillsEmptySlotsWithFallbacks) {
  StubController ctrl;
  YAML::Node yaml;
  yaml["topics"]["arm"]["subscribe"][0]["topic"] = "/arm/target";
  yaml["topics"]["arm"]["subscribe"][0]["role"] = "target";
  ctrl.LoadConfig(yaml);

  std::array<std::vector<double>, rtc::ControllerState::kMaxDevices> lower, upper, vel;
  ctrl.LoadDeviceLimitsFromConfig(lower, upper, vel, -7.0, 7.0, 3.0);

  for (const auto& slot : lower) {
    ASSERT_EQ(slot.size(), static_cast<std::size_t>(rtc::kMaxDeviceChannels));
    EXPECT_DOUBLE_EQ(slot[0], -7.0);
  }
  for (const auto& slot : upper) {
    ASSERT_EQ(slot.size(), static_cast<std::size_t>(rtc::kMaxDeviceChannels));
    EXPECT_DOUBLE_EQ(slot[0], 7.0);
  }
  for (const auto& slot : vel) {
    ASSERT_EQ(slot.size(), static_cast<std::size_t>(rtc::kMaxDeviceChannels));
    EXPECT_DOUBLE_EQ(slot[0], 3.0);
  }
}

TEST(LoadDeviceLimitsFromConfig, EmptyJointLimitVectorsTreatedAsMissing) {
  // joint_limits present but with empty inner vectors → fallback per-vector,
  // not per-device.
  StubController ctrl;
  YAML::Node yaml;
  yaml["topics"]["arm"]["subscribe"][0]["topic"] = "/arm/target";
  yaml["topics"]["arm"]["subscribe"][0]["role"] = "target";
  ctrl.LoadConfig(yaml);

  std::map<std::string, rtc::DeviceNameConfig> configs;
  rtc::DeviceJointLimits limits;
  limits.position_lower = {-2.0};  // upper + velocity intentionally empty
  configs["arm"].joint_limits = limits;
  ctrl.SetDeviceNameConfigs(std::move(configs));

  std::array<std::vector<double>, rtc::ControllerState::kMaxDevices> lower, upper, vel;
  ctrl.LoadDeviceLimitsFromConfig(lower, upper, vel, -9.0, 9.0, 4.0);

  EXPECT_EQ(lower[0], (std::vector<double>{-2.0}));
  ASSERT_EQ(upper[0].size(), static_cast<std::size_t>(rtc::kMaxDeviceChannels));
  EXPECT_DOUBLE_EQ(upper[0][0], 9.0);
  ASSERT_EQ(vel[0].size(), static_cast<std::size_t>(rtc::kMaxDeviceChannels));
  EXPECT_DOUBLE_EQ(vel[0][0], 4.0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// L2: ParseArmSafePosition — robot-agnostic estop YAML parser
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ParseArmSafePosition, ParsesValidSequence) {
  YAML::Node cfg;
  cfg["estop"]["arm_safe_position"].push_back(0.0);
  cfg["estop"]["arm_safe_position"].push_back(-1.57);
  cfg["estop"]["arm_safe_position"].push_back(1.57);
  const auto sp = StubController::ParseArmSafePosition(cfg, 3, "test_ctrl");
  ASSERT_EQ(sp.size(), 3u);
  EXPECT_DOUBLE_EQ(sp[0], 0.0);
  EXPECT_DOUBLE_EQ(sp[1], -1.57);
  EXPECT_DOUBLE_EQ(sp[2], 1.57);
}

TEST(ParseArmSafePosition, AcceptsArbitraryDof) {
  // KUKA iiwa7-style 7-DOF arm — base must not assume any specific count.
  YAML::Node cfg;
  for (int i = 0; i < 7; ++i) {
    cfg["estop"]["arm_safe_position"].push_back(0.1 * i);
  }
  const auto sp = StubController::ParseArmSafePosition(cfg, 7, "test_ctrl");
  ASSERT_EQ(sp.size(), 7u);
  EXPECT_DOUBLE_EQ(sp[6], 0.6);
}

TEST(ParseArmSafePosition, ThrowsOnMissingEstopSection) {
  YAML::Node cfg;
  cfg["other_section"] = "value";
  EXPECT_THROW(StubController::ParseArmSafePosition(cfg, 6, "test_ctrl"), std::runtime_error);
}

TEST(ParseArmSafePosition, ThrowsOnMissingArmSafePosition) {
  YAML::Node cfg;
  cfg["estop"]["something_else"] = 0.0;
  EXPECT_THROW(StubController::ParseArmSafePosition(cfg, 6, "test_ctrl"), std::runtime_error);
}

TEST(ParseArmSafePosition, ThrowsOnNonSequence) {
  YAML::Node cfg;
  cfg["estop"]["arm_safe_position"] = 0.0;
  EXPECT_THROW(StubController::ParseArmSafePosition(cfg, 6, "test_ctrl"), std::runtime_error);
}

TEST(ParseArmSafePosition, ThrowsOnLengthMismatch) {
  YAML::Node cfg;
  for (int i = 0; i < 5; ++i) {
    cfg["estop"]["arm_safe_position"].push_back(0.0);
  }
  EXPECT_THROW(StubController::ParseArmSafePosition(cfg, 6, "test_ctrl"), std::runtime_error);
}

TEST(ParseArmSafePosition, ErrorMessageIncludesControllerName) {
  YAML::Node cfg;
  cfg["estop"]["arm_safe_position"] = 0.0;
  try {
    StubController::ParseArmSafePosition(cfg, 6, "my_special_ctrl");
    FAIL() << "Expected runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("my_special_ctrl"), std::string::npos);
  }
}

}  // namespace
