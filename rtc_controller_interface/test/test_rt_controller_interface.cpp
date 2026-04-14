// ── test_rt_controller_interface.cpp ──────────────────────────────────────────
// Unit tests for RTControllerInterface base class and TopicConfig.
//
// Uses a minimal concrete subclass (StubController) to exercise the protected
// and virtual method behaviour of the abstract interface.
//
// Covers: default construction, control rate, device name configuration,
// system model configuration, MakeDefaultTopicConfig, ParseTopicConfig
// (valid / deprecated / unknown role / backward-compat / capability inference),
// LoadConfig, TopicConfig struct operations, and default virtual methods.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_interface/rt_controller_interface.hpp>
#include <rtc_urdf_bridge/types.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <string>
#include <vector>

namespace
{

// ── Stub controller — exposes protected helpers for testing ──────────────────
class StubController : public rtc::RTControllerInterface
{
public:
  StubController() = default;

  [[nodiscard]] rtc::ControllerOutput Compute(
    const rtc::ControllerState &) noexcept override
  {
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }
  void SetDeviceTarget(int, std::span<const double>) noexcept override {}
  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "StubController";
  }
  void InitializeHoldPosition(const rtc::ControllerState &) noexcept override {}

  // Expose protected statics for direct testing.
  using RTControllerInterface::MakeDefaultTopicConfig;
  using RTControllerInterface::ParseTopicConfig;

  // Callback counters.
  int device_config_set_count{0};
  int model_config_set_count{0};

protected:
  void OnDeviceConfigsSet() override {++device_config_set_count;}
  void OnSystemModelConfigSet() override {++model_config_set_count;}
};

// ═══════════════════════════════════════════════════════════════════════════════
// Default construction
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, DefaultControlRate)
{
  StubController ctrl;
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.002);
}

TEST(RTControllerInterfaceTest, DefaultTopicConfigHasUr5e)
{
  StubController ctrl;
  EXPECT_TRUE(ctrl.GetTopicConfig().HasGroup("ur5e"));
}

TEST(RTControllerInterfaceTest, DefaultVirtualMethods)
{
  StubController ctrl;
  EXPECT_FALSE(ctrl.IsEstopped());
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);
  EXPECT_TRUE(ctrl.GetCurrentGains().empty());
}

TEST(RTControllerInterfaceTest, NameReturnsStub)
{
  StubController ctrl;
  EXPECT_EQ(ctrl.Name(), "StubController");
}

TEST(RTControllerInterfaceTest, ComputeReturnsValid)
{
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

TEST(RTControllerInterfaceTest, EstopDefaultNoOps)
{
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

TEST(RTControllerInterfaceTest, SetControlRate)
{
  StubController ctrl;
  ctrl.SetControlRate(1000.0);
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.001);
}

TEST(RTControllerInterfaceTest, SetControlRateZeroFallback)
{
  StubController ctrl;
  ctrl.SetControlRate(0.0);
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.002);
}

TEST(RTControllerInterfaceTest, SetControlRateNegativeFallback)
{
  StubController ctrl;
  ctrl.SetControlRate(-500.0);
  EXPECT_DOUBLE_EQ(ctrl.GetDefaultDt(), 0.002);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Device name configuration
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, SetDeviceNameConfigsStoresAndCallsHook)
{
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

TEST(RTControllerInterfaceTest, GetDeviceNameConfigReturnsNullptrForMissing)
{
  StubController ctrl;
  EXPECT_EQ(ctrl.GetDeviceNameConfig("nonexistent"), nullptr);
}

TEST(RTControllerInterfaceTest, GetPrimaryDeviceNameFromTopicConfig)
{
  StubController ctrl;
  EXPECT_EQ(ctrl.GetPrimaryDeviceName(), "ur5e");
}

// ═══════════════════════════════════════════════════════════════════════════════
// System model configuration
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, SystemModelConfigDefaultNull)
{
  StubController ctrl;
  EXPECT_EQ(ctrl.GetSystemModelConfig(), nullptr);
}

TEST(RTControllerInterfaceTest, SetSystemModelConfigStoresAndCallsHook)
{
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

TEST(RTControllerInterfaceTest, SetSystemModelConfigOverwrites)
{
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
// MakeDefaultTopicConfig
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigSubscribeRoles)
{
  const auto cfg = StubController::MakeDefaultTopicConfig("my_robot");
  EXPECT_TRUE(cfg.HasGroup("my_robot"));
  EXPECT_TRUE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kState));
  EXPECT_TRUE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kTarget));
  EXPECT_FALSE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kMotorState));
  EXPECT_FALSE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kSensorState));
}

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigTopicNames)
{
  const auto cfg = StubController::MakeDefaultTopicConfig("my_robot");
  EXPECT_EQ(cfg.GetSubscribeTopicName("my_robot", rtc::SubscribeRole::kState),
            "/joint_states");
  EXPECT_EQ(cfg.GetSubscribeTopicName("my_robot", rtc::SubscribeRole::kTarget),
            "/my_robot/target_joint_positions");
}

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigCapability)
{
  const auto cfg = StubController::MakeDefaultTopicConfig("ur5e");

  for (const auto & [name, group] : cfg.groups) {
    if (name == "ur5e") {
      EXPECT_TRUE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kJointState));
      EXPECT_FALSE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kMotorState));
      EXPECT_FALSE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kSensorData));
      return;
    }
  }
  FAIL() << "ur5e group not found in default topic config";
}

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigPublishEntries)
{
  const auto cfg = StubController::MakeDefaultTopicConfig("ur5e");

  bool has_joint_command = false;
  bool has_ros2_command = false;
  bool has_gui_position = false;
  bool has_robot_target = false;
  bool has_state_log = false;

  for (const auto & [name, group] : cfg.groups) {
    if (name != "ur5e") {continue;}
    for (const auto & pub : group.publish) {
      switch (pub.role) {
        case rtc::PublishRole::kJointCommand:   has_joint_command = true; break;
        case rtc::PublishRole::kRos2Command:    has_ros2_command = true; break;
        case rtc::PublishRole::kGuiPosition:    has_gui_position = true; break;
        case rtc::PublishRole::kRobotTarget:    has_robot_target = true; break;
        case rtc::PublishRole::kDeviceStateLog: has_state_log = true; break;
        default: break;
      }
    }
  }

  EXPECT_TRUE(has_joint_command);
  EXPECT_TRUE(has_ros2_command);
  EXPECT_TRUE(has_gui_position);
  EXPECT_TRUE(has_robot_target);
  EXPECT_TRUE(has_state_log);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — valid YAML
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigValidMultiDevice)
{
  const auto node =
    YAML::Load(
      R"(
ur5e:
  subscribe:
    - {topic: /joint_states, role: state}
    - {topic: /ur5e/target, role: target}
  publish:
    - {topic: /ur5e/joint_command, role: joint_command}
    - {topic: /ur5e/gui_pos, role: gui_position}
hand:
  subscribe:
    - {topic: /hand/joint_states, role: state}
    - {topic: /hand/motor_states, role: motor_state}
    - {topic: /hand/sensor_states, role: sensor_state}
  publish:
    - {topic: /hand/joint_command, role: joint_command}
    - {topic: /hand/grasp_state, role: grasp_state, data_size: 20}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  EXPECT_TRUE(cfg.HasGroup("ur5e"));
  EXPECT_TRUE(cfg.HasGroup("hand"));
  EXPECT_FALSE(cfg.HasGroup("missing"));

  EXPECT_TRUE(cfg.HasSubscribeRole("ur5e", rtc::SubscribeRole::kState));
  EXPECT_TRUE(cfg.HasSubscribeRole("ur5e", rtc::SubscribeRole::kTarget));
  EXPECT_TRUE(cfg.HasSubscribeRole("hand", rtc::SubscribeRole::kMotorState));
  EXPECT_TRUE(cfg.HasSubscribeRole("hand", rtc::SubscribeRole::kSensorState));

  EXPECT_EQ(cfg.GetSubscribeTopicName("ur5e", rtc::SubscribeRole::kState),
            "/joint_states");
  EXPECT_EQ(cfg.GetSubscribeTopicName("hand", rtc::SubscribeRole::kMotorState),
            "/hand/motor_states");
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — capability inference
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigCapabilityInference)
{
  const auto node =
    YAML::Load(
      R"(
hand:
  subscribe:
    - {topic: /hand/js, role: state}
    - {topic: /hand/ms, role: motor_state}
    - {topic: /hand/ss, role: sensor_state}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto & [name, group] : cfg.groups) {
    if (name == "hand") {
      EXPECT_TRUE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kJointState));
      EXPECT_TRUE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kMotorState));
      EXPECT_TRUE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kSensorData));
      EXPECT_TRUE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kInference));
      return;
    }
  }
  FAIL() << "hand group not found";
}

TEST(RTControllerInterfaceTest, ParseTopicConfigTargetOnlyNoCapability)
{
  const auto node = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /robot/target, role: target}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto & [name, group] : cfg.groups) {
    if (name == "robot") {
      EXPECT_FALSE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kJointState));
      EXPECT_FALSE(rtc::HasCapability(
        group.capability, rtc::DeviceCapability::kMotorState));
      return;
    }
  }
  FAIL() << "robot group not found";
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — backward compatibility
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigBackwardCompatSubscribe)
{
  const auto node =
    YAML::Load(
      R"(
robot:
  subscribe:
    - {topic: /js, role: joint_state}
    - {topic: /hs, role: hand_state}
    - {topic: /goal, role: goal}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  // joint_state -> kState, hand_state -> kState, goal -> kTarget
  EXPECT_TRUE(cfg.HasSubscribeRole("robot", rtc::SubscribeRole::kState));
  EXPECT_TRUE(cfg.HasSubscribeRole("robot", rtc::SubscribeRole::kTarget));
}

TEST(RTControllerInterfaceTest, ParseTopicConfigBackwardCompatPublish)
{
  const auto node =
    YAML::Load(
      R"(
robot:
  publish:
    - {topic: /cmd, role: position_command}
    - {topic: /tcmd, role: torque_command}
    - {topic: /hcmd, role: hand_command}
    - {topic: /jgoal, role: joint_goal}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  bool has_ros2_cmd = false;
  bool has_joint_cmd = false;
  bool has_robot_target = false;

  for (const auto & [name, group] : cfg.groups) {
    if (name != "robot") {continue;}
    for (const auto & pub : group.publish) {
      if (pub.role == rtc::PublishRole::kRos2Command) {has_ros2_cmd = true;}
      if (pub.role == rtc::PublishRole::kJointCommand) {has_joint_cmd = true;}
      if (pub.role == rtc::PublishRole::kRobotTarget) {has_robot_target = true;}
    }
  }

  // position_command/torque_command -> kRos2Command
  EXPECT_TRUE(has_ros2_cmd);
  // hand_command -> kJointCommand
  EXPECT_TRUE(has_joint_cmd);
  // joint_goal -> kRobotTarget
  EXPECT_TRUE(has_robot_target);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — error cases
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigDeprecatedFlatFormatThrows)
{
  const auto node =
    YAML::Load(
      R"(
subscribe:
  - {topic: /joint_states, role: state}
publish:
  - {topic: /cmd, role: joint_command}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(node), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigUnknownSubscribeRoleThrows)
{
  const auto node = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /data, role: unknown_role}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(node), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigUnknownPublishRoleThrows)
{
  const auto node =
    YAML::Load(R"(
robot:
  publish:
    - {topic: /data, role: invalid_publish_role}
)");
  EXPECT_THROW(StubController::ParseTopicConfig(node), std::runtime_error);
}

TEST(RTControllerInterfaceTest, ParseTopicConfigNonMapGroupSkipped)
{
  const auto node =
    YAML::Load(R"(
ur5e:
  subscribe:
    - {topic: /joint_states, role: state}
scalar_value: 42
)");
  const auto cfg = StubController::ParseTopicConfig(node);
  EXPECT_TRUE(cfg.HasGroup("ur5e"));
  EXPECT_FALSE(cfg.HasGroup("scalar_value"));
}

TEST(RTControllerInterfaceTest, ParseTopicConfigDataSizePreserved)
{
  const auto node =
    YAML::Load(R"(
robot:
  publish:
    - {topic: /log, role: device_state_log, data_size: 30}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto & [name, group] : cfg.groups) {
    if (name == "robot") {
      ASSERT_EQ(group.publish.size(), std::size_t{1});
      EXPECT_EQ(group.publish[0].data_size, 30);
      return;
    }
  }
  FAIL() << "robot group not found";
}

TEST(RTControllerInterfaceTest, ParseTopicConfigPreservesInsertionOrder)
{
  const auto node =
    YAML::Load(
      R"(
beta_device:
  subscribe:
    - {topic: /b, role: state}
alpha_device:
  subscribe:
    - {topic: /a, role: state}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  ASSERT_GE(cfg.groups.size(), std::size_t{2});
  EXPECT_EQ(cfg.groups[0].first, "beta_device");
  EXPECT_EQ(cfg.groups[1].first, "alpha_device");
}

TEST(RTControllerInterfaceTest, ParseTopicConfigAllPublishRoles)
{
  const auto node =
    YAML::Load(
      R"(
robot:
  publish:
    - {topic: /a, role: joint_command}
    - {topic: /b, role: ros2_command}
    - {topic: /c, role: gui_position}
    - {topic: /d, role: robot_target}
    - {topic: /e, role: device_state_log}
    - {topic: /f, role: device_sensor_log}
    - {topic: /g, role: grasp_state}
    - {topic: /h, role: tof_snapshot}
    - {topic: /i, role: digital_twin_state}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto & [name, group] : cfg.groups) {
    if (name == "robot") {
      EXPECT_EQ(group.publish.size(), std::size_t{9});
      return;
    }
  }
  FAIL() << "robot group not found";
}

// ═══════════════════════════════════════════════════════════════════════════════
// LoadConfig
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, LoadConfigNullNode)
{
  StubController ctrl;
  YAML::Node null_node;
  EXPECT_NO_THROW(ctrl.LoadConfig(null_node));
  // Default topic config should remain
  EXPECT_TRUE(ctrl.GetTopicConfig().HasGroup("ur5e"));
}

TEST(RTControllerInterfaceTest, LoadConfigWithTopics)
{
  StubController ctrl;
  const auto node =
    YAML::Load(
      R"(
topics:
  custom_robot:
    subscribe:
      - {topic: /custom/states, role: state}
    publish:
      - {topic: /custom/cmd, role: joint_command}
)");
  ctrl.LoadConfig(node);

  EXPECT_TRUE(ctrl.GetTopicConfig().HasGroup("custom_robot"));
  EXPECT_FALSE(ctrl.GetTopicConfig().HasGroup("ur5e"));
}

TEST(RTControllerInterfaceTest, LoadConfigWithoutTopicsKeepsDefault)
{
  StubController ctrl;
  const auto node = YAML::Load(R"(
some_gain: 1.5
some_flag: true
)");
  ctrl.LoadConfig(node);
  EXPECT_TRUE(ctrl.GetTopicConfig().HasGroup("ur5e"));
}

TEST(RTControllerInterfaceTest, LoadConfigDeprecatedEnableFlagsDoNotThrow)
{
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

TEST(TopicConfigTest, OperatorBracketInsertsNew)
{
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/data", rtc::SubscribeRole::kState});
  EXPECT_TRUE(cfg.HasGroup("robot"));
  EXPECT_EQ(cfg.groups.size(), std::size_t{1});
}

TEST(TopicConfigTest, OperatorBracketAccessesExisting)
{
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/data", rtc::SubscribeRole::kState});
  cfg["robot"].subscribe.push_back({"/more", rtc::SubscribeRole::kTarget});
  EXPECT_EQ(cfg.groups.size(), std::size_t{1});
  EXPECT_EQ(cfg.groups[0].second.subscribe.size(), std::size_t{2});
}

TEST(TopicConfigTest, HasGroupFalseWhenEmpty)
{
  rtc::TopicConfig cfg;
  EXPECT_FALSE(cfg.HasGroup("anything"));
}

TEST(TopicConfigTest, HasGroupFalseForEmptyEntries)
{
  rtc::TopicConfig cfg;
  cfg.groups.emplace_back("empty", rtc::DeviceTopicGroup{});
  EXPECT_FALSE(cfg.HasGroup("empty"));
}

TEST(TopicConfigTest, HasSubscribeRoleNotFound)
{
  rtc::TopicConfig cfg;
  EXPECT_FALSE(cfg.HasSubscribeRole("robot", rtc::SubscribeRole::kState));
}

TEST(TopicConfigTest, HasSubscribeRoleWrongGroup)
{
  rtc::TopicConfig cfg;
  cfg["hand"].subscribe.push_back({"/hand/js", rtc::SubscribeRole::kState});
  EXPECT_FALSE(cfg.HasSubscribeRole("robot", rtc::SubscribeRole::kState));
  EXPECT_TRUE(cfg.HasSubscribeRole("hand", rtc::SubscribeRole::kState));
}

TEST(TopicConfigTest, GetSubscribeTopicNameReturnsEmpty)
{
  rtc::TopicConfig cfg;
  EXPECT_EQ(cfg.GetSubscribeTopicName("robot", rtc::SubscribeRole::kState), "");
}

TEST(TopicConfigTest, GetSubscribeTopicNameReturnsFirst)
{
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/first", rtc::SubscribeRole::kState});
  cfg["robot"].subscribe.push_back({"/second", rtc::SubscribeRole::kState});
  EXPECT_EQ(cfg.GetSubscribeTopicName("robot", rtc::SubscribeRole::kState),
            "/first");
}

TEST(TopicConfigTest, InsertionOrderPreserved)
{
  rtc::TopicConfig cfg;
  cfg["charlie"];
  cfg["alpha"];
  cfg["bravo"];

  ASSERT_EQ(cfg.groups.size(), std::size_t{3});
  EXPECT_EQ(cfg.groups[0].first, "charlie");
  EXPECT_EQ(cfg.groups[1].first, "alpha");
  EXPECT_EQ(cfg.groups[2].first, "bravo");
}

}  // namespace
