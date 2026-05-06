// ── test_rt_controller_interface.cpp
// ────────────────────────────────────────── Unit tests for
// RTControllerInterface base class and TopicConfig.
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
  using RTControllerInterface::MakeDefaultTopicConfig;
  using RTControllerInterface::ParseArmSafePosition;
  using RTControllerInterface::ParseTopicConfig;

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
  // topic_config_ empty; LoadConfig() (or an explicit
  // MakeDefaultTopicConfig(<device>) call from a robot-specific bringup)
  // populates it.
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

TEST(RTControllerInterfaceTest, GetPrimaryDeviceNameFromMakeDefaultTopicConfig) {
  // Robot-specific bringups can still call MakeDefaultTopicConfig(<device>)
  // explicitly to seed the topic config — verify primary name flows through.
  const auto cfg = StubController::MakeDefaultTopicConfig("custom_robot");
  ASSERT_FALSE(cfg.groups.empty());
  EXPECT_EQ(cfg.groups.front().first, "custom_robot");
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
// MakeDefaultTopicConfig
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigSubscribeRoles) {
  const auto cfg = StubController::MakeDefaultTopicConfig("my_robot");
  EXPECT_TRUE(cfg.HasGroup("my_robot"));
  EXPECT_TRUE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kState));
  EXPECT_TRUE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kTarget));
  EXPECT_FALSE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kMotorState));
  EXPECT_FALSE(cfg.HasSubscribeRole("my_robot", rtc::SubscribeRole::kSensorState));
}

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigTopicNames) {
  const auto cfg = StubController::MakeDefaultTopicConfig("my_robot");
  EXPECT_EQ(cfg.GetSubscribeTopicName("my_robot", rtc::SubscribeRole::kState), "/joint_states");
  EXPECT_EQ(cfg.GetSubscribeTopicName("my_robot", rtc::SubscribeRole::kTarget),
            "/my_robot/target_joint_positions");
}

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigCapability) {
  const auto cfg = StubController::MakeDefaultTopicConfig("ur5e");

  for (const auto& [name, group] : cfg.groups) {
    if (name == "ur5e") {
      EXPECT_TRUE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kJointState));
      EXPECT_FALSE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kMotorState));
      EXPECT_FALSE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kSensorData));
      return;
    }
  }
  FAIL() << "ur5e group not found in default topic config";
}

TEST(RTControllerInterfaceTest, MakeDefaultTopicConfigPublishEntries) {
  const auto cfg = StubController::MakeDefaultTopicConfig("ur5e");

  bool has_joint_command = false;
  bool has_ros2_command = false;
  bool has_robot_target = false;

  for (const auto& [name, group] : cfg.groups) {
    if (name != "ur5e") {
      continue;
    }
    for (const auto& pub : group.publish) {
      switch (pub.role) {
        case rtc::PublishRole::kJointCommand:
          has_joint_command = true;
          break;
        case rtc::PublishRole::kRos2Command:
          has_ros2_command = true;
          break;
        case rtc::PublishRole::kRobotTarget:
          has_robot_target = true;
          break;
        default:
          break;
      }
    }
  }

  EXPECT_TRUE(has_joint_command);
  EXPECT_TRUE(has_ros2_command);
  EXPECT_TRUE(has_robot_target);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — valid YAML
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigValidMultiDevice) {
  const auto node = YAML::Load(
      R"(
ur5e:
  subscribe:
    - {topic: /joint_states, role: state}
    - {topic: /ur5e/target, role: target}
  publish:
    - {topic: /ur5e/joint_command, role: joint_command}
    - {topic: /ur5e/robot_target, role: robot_target}
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

  EXPECT_EQ(cfg.GetSubscribeTopicName("ur5e", rtc::SubscribeRole::kState), "/joint_states");
  EXPECT_EQ(cfg.GetSubscribeTopicName("hand", rtc::SubscribeRole::kMotorState),
            "/hand/motor_states");
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — capability inference
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigCapabilityInference) {
  const auto node = YAML::Load(
      R"(
hand:
  subscribe:
    - {topic: /hand/js, role: state}
    - {topic: /hand/ms, role: motor_state}
    - {topic: /hand/ss, role: sensor_state}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto& [name, group] : cfg.groups) {
    if (name == "hand") {
      EXPECT_TRUE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kJointState));
      EXPECT_TRUE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kMotorState));
      EXPECT_TRUE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kSensorData));
      EXPECT_TRUE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kInference));
      return;
    }
  }
  FAIL() << "hand group not found";
}

TEST(RTControllerInterfaceTest, ParseTopicConfigTargetOnlyNoCapability) {
  const auto node = YAML::Load(R"(
robot:
  subscribe:
    - {topic: /robot/target, role: target}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto& [name, group] : cfg.groups) {
    if (name == "robot") {
      EXPECT_FALSE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kJointState));
      EXPECT_FALSE(rtc::HasCapability(group.capability, rtc::DeviceCapability::kMotorState));
      return;
    }
  }
  FAIL() << "robot group not found";
}

// ═══════════════════════════════════════════════════════════════════════════════
// ParseTopicConfig — backward compatibility
// ═══════════════════════════════════════════════════════════════════════════════

TEST(RTControllerInterfaceTest, ParseTopicConfigBackwardCompatSubscribe) {
  const auto node = YAML::Load(
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

TEST(RTControllerInterfaceTest, ParseTopicConfigBackwardCompatPublish) {
  const auto node = YAML::Load(
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

  for (const auto& [name, group] : cfg.groups) {
    if (name != "robot") {
      continue;
    }
    for (const auto& pub : group.publish) {
      if (pub.role == rtc::PublishRole::kRos2Command) {
        has_ros2_cmd = true;
      }
      if (pub.role == rtc::PublishRole::kJointCommand) {
        has_joint_cmd = true;
      }
      if (pub.role == rtc::PublishRole::kRobotTarget) {
        has_robot_target = true;
      }
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

TEST(RTControllerInterfaceTest, ParseTopicConfigDeprecatedFlatFormatThrows) {
  const auto node = YAML::Load(
      R"(
subscribe:
  - {topic: /joint_states, role: state}
publish:
  - {topic: /cmd, role: joint_command}
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
    - {topic: /joint_states, role: state}
scalar_value: 42
)");
  const auto cfg = StubController::ParseTopicConfig(node);
  EXPECT_TRUE(cfg.HasGroup("ur5e"));
  EXPECT_FALSE(cfg.HasGroup("scalar_value"));
}

TEST(RTControllerInterfaceTest, ParseTopicConfigDataSizePreserved) {
  const auto node = YAML::Load(R"(
robot:
  publish:
    - {topic: /cmd, role: ros2_command, data_size: 30}
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

TEST(RTControllerInterfaceTest, ParseTopicConfigAllPublishRoles) {
  // Phase C: device_state_log / device_sensor_log roles removed; controller
  // data CSVs flow through ControllerLogSet under the top-level `logs:`
  // section instead.
  const auto node = YAML::Load(
      R"(
robot:
  publish:
    - {topic: /a, role: joint_command}
    - {topic: /b, role: ros2_command}
    - {topic: /d, role: robot_target}
    - {topic: /g, role: grasp_state}
    - {topic: /h, role: tof_snapshot}
    - {topic: /i, role: digital_twin_state}
    - {topic: /j, role: robot_transforms}
)");
  const auto cfg = StubController::ParseTopicConfig(node);

  for (const auto& [name, group] : cfg.groups) {
    if (name == "robot") {
      EXPECT_EQ(group.publish.size(), std::size_t{7});
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
  StubController ctrl;
  const auto node = YAML::Load(
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

TEST(RTControllerInterfaceTest, LoadConfigWithoutTopicsLeavesEmpty) {
  // ARCH-1: topic_config_ starts empty; YAML without a `topics:` section
  // leaves it empty (controllers that need a fallback must call
  // MakeDefaultTopicConfig(<device>) explicitly with a robot-specific name).
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
  cfg["robot"].subscribe.push_back({"/data", rtc::SubscribeRole::kState});
  EXPECT_TRUE(cfg.HasGroup("robot"));
  EXPECT_EQ(cfg.groups.size(), std::size_t{1});
}

TEST(TopicConfigTest, OperatorBracketAccessesExisting) {
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/data", rtc::SubscribeRole::kState});
  cfg["robot"].subscribe.push_back({"/more", rtc::SubscribeRole::kTarget});
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

TEST(TopicConfigTest, HasSubscribeRoleNotFound) {
  rtc::TopicConfig cfg;
  EXPECT_FALSE(cfg.HasSubscribeRole("robot", rtc::SubscribeRole::kState));
}

TEST(TopicConfigTest, HasSubscribeRoleWrongGroup) {
  rtc::TopicConfig cfg;
  cfg["hand"].subscribe.push_back({"/hand/js", rtc::SubscribeRole::kState});
  EXPECT_FALSE(cfg.HasSubscribeRole("robot", rtc::SubscribeRole::kState));
  EXPECT_TRUE(cfg.HasSubscribeRole("hand", rtc::SubscribeRole::kState));
}

TEST(TopicConfigTest, GetSubscribeTopicNameReturnsEmpty) {
  rtc::TopicConfig cfg;
  EXPECT_EQ(cfg.GetSubscribeTopicName("robot", rtc::SubscribeRole::kState), "");
}

TEST(TopicConfigTest, GetSubscribeTopicNameReturnsFirst) {
  rtc::TopicConfig cfg;
  cfg["robot"].subscribe.push_back({"/first", rtc::SubscribeRole::kState});
  cfg["robot"].subscribe.push_back({"/second", rtc::SubscribeRole::kState});
  EXPECT_EQ(cfg.GetSubscribeTopicName("robot", rtc::SubscribeRole::kState), "/first");
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
  yaml["topics"]["wrist"]["subscribe"][0]["topic"] = "/wrist/state";
  yaml["topics"]["wrist"]["subscribe"][0]["role"] = "state";
  yaml["topics"]["base"]["subscribe"][0]["topic"] = "/base/state";
  yaml["topics"]["base"]["subscribe"][0]["role"] = "state";
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
  yaml["topics"]["arm"]["subscribe"][0]["topic"] = "/arm/state";
  yaml["topics"]["arm"]["subscribe"][0]["role"] = "state";
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
  yaml["topics"]["arm"]["subscribe"][0]["topic"] = "/arm/state";
  yaml["topics"]["arm"]["subscribe"][0]["role"] = "state";
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
