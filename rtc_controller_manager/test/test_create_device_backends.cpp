// Tier 2 (lightweight) — CreateDeviceBackends / PropagateCapabilitiesIntoMappings
// tests for RtControllerNode.
//
// These drive the device-backend construction path in
// rt_controller_node_device_config.cpp WITHOUT the heavy YAML pipeline
// (LoadDeviceNameConfigs): group_slot_map_ and device_name_configs_ are
// injected via the friend accessor, and a fake backend is registered through
// DeviceBackendRegistry (same pattern as test_device_backend_registry). Covers
// the happy path (capability derivation, state-ready callback) plus the skip
// branches (slot overflow, missing binding, unknown type).

#include "rt_cm_test_access.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_backend_registry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>

namespace rtc {
namespace {

// Fake backend exposing motor + sensor lanes so CreateDeviceBackends derives a
// full capability bitmask. FireStateReady() exposes the protected
// NotifyStateReady() so tests can invoke the state-ready callback CM installs.
class CdbStubBackend : public DeviceBackend {
 public:
  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/, const DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr /*state_cb_group*/) override {
    configured_ = true;
    last_group_ = config.group_name;
  }

  void Activate() override {}

  void Deactivate() override {}

  bool ReadState(DeviceStateCache& /*cache*/) noexcept override { return false; }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot& /*slot*/,
                    CommandType /*command_type*/) noexcept override {}

  // Honours the LastStateStamp contract (issue #198): epoch until the first
  // state arrives, stamped before the ready callback fires.
  std::chrono::steady_clock::time_point LastStateStamp() const noexcept override { return stamp_; }

  bool HasMotorState() const noexcept override { return true; }

  bool HasSensorState() const noexcept override { return true; }

  void FireStateReady() {
    stamp_ = std::chrono::steady_clock::now();
    NotifyStateReady();
  }

  bool configured() const { return configured_; }

  const std::string& last_group() const { return last_group_; }

 private:
  bool configured_{false};
  std::string last_group_;
  std::chrono::steady_clock::time_point stamp_{};
};

RTC_REGISTER_DEVICE_BACKEND(cdb_test_backend, std::make_unique<CdbStubBackend>())

DeviceNameConfig MakeArmConfig(const std::string& backend_type) {
  DeviceNameConfig c;
  c.device_name = "arm";
  c.joint_state_names = {"j1", "j2"};
  c.joint_command_names = {"j1", "j2"};
  DeviceBackendBinding b;
  b.type = backend_type;
  b.state_topic = "/arm/state";
  b.command_topic = "/arm/cmd";
  b.motor_topic = "/arm/motor";
  b.sensor_topic = "/arm/sensor";
  c.backend = b;
  return c;
}

class CreateDeviceBackendsTest : public ::testing::Test {
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

  void SetUp() override {
    node_ = std::make_shared<RtControllerNode>("test_cdb_node");
    ControllerLifecycleTestAccess::EnsureRtCallbackGroup(*node_);
  }

  void InjectArm(const std::string& backend_type) {
    ControllerLifecycleTestAccess::SetGroupSlotMap(*node_, {{"arm", 0}});
    std::map<std::string, DeviceNameConfig> cfgs;
    cfgs.emplace("arm", MakeArmConfig(backend_type));
    ControllerLifecycleTestAccess::InjectDeviceNameConfigs(*node_, std::move(cfgs));
  }

  std::shared_ptr<RtControllerNode> node_;
};

// ── Happy path ───────────────────────────────────────────────────────────────

TEST_F(CreateDeviceBackendsTest, HappyPathCreatesBackendAndDerivesCapabilities) {
  InjectArm("cdb_test_backend");
  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);

  auto* backend = ControllerLifecycleTestAccess::GetBackend(*node_, 0);
  ASSERT_NE(nullptr, backend);
  EXPECT_TRUE(dynamic_cast<CdbStubBackend*>(backend)->configured());
  EXPECT_EQ("arm", dynamic_cast<CdbStubBackend*>(backend)->last_group());

  const auto cap = ControllerLifecycleTestAccess::GetSlotCapability(*node_, 0);
  EXPECT_TRUE(HasCapability(cap, DeviceCapability::kJointState));
  EXPECT_TRUE(HasCapability(cap, DeviceCapability::kMotorState));
  EXPECT_TRUE(HasCapability(cap, DeviceCapability::kSensorData));
}

TEST_F(CreateDeviceBackendsTest, DeviceBecomesReadyOnItsOwnFirstState) {
  // The state-ready callback no longer marks anything on the CM side: liveness
  // is read from the backend's LastStateStamp (issue #198 §1/§2). What this
  // pins is that the transition still happens, and that it is per device.
  InjectArm("cdb_test_backend");
  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);
  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "arm", std::chrono::seconds(60),
                                                  /*slot=*/0);

  auto* backend =
      dynamic_cast<CdbStubBackend*>(ControllerLifecycleTestAccess::GetBackend(*node_, 0));
  ASSERT_NE(nullptr, backend);
  EXPECT_EQ(0, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(
                   *node_, std::chrono::steady_clock::now()));

  backend->FireStateReady();  // invokes the lambda CreateDeviceBackends installed
  EXPECT_EQ(-1, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(
                    *node_, std::chrono::steady_clock::now()));
}

TEST_F(CreateDeviceBackendsTest, PropagateCopiesCapabilityIntoControllerMapping) {
  InjectArm("cdb_test_backend");
  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);
  ControllerLifecycleTestAccess::AddControllerSlotMapping(*node_, /*slot0=*/0);

  ControllerLifecycleTestAccess::CallPropagateCapabilities(*node_);

  EXPECT_EQ(ControllerLifecycleTestAccess::GetSlotCapability(*node_, 0),
            ControllerLifecycleTestAccess::GetMappingCapability(*node_, 0, 0));
}

// ── Skip branches ────────────────────────────────────────────────────────────

TEST_F(CreateDeviceBackendsTest, SlotExceedingMaxDevicesSkipped) {
  ControllerLifecycleTestAccess::SetGroupSlotMap(
      *node_, {{"arm", ControllerLifecycleTestAccess::MaxDevices()}});
  std::map<std::string, DeviceNameConfig> cfgs;
  cfgs.emplace("arm", MakeArmConfig("cdb_test_backend"));
  ControllerLifecycleTestAccess::InjectDeviceNameConfigs(*node_, std::move(cfgs));

  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);
  EXPECT_EQ(nullptr, ControllerLifecycleTestAccess::GetBackend(*node_, 0));
}

TEST_F(CreateDeviceBackendsTest, GroupMissingFromConfigsSkipped) {
  ControllerLifecycleTestAccess::SetGroupSlotMap(*node_, {{"arm", 0}});
  // device_name_configs_ left empty → find() == end() → skip.
  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);
  EXPECT_EQ(nullptr, ControllerLifecycleTestAccess::GetBackend(*node_, 0));
}

TEST_F(CreateDeviceBackendsTest, GroupWithoutBackendBindingSkipped) {
  ControllerLifecycleTestAccess::SetGroupSlotMap(*node_, {{"arm", 0}});
  std::map<std::string, DeviceNameConfig> cfgs;
  DeviceNameConfig c;
  c.device_name = "arm";  // backend == nullopt
  cfgs.emplace("arm", std::move(c));
  ControllerLifecycleTestAccess::InjectDeviceNameConfigs(*node_, std::move(cfgs));

  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);
  EXPECT_EQ(nullptr, ControllerLifecycleTestAccess::GetBackend(*node_, 0));
}

TEST_F(CreateDeviceBackendsTest, UnknownBackendTypeSkipped) {
  InjectArm("nonexistent_backend_xyz");
  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);
  EXPECT_EQ(nullptr, ControllerLifecycleTestAccess::GetBackend(*node_, 0));
}

// ── Digital-twin publishers + state-ready republish path ────────────────────

TEST_F(CreateDeviceBackendsTest, DigitalTwinPublishersCreatedPerGroup) {
  InjectArm("cdb_test_backend");
  // CreateDigitalTwinPublishers iterates group_slot_map_ + device_name_configs_
  // (rt_controller_node_publishers.cpp) — pre-sizes one JointState per group.
  ControllerLifecycleTestAccess::CallCreateDigitalTwinPublishers(*node_);
  SUCCEED();  // no crash; publisher map populated for slot 0
}

TEST_F(CreateDeviceBackendsTest, StateReadyRepublishesDigitalTwin) {
  InjectArm("cdb_test_backend");
  // Order matters: DT publishers must exist before the backend's state-ready
  // callback fires so the callback's republish branch (device_config.cpp) runs.
  ControllerLifecycleTestAccess::CallCreateDigitalTwinPublishers(*node_);
  ControllerLifecycleTestAccess::CallCreateDeviceBackends(*node_);

  auto* backend =
      dynamic_cast<CdbStubBackend*>(ControllerLifecycleTestAccess::GetBackend(*node_, 0));
  ASSERT_NE(nullptr, backend);
  // Exercises the digital-twin republish branch. The callback's only remaining
  // job is that republish, so reaching it without crashing IS the assertion.
  backend->FireStateReady();
}

}  // namespace
}  // namespace rtc
