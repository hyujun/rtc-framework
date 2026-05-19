// ── test_device_backend_registry.cpp ────────────────────────────────────────
// Unit tests for rtc::DeviceBackendRegistry.
//
// Covers: singleton identity, Register/Find/Create, duplicate-tag shadow
// warning (registers but second entry overrides), unknown-tag lookup returns
// nullptr, RTC_REGISTER_DEVICE_BACKEND macro path.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_manager/device_backend.hpp>
#include <rtc_controller_manager/device_backend_registry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

namespace {

// Minimal stub backend — all methods are no-ops. Used only to confirm that
// the registry can produce a non-null unique_ptr for a registered tag.
class StubBackend : public rtc::DeviceBackend {
 public:
  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/, const rtc::DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr /*state_cb_group*/) override {
    last_group_ = config.group_name;
  }

  void Activate() override {}

  void Deactivate() override {}

  [[nodiscard]] bool ReadState(rtc::DeviceStateCache& /*cache*/) noexcept override { return false; }

  void WriteCommand(const rtc::PublishSnapshot::GroupCommandSlot& /*slot*/,
                    rtc::CommandType /*command_type*/) noexcept override {}

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    return {};
  }

  [[nodiscard]] const std::string& last_group() const { return last_group_; }

 private:
  std::string last_group_;
};

// Register a stub via the macro to exercise the static-init path.
RTC_REGISTER_DEVICE_BACKEND(test_stub_native, std::make_unique<StubBackend>())

}  // namespace

TEST(DeviceBackendRegistryTest, SingletonIdentity) {
  auto& a = rtc::DeviceBackendRegistry::Instance();
  auto& b = rtc::DeviceBackendRegistry::Instance();
  EXPECT_EQ(&a, &b);
}

TEST(DeviceBackendRegistryTest, MacroRegistration) {
  // The macro above runs at static init; the registry must already contain
  // `test_stub_native` by the time the test body executes.
  const auto* entry = rtc::DeviceBackendRegistry::Instance().Find("test_stub_native");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->type_tag, "test_stub_native");
  ASSERT_TRUE(static_cast<bool>(entry->factory));
}

TEST(DeviceBackendRegistryTest, CreateProducesInstance) {
  auto backend = rtc::DeviceBackendRegistry::Instance().Create("test_stub_native");
  ASSERT_NE(backend, nullptr);

  rtc::DeviceBackendConfig cfg;
  cfg.group_name = "iiwa7";
  cfg.type = "test_stub_native";
  // Test fixture has no executor — pass a null callback group; backends must
  // tolerate this and fall back to the node's default group (StubBackend
  // ignores the arg entirely).
  backend->Configure(nullptr, cfg, nullptr);

  auto* stub = dynamic_cast<StubBackend*>(backend.get());
  ASSERT_NE(stub, nullptr);
  EXPECT_EQ(stub->last_group(), "iiwa7");
}

TEST(DeviceBackendRegistryTest, UnknownTagReturnsNull) {
  EXPECT_EQ(rtc::DeviceBackendRegistry::Instance().Find("definitely_not_registered"), nullptr);
  EXPECT_EQ(rtc::DeviceBackendRegistry::Instance().Create("definitely_not_registered"), nullptr);
}

TEST(DeviceBackendRegistryTest, DuplicateTagShadowsFirst) {
  // Direct Register call (not via macro) so we control timing relative to the
  // assertion below. Registering a second `test_stub_native` should warn but
  // succeed; Find returns the first registration (linear scan from front).
  const std::size_t before = rtc::DeviceBackendRegistry::Instance().GetEntries().size();

  rtc::DeviceBackendRegistry::Instance().Register(
      {"test_stub_native", [] { return std::unique_ptr<rtc::DeviceBackend>(new StubBackend); }});

  const auto& entries = rtc::DeviceBackendRegistry::Instance().GetEntries();
  EXPECT_EQ(entries.size(), before + 1);

  // Linear scan returns the first match — first registration wins on lookup.
  const auto* first = rtc::DeviceBackendRegistry::Instance().Find("test_stub_native");
  ASSERT_NE(first, nullptr);
  std::size_t match_count = 0;
  for (const auto& entry : entries) {
    if (entry.type_tag == "test_stub_native")
      ++match_count;
  }
  EXPECT_GE(match_count, 2U);  // macro registration + this one
}
