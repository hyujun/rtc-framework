// ── test_controller_registry.cpp ─────────────────────────────────────────────
// Unit tests for rtc::ControllerRegistry singleton and ControllerEntry factory.
//
// Covers: singleton identity, Register / GetEntries, factory invocation,
// multiple registrations, and entry field preservation.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_interface/controller_registry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace {

// Minimal concrete controller for factory tests.
class StubRegistryController : public rtc::RTControllerInterface {
public:
  explicit StubRegistryController(const std::string &urdf_path)
      : urdf_path_(urdf_path) {}

  [[nodiscard]] rtc::ControllerOutput
  Compute(const rtc::ControllerState &) noexcept override {
    return {};
  }
  void SetDeviceTarget(int, std::span<const double>) noexcept override {}
  [[nodiscard]] std::string_view Name() const noexcept override {
    return "stub";
  }
  void InitializeHoldPosition(const rtc::ControllerState &) noexcept override {}

  [[nodiscard]] const std::string &urdf_path() const { return urdf_path_; }

private:
  std::string urdf_path_;
};

// ── Singleton identity ──────────────────────────────────────────────────────
// Two calls to Instance() must return the same object.
TEST(ControllerRegistryTest, SingletonIdentity) {
  auto &a = rtc::ControllerRegistry::Instance();
  auto &b = rtc::ControllerRegistry::Instance();
  EXPECT_EQ(&a, &b);
}

// ── Register adds entry ─────────────────────────────────────────────────────
TEST(ControllerRegistryTest, RegisterAndGetEntries) {
  auto &reg = rtc::ControllerRegistry::Instance();
  const auto initial_count = reg.GetEntries().size();

  reg.Register({"test_stub_ctrl", "indirect/", "rtc_controller_interface",
                [](const std::string &urdf) {
                  return std::make_unique<StubRegistryController>(urdf);
                }});

  EXPECT_EQ(reg.GetEntries().size(), initial_count + 1);

  const auto &last = reg.GetEntries().back();
  EXPECT_EQ(last.config_key, "test_stub_ctrl");
  EXPECT_EQ(last.config_subdir, "indirect/");
  EXPECT_EQ(last.config_package, "rtc_controller_interface");
}

// ── Factory produces a valid controller ──────────────────────────────────────
TEST(ControllerRegistryTest, FactoryCreatesController) {
  auto &reg = rtc::ControllerRegistry::Instance();

  reg.Register(
      {"test_factory_ctrl", "direct/", "test_pkg", [](const std::string &urdf) {
         return std::make_unique<StubRegistryController>(urdf);
       }});

  const auto &entry = reg.GetEntries().back();
  auto ctrl = entry.factory("/path/to/robot.urdf");

  ASSERT_NE(ctrl, nullptr);
  EXPECT_EQ(ctrl->Name(), "stub");

  auto *stub = dynamic_cast<StubRegistryController *>(ctrl.get());
  ASSERT_NE(stub, nullptr);
  EXPECT_EQ(stub->urdf_path(), "/path/to/robot.urdf");
}

// ── Multiple registrations accumulate ────────────────────────────────────────
TEST(ControllerRegistryTest, MultipleRegistrationsAccumulate) {
  auto &reg = rtc::ControllerRegistry::Instance();
  const auto base_count = reg.GetEntries().size();

  for (int i = 0; i < 3; ++i) {
    reg.Register({"accum_" + std::to_string(i), "indirect/", "test_pkg",
                  [](const std::string &urdf) {
                    return std::make_unique<StubRegistryController>(urdf);
                  }});
  }

  EXPECT_EQ(reg.GetEntries().size(), base_count + 3);
}

// ── Entry fields are correctly stored ────────────────────────────────────────
TEST(ControllerRegistryTest, EntryFieldsPreserved) {
  auto &reg = rtc::ControllerRegistry::Instance();

  reg.Register({"fields_test_ctrl", "direct/", "my_bringup_pkg",
                [](const std::string &urdf) {
                  return std::make_unique<StubRegistryController>(urdf);
                }});

  const auto &entry = reg.GetEntries().back();
  EXPECT_EQ(entry.config_key, "fields_test_ctrl");
  EXPECT_EQ(entry.config_subdir, "direct/");
  EXPECT_EQ(entry.config_package, "my_bringup_pkg");
  EXPECT_NE(entry.factory, nullptr);
}

// ── Factory receives URDF path correctly ─────────────────────────────────────
TEST(ControllerRegistryTest, FactoryReceivesUrdfPath) {
  auto &reg = rtc::ControllerRegistry::Instance();

  reg.Register(
      {"urdf_path_test", "indirect/", "test_pkg", [](const std::string &urdf) {
         return std::make_unique<StubRegistryController>(urdf);
       }});

  const auto &entry = reg.GetEntries().back();

  const std::string test_path = "/opt/ros/urdf/test_robot.urdf.xacro";
  auto ctrl = entry.factory(test_path);
  auto *stub = dynamic_cast<StubRegistryController *>(ctrl.get());
  ASSERT_NE(stub, nullptr);
  EXPECT_EQ(stub->urdf_path(), test_path);
}

// ── GetEntries returns const reference (compile-time check) ──────────────────
TEST(ControllerRegistryTest, GetEntriesIsConst) {
  const auto &reg = rtc::ControllerRegistry::Instance();
  const auto &entries = reg.GetEntries();
  EXPECT_GE(entries.size(), std::size_t{0});
}

// ── Duplicate config_key still appends but emits a warning ──────────────────
// Static-init ordering across TUs makes throwing on duplicates fragile, so
// Register() warns and shadows. This test pins the shadowing behaviour and
// the count delta so a future "throw on duplicate" change is intentional.
TEST(ControllerRegistryTest, RegisterDuplicateShadowsAndAppends) {
  auto &reg = rtc::ControllerRegistry::Instance();
  const auto initial_count = reg.GetEntries().size();

  auto factory_fn = [](const std::string &urdf) {
    return std::make_unique<StubRegistryController>(urdf);
  };

  reg.Register({"dup_key_canary", "indirect/", "pkg_first", factory_fn});
  reg.Register({"dup_key_canary", "indirect/", "pkg_second", factory_fn});

  EXPECT_EQ(reg.GetEntries().size(), initial_count + 2);
  // Lookup-by-key returns the latest (push_back order); pkg_second shadows.
  const auto &entries = reg.GetEntries();
  std::size_t hits = 0;
  std::string last_pkg;
  for (const auto &e : entries) {
    if (e.config_key == "dup_key_canary") {
      ++hits;
      last_pkg = e.config_package;
    }
  }
  EXPECT_EQ(hits, std::size_t{2});
  EXPECT_EQ(last_pkg, "pkg_second");
}

} // namespace
