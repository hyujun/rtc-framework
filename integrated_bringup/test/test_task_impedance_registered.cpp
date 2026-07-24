// ── Controller registration smoke test ──────────────────────────────────────
// Confirms task_impedance_controller (registered in controller_registration.cpp)
// reaches the ControllerRegistry singleton at static-init time via
// --whole-archive. Failure here means the linker stripped the registration TU
// or the RTC_REGISTER_CONTROLLER wiring is broken. Construction of the
// controller itself is exercised by rtc_controllers unit tests.
#include <rtc_controller_interface/controller_registry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

TEST(TaskImpedanceRegisteredTest, EntryPresent) {
  const auto& entries = rtc::ControllerRegistry::Instance().GetEntries();
  const auto it = std::find_if(entries.begin(), entries.end(), [](const rtc::ControllerEntry& e) {
    return e.config_key == "task_impedance_controller";
  });
  ASSERT_NE(it, entries.end()) << "task_impedance_controller not registered (TU stripped?)";
  EXPECT_EQ(it->config_package, "integrated_bringup");
  EXPECT_TRUE(static_cast<bool>(it->factory)) << "factory is empty";
}

// The demo controllers must still be present (registration is additive).
TEST(TaskImpedanceRegisteredTest, DemoControllersStillRegistered) {
  const auto& entries = rtc::ControllerRegistry::Instance().GetEntries();
  for (const char* key : {"demo_joint_controller", "demo_task_controller", "demo_wbc_controller"}) {
    const std::string want = key;
    EXPECT_TRUE(std::any_of(entries.begin(), entries.end(),
                            [&](const rtc::ControllerEntry& e) { return e.config_key == want; }))
        << key << " missing after adding task_impedance_controller";
  }
}
