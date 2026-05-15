// ── test_device_backends_registered.cpp ─────────────────────────────────────
// Smoke test: confirm that the 3 backend translation units in
// integrated_bringup (mujoco_native / ur_driver_native / udp_hand_native)
// reach DeviceBackendRegistry at static-init time. Failure here means the
// linker stripped the registration TUs (--whole-archive is missing somewhere)
// or the registry wiring is broken.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_manager/device_backend_registry.hpp>

#include <gtest/gtest.h>

TEST(DeviceBackendsRegisteredTest, MujocoNative) {
  const auto* entry = rtc::DeviceBackendRegistry::Instance().Find("mujoco_native");
  ASSERT_NE(entry, nullptr);
  ASSERT_TRUE(static_cast<bool>(entry->factory));
  EXPECT_NE(entry->factory(), nullptr);
}

TEST(DeviceBackendsRegisteredTest, UrDriverNative) {
  const auto* entry = rtc::DeviceBackendRegistry::Instance().Find("ur_driver_native");
  ASSERT_NE(entry, nullptr);
  ASSERT_TRUE(static_cast<bool>(entry->factory));
  EXPECT_NE(entry->factory(), nullptr);
}

TEST(DeviceBackendsRegisteredTest, UdpHandNative) {
  const auto* entry = rtc::DeviceBackendRegistry::Instance().Find("udp_hand_native");
  ASSERT_NE(entry, nullptr);
  ASSERT_TRUE(static_cast<bool>(entry->factory));
  EXPECT_NE(entry->factory(), nullptr);
}

TEST(DeviceBackendsRegisteredTest, AllReportCorrectCapabilities) {
  auto mujoco = rtc::DeviceBackendRegistry::Instance().Create("mujoco_native");
  ASSERT_NE(mujoco, nullptr);
  EXPECT_FALSE(mujoco->HasMotorState());
  EXPECT_FALSE(mujoco->HasSensorState());

  auto ur = rtc::DeviceBackendRegistry::Instance().Create("ur_driver_native");
  ASSERT_NE(ur, nullptr);
  EXPECT_FALSE(ur->HasMotorState());
  EXPECT_FALSE(ur->HasSensorState());

  auto hand = rtc::DeviceBackendRegistry::Instance().Create("udp_hand_native");
  ASSERT_NE(hand, nullptr);
  EXPECT_TRUE(hand->HasMotorState());
  EXPECT_TRUE(hand->HasSensorState());
}
