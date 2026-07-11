// Unit tests for fake_hand_firmware.hpp — the full-SIL loopback firmware
// simulator (device side). Exercises BuildResponse dispatch (per-frame size,
// CMD echo, MODE mirror), the LPF joint model, and sensor synthesis, all without
// a socket (BuildResponse is called directly with crafted request buffers).

#include "udp_hand_driver/fake_hand_firmware.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace udp_hand_driver::test {

using namespace packets;

namespace {

FakeHandFirmware::Config DefaultConfig() {
  FakeHandFirmware::Config cfg;
  cfg.num_fingertips = kDefaultNumFingertips;
  cfg.proto_1b = false;
  cfg.lpf_time_constant_s = 0.1;
  cfg.effort_stiffness = 2.0;
  cfg.effort_damping = 0.1;
  cfg.step_rate_hz = 500.0;
  cfg.sensor_base_min = 1000;
  cfg.sensor_base_max = 2000;
  cfg.sensor_noise_stddev = 5.0;
  cfg.seed = 7;
  return cfg;
}

// Build a 3-byte header-only request (read / set-mode style).
std::array<uint8_t, kHeaderSize> HeaderReq(Command cmd, uint8_t mode) {
  return {kDeviceId, static_cast<uint8_t>(cmd), mode};
}

// Build a 43-byte write-position request carrying `positions`.
std::array<uint8_t, kMotorPacketSize> WriteReq(const std::array<float, kNumHandMotors>& positions,
                                               uint8_t mode) {
  MotorPacket pkt{};
  pkt.id = kDeviceId;
  pkt.cmd = static_cast<uint8_t>(Command::kWritePosition);
  pkt.mode = mode;
  for (std::size_t i = 0; i < kNumHandMotors; ++i)
    pkt.data[i] = FloatToUint32(positions[i]);
  std::array<uint8_t, kMotorPacketSize> buf{};
  std::memcpy(buf.data(), &pkt, kMotorPacketSize);
  return buf;
}

}  // namespace

// ── Dispatch: per-frame size, CMD echo, MODE mirror ─────────────────────────

TEST(FakeHandFirmware, WriteEcho_43B_EchoesCmd_MirrorsMode) {
  FakeHandFirmware fw(DefaultConfig());
  std::array<float, kNumHandMotors> cmd{};
  cmd.fill(0.5f);
  const auto req = WriteReq(cmd, static_cast<uint8_t>(JointMode::kJoint));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());

  EXPECT_EQ(n, kMotorPacketSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kWritePosition));  // CMD echo
  EXPECT_EQ(out[2], static_cast<uint8_t>(JointMode::kJoint));        // MODE mirror
}

TEST(FakeHandFirmware, ReadPosition_43B) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(Command::kReadPosition, static_cast<uint8_t>(JointMode::kMotor));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_EQ(n, kMotorPacketSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kReadPosition));
  EXPECT_EQ(out[2], static_cast<uint8_t>(JointMode::kMotor));
}

TEST(FakeHandFirmware, ReadAllMotors_123B_GroupedLayout) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(Command::kReadAllMotors, static_cast<uint8_t>(JointMode::kJoint));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_EQ(n, kAllMotorResponseSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kReadAllMotors));
  EXPECT_EQ(out[2], static_cast<uint8_t>(JointMode::kJoint));
}

TEST(FakeHandFirmware, FingerSensor_67B) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(Command::kReadSensor2, static_cast<uint8_t>(SensorMode::kRaw));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_EQ(n, kSensorResponseSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kReadSensor2));
  EXPECT_EQ(out[2], static_cast<uint8_t>(SensorMode::kRaw));
}

TEST(FakeHandFirmware, AllSensors1a_259B_MirrorsMode) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(Command::kReadAllSensors, static_cast<uint8_t>(SensorMode::kNn));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_EQ(n, kAllSensorResponseSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kReadAllSensors));
  EXPECT_EQ(out[2], static_cast<uint8_t>(SensorMode::kNn));  // 1a strict MODE mirror
}

TEST(FakeHandFirmware, AllSensors1b_99B) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.proto_1b = true;
  FakeHandFirmware fw(cfg);
  const auto req = HeaderReq(Command::kReadAllSensors, static_cast<uint8_t>(SensorMode::kRaw));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_EQ(n, kP1bSensorResponseSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kReadAllSensors));
}

TEST(FakeHandFirmware, SetSensorMode_3B_Ack) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(Command::kSetSensorMode, static_cast<uint8_t>(SensorMode::kNn));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_EQ(n, kSensorRequestSize);
  EXPECT_EQ(out[1], static_cast<uint8_t>(Command::kSetSensorMode));
  EXPECT_EQ(out[2], static_cast<uint8_t>(SensorMode::kNn));
}

TEST(FakeHandFirmware, ShortHeader_Dropped) {
  FakeHandFirmware fw(DefaultConfig());
  std::array<uint8_t, 2> req{kDeviceId, static_cast<uint8_t>(Command::kReadPosition)};
  std::array<uint8_t, kMaxPacketSize> out{};
  EXPECT_EQ(fw.BuildResponse(req.data(), req.size(), out.data(), out.size()), 0u);
}

TEST(FakeHandFirmware, UnknownCommand_Dropped) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(static_cast<Command>(0xEE), 0x00);
  std::array<uint8_t, kMaxPacketSize> out{};
  EXPECT_EQ(fw.BuildResponse(req.data(), req.size(), out.data(), out.size()), 0u);
}

TEST(FakeHandFirmware, InsufficientCapacity_Dropped) {
  FakeHandFirmware fw(DefaultConfig());
  const auto req = HeaderReq(Command::kReadAllSensors, 0x00);
  std::array<uint8_t, 10> out{};  // too small for 259 B
  EXPECT_EQ(fw.BuildResponse(req.data(), req.size(), out.data(), out.size()), 0u);
}

// ── LPF joint model ─────────────────────────────────────────────────────────

TEST(FakeHandFirmware, Write_LatchesTarget_AdvancesLpfOneStep) {
  FakeHandFirmware fw(DefaultConfig());
  const double alpha = fw.alpha();
  ASSERT_GT(alpha, 0.0);

  std::array<float, kNumHandMotors> cmd{};
  cmd.fill(1.0f);
  const auto req = WriteReq(cmd, static_cast<uint8_t>(JointMode::kJoint));
  std::array<uint8_t, kMaxPacketSize> out{};
  fw.BuildResponse(req.data(), req.size(), out.data(), out.size());

  // One LPF step from pos=0 toward target=1: pos == alpha·(1 − 0).
  EXPECT_NEAR(fw.target()[0], 1.0f, 1e-6f);
  EXPECT_NEAR(fw.pos()[0], static_cast<float>(alpha), 1e-5f);

  // The 43 B echo carries the stepped position (not the raw target).
  MotorPacket echo{};
  std::memcpy(&echo, out.data(), kMotorPacketSize);
  EXPECT_NEAR(Uint32ToFloat(echo.data[0]), static_cast<float>(alpha), 1e-5f);
}

TEST(FakeHandFirmware, RepeatedWrites_ConvergeTowardTarget) {
  FakeHandFirmware fw(DefaultConfig());
  std::array<float, kNumHandMotors> cmd{};
  cmd.fill(1.0f);
  const auto req = WriteReq(cmd, static_cast<uint8_t>(JointMode::kJoint));
  std::array<uint8_t, kMaxPacketSize> out{};
  for (int i = 0; i < 5000; ++i)
    fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  EXPECT_NEAR(fw.pos()[0], 1.0f, 1e-3f);
  EXPECT_NEAR(fw.vel()[0], 0.0f, 1e-2f);  // velocity decays as position settles
}

TEST(FakeHandFirmware, ReadsSnapshotWithoutMovingModel) {
  FakeHandFirmware fw(DefaultConfig());
  std::array<float, kNumHandMotors> cmd{};
  cmd.fill(1.0f);
  const auto wreq = WriteReq(cmd, static_cast<uint8_t>(JointMode::kJoint));
  std::array<uint8_t, kMaxPacketSize> out{};
  fw.BuildResponse(wreq.data(), wreq.size(), out.data(), out.size());
  const float pos_after_write = fw.pos()[0];

  // Several reads must not advance the model.
  const auto rreq = HeaderReq(Command::kReadPosition, static_cast<uint8_t>(JointMode::kJoint));
  for (int i = 0; i < 10; ++i)
    fw.BuildResponse(rreq.data(), rreq.size(), out.data(), out.size());
  EXPECT_FLOAT_EQ(fw.pos()[0], pos_after_write);
}

// ── Sensor synthesis ────────────────────────────────────────────────────────

TEST(FakeHandFirmware, AllSensors1a_ValuesNearBaseBand) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.sensor_base_min = 1500;
  cfg.sensor_base_max = 1500;  // fixed base → value == 1500 + small noise
  cfg.sensor_noise_stddev = 3.0;
  FakeHandFirmware fw(cfg);

  const auto req = HeaderReq(Command::kReadAllSensors, static_cast<uint8_t>(SensorMode::kRaw));
  std::array<uint8_t, kMaxPacketSize> out{};
  fw.BuildResponse(req.data(), req.size(), out.data(), out.size());

  AllSensorResponsePacket pkt{};
  std::memcpy(&pkt, out.data(), kAllSensorResponseSize);
  // Active fingers (all 4) carry base+noise; values stay within a few stddev.
  for (std::size_t k = 0; k < kAllSensorDataCount; ++k)
    EXPECT_NEAR(pkt.data[k], 1500, 40);
}

TEST(FakeHandFirmware, InactiveFingersZeroed) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.num_fingertips = 2;  // only fingers 0,1 active
  FakeHandFirmware fw(cfg);

  const auto req = HeaderReq(Command::kReadAllSensors, static_cast<uint8_t>(SensorMode::kRaw));
  std::array<uint8_t, kMaxPacketSize> out{};
  fw.BuildResponse(req.data(), req.size(), out.data(), out.size());

  AllSensorResponsePacket pkt{};
  std::memcpy(&pkt, out.data(), kAllSensorResponseSize);
  // Fingers 2,3 (channels 32..63) must be zero.
  for (std::size_t k = 2 * kSensorDataPerPacket; k < kAllSensorDataCount; ++k)
    EXPECT_EQ(pkt.data[k], 0);
}

TEST(FakeHandFirmware, Deterministic_SameSeedSameStream) {
  FakeHandFirmware fw_a(DefaultConfig());
  FakeHandFirmware fw_b(DefaultConfig());
  const auto req = HeaderReq(Command::kReadAllSensors, static_cast<uint8_t>(SensorMode::kRaw));
  std::array<uint8_t, kMaxPacketSize> a{}, b{};
  fw_a.BuildResponse(req.data(), req.size(), a.data(), a.size());
  fw_b.BuildResponse(req.data(), req.size(), b.data(), b.size());
  EXPECT_EQ(std::memcmp(a.data(), b.data(), kAllSensorResponseSize), 0);
}

TEST(FakeHandFirmware, Proto1b_FloatForcesNearBase) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.proto_1b = true;
  cfg.sensor_base_min = 10;
  cfg.sensor_base_max = 10;
  cfg.sensor_noise_stddev = 1.0;
  FakeHandFirmware fw(cfg);

  const auto req = HeaderReq(Command::kReadAllSensors, static_cast<uint8_t>(SensorMode::kRaw));
  std::array<uint8_t, kMaxPacketSize> out{};
  const std::size_t n = fw.BuildResponse(req.data(), req.size(), out.data(), out.size());
  ASSERT_EQ(n, kP1bSensorResponseSize);

  P1bSensorResponsePacket pkt{};
  std::memcpy(&pkt, out.data(), kP1bSensorResponseSize);
  for (std::size_t k = 0; k < kP1bSensorFloatCount; ++k)
    EXPECT_NEAR(pkt.data[k], 10.0f, 8.0f);
}

}  // namespace udp_hand_driver::test
