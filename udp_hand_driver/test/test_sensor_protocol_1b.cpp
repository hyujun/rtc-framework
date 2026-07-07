// Unit tests for the Proto_1b sensor protocol seam.
//
// Covers the 99-byte bulk-sensor wire packet (round-trip, size guard, little-
// endian decode) and the SensorProtocol1a/1b factory + capabilities + decode.
//
// Tier 1: Pure computation, no ROS2 or network dependencies.

#include "udp_hand_driver/protocol/sensor_protocol.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"
#include "udp_hand_driver/udp_hand_state.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

namespace udp_hand_driver::test {

namespace {

// Build a 99-byte 1b bulk-sensor response: [id, cmd, mode] + 24 float32 LE.
std::array<uint8_t, packets::kP1bSensorResponseSize> MakeP1bBuffer(
    uint8_t cmd, uint8_t mode, const std::array<float, packets::kP1bSensorFloatCount>& vals) {
  std::array<uint8_t, packets::kP1bSensorResponseSize> buf{};
  buf[0] = packets::kDeviceId;
  buf[1] = cmd;
  buf[2] = mode;
  std::memcpy(buf.data() + packets::kHeaderSize, vals.data(), vals.size() * sizeof(float));
  return buf;
}

// 24 distinct sentinel values (0.5, 1.5, ... 23.5) so a mis-indexed decode is
// visible.
std::array<float, packets::kP1bSensorFloatCount> SentinelValues() {
  std::array<float, packets::kP1bSensorFloatCount> vals{};
  for (std::size_t i = 0; i < vals.size(); ++i) {
    vals[i] = static_cast<float>(i) + 0.5f;
  }
  return vals;
}

}  // namespace

// ── Wire packet ─────────────────────────────────────────────────────────────

TEST(P1bSensorPacket, SizeIs99) {
  EXPECT_EQ(sizeof(packets::P1bSensorResponsePacket), 99u);
  EXPECT_EQ(packets::kP1bSensorResponseSize, 99u);
  EXPECT_EQ(packets::kP1bSensorFloatCount, 24u);
  EXPECT_EQ(packets::kP1bSensorFingertipCount, 4u);
  EXPECT_EQ(packets::kP1bSensorFloatsPerFingertip, 6u);
  // 1a bulk sensor (259) remains the largest response.
  EXPECT_EQ(packets::kMaxPacketSize, packets::kAllSensorResponseSize);
}

TEST(P1bSensorPacket, DecodeRoundtrip) {
  const auto vals = SentinelValues();
  const auto buf =
      MakeP1bBuffer(static_cast<uint8_t>(packets::Command::kReadAllSensors), 0x00, vals);

  packets::P1bSensorResponsePacket pkt{};
  ASSERT_TRUE(packets::DecodeP1bSensorResponse(buf.data(), buf.size(), pkt));
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(packets::Command::kReadAllSensors));
  for (std::size_t i = 0; i < vals.size(); ++i) {
    EXPECT_FLOAT_EQ(pkt.data[i], vals[i]) << "mismatch at index " << i;
  }
}

TEST(P1bSensorPacket, RejectsShortBuffer) {
  const auto vals = SentinelValues();
  const auto buf = MakeP1bBuffer(0x19, 0x00, vals);
  packets::P1bSensorResponsePacket pkt{};
  EXPECT_FALSE(
      packets::DecodeP1bSensorResponse(buf.data(), packets::kP1bSensorResponseSize - 1, pkt));
}

TEST(P1bSensorPacket, LittleEndianDecode) {
  // 1.0f = 0x3F800000; little-endian byte order = {00, 00, 80, 3F}.
  std::array<uint8_t, packets::kP1bSensorResponseSize> buf{};
  buf[0] = packets::kDeviceId;
  buf[1] = static_cast<uint8_t>(packets::Command::kReadAllSensors);
  buf[2] = 0x00;
  buf[packets::kHeaderSize + 0] = 0x00;
  buf[packets::kHeaderSize + 1] = 0x00;
  buf[packets::kHeaderSize + 2] = 0x80;
  buf[packets::kHeaderSize + 3] = 0x3F;

  packets::P1bSensorResponsePacket pkt{};
  ASSERT_TRUE(packets::DecodeP1bSensorResponse(buf.data(), buf.size(), pkt));
  EXPECT_FLOAT_EQ(pkt.data[0], 1.0f);
}

// ── Factory + capabilities ──────────────────────────────────────────────────

TEST(SensorProtocolFactory, CreatesKnownVersions) {
  auto p1a = CreateSensorProtocol("1a");
  ASSERT_NE(p1a, nullptr);
  EXPECT_STREQ(p1a->Version(), "1a");
  EXPECT_EQ(p1a->ResponseSize(), packets::kAllSensorResponseSize);  // 259
  EXPECT_TRUE(p1a->HasMotorSpaceRead());
  EXPECT_EQ(p1a->JointIoMode(), packets::JointMode::kJoint);  // 1a: gear-mapped joint mode
  EXPECT_TRUE(p1a->RunsSensorPostProcess());
  EXPECT_TRUE(p1a->VerifiesResponseMode());  // 1a: MODE echo is meaningful

  auto p1b = CreateSensorProtocol("1b");
  ASSERT_NE(p1b, nullptr);
  EXPECT_STREQ(p1b->Version(), "1b");
  EXPECT_EQ(p1b->ResponseSize(), packets::kP1bSensorResponseSize);  // 99
  EXPECT_FALSE(p1b->HasMotorSpaceRead());
  EXPECT_EQ(p1b->JointIoMode(), packets::JointMode::kMotor);  // 1b: joint served under kMotor
  EXPECT_FALSE(p1b->RunsSensorPostProcess());
  EXPECT_TRUE(p1b->VerifiesResponseMode());  // 1b: firmware echoes requested MODE accurately
}

TEST(SensorProtocolFactory, RejectsUnknownVersion) {
  EXPECT_EQ(CreateSensorProtocol("2a"), nullptr);
  EXPECT_EQ(CreateSensorProtocol(""), nullptr);
  EXPECT_EQ(CreateSensorProtocol("1"), nullptr);
}

// ── SensorProtocol1b::DecodeAllSensors ──────────────────────────────────────

TEST(SensorProtocol1bDecode, FillsForceForAllFingertips) {
  auto proto = CreateSensorProtocol("1b");
  ASSERT_NE(proto, nullptr);

  const auto vals = SentinelValues();
  const auto buf =
      MakeP1bBuffer(static_cast<uint8_t>(packets::Command::kReadAllSensors), 0x00, vals);

  UdpHandState state{};
  state.num_fingertips = 4;
  ASSERT_TRUE(proto->DecodeAllSensors(buf.data(), buf.size(), state));

  // All 24 float slots populated from the wire.
  for (std::size_t i = 0; i < vals.size(); ++i) {
    EXPECT_FLOAT_EQ(state.sensor_force[i], vals[i]) << "mismatch at index " << i;
  }
  // Tail beyond the 4 wire fingertips stays zero.
  for (std::size_t i = vals.size(); i < state.sensor_force.size(); ++i) {
    EXPECT_FLOAT_EQ(state.sensor_force[i], 0.0f);
  }
  // 1b does not touch the int32 pipeline.
  for (const auto v : state.sensor_data) {
    EXPECT_EQ(v, 0);
  }
}

TEST(SensorProtocol1bDecode, PartialFingertipsLeaveTailZero) {
  auto proto = CreateSensorProtocol("1b");
  ASSERT_NE(proto, nullptr);

  const auto vals = SentinelValues();
  const auto buf =
      MakeP1bBuffer(static_cast<uint8_t>(packets::Command::kReadAllSensors), 0x00, vals);

  UdpHandState state{};
  state.num_fingertips = 2;  // only 2 of 4 wire fingertips consumed
  ASSERT_TRUE(proto->DecodeAllSensors(buf.data(), buf.size(), state));

  constexpr std::size_t kFilled = 2 * kP1bValuesPerFingertip;  // 12
  for (std::size_t i = 0; i < kFilled; ++i) {
    EXPECT_FLOAT_EQ(state.sensor_force[i], vals[i]);
  }
  for (std::size_t i = kFilled; i < state.sensor_force.size(); ++i) {
    EXPECT_FLOAT_EQ(state.sensor_force[i], 0.0f) << "unexpected fill at index " << i;
  }
}

TEST(SensorProtocol1bDecode, ClampsToWireFingertipCount) {
  auto proto = CreateSensorProtocol("1b");
  ASSERT_NE(proto, nullptr);

  const auto vals = SentinelValues();
  const auto buf =
      MakeP1bBuffer(static_cast<uint8_t>(packets::Command::kReadAllSensors), 0x00, vals);

  UdpHandState state{};
  state.num_fingertips = kMaxFingertips;  // 8 requested, wire only carries 4
  ASSERT_TRUE(proto->DecodeAllSensors(buf.data(), buf.size(), state));

  constexpr std::size_t kWireFloats = 4 * kP1bValuesPerFingertip;  // 24
  for (std::size_t i = 0; i < kWireFloats; ++i) {
    EXPECT_FLOAT_EQ(state.sensor_force[i], vals[i]);
  }
  for (std::size_t i = kWireFloats; i < state.sensor_force.size(); ++i) {
    EXPECT_FLOAT_EQ(state.sensor_force[i], 0.0f);
  }
}

TEST(SensorProtocol1bDecode, RejectsWrongCommand) {
  auto proto = CreateSensorProtocol("1b");
  ASSERT_NE(proto, nullptr);

  const auto vals = SentinelValues();
  const auto buf =
      MakeP1bBuffer(static_cast<uint8_t>(packets::Command::kReadAllMotors), 0x00, vals);

  UdpHandState state{};
  state.num_fingertips = 4;
  EXPECT_FALSE(proto->DecodeAllSensors(buf.data(), buf.size(), state));
}

TEST(SensorProtocol1bDecode, RejectsShortBuffer) {
  auto proto = CreateSensorProtocol("1b");
  ASSERT_NE(proto, nullptr);

  const auto vals = SentinelValues();
  const auto buf =
      MakeP1bBuffer(static_cast<uint8_t>(packets::Command::kReadAllSensors), 0x00, vals);

  UdpHandState state{};
  state.num_fingertips = 4;
  EXPECT_FALSE(proto->DecodeAllSensors(buf.data(), packets::kP1bSensorResponseSize - 1, state));
}

}  // namespace udp_hand_driver::test
