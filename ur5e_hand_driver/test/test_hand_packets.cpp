// Unit tests for hand_packets.hpp — packet definitions, encoding, decoding.
//
// Tier 1: Pure computation, no ROS2 or network dependencies.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_packets.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace rtc::hand_packets::test {

// ── Protocol constants ──────────────────────────────────────────────────────

TEST(HandPacketsConstants, PacketSizes)
{
  EXPECT_EQ(kHeaderSize, 3u);
  EXPECT_EQ(kMotorPacketSize, 43u);
  EXPECT_EQ(kSensorRequestSize, 3u);
  EXPECT_EQ(kSensorResponseSize, 67u);
  EXPECT_EQ(kAllMotorRequestSize, 3u);
  EXPECT_EQ(kAllMotorResponseSize, 123u);
  EXPECT_EQ(kAllSensorRequestSize, 3u);
  EXPECT_EQ(kAllSensorResponseSize, 259u);
  EXPECT_EQ(kMaxPacketSize, kAllSensorResponseSize);
}

TEST(HandPacketsConstants, DataCounts)
{
  EXPECT_EQ(kMotorDataCount, 10u);
  EXPECT_EQ(kSensorResponseDataCount, static_cast<std::size_t>(kSensorDataPerPacket));
  EXPECT_EQ(kAllMotorDataCount, 30u);  // 10 * 3
  EXPECT_EQ(kAllSensorDataCount, 64u);  // 4 * 16
}

// ── Enums: SensorCommand ────────────────────────────────────────────────────

TEST(HandPacketsEnums, SensorCommand_MapsCorrectly)
{
  EXPECT_EQ(SensorCommand(0), Command::kReadSensor0);
  EXPECT_EQ(SensorCommand(1), Command::kReadSensor1);
  EXPECT_EQ(SensorCommand(2), Command::kReadSensor2);
  EXPECT_EQ(SensorCommand(3), Command::kReadSensor3);
}

TEST(HandPacketsEnums, SensorCommand_RawValues)
{
  EXPECT_EQ(static_cast<uint8_t>(SensorCommand(0)), 0x14);
  EXPECT_EQ(static_cast<uint8_t>(SensorCommand(1)), 0x15);
  EXPECT_EQ(static_cast<uint8_t>(SensorCommand(2)), 0x16);
  EXPECT_EQ(static_cast<uint8_t>(SensorCommand(3)), 0x17);
}

// ── Enums: IsJointCommand ───────────────────────────────────────────────────

TEST(HandPacketsEnums, IsJointCommand)
{
  EXPECT_TRUE(IsJointCommand(Command::kWritePosition));
  EXPECT_TRUE(IsJointCommand(Command::kReadAllMotors));
  EXPECT_TRUE(IsJointCommand(Command::kReadPosition));
  EXPECT_TRUE(IsJointCommand(Command::kReadVelocity));

  EXPECT_FALSE(IsJointCommand(Command::kSetSensorMode));
  EXPECT_FALSE(IsJointCommand(Command::kReadSensor0));
  EXPECT_FALSE(IsJointCommand(Command::kReadSensor1));
  EXPECT_FALSE(IsJointCommand(Command::kReadSensor2));
  EXPECT_FALSE(IsJointCommand(Command::kReadSensor3));
  EXPECT_FALSE(IsJointCommand(Command::kReadAllSensors));
}

// ── Enums: IsSensorCommand ──────────────────────────────────────────────────

TEST(HandPacketsEnums, IsSensorCommand_EnumOverload)
{
  EXPECT_TRUE(IsSensorCommand(Command::kSetSensorMode));
  EXPECT_TRUE(IsSensorCommand(Command::kReadSensor0));
  EXPECT_TRUE(IsSensorCommand(Command::kReadSensor1));
  EXPECT_TRUE(IsSensorCommand(Command::kReadSensor2));
  EXPECT_TRUE(IsSensorCommand(Command::kReadSensor3));
  EXPECT_TRUE(IsSensorCommand(Command::kReadAllSensors));

  EXPECT_FALSE(IsSensorCommand(Command::kWritePosition));
  EXPECT_FALSE(IsSensorCommand(Command::kReadAllMotors));
  EXPECT_FALSE(IsSensorCommand(Command::kReadPosition));
  EXPECT_FALSE(IsSensorCommand(Command::kReadVelocity));
}

TEST(HandPacketsEnums, IsSensorCommand_Uint8Overload)
{
  EXPECT_TRUE(IsSensorCommand(uint8_t{0x04}));
  EXPECT_TRUE(IsSensorCommand(uint8_t{0x14}));
  EXPECT_TRUE(IsSensorCommand(uint8_t{0x17}));
  EXPECT_TRUE(IsSensorCommand(uint8_t{0x19}));

  EXPECT_FALSE(IsSensorCommand(uint8_t{0x01}));
  EXPECT_FALSE(IsSensorCommand(uint8_t{0x10}));
  EXPECT_FALSE(IsSensorCommand(uint8_t{0x11}));
  EXPECT_FALSE(IsSensorCommand(uint8_t{0x12}));
  // Out of range
  EXPECT_FALSE(IsSensorCommand(uint8_t{0x18}));
  EXPECT_FALSE(IsSensorCommand(uint8_t{0x00}));
  EXPECT_FALSE(IsSensorCommand(uint8_t{0xFF}));
}

// ── Conversion helpers ──────────────────────────────────────────────────────

TEST(HandPacketsConversion, Uint32ToFloat_Roundtrip_Positive)
{
  const float original = 3.14159f;
  const uint32_t encoded = FloatToUint32(original);
  const float decoded = Uint32ToFloat(encoded);
  EXPECT_FLOAT_EQ(decoded, original);
}

TEST(HandPacketsConversion, Uint32ToFloat_Roundtrip_Negative)
{
  const float original = -42.5f;
  const uint32_t encoded = FloatToUint32(original);
  const float decoded = Uint32ToFloat(encoded);
  EXPECT_FLOAT_EQ(decoded, original);
}

TEST(HandPacketsConversion, Uint32ToFloat_Zero)
{
  const float original = 0.0f;
  const uint32_t encoded = FloatToUint32(original);
  const float decoded = Uint32ToFloat(encoded);
  EXPECT_FLOAT_EQ(decoded, original);
  EXPECT_EQ(encoded, 0u);
}

TEST(HandPacketsConversion, Uint32ToFloat_SpecialValues)
{
  // Infinity
  {
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_FLOAT_EQ(Uint32ToFloat(FloatToUint32(inf)), inf);
  }
  // Negative infinity
  {
    const float ninf = -std::numeric_limits<float>::infinity();
    EXPECT_FLOAT_EQ(Uint32ToFloat(FloatToUint32(ninf)), ninf);
  }
  // NaN (bitwise preservation)
  {
    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    const uint32_t encoded = FloatToUint32(nan_val);
    const float decoded = Uint32ToFloat(encoded);
    EXPECT_TRUE(std::isnan(decoded));
  }
  // Max float
  {
    const float max_val = std::numeric_limits<float>::max();
    EXPECT_FLOAT_EQ(Uint32ToFloat(FloatToUint32(max_val)), max_val);
  }
  // Denormalized
  {
    const float denorm = std::numeric_limits<float>::denorm_min();
    EXPECT_FLOAT_EQ(Uint32ToFloat(FloatToUint32(denorm)), denorm);
  }
}

// ── Encode helpers: MakeReadRequest ─────────────────────────────────────────

TEST(HandPacketsEncode, MakeReadRequest)
{
  const auto pkt = MakeReadRequest(Command::kReadPosition);
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kReadPosition));
  EXPECT_EQ(pkt.mode, kDefaultMode);
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_EQ(pkt.data[i], 0u);
  }
}

TEST(HandPacketsEncode, MakeMotorReadRequest_DefaultMode)
{
  const auto pkt = MakeMotorReadRequest(Command::kReadPosition);
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kReadPosition));
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(JointMode::kMotor));
}

TEST(HandPacketsEncode, MakeMotorReadRequest_JointMode)
{
  const auto pkt = MakeMotorReadRequest(Command::kReadPosition, JointMode::kJoint);
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(JointMode::kJoint));
}

TEST(HandPacketsEncode, MakeSensorReadRequest_DefaultRaw)
{
  const auto pkt = MakeSensorReadRequest(Command::kReadSensor0);
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kReadSensor0));
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(SensorMode::kRaw));
}

TEST(HandPacketsEncode, MakeSensorReadRequest_NnMode)
{
  const auto pkt = MakeSensorReadRequest(Command::kReadSensor1, SensorMode::kNn);
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(SensorMode::kNn));
}

TEST(HandPacketsEncode, MakeSetSensorMode)
{
  const auto pkt = MakeSetSensorMode(SensorMode::kRaw);
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kSetSensorMode));
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(SensorMode::kRaw));
}

TEST(HandPacketsEncode, MakeWritePosition)
{
  std::array<float, kNumHandMotors> positions{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    positions[static_cast<std::size_t>(i)] = static_cast<float>(i) * 0.1f;
  }

  const auto pkt = MakeWritePosition(positions);
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kWritePosition));
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(JointMode::kMotor));

  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    EXPECT_FLOAT_EQ(Uint32ToFloat(pkt.data[i]), positions[i]);
  }
}

TEST(HandPacketsEncode, MakeWritePosition_JointMode)
{
  std::array<float, kNumHandMotors> positions{};
  positions[0] = 1.0f;
  const auto pkt = MakeWritePosition(positions, JointMode::kJoint);
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(JointMode::kJoint));
}

TEST(HandPacketsEncode, MakeReadAllMotorsRequest)
{
  const auto pkt = MakeReadAllMotorsRequest();
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kReadAllMotors));
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(JointMode::kMotor));
}

TEST(HandPacketsEncode, MakeReadAllSensorsRequest)
{
  const auto pkt = MakeReadAllSensorsRequest();
  EXPECT_EQ(pkt.id, kDeviceId);
  EXPECT_EQ(pkt.cmd, static_cast<uint8_t>(Command::kReadAllSensors));
  EXPECT_EQ(pkt.mode, static_cast<uint8_t>(SensorMode::kRaw));
}

// ── Decode helpers ──────────────────────────────────────────────────────────

TEST(HandPacketsDecode, DecodeMotorPacket_Valid)
{
  // Build a motor packet and serialize
  auto src = MakeReadRequest(Command::kReadPosition);
  src.data[0] = FloatToUint32(1.5f);
  src.data[9] = FloatToUint32(-2.5f);

  std::array<uint8_t, kMotorPacketSize> buf{};
  std::memcpy(buf.data(), &src, kMotorPacketSize);

  MotorPacket out{};
  EXPECT_TRUE(DecodeMotorPacket(buf.data(), buf.size(), out));
  EXPECT_EQ(out.id, src.id);
  EXPECT_EQ(out.cmd, src.cmd);
  EXPECT_FLOAT_EQ(Uint32ToFloat(out.data[0]), 1.5f);
  EXPECT_FLOAT_EQ(Uint32ToFloat(out.data[9]), -2.5f);
}

TEST(HandPacketsDecode, DecodeMotorPacket_TooShort)
{
  std::array<uint8_t, 42> buf{};
  MotorPacket out{};
  EXPECT_FALSE(DecodeMotorPacket(buf.data(), buf.size(), out));
}

TEST(HandPacketsDecode, DecodeSensorResponse_Valid)
{
  SensorResponsePacket src{};
  src.id = kDeviceId;
  src.cmd = static_cast<uint8_t>(Command::kReadSensor0);
  src.mode = 0x00;
  for (int i = 0; i < static_cast<int>(kSensorResponseDataCount); ++i) {
    src.data[static_cast<std::size_t>(i)] = (i + 1) * 100;
  }

  std::array<uint8_t, kSensorResponseSize> buf{};
  std::memcpy(buf.data(), &src, kSensorResponseSize);

  SensorResponsePacket out{};
  EXPECT_TRUE(DecodeSensorResponse(buf.data(), buf.size(), out));
  EXPECT_EQ(out.cmd, src.cmd);
  EXPECT_EQ(out.data[0], 100);
  EXPECT_EQ(out.data[15], 1600);
}

TEST(HandPacketsDecode, DecodeSensorResponse_TooShort)
{
  std::array<uint8_t, 66> buf{};
  SensorResponsePacket out{};
  EXPECT_FALSE(DecodeSensorResponse(buf.data(), buf.size(), out));
}

TEST(HandPacketsDecode, DecodeAllMotorResponse_Valid)
{
  AllMotorResponsePacket src{};
  src.id = kDeviceId;
  src.cmd = static_cast<uint8_t>(Command::kReadAllMotors);
  // pos[0..9], vel[0..9], cur[0..9]
  for (std::size_t i = 0; i < kAllMotorDataCount; ++i) {
    src.data[i] = FloatToUint32(static_cast<float>(i));
  }

  std::array<uint8_t, kAllMotorResponseSize> buf{};
  std::memcpy(buf.data(), &src, kAllMotorResponseSize);

  AllMotorResponsePacket out{};
  EXPECT_TRUE(DecodeAllMotorResponse(buf.data(), buf.size(), out));
  EXPECT_EQ(out.cmd, src.cmd);
  EXPECT_FLOAT_EQ(Uint32ToFloat(out.data[0]), 0.0f);
  EXPECT_FLOAT_EQ(Uint32ToFloat(out.data[29]), 29.0f);
}

TEST(HandPacketsDecode, DecodeAllMotorResponse_TooShort)
{
  std::array<uint8_t, 122> buf{};
  AllMotorResponsePacket out{};
  EXPECT_FALSE(DecodeAllMotorResponse(buf.data(), buf.size(), out));
}

TEST(HandPacketsDecode, DecodeAllSensorResponse_Valid)
{
  AllSensorResponsePacket src{};
  src.id = kDeviceId;
  src.cmd = static_cast<uint8_t>(Command::kReadAllSensors);
  for (std::size_t i = 0; i < kAllSensorDataCount; ++i) {
    src.data[i] = static_cast<int32_t>(i * 10);
  }

  std::array<uint8_t, kAllSensorResponseSize> buf{};
  std::memcpy(buf.data(), &src, kAllSensorResponseSize);

  AllSensorResponsePacket out{};
  EXPECT_TRUE(DecodeAllSensorResponse(buf.data(), buf.size(), out));
  EXPECT_EQ(out.data[0], 0);
  EXPECT_EQ(out.data[63], 630);
}

TEST(HandPacketsDecode, DecodeAllSensorResponse_TooShort)
{
  std::array<uint8_t, 258> buf{};
  AllSensorResponsePacket out{};
  EXPECT_FALSE(DecodeAllSensorResponse(buf.data(), buf.size(), out));
}

// ── Extract helpers ─────────────────────────────────────────────────────────

TEST(HandPacketsExtract, ExtractMotorFloats)
{
  MotorPacket pkt{};
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    pkt.data[i] = FloatToUint32(static_cast<float>(i) * 1.1f);
  }

  std::array<float, kMotorDataCount> out{};
  ExtractMotorFloats(pkt, out);

  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_FLOAT_EQ(out[i], static_cast<float>(i) * 1.1f);
  }
}

TEST(HandPacketsExtract, ExtractSensorValues_SkipsReserved)
{
  // Layout: barometer[0..7] + reserved[8..12] + tof[13..15]
  SensorResponsePacket pkt{};
  pkt.data.fill(0);
  // barometer
  for (int i = 0; i < kBarometerCount; ++i) {
    pkt.data[static_cast<std::size_t>(i)] = (i + 1) * 1000;
  }
  // reserved
  for (int i = kBarometerCount; i < kBarometerCount + kReservedCount; ++i) {
    pkt.data[static_cast<std::size_t>(i)] = 99999;  // should be skipped
  }
  // tof
  for (int i = 0; i < kTofCount; ++i) {
    pkt.data[static_cast<std::size_t>(kBarometerCount + kReservedCount + i)] = (i + 1) * 500;
  }

  std::array<float, kSensorValuesPerFingertip> out{};
  ExtractSensorValues(pkt, out);

  // barometer: out[0..7]
  for (int i = 0; i < kBarometerCount; ++i) {
    EXPECT_FLOAT_EQ(out[static_cast<std::size_t>(i)],
                    static_cast<float>((i + 1) * 1000));
  }
  // tof: out[8..10]
  for (int i = 0; i < kTofCount; ++i) {
    EXPECT_FLOAT_EQ(out[static_cast<std::size_t>(kBarometerCount + i)],
                    static_cast<float>((i + 1) * 500));
  }
}

TEST(HandPacketsExtract, ExtractSensorValuesRaw_SkipsReserved)
{
  SensorResponsePacket pkt{};
  for (int i = 0; i < kBarometerCount; ++i) {
    pkt.data[static_cast<std::size_t>(i)] = i + 10;
  }
  for (int i = 0; i < kReservedCount; ++i) {
    pkt.data[static_cast<std::size_t>(kBarometerCount + i)] = -1;
  }
  for (int i = 0; i < kTofCount; ++i) {
    pkt.data[static_cast<std::size_t>(kBarometerCount + kReservedCount + i)] = i + 100;
  }

  std::array<int32_t, kSensorValuesPerFingertip> out{};
  ExtractSensorValuesRaw(pkt, out);

  for (int i = 0; i < kBarometerCount; ++i) {
    EXPECT_EQ(out[static_cast<std::size_t>(i)], i + 10);
  }
  for (int i = 0; i < kTofCount; ++i) {
    EXPECT_EQ(out[static_cast<std::size_t>(kBarometerCount + i)], i + 100);
  }
}

TEST(HandPacketsExtract, ExtractAllMotorFloats_GroupedLayout)
{
  AllMotorResponsePacket pkt{};
  // pos[0..9] = 1.0~10.0, vel[10..19] = 11.0~20.0, cur[20..29] = 21.0~30.0
  for (std::size_t i = 0; i < kAllMotorDataCount; ++i) {
    pkt.data[i] = FloatToUint32(static_cast<float>(i + 1));
  }

  std::array<float, kMotorDataCount> pos{}, vel{}, cur{};
  ExtractAllMotorFloats(pkt, pos, vel, cur);

  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_FLOAT_EQ(pos[i], static_cast<float>(i + 1));
    EXPECT_FLOAT_EQ(vel[i], static_cast<float>(kMotorDataCount + i + 1));
    EXPECT_FLOAT_EQ(cur[i], static_cast<float>(2 * kMotorDataCount + i + 1));
  }
}

TEST(HandPacketsExtract, ExtractAllSensorValuesRaw_MultiFingerSkipsReserved)
{
  AllSensorResponsePacket pkt{};
  pkt.data.fill(0);

  // Fill 4 fingers: per finger = [baro0..7, reserved0..4, tof0..2]
  for (int f = 0; f < kDefaultNumFingertips; ++f) {
    const auto base = static_cast<std::size_t>(f) * kSensorDataPerPacket;
    for (int b = 0; b < kBarometerCount; ++b) {
      pkt.data[base + static_cast<std::size_t>(b)] =
          static_cast<int32_t>(f * 100 + b);
    }
    for (int r = 0; r < kReservedCount; ++r) {
      pkt.data[base + static_cast<std::size_t>(kBarometerCount + r)] = -999;
    }
    for (int t = 0; t < kTofCount; ++t) {
      pkt.data[base + static_cast<std::size_t>(kBarometerCount + kReservedCount + t)] =
          static_cast<int32_t>(f * 1000 + t);
    }
  }

  // Output: 4 * kSensorValuesPerFingertip = 4 * 11 = 44
  std::array<int32_t, 44> out{};
  ExtractAllSensorValuesRaw(pkt, out.data(), kDefaultNumFingertips);

  for (int f = 0; f < kDefaultNumFingertips; ++f) {
    const auto out_base = static_cast<std::size_t>(f) * kSensorValuesPerFingertip;
    for (int b = 0; b < kBarometerCount; ++b) {
      EXPECT_EQ(out[out_base + static_cast<std::size_t>(b)], f * 100 + b);
    }
    for (int t = 0; t < kTofCount; ++t) {
      EXPECT_EQ(out[out_base + static_cast<std::size_t>(kBarometerCount + t)],
                f * 1000 + t);
    }
  }
}

// ── Serialization roundtrip ─────────────────────────────────────────────────

TEST(HandPacketsSerialize, MotorPacket_Roundtrip)
{
  std::array<float, kNumHandMotors> positions{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    positions[static_cast<std::size_t>(i)] = static_cast<float>(i) * 0.5f;
  }
  const auto original = MakeWritePosition(positions);

  std::array<uint8_t, kMotorPacketSize> buf{};
  SerializePacket(original, buf);

  MotorPacket decoded{};
  ASSERT_TRUE(DecodeMotorPacket(buf.data(), buf.size(), decoded));
  EXPECT_EQ(decoded.id, original.id);
  EXPECT_EQ(decoded.cmd, original.cmd);
  EXPECT_EQ(decoded.mode, original.mode);
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_EQ(decoded.data[i], original.data[i]);
  }
}

TEST(HandPacketsSerialize, SensorRequest_Roundtrip)
{
  const auto original = MakeSensorReadRequest(Command::kReadSensor2, SensorMode::kNn);

  std::array<uint8_t, kSensorRequestSize> buf{};
  SerializeSensorRequest(original, buf);

  // Manual decode (3 bytes)
  EXPECT_EQ(buf[0], original.id);
  EXPECT_EQ(buf[1], original.cmd);
  EXPECT_EQ(buf[2], original.mode);
}

// ── Legacy aliases ──────────────────────────────────────────────────────────

TEST(HandPacketsLegacy, DecodePacket_IsDecodeMotorPacket)
{
  auto src = MakeReadRequest(Command::kReadPosition);
  std::array<uint8_t, kMotorPacketSize> buf{};
  std::memcpy(buf.data(), &src, kMotorPacketSize);

  MotorPacket out1{}, out2{};
  const bool r1 = DecodeMotorPacket(buf.data(), buf.size(), out1);
  const bool r2 = DecodePacket(buf.data(), buf.size(), out2);
  EXPECT_EQ(r1, r2);
  EXPECT_EQ(std::memcmp(&out1, &out2, sizeof(MotorPacket)), 0);
}

TEST(HandPacketsLegacy, ExtractFloats_IsExtractMotorFloats)
{
  MotorPacket pkt{};
  pkt.data[0] = FloatToUint32(42.0f);

  std::array<float, kMotorDataCount> out1{}, out2{};
  ExtractMotorFloats(pkt, out1);
  ExtractFloats(pkt, out2);
  EXPECT_EQ(out1, out2);
}

}  // namespace rtc::hand_packets::test
