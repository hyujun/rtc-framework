// Unit tests for hand_udp_codec.hpp — encode/decode request-response codec.
//
// Tier 1: Pure computation, no ROS2 or network dependencies.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_udp_codec.hpp"

#include <array>
#include <cstdint>
#include <cstring>

namespace rtc::hand_udp_codec::test {

using namespace hand_packets;

// ── Encode requests ─────────────────────────────────────────────────────────

TEST(HandUdpCodecEncode, EncodeReadRequest)
{
  std::array<uint8_t, kMotorPacketBytes> buf{};
  EncodeReadRequest(Command::kReadPosition, buf);

  // Header check
  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kReadPosition));
  EXPECT_EQ(buf[2], kDefaultMode);

  // Data should be zeroed
  for (std::size_t i = kHeaderSize; i < kMotorPacketBytes; ++i) {
    EXPECT_EQ(buf[i], 0u);
  }
}

TEST(HandUdpCodecEncode, EncodeMotorReadRequest)
{
  std::array<uint8_t, kSensorRequestBytes> buf{};
  EncodeMotorReadRequest(Command::kReadVelocity, buf, JointMode::kJoint);

  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kReadVelocity));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(JointMode::kJoint));
}

TEST(HandUdpCodecEncode, EncodeSensorReadRequest)
{
  std::array<uint8_t, kSensorRequestBytes> buf{};
  EncodeSensorReadRequest(Command::kReadSensor0, buf, SensorMode::kNn);

  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kReadSensor0));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(SensorMode::kNn));
}

TEST(HandUdpCodecEncode, EncodeSetSensorMode)
{
  std::array<uint8_t, kSensorRequestBytes> buf{};
  EncodeSetSensorMode(SensorMode::kRaw, buf);

  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kSetSensorMode));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(SensorMode::kRaw));
}

TEST(HandUdpCodecEncode, EncodeReadAllMotorsRequest)
{
  std::array<uint8_t, kAllMotorRequestBytes> buf{};
  EncodeReadAllMotorsRequest(buf, JointMode::kMotor);

  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kReadAllMotors));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(JointMode::kMotor));
}

TEST(HandUdpCodecEncode, EncodeReadAllSensorsRequest)
{
  std::array<uint8_t, kAllSensorRequestBytes> buf{};
  EncodeReadAllSensorsRequest(buf, SensorMode::kRaw);

  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kReadAllSensors));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(SensorMode::kRaw));
}

TEST(HandUdpCodecEncode, EncodeWritePosition)
{
  std::array<float, kNumHandMotors> positions{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    positions[static_cast<std::size_t>(i)] = static_cast<float>(i) + 0.5f;
  }

  std::array<uint8_t, kMotorPacketBytes> buf{};
  EncodeWritePosition(positions, buf);

  EXPECT_EQ(buf[0], kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(Command::kWritePosition));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(JointMode::kMotor));

  // Decode to verify float data
  MotorPacket pkt{};
  ASSERT_TRUE(DecodeMotorPacket(buf.data(), buf.size(), pkt));
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_FLOAT_EQ(Uint32ToFloat(pkt.data[i]), positions[i]);
  }
}

TEST(HandUdpCodecEncode, EncodeWritePosition_JointMode)
{
  std::array<float, kNumHandMotors> positions{};
  std::array<uint8_t, kMotorPacketBytes> buf{};
  EncodeWritePosition(positions, buf, JointMode::kJoint);

  EXPECT_EQ(buf[2], static_cast<uint8_t>(JointMode::kJoint));
}

// ── Decode responses ────────────────────────────────────────────────────────

class DecodeMotorResponseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build a synthetic motor response packet
    MotorPacket src{};
    src.id = kDeviceId;
    src.cmd = static_cast<uint8_t>(Command::kReadPosition);
    src.mode = static_cast<uint8_t>(JointMode::kJoint);
    for (std::size_t i = 0; i < kMotorDataCount; ++i) {
      src.data[i] = FloatToUint32(static_cast<float>(i) * 2.0f);
    }
    std::memcpy(buf_.data(), &src, kMotorPacketSize);
  }

  std::array<uint8_t, kMotorPacketSize> buf_{};
};

TEST_F(DecodeMotorResponseTest, ValidDecode)
{
  uint8_t cmd_out{}, mode_out{};
  std::array<float, kMotorDataCount> data_out{};

  ASSERT_TRUE(DecodeMotorResponse(buf_.data(), buf_.size(),
                                  cmd_out, mode_out, data_out));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(Command::kReadPosition));
  EXPECT_EQ(mode_out, static_cast<uint8_t>(JointMode::kJoint));

  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_FLOAT_EQ(data_out[i], static_cast<float>(i) * 2.0f);
  }
}

TEST(HandUdpCodecDecode, DecodeMotorResponse_TooShort)
{
  std::array<uint8_t, 42> buf{};
  uint8_t cmd, mode;
  std::array<float, kMotorDataCount> data{};
  EXPECT_FALSE(DecodeMotorResponse(buf.data(), buf.size(), cmd, mode, data));
}

class DecodeSensorResponseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    SensorResponsePacket src{};
    src.id = kDeviceId;
    src.cmd = static_cast<uint8_t>(Command::kReadSensor0);
    src.mode = static_cast<uint8_t>(SensorMode::kRaw);
    // barometer[0..7] = 100..800
    for (int i = 0; i < kBarometerCount; ++i) {
      src.data[static_cast<std::size_t>(i)] = (i + 1) * 100;
    }
    // reserved[8..12] = 99999
    for (int i = kBarometerCount; i < kBarometerCount + kReservedCount; ++i) {
      src.data[static_cast<std::size_t>(i)] = 99999;
    }
    // tof[13..15] = 10, 20, 30
    for (int i = 0; i < kTofCount; ++i) {
      src.data[static_cast<std::size_t>(kBarometerCount + kReservedCount + i)] =
          (i + 1) * 10;
    }
    std::memcpy(buf_.data(), &src, kSensorResponseSize);
  }

  std::array<uint8_t, kSensorResponseSize> buf_{};
};

TEST_F(DecodeSensorResponseTest, ValidDecode_Float)
{
  uint8_t cmd_out{}, mode_out{};
  std::array<float, kSensorValuesPerFingertip> data_out{};

  ASSERT_TRUE(DecodeSensorResponse(buf_.data(), buf_.size(),
                                   cmd_out, mode_out, data_out));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(Command::kReadSensor0));
  EXPECT_EQ(mode_out, static_cast<uint8_t>(SensorMode::kRaw));

  // barometer: out[0..7]
  for (int i = 0; i < kBarometerCount; ++i) {
    EXPECT_FLOAT_EQ(data_out[static_cast<std::size_t>(i)],
                    static_cast<float>((i + 1) * 100));
  }
  // tof: out[8..10]
  for (int i = 0; i < kTofCount; ++i) {
    EXPECT_FLOAT_EQ(data_out[static_cast<std::size_t>(kBarometerCount + i)],
                    static_cast<float>((i + 1) * 10));
  }
}

TEST_F(DecodeSensorResponseTest, ValidDecode_Raw)
{
  uint8_t cmd_out{}, mode_out{};
  std::array<int32_t, kSensorValuesPerFingertip> data_out{};

  ASSERT_TRUE(DecodeSensorResponseRaw(buf_.data(), buf_.size(),
                                      cmd_out, mode_out, data_out));
  EXPECT_EQ(data_out[0], 100);
  EXPECT_EQ(data_out[7], 800);
  EXPECT_EQ(data_out[8], 10);   // tof[0]
  EXPECT_EQ(data_out[10], 30);  // tof[2]
}

TEST(HandUdpCodecDecode, DecodeSensorResponse_TooShort)
{
  std::array<uint8_t, 66> buf{};
  uint8_t cmd, mode;
  std::array<float, kSensorValuesPerFingertip> data{};
  EXPECT_FALSE(DecodeSensorResponse(buf.data(), buf.size(), cmd, mode, data));
}

// ── Bulk motor decode ───────────────────────────────────────────────────────

TEST(HandUdpCodecDecode, DecodeAllMotorResponse_Valid)
{
  AllMotorResponsePacket src{};
  src.id = kDeviceId;
  src.cmd = static_cast<uint8_t>(Command::kReadAllMotors);
  src.mode = static_cast<uint8_t>(JointMode::kMotor);

  // pos[0..9]=1~10, vel[10..19]=11~20, cur[20..29]=21~30
  for (std::size_t i = 0; i < kAllMotorDataCount; ++i) {
    src.data[i] = FloatToUint32(static_cast<float>(i + 1));
  }

  std::array<uint8_t, kAllMotorResponseSize> buf{};
  std::memcpy(buf.data(), &src, kAllMotorResponseSize);

  uint8_t cmd_out{}, mode_out{};
  std::array<float, kMotorDataCount> pos{}, vel{}, cur{};

  ASSERT_TRUE(DecodeAllMotorResponse(buf.data(), buf.size(),
                                     cmd_out, mode_out, pos, vel, cur));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(Command::kReadAllMotors));
  EXPECT_EQ(mode_out, static_cast<uint8_t>(JointMode::kMotor));

  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_FLOAT_EQ(pos[i], static_cast<float>(i + 1));
    EXPECT_FLOAT_EQ(vel[i], static_cast<float>(kMotorDataCount + i + 1));
    EXPECT_FLOAT_EQ(cur[i], static_cast<float>(2 * kMotorDataCount + i + 1));
  }
}

TEST(HandUdpCodecDecode, DecodeAllMotorResponse_TooShort)
{
  std::array<uint8_t, 122> buf{};
  uint8_t cmd, mode;
  std::array<float, kMotorDataCount> pos{}, vel{}, cur{};
  EXPECT_FALSE(DecodeAllMotorResponse(buf.data(), buf.size(),
                                      cmd, mode, pos, vel, cur));
}

// ── Bulk sensor decode ──────────────────────────────────────────────────────

TEST(HandUdpCodecDecode, DecodeAllSensorResponseRaw_Valid)
{
  AllSensorResponsePacket src{};
  src.id = kDeviceId;
  src.cmd = static_cast<uint8_t>(Command::kReadAllSensors);
  src.mode = static_cast<uint8_t>(SensorMode::kRaw);

  // Fill: finger f, baro b = f*100+b, reserved=-1, tof t = f*1000+t
  for (int f = 0; f < kDefaultNumFingertips; ++f) {
    const auto base = static_cast<std::size_t>(f) * kSensorDataPerPacket;
    for (int b = 0; b < kBarometerCount; ++b) {
      src.data[base + static_cast<std::size_t>(b)] =
          static_cast<int32_t>(f * 100 + b);
    }
    for (int r = 0; r < kReservedCount; ++r) {
      src.data[base + static_cast<std::size_t>(kBarometerCount + r)] = -1;
    }
    for (int t = 0; t < kTofCount; ++t) {
      src.data[base + static_cast<std::size_t>(kBarometerCount + kReservedCount + t)] =
          static_cast<int32_t>(f * 1000 + t);
    }
  }

  std::array<uint8_t, kAllSensorResponseSize> buf{};
  std::memcpy(buf.data(), &src, kAllSensorResponseSize);

  uint8_t cmd_out{}, mode_out{};
  std::array<int32_t, kDefaultNumFingertips * kSensorValuesPerFingertip> out{};

  ASSERT_TRUE(DecodeAllSensorResponseRaw(buf.data(), buf.size(),
                                         cmd_out, mode_out,
                                         out.data(), kDefaultNumFingertips));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(Command::kReadAllSensors));

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

// ── Legacy API ──────────────────────────────────────────────────────────────

TEST(HandUdpCodecLegacy, DecodeResponse_WithoutMode)
{
  MotorPacket src{};
  src.id = kDeviceId;
  src.cmd = static_cast<uint8_t>(Command::kReadPosition);
  src.mode = static_cast<uint8_t>(JointMode::kJoint);
  src.data[0] = FloatToUint32(7.77f);

  std::array<uint8_t, kMotorPacketSize> buf{};
  std::memcpy(buf.data(), &src, kMotorPacketSize);

  uint8_t cmd_out{};
  std::array<float, kMotorDataCount> data_out{};
  ASSERT_TRUE(DecodeResponse(buf.data(), buf.size(), cmd_out, data_out));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(Command::kReadPosition));
  EXPECT_FLOAT_EQ(data_out[0], 7.77f);
}

// ── Encode-Decode Roundtrip ─────────────────────────────────────────────────

TEST(HandUdpCodecRoundtrip, WritePosition_EncodeDecodeRoundtrip)
{
  std::array<float, kNumHandMotors> original{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    original[static_cast<std::size_t>(i)] = static_cast<float>(i) * 3.14f;
  }

  // Encode
  std::array<uint8_t, kMotorPacketBytes> buf{};
  EncodeWritePosition(original, buf, JointMode::kJoint);

  // Decode
  uint8_t cmd_out{}, mode_out{};
  std::array<float, kMotorDataCount> decoded{};
  ASSERT_TRUE(DecodeMotorResponse(buf.data(), buf.size(),
                                  cmd_out, mode_out, decoded));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(Command::kWritePosition));
  EXPECT_EQ(mode_out, static_cast<uint8_t>(JointMode::kJoint));

  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    EXPECT_FLOAT_EQ(decoded[i], original[i]);
  }
}

// ── Bulk sensor decode: too-short buffer ────────────────────────────────────

TEST(HandUdpCodecDecode, DecodeAllSensorResponseRaw_TooShort)
{
  std::array<uint8_t, 258> buf{};
  uint8_t cmd, mode;
  std::array<int32_t, kDefaultNumFingertips * kSensorValuesPerFingertip> out{};
  EXPECT_FALSE(DecodeAllSensorResponseRaw(buf.data(), buf.size(),
                                          cmd, mode,
                                          out.data(), kDefaultNumFingertips));
}

// ── Sensor mode encode-decode roundtrip ─────────────────────────────────────

TEST(HandUdpCodecRoundtrip, SensorMode_EncodeVerify)
{
  std::array<uint8_t, kSensorRequestBytes> buf{};
  EncodeSetSensorMode(hand_packets::SensorMode::kNn, buf);

  EXPECT_EQ(buf[0], hand_packets::kDeviceId);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(hand_packets::Command::kSetSensorMode));
  EXPECT_EQ(buf[2], static_cast<uint8_t>(hand_packets::SensorMode::kNn));
}

// ── Bulk motor encode with JointMode::kJoint ────────────────────────────────

TEST(HandUdpCodecEncode, EncodeReadAllMotorsRequest_JointMode)
{
  std::array<uint8_t, kAllMotorRequestBytes> buf{};
  EncodeReadAllMotorsRequest(buf, hand_packets::JointMode::kJoint);

  EXPECT_EQ(buf[2], static_cast<uint8_t>(hand_packets::JointMode::kJoint));
}

// ── Bulk sensor encode with NnMode ──────────────────────────────────────────

TEST(HandUdpCodecEncode, EncodeReadAllSensorsRequest_NnMode)
{
  std::array<uint8_t, kAllSensorRequestBytes> buf{};
  EncodeReadAllSensorsRequest(buf, hand_packets::SensorMode::kNn);

  EXPECT_EQ(buf[2], static_cast<uint8_t>(hand_packets::SensorMode::kNn));
}

// ── DecodeMotorResponse with wrong cmd: still decodes bytes ─────────────────

TEST(HandUdpCodecDecode, DecodeMotorResponse_AnyCmd)
{
  // Build packet with kReadVelocity cmd
  hand_packets::MotorPacket src{};
  src.id = hand_packets::kDeviceId;
  src.cmd = static_cast<uint8_t>(hand_packets::Command::kReadVelocity);
  src.mode = static_cast<uint8_t>(hand_packets::JointMode::kMotor);
  src.data[0] = hand_packets::FloatToUint32(99.9f);

  std::array<uint8_t, hand_packets::kMotorPacketSize> buf{};
  std::memcpy(buf.data(), &src, hand_packets::kMotorPacketSize);

  uint8_t cmd_out{}, mode_out{};
  std::array<float, hand_packets::kMotorDataCount> data_out{};
  ASSERT_TRUE(DecodeMotorResponse(buf.data(), buf.size(),
                                  cmd_out, mode_out, data_out));
  EXPECT_EQ(cmd_out, static_cast<uint8_t>(hand_packets::Command::kReadVelocity));
  EXPECT_FLOAT_EQ(data_out[0], 99.9f);
}

// ── Protocol constant aliases ───────────────────────────────────────────────

TEST(HandUdpCodecConstants, Aliases)
{
  EXPECT_EQ(kMotorPacketBytes, 43u);
  EXPECT_EQ(kSensorRequestBytes, 3u);
  EXPECT_EQ(kSensorResponseBytes, 67u);
  EXPECT_EQ(kAllMotorRequestBytes, 3u);
  EXPECT_EQ(kAllMotorResponseBytes, 123u);
  EXPECT_EQ(kAllSensorRequestBytes, 3u);
  EXPECT_EQ(kAllSensorResponseBytes, 259u);
  EXPECT_EQ(kMaxPacketBytes, 259u);
  EXPECT_EQ(kPacketBytes, kMotorPacketBytes);  // legacy alias
}

}  // namespace rtc::hand_udp_codec::test
