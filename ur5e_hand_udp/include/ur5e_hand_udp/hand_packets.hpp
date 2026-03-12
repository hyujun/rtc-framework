#ifndef UR5E_HAND_UDP_HAND_PACKETS_HPP_
#define UR5E_HAND_UDP_HAND_PACKETS_HPP_

// Wire-format packet definitions for the hand UDP protocol.
//
// Request packet:  [ID: 1B] [CMD: 1B] [MODE: 1B] [data...]
//   - Motor cmd:   data = 10 × uint32_t (float reinterpret)  → total 43 bytes
//   - Sensor cmd:  no data                                    → total  3 bytes
//
// Response packet: [ID: 1B] [CMD: 1B] [MODE: 1B] [data...]
//   - ID, CMD = echo of request
//   - MODE    = current state mode (0=motor, 1=fingertip sensor)
//   - Motor response:  10 × uint32_t (float) → total 43 bytes
//   - Sensor response: 16 × uint32_t         → total 67 bytes
//     Layout per fingertip: barometer[8] + reserved[5] + tof[3]

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "ur5e_rt_base/types/types.hpp"

namespace ur5e_rt_controller {
namespace hand_packets {

// ── Protocol constants ──────────────────────────────────────────────────────
inline constexpr uint8_t kDeviceId     = 0x01;
inline constexpr uint8_t kDefaultMode  = 0x00;

inline constexpr std::size_t kHeaderSize = 3;  // ID + CMD + MODE

// Motor packet constants
inline constexpr std::size_t kMotorDataCount  = 10;
inline constexpr std::size_t kMotorPacketSize = kHeaderSize + kMotorDataCount * sizeof(uint32_t);  // 43

// Sensor packet constants
inline constexpr std::size_t kSensorRequestSize    = kHeaderSize;  // 3 (no data)
inline constexpr std::size_t kSensorResponseDataCount = kSensorDataPerPacket;  // 16
inline constexpr std::size_t kSensorResponseSize   = kHeaderSize + kSensorResponseDataCount * sizeof(uint32_t);  // 67

// Max packet size (for receive buffer allocation)
inline constexpr std::size_t kMaxPacketSize = kSensorResponseSize;  // 67

// Legacy alias
inline constexpr std::size_t kDataCount  = kMotorDataCount;
inline constexpr std::size_t kPacketSize = kMotorPacketSize;

static_assert(kMotorPacketSize == 43, "Motor packet size must be 43 bytes");
static_assert(kSensorRequestSize == 3, "Sensor request size must be 3 bytes");
static_assert(kSensorResponseSize == 67, "Sensor response size must be 67 bytes");

// ── State mode (response mode field) ────────────────────────────────────────
enum class StateMode : uint8_t {
  kMotor           = 0,
  kFingertipSensor = 1,
};

// ── Sensor sub-mode ─────────────────────────────────────────────────────────
enum class SensorMode : uint8_t {
  kRaw = 0,
  kNn  = 1,
};

// ── Command definitions ─────────────────────────────────────────────────────
enum class Command : uint8_t {
  kWritePosition = 0x01,
  kReadPosition  = 0x11,
  kReadVelocity  = 0x12,
  kReadSensor0   = 0x14,
  kReadSensor1   = 0x15,
  kReadSensor2   = 0x16,
  kReadSensor3   = 0x17,
};

// Sensor command for fingertip index [0..3].
inline constexpr Command SensorCommand(int fingertip_idx) noexcept {
  return static_cast<Command>(
      static_cast<uint8_t>(Command::kReadSensor0) + fingertip_idx);
}

// Returns true if the command is a sensor read command.
inline constexpr bool IsSensorCommand(Command cmd) noexcept {
  return cmd >= Command::kReadSensor0 && cmd <= Command::kReadSensor3;
}

inline constexpr bool IsSensorCommand(uint8_t cmd) noexcept {
  return cmd >= static_cast<uint8_t>(Command::kReadSensor0) &&
         cmd <= static_cast<uint8_t>(Command::kReadSensor3);
}

// ── Wire packets (packed) ───────────────────────────────────────────────────

// Motor packet: 43 bytes (used for motor request and response)
#pragma pack(push, 1)
struct MotorPacket {
  uint8_t  id;
  uint8_t  cmd;
  uint8_t  mode;
  std::array<uint32_t, kMotorDataCount> data;
};
#pragma pack(pop)

static_assert(sizeof(MotorPacket) == kMotorPacketSize, "MotorPacket size mismatch");
static_assert(std::is_trivially_copyable_v<MotorPacket>, "Must be trivially copyable");

// Sensor request: 3 bytes (header only, no data)
#pragma pack(push, 1)
struct SensorRequestPacket {
  uint8_t id;
  uint8_t cmd;
  uint8_t mode;
};
#pragma pack(pop)

static_assert(sizeof(SensorRequestPacket) == kSensorRequestSize, "SensorRequestPacket size mismatch");

// Sensor response: 67 bytes
#pragma pack(push, 1)
struct SensorResponsePacket {
  uint8_t  id;
  uint8_t  cmd;
  uint8_t  mode;
  std::array<uint32_t, kSensorResponseDataCount> data;  // barometer[8] + reserved[5] + tof[3]
};
#pragma pack(pop)

static_assert(sizeof(SensorResponsePacket) == kSensorResponseSize, "SensorResponsePacket size mismatch");

// Legacy alias
using HandPacket = MotorPacket;

// ── Conversion helpers (uint32 ↔ float, little-endian) ──────────────────────

inline float Uint32ToFloat(uint32_t raw) noexcept {
  float f;
  std::memcpy(&f, &raw, sizeof(float));
  return f;
}

inline uint32_t FloatToUint32(float f) noexcept {
  uint32_t raw;
  std::memcpy(&raw, &f, sizeof(uint32_t));
  return raw;
}

// ── Encode helpers ──────────────────────────────────────────────────────────

// Build a motor read-request packet (data filled with zeros).
inline MotorPacket MakeReadRequest(Command cmd) noexcept {
  MotorPacket pkt{};
  pkt.id   = kDeviceId;
  pkt.cmd  = static_cast<uint8_t>(cmd);
  pkt.mode = kDefaultMode;
  pkt.data = {};
  return pkt;
}

// Build a sensor read-request packet (header only, 3 bytes).
inline SensorRequestPacket MakeSensorReadRequest(Command cmd) noexcept {
  SensorRequestPacket pkt{};
  pkt.id   = kDeviceId;
  pkt.cmd  = static_cast<uint8_t>(cmd);
  pkt.mode = kDefaultMode;
  return pkt;
}

// Build a write-position packet with float data.
inline MotorPacket MakeWritePosition(
    const std::array<float, kNumHandMotors>& positions) noexcept {
  MotorPacket pkt{};
  pkt.id   = kDeviceId;
  pkt.cmd  = static_cast<uint8_t>(Command::kWritePosition);
  pkt.mode = kDefaultMode;
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    pkt.data[i] = FloatToUint32(positions[i]);
  }
  return pkt;
}

// ── Decode helpers ──────────────────────────────────────────────────────────

// Decode raw bytes into a MotorPacket. Returns false if buffer too small.
[[nodiscard]] inline bool DecodeMotorPacket(
    const uint8_t* buf, std::size_t len, MotorPacket& out) noexcept {
  if (len < kMotorPacketSize) return false;
  std::memcpy(&out, buf, kMotorPacketSize);
  return true;
}

// Decode raw bytes into a SensorResponsePacket. Returns false if buffer too small.
[[nodiscard]] inline bool DecodeSensorResponse(
    const uint8_t* buf, std::size_t len, SensorResponsePacket& out) noexcept {
  if (len < kSensorResponseSize) return false;
  std::memcpy(&out, buf, kSensorResponseSize);
  return true;
}

// Extract 10 float values from a motor packet's data field.
inline void ExtractMotorFloats(
    const MotorPacket& pkt,
    std::array<float, kMotorDataCount>& out) noexcept {
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    out[i] = Uint32ToFloat(pkt.data[i]);
  }
}

// Extract sensor values from a sensor response, skipping reserved fields.
// Output: barometer[8] + tof[3] = 11 useful values.
inline void ExtractSensorValues(
    const SensorResponsePacket& pkt,
    std::array<float, kSensorValuesPerFingertip>& out) noexcept {
  // barometer: data[0..7] → out[0..7]
  for (std::size_t i = 0; i < kBarometerCount; ++i) {
    out[i] = Uint32ToFloat(pkt.data[i]);
  }
  // skip reserved: data[8..12]
  // tof: data[13..15] → out[8..10]
  for (std::size_t i = 0; i < kTofCount; ++i) {
    out[kBarometerCount + i] = Uint32ToFloat(pkt.data[kBarometerCount + kReservedCount + i]);
  }
}

// Serialize a MotorPacket into a byte buffer.
inline void SerializePacket(
    const MotorPacket& pkt,
    std::array<uint8_t, kMotorPacketSize>& out) noexcept {
  std::memcpy(out.data(), &pkt, kMotorPacketSize);
}

// Serialize a SensorRequestPacket into a byte buffer.
inline void SerializeSensorRequest(
    const SensorRequestPacket& pkt,
    std::array<uint8_t, kSensorRequestSize>& out) noexcept {
  std::memcpy(out.data(), &pkt, kSensorRequestSize);
}

// ── Legacy aliases ──────────────────────────────────────────────────────────
// Keep backward compatibility for code using old names.
[[nodiscard]] inline bool DecodePacket(
    const uint8_t* buf, std::size_t len, MotorPacket& out) noexcept {
  return DecodeMotorPacket(buf, len, out);
}

inline void ExtractFloats(
    const MotorPacket& pkt,
    std::array<float, kMotorDataCount>& out) noexcept {
  ExtractMotorFloats(pkt, out);
}

}  // namespace hand_packets
}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_PACKETS_HPP_
