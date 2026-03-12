#ifndef UR5E_HAND_UDP_HAND_PACKETS_HPP_
#define UR5E_HAND_UDP_HAND_PACKETS_HPP_

// Wire-format packet definitions for the hand UDP protocol.
//
// Packet format (43 bytes, little-endian):
//   [ID: 1B] [CMD: 1B] [MODE: 1B] [data: 10 × uint32_t (float reinterpret)]
//
// Communication is request-response:
//   1. Write position:  send(0x01, cmd=0x01, data=positions)
//   2. Read position:   send(0x11) → recv(positions)
//   3. Read velocity:   send(0x12) → recv(velocities)
//   4. Read sensor 0-3: send(0x14..0x17) → recv(sensor data) × 4

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

inline constexpr std::size_t kHeaderSize   = 3;   // ID + CMD + MODE
inline constexpr std::size_t kDataCount    = 10;   // 10 float values per packet
inline constexpr std::size_t kPacketSize   = kHeaderSize + kDataCount * sizeof(uint32_t);  // 43

static_assert(kPacketSize == 43, "Packet size must be 43 bytes");

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

// ── Wire packet (packed, 43 bytes) ──────────────────────────────────────────
#pragma pack(push, 1)
struct HandPacket {
  uint8_t  id;
  uint8_t  cmd;
  uint8_t  mode;
  std::array<uint32_t, kDataCount> data;
};
#pragma pack(pop)

static_assert(sizeof(HandPacket) == kPacketSize, "HandPacket size mismatch");
static_assert(std::is_trivially_copyable_v<HandPacket>, "Must be trivially copyable");

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

// Build a read-request packet (data filled with zeros).
inline HandPacket MakeReadRequest(Command cmd) noexcept {
  HandPacket pkt{};
  pkt.id   = kDeviceId;
  pkt.cmd  = static_cast<uint8_t>(cmd);
  pkt.mode = kDefaultMode;
  pkt.data = {};
  return pkt;
}

// Build a write-position packet with float data.
inline HandPacket MakeWritePosition(
    const std::array<float, kNumHandMotors>& positions) noexcept {
  HandPacket pkt{};
  pkt.id   = kDeviceId;
  pkt.cmd  = static_cast<uint8_t>(Command::kWritePosition);
  pkt.mode = kDefaultMode;
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    pkt.data[i] = FloatToUint32(positions[i]);
  }
  return pkt;
}

// ── Decode helpers ──────────────────────────────────────────────────────────

// Decode raw bytes into a HandPacket. Returns false if buffer too small.
[[nodiscard]] inline bool DecodePacket(
    const uint8_t* buf, std::size_t len, HandPacket& out) noexcept {
  if (len < kPacketSize) return false;
  std::memcpy(&out, buf, kPacketSize);
  return true;
}

// Extract 10 float values from a decoded packet's data field.
inline void ExtractFloats(
    const HandPacket& pkt,
    std::array<float, kDataCount>& out) noexcept {
  for (std::size_t i = 0; i < kDataCount; ++i) {
    out[i] = Uint32ToFloat(pkt.data[i]);
  }
}

// Serialize a HandPacket into a byte buffer.
inline void SerializePacket(
    const HandPacket& pkt,
    std::array<uint8_t, kPacketSize>& out) noexcept {
  std::memcpy(out.data(), &pkt, kPacketSize);
}

}  // namespace hand_packets
}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_PACKETS_HPP_
