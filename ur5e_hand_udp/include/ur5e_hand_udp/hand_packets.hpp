#ifndef UR5E_HAND_UDP_HAND_PACKETS_HPP_
#define UR5E_HAND_UDP_HAND_PACKETS_HPP_

// Wire-format packet definitions for the hand UDP protocol.
//
// All structs use #pragma pack(1) so that sizeof(T) == wire size.
// This eliminates manual byte-offset arithmetic — decode/encode becomes
// a single memcpy of sizeof(T) bytes.
//
// Platform assumption: little-endian, IEEE 754 doubles (x86-64).

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "ur5e_rt_base/types/types.hpp"

namespace ur5e_rt_controller {
namespace hand_packets {

#pragma pack(push, 1)

// ── Full receive packet (616 bytes) ──────────────────────────────────────────
// Wire layout: [pos:11][vel:11][cur:11][sensor:44] = 77 doubles
struct HandRecvPacket {
  std::array<double, kNumHandJoints>  motor_positions;   // [ 0..10]
  std::array<double, kNumHandJoints>  motor_velocities;  // [11..21]
  std::array<double, kNumHandJoints>  motor_currents;    // [22..32]
  std::array<double, kNumHandSensors> sensor_data;       // [33..76]
};

// ── Send packet (88 bytes) ───────────────────────────────────────────────────
// Wire layout: [cmd:11] = 11 doubles
struct HandSendPacket {
  std::array<double, kNumHandJoints> motor_commands;
};

// ── Position-only packet (88 bytes) ──────────────────────────────────────────
// Wire layout: [pos:11] = 11 doubles
// Use when only motor positions are needed (lightweight decode).
struct HandPositionPacket {
  std::array<double, kNumHandJoints> motor_positions;
};

// ── Velocity-only packet (88 bytes) ──────────────────────────────────────────
// Wire layout: [vel:11] = 11 doubles
// Starts at byte offset sizeof(HandPositionPacket) in the full recv stream.
struct HandVelocityPacket {
  std::array<double, kNumHandJoints> motor_velocities;
};

#pragma pack(pop)

// ── Compile-time size checks ─────────────────────────────────────────────────
static_assert(sizeof(HandRecvPacket)     == 77 * sizeof(double), "HandRecvPacket size mismatch");
static_assert(sizeof(HandSendPacket)     == 11 * sizeof(double), "HandSendPacket size mismatch");
static_assert(sizeof(HandPositionPacket) == 11 * sizeof(double), "HandPositionPacket size mismatch");
static_assert(sizeof(HandVelocityPacket) == 11 * sizeof(double), "HandVelocityPacket size mismatch");

static_assert(std::is_trivially_copyable_v<HandRecvPacket>,     "Must be trivially copyable");
static_assert(std::is_trivially_copyable_v<HandSendPacket>,     "Must be trivially copyable");
static_assert(std::is_trivially_copyable_v<HandPositionPacket>, "Must be trivially copyable");
static_assert(std::is_trivially_copyable_v<HandVelocityPacket>, "Must be trivially copyable");

// ── Wire-size constants (derived from sizeof) ────────────────────────────────
inline constexpr std::size_t kHandRecvBytes     = sizeof(HandRecvPacket);      // 616
inline constexpr std::size_t kHandSendBytes     = sizeof(HandSendPacket);      // 88
inline constexpr std::size_t kHandPositionBytes = sizeof(HandPositionPacket);  // 88
inline constexpr std::size_t kHandVelocityBytes = sizeof(HandVelocityPacket);  // 88

// Byte offset of velocity data within the full recv packet.
inline constexpr std::size_t kVelocityOffset = offsetof(HandRecvPacket, motor_velocities);

// ── Decode helpers (noexcept, allocation-free) ───────────────────────────────

// Decode full recv packet → HandState.
[[nodiscard]] inline bool Decode(
    std::span<const char> buf, HandState& out) noexcept {
  if (buf.size() < kHandRecvBytes) return false;

  HandRecvPacket pkt;
  std::memcpy(&pkt, buf.data(), sizeof(pkt));

  out.motor_positions  = pkt.motor_positions;
  out.motor_velocities = pkt.motor_velocities;
  out.motor_currents   = pkt.motor_currents;
  out.sensor_data      = pkt.sensor_data;
  out.valid            = true;
  return true;
}

// Decode position-only (first 11 doubles of recv packet).
[[nodiscard]] inline bool DecodePosition(
    std::span<const char> buf,
    std::span<double, kNumHandJoints> out) noexcept {
  if (buf.size() < kHandRecvBytes) return false;

  HandPositionPacket pkt;
  std::memcpy(&pkt, buf.data(), sizeof(pkt));
  std::memcpy(out.data(), pkt.motor_positions.data(), sizeof(pkt));
  return true;
}

// Decode velocity-only (doubles [11..21] of recv packet).
[[nodiscard]] inline bool DecodeVelocity(
    std::span<const char> buf,
    std::span<double, kNumHandJoints> out) noexcept {
  if (buf.size() < kHandRecvBytes) return false;

  HandVelocityPacket pkt;
  std::memcpy(&pkt, buf.data() + kVelocityOffset, sizeof(pkt));
  std::memcpy(out.data(), pkt.motor_velocities.data(), sizeof(pkt));
  return true;
}

// ── Encode helper (noexcept, allocation-free) ────────────────────────────────

// Encode hand commands into a pre-allocated byte buffer.
inline void Encode(
    std::span<const double, kNumHandJoints> commands,
    std::span<uint8_t, kHandSendBytes> out) noexcept {
  HandSendPacket pkt;
  pkt.motor_commands = {{}};
  std::memcpy(pkt.motor_commands.data(), commands.data(),
              kNumHandJoints * sizeof(double));
  std::memcpy(out.data(), &pkt, sizeof(pkt));
}

}  // namespace hand_packets

// ── Codec adapter for UdpTransceiver<HandCodec> ─────────────────────────────
// Satisfies the UdpPacketCodec concept (ur5e_rt_base/udp_codec.hpp).
struct HandCodec {
  using RecvPacket = hand_packets::HandRecvPacket;
  using SendPacket = hand_packets::HandSendPacket;
  using State      = HandState;

  [[nodiscard]] static bool Decode(
      std::span<const char> buf, HandState& out) noexcept {
    return hand_packets::Decode(buf, out);
  }
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_PACKETS_HPP_
