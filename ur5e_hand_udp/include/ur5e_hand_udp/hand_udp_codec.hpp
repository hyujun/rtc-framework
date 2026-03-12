#ifndef UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
#define UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_

// Packet encoding / decoding for the hand UDP protocol.
//
// All functions are header-only, noexcept, and allocation-free — safe for use
// on SCHED_FIFO real-time threads.
//
// Receive packet layout (616 bytes = 77 doubles, little-endian):
//   [ 0..10] motor_positions  (11)
//   [11..21] motor_velocities (11)
//   [22..32] motor_currents   (11)
//   [33..76] sensor_data      (44)
//
// Send packet layout (88 bytes = 11 doubles, little-endian):
//   [ 0..10] motor_commands   (11)

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "ur5e_rt_base/types.hpp"

namespace ur5e_rt_controller {
namespace hand_udp_codec {

// ── Protocol constants ──────────────────────────────────────────────────────
inline constexpr std::size_t kRecvDoubles = 77;  // 11 + 11 + 11 + 44
inline constexpr std::size_t kRecvBytes   = kRecvDoubles * sizeof(double);  // 616

inline constexpr std::size_t kSendDoubles = kNumHandJoints;  // 11
inline constexpr std::size_t kSendBytes   = kSendDoubles * sizeof(double);  // 88

// ── Decode ──────────────────────────────────────────────────────────────────

// Decodes a received UDP packet into a full HandState.
// Returns false if the buffer is too small.
[[nodiscard]] inline bool DecodeRecvPacket(
    std::span<const char> buffer,
    HandState& out) noexcept {
  if (buffer.size() < kRecvBytes) {
    return false;
  }

  const auto* src = buffer.data();

  std::memcpy(out.motor_positions.data(),  src,
              kNumHandJoints * sizeof(double));
  std::memcpy(out.motor_velocities.data(), src + kNumHandJoints * sizeof(double),
              kNumHandJoints * sizeof(double));
  std::memcpy(out.motor_currents.data(),   src + 2U * kNumHandJoints * sizeof(double),
              kNumHandJoints * sizeof(double));
  std::memcpy(out.sensor_data.data(),      src + 3U * kNumHandJoints * sizeof(double),
              kNumHandSensors * sizeof(double));

  out.valid = true;
  return true;
}

// Lightweight version: decodes only motor_positions (first 11 doubles).
// Use when velocity / current / sensor data is not needed.
[[nodiscard]] inline bool DecodeMotorPositions(
    std::span<const char> buffer,
    std::span<double, kNumHandJoints> out) noexcept {
  if (buffer.size() < kRecvBytes) {
    return false;
  }

  std::memcpy(out.data(), buffer.data(), kNumHandJoints * sizeof(double));
  return true;
}

// ── Encode ──────────────────────────────────────────────────────────────────

// Encodes hand motor commands into a pre-allocated byte buffer.
// Zero heap allocation — the caller owns the output buffer.
inline void EncodeSendPacket(
    std::span<const double, kNumHandJoints> commands,
    std::span<uint8_t, kSendBytes> out) noexcept {
  std::memcpy(out.data(), commands.data(), kSendBytes);
}

}  // namespace hand_udp_codec
}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
