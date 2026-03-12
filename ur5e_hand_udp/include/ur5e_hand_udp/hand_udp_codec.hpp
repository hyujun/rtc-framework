#ifndef UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
#define UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_

// Packet encoding / decoding for the hand UDP protocol.
//
// All functions are header-only, noexcept, and allocation-free — safe for use
// on SCHED_FIFO real-time threads.
//
// Wire format is defined by packed structs in hand_packets.hpp.
// sizeof(PacketStruct) == wire size — no manual byte-offset arithmetic.
//
// This header wraps hand_packets.hpp and provides the legacy API names
// (DecodeRecvPacket, DecodeMotorPositions, EncodeSendPacket) so that
// existing call-sites continue to compile without changes.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "ur5e_rt_base/types.hpp"
#include "ur5e_hand_udp/hand_packets.hpp"

namespace ur5e_rt_controller {
namespace hand_udp_codec {

// ── Protocol constants (derived from packed struct sizeof) ────────────────
inline constexpr std::size_t kRecvDoubles = 77;  // 11 + 11 + 11 + 44
inline constexpr std::size_t kRecvBytes   = hand_packets::kHandRecvBytes;   // 616
inline constexpr std::size_t kSendDoubles = kNumHandJoints;                 // 11
inline constexpr std::size_t kSendBytes   = hand_packets::kHandSendBytes;   // 88

// ── Decode ───────────────────────────────────────────────────────────────

// Decodes a received UDP packet into a full HandState.
[[nodiscard]] inline bool DecodeRecvPacket(
    std::span<const char> buffer,
    HandState& out) noexcept {
  return hand_packets::Decode(buffer, out);
}

// Lightweight: decodes only motor_positions (first 11 doubles).
[[nodiscard]] inline bool DecodeMotorPositions(
    std::span<const char> buffer,
    std::span<double, kNumHandJoints> out) noexcept {
  return hand_packets::DecodePosition(buffer, out);
}

// Lightweight: decodes only motor_velocities (doubles [11..21]).
[[nodiscard]] inline bool DecodeMotorVelocities(
    std::span<const char> buffer,
    std::span<double, kNumHandJoints> out) noexcept {
  return hand_packets::DecodeVelocity(buffer, out);
}

// ── Encode ───────────────────────────────────────────────────────────────

// Encodes hand motor commands into a pre-allocated byte buffer.
inline void EncodeSendPacket(
    std::span<const double, kNumHandJoints> commands,
    std::span<uint8_t, kSendBytes> out) noexcept {
  hand_packets::Encode(commands, out);
}

}  // namespace hand_udp_codec
}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
