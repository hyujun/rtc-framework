#ifndef UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
#define UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_

// Packet encoding / decoding for the hand UDP request-response protocol.
//
// All functions are header-only, noexcept, and allocation-free — safe for use
// on SCHED_FIFO real-time threads.

#include <array>
#include <cstddef>
#include <cstdint>

#include "ur5e_rt_base/types/types.hpp"
#include "ur5e_hand_udp/hand_packets.hpp"

namespace ur5e_rt_controller {
namespace hand_udp_codec {

// ── Protocol constants ───────────────────────────────────────────────────
inline constexpr std::size_t kPacketBytes = hand_packets::kPacketSize;  // 43

// ── Encode requests ──────────────────────────────────────────────────────

// Encode a read-request packet into a byte buffer.
inline void EncodeReadRequest(
    hand_packets::Command cmd,
    std::array<uint8_t, kPacketBytes>& out) noexcept {
  auto pkt = hand_packets::MakeReadRequest(cmd);
  hand_packets::SerializePacket(pkt, out);
}

// Encode a write-position packet into a byte buffer.
inline void EncodeWritePosition(
    const std::array<float, kNumHandMotors>& positions,
    std::array<uint8_t, kPacketBytes>& out) noexcept {
  auto pkt = hand_packets::MakeWritePosition(positions);
  hand_packets::SerializePacket(pkt, out);
}

// ── Decode response ──────────────────────────────────────────────────────

// Decode a response packet, extracting 10 float values.
// Returns false if the buffer is too small.
[[nodiscard]] inline bool DecodeResponse(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out,
    std::array<float, hand_packets::kDataCount>& data_out) noexcept {
  hand_packets::HandPacket pkt{};
  if (!hand_packets::DecodePacket(buf, len, pkt)) return false;
  cmd_out = pkt.cmd;
  hand_packets::ExtractFloats(pkt, data_out);
  return true;
}

}  // namespace hand_udp_codec
}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
