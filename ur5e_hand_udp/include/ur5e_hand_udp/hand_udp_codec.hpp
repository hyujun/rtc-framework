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
inline constexpr std::size_t kMotorPacketBytes    = hand_packets::kMotorPacketSize;     // 43
inline constexpr std::size_t kSensorRequestBytes  = hand_packets::kSensorRequestSize;   // 3
inline constexpr std::size_t kSensorResponseBytes = hand_packets::kSensorResponseSize;  // 67
inline constexpr std::size_t kMaxPacketBytes       = hand_packets::kMaxPacketSize;       // 67

// Legacy alias
inline constexpr std::size_t kPacketBytes = kMotorPacketBytes;

// ── Encode requests ──────────────────────────────────────────────────────

// Encode a motor read-request packet (43 bytes).
inline void EncodeReadRequest(
    hand_packets::Command cmd,
    std::array<uint8_t, kMotorPacketBytes>& out) noexcept {
  auto pkt = hand_packets::MakeReadRequest(cmd);
  hand_packets::SerializePacket(pkt, out);
}

// Encode a sensor read-request packet (3 bytes, header only).
inline void EncodeSensorReadRequest(
    hand_packets::Command cmd,
    std::array<uint8_t, kSensorRequestBytes>& out) noexcept {
  auto pkt = hand_packets::MakeSensorReadRequest(cmd);
  hand_packets::SerializeSensorRequest(pkt, out);
}

// Encode a write-position packet (43 bytes).
inline void EncodeWritePosition(
    const std::array<float, kNumHandMotors>& positions,
    std::array<uint8_t, kMotorPacketBytes>& out) noexcept {
  auto pkt = hand_packets::MakeWritePosition(positions);
  hand_packets::SerializePacket(pkt, out);
}

// ── Decode responses ─────────────────────────────────────────────────────

// Decode a motor response packet (43 bytes), extracting 10 float values.
[[nodiscard]] inline bool DecodeMotorResponse(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out, uint8_t& mode_out,
    std::array<float, hand_packets::kMotorDataCount>& data_out) noexcept {
  hand_packets::MotorPacket pkt{};
  if (!hand_packets::DecodeMotorPacket(buf, len, pkt)) return false;
  cmd_out  = pkt.cmd;
  mode_out = pkt.mode;
  hand_packets::ExtractMotorFloats(pkt, data_out);
  return true;
}

// Decode a sensor response packet (67 bytes), extracting 11 useful values
// (barometer[8] + tof[3], skipping reserved[5]).
[[nodiscard]] inline bool DecodeSensorResponse(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out, uint8_t& mode_out,
    std::array<float, kSensorValuesPerFingertip>& data_out) noexcept {
  hand_packets::SensorResponsePacket pkt{};
  if (!hand_packets::DecodeSensorResponse(buf, len, pkt)) return false;
  cmd_out  = pkt.cmd;
  mode_out = pkt.mode;
  hand_packets::ExtractSensorValues(pkt, data_out);
  return true;
}

// Legacy: Decode a motor response (without mode output).
[[nodiscard]] inline bool DecodeResponse(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out,
    std::array<float, hand_packets::kMotorDataCount>& data_out) noexcept {
  uint8_t mode_unused;
  return DecodeMotorResponse(buf, len, cmd_out, mode_unused, data_out);
}

}  // namespace hand_udp_codec
}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_UDP_CODEC_HPP_
