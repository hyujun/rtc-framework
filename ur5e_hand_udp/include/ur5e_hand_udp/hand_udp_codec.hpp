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
inline constexpr std::size_t kMotorPacketBytes         = hand_packets::kMotorPacketSize;         // 43
inline constexpr std::size_t kSensorRequestBytes       = hand_packets::kSensorRequestSize;       // 3
inline constexpr std::size_t kSensorResponseBytes      = hand_packets::kSensorResponseSize;      // 67
inline constexpr std::size_t kAllMotorRequestBytes     = hand_packets::kAllMotorRequestSize;     // 3
inline constexpr std::size_t kAllMotorResponseBytes    = hand_packets::kAllMotorResponseSize;    // 123
inline constexpr std::size_t kAllSensorRequestBytes    = hand_packets::kAllSensorRequestSize;    // 3
inline constexpr std::size_t kAllSensorResponseBytes   = hand_packets::kAllSensorResponseSize;   // 259
inline constexpr std::size_t kMaxPacketBytes            = hand_packets::kMaxPacketSize;           // 259

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
// MODE field carries the desired sensor mode (kRaw=0 or kNn=1, default kRaw).
inline void EncodeSensorReadRequest(
    hand_packets::Command cmd,
    std::array<uint8_t, kSensorRequestBytes>& out,
    hand_packets::SensorMode sensor_mode = hand_packets::SensorMode::kRaw) noexcept {
  auto pkt = hand_packets::MakeSensorReadRequest(cmd, sensor_mode);
  hand_packets::SerializeSensorRequest(pkt, out);
}

// Encode a set-sensor-mode request packet (3 bytes).
// CMD=0x04, MODE field = desired SensorMode. Sensor init before reading data.
inline void EncodeSetSensorMode(
    hand_packets::SensorMode sensor_mode,
    std::array<uint8_t, kSensorRequestBytes>& out) noexcept {
  auto pkt = hand_packets::MakeSetSensorMode(sensor_mode);
  hand_packets::SerializeSensorRequest(pkt, out);
}

// Encode a bulk motor read-request packet (3 bytes, cmd=0x10).
inline void EncodeReadAllMotorsRequest(
    std::array<uint8_t, kAllMotorRequestBytes>& out) noexcept {
  auto pkt = hand_packets::MakeReadAllMotorsRequest();
  hand_packets::SerializeSensorRequest(pkt, out);
}

// Encode a bulk sensor read-request packet (3 bytes, cmd=0x19).
inline void EncodeReadAllSensorsRequest(
    std::array<uint8_t, kAllSensorRequestBytes>& out,
    hand_packets::SensorMode sensor_mode = hand_packets::SensorMode::kRaw) noexcept {
  auto pkt = hand_packets::MakeReadAllSensorsRequest(sensor_mode);
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

// Decode a sensor response packet (67 bytes), extracting 11 raw uint32 values
// (barometer[8] + tof[3], skipping reserved[5]). No float conversion.
[[nodiscard]] inline bool DecodeSensorResponseRaw(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out, uint8_t& mode_out,
    std::array<uint32_t, kSensorValuesPerFingertip>& data_out) noexcept {
  hand_packets::SensorResponsePacket pkt{};
  if (!hand_packets::DecodeSensorResponse(buf, len, pkt)) return false;
  cmd_out  = pkt.cmd;
  mode_out = pkt.mode;
  hand_packets::ExtractSensorValuesRaw(pkt, data_out);
  return true;
}

// Decode a bulk motor response (123 bytes), extracting positions[10] and velocities[10].
// Data layout: grouped [pos0..9, vel0..9, cur0..9].
[[nodiscard]] inline bool DecodeAllMotorResponse(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out, uint8_t& mode_out,
    std::array<float, hand_packets::kMotorDataCount>& positions,
    std::array<float, hand_packets::kMotorDataCount>& velocities) noexcept {
  hand_packets::AllMotorResponsePacket pkt{};
  if (!hand_packets::DecodeAllMotorResponse(buf, len, pkt)) return false;
  cmd_out  = pkt.cmd;
  mode_out = pkt.mode;
  hand_packets::ExtractAllMotorFloats(pkt, positions, velocities);
  return true;
}

// Decode a bulk sensor response (259 bytes), extracting all fingertip data.
// Output: concatenated barometer[8]+tof[3] per finger (num_fingertips × 11 raw uint32).
[[nodiscard]] inline bool DecodeAllSensorResponseRaw(
    const uint8_t* buf, std::size_t len,
    uint8_t& cmd_out, uint8_t& mode_out,
    uint32_t* out, int num_fingertips) noexcept {
  hand_packets::AllSensorResponsePacket pkt{};
  if (!hand_packets::DecodeAllSensorResponse(buf, len, pkt)) return false;
  cmd_out  = pkt.cmd;
  mode_out = pkt.mode;
  hand_packets::ExtractAllSensorValuesRaw(pkt, out, num_fingertips);
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
