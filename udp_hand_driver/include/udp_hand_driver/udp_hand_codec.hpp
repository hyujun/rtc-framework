#ifndef UDP_HAND_DRIVER_UDP_HAND_CODEC_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_CODEC_HPP_

// Packet encoding / decoding for the hand UDP request-response protocol.
//
// All functions are header-only, noexcept, and allocation-free — safe for use
// on SCHED_FIFO real-time threads.

#include "rtc_base/types/types.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace udp_hand_driver::codec {

// ── Protocol constants ───────────────────────────────────────────────────
inline constexpr std::size_t kMotorPacketBytes = packets::kMotorPacketSize;              // 43
inline constexpr std::size_t kSensorRequestBytes = packets::kSensorRequestSize;          // 3
inline constexpr std::size_t kSensorResponseBytes = packets::kSensorResponseSize;        // 67
inline constexpr std::size_t kAllMotorRequestBytes = packets::kAllMotorRequestSize;      // 3
inline constexpr std::size_t kAllMotorResponseBytes = packets::kAllMotorResponseSize;    // 123
inline constexpr std::size_t kAllSensorRequestBytes = packets::kAllSensorRequestSize;    // 3
inline constexpr std::size_t kAllSensorResponseBytes = packets::kAllSensorResponseSize;  // 259
inline constexpr std::size_t kMaxPacketBytes = packets::kMaxPacketSize;                  // 259

// Legacy alias
inline constexpr std::size_t kPacketBytes = kMotorPacketBytes;

// ── Encode requests ──────────────────────────────────────────────────────

// Encode a motor read-request packet (43 bytes).
// Legacy: sends full MotorPacket with zero data. Prefer EncodeMotorReadRequest() (3 bytes).
inline void EncodeReadRequest(packets::Command cmd,
                              std::array<uint8_t, kMotorPacketBytes>& out) noexcept {
  auto pkt = packets::MakeReadRequest(cmd);
  packets::SerializePacket(pkt, out);
}

// Encode a motor read-request packet (3 bytes, header only).
// Read position/velocity requests only need the header — no data payload required.
// joint_mode selects motor (0x00) or joint (0x01) data from firmware.
inline void EncodeMotorReadRequest(
    packets::Command cmd, std::array<uint8_t, kSensorRequestBytes>& out,
    packets::JointMode joint_mode = packets::JointMode::kMotor) noexcept {
  auto pkt = packets::MakeMotorReadRequest(cmd, joint_mode);
  packets::SerializeSensorRequest(pkt, out);
}

// Encode a sensor read-request packet (3 bytes, header only).
// MODE field carries the desired sensor mode (kRaw=0 or kNn=1, default kRaw).
inline void EncodeSensorReadRequest(
    packets::Command cmd, std::array<uint8_t, kSensorRequestBytes>& out,
    packets::SensorMode sensor_mode = packets::SensorMode::kRaw) noexcept {
  auto pkt = packets::MakeSensorReadRequest(cmd, sensor_mode);
  packets::SerializeSensorRequest(pkt, out);
}

// Encode a set-sensor-mode request packet (3 bytes).
// CMD=0x04, MODE field = desired SensorMode. Sensor init before reading data.
inline void EncodeSetSensorMode(packets::SensorMode sensor_mode,
                                std::array<uint8_t, kSensorRequestBytes>& out) noexcept {
  auto pkt = packets::MakeSetSensorMode(sensor_mode);
  packets::SerializeSensorRequest(pkt, out);
}

// Encode a bulk motor read-request packet (3 bytes, cmd=0x10).
// joint_mode selects motor (0x00) or joint (0x01) data from firmware.
inline void EncodeReadAllMotorsRequest(
    std::array<uint8_t, kAllMotorRequestBytes>& out,
    packets::JointMode joint_mode = packets::JointMode::kMotor) noexcept {
  auto pkt = packets::MakeReadAllMotorsRequest(joint_mode);
  packets::SerializeSensorRequest(pkt, out);
}

// Encode a bulk sensor read-request packet (3 bytes, cmd=0x19).
inline void EncodeReadAllSensorsRequest(
    std::array<uint8_t, kAllSensorRequestBytes>& out,
    packets::SensorMode sensor_mode = packets::SensorMode::kRaw) noexcept {
  auto pkt = packets::MakeReadAllSensorsRequest(sensor_mode);
  packets::SerializeSensorRequest(pkt, out);
}

// Encode a write-position packet (43 bytes).
// joint_mode selects motor (0x00, default) or joint (0x01) position interpretation.
inline void EncodeWritePosition(
    const std::array<float, kNumHandMotors>& positions, std::array<uint8_t, kMotorPacketBytes>& out,
    packets::JointMode joint_mode = packets::JointMode::kMotor) noexcept {
  auto pkt = packets::MakeWritePosition(positions, joint_mode);
  packets::SerializePacket(pkt, out);
}

// ── Decode responses ─────────────────────────────────────────────────────

// Decode a motor response packet (43 bytes), extracting 10 float values.
[[nodiscard]] inline bool DecodeMotorResponse(
    const uint8_t* buf, std::size_t len, uint8_t& cmd_out, uint8_t& mode_out,
    std::array<float, packets::kMotorDataCount>& data_out) noexcept {
  packets::MotorPacket pkt{};
  if (!packets::DecodeMotorPacket(buf, len, pkt))
    return false;
  cmd_out = pkt.cmd;
  mode_out = pkt.mode;
  packets::ExtractMotorFloats(pkt, data_out);
  return true;
}

// Decode a sensor response packet (67 bytes), extracting 11 useful values
// (barometer[8] + tof[3], skipping reserved[5]).
[[nodiscard]] inline bool DecodeSensorResponse(
    const uint8_t* buf, std::size_t len, uint8_t& cmd_out, uint8_t& mode_out,
    std::array<float, udp_hand_driver::kSensorValuesPerFingertip>& data_out) noexcept {
  packets::SensorResponsePacket pkt{};
  if (!packets::DecodeSensorResponse(buf, len, pkt))
    return false;
  cmd_out = pkt.cmd;
  mode_out = pkt.mode;
  packets::ExtractSensorValues(pkt, data_out);
  return true;
}

// Decode a sensor response packet (67 bytes), extracting 11 raw int32 values
// (barometer[8] + tof[3], skipping reserved[5]). No float conversion.
[[nodiscard]] inline bool DecodeSensorResponseRaw(
    const uint8_t* buf, std::size_t len, uint8_t& cmd_out, uint8_t& mode_out,
    std::array<int32_t, udp_hand_driver::kSensorValuesPerFingertip>& data_out) noexcept {
  packets::SensorResponsePacket pkt{};
  if (!packets::DecodeSensorResponse(buf, len, pkt))
    return false;
  cmd_out = pkt.cmd;
  mode_out = pkt.mode;
  packets::ExtractSensorValuesRaw(pkt, data_out);
  return true;
}

// Decode a bulk motor response (123 bytes), extracting positions[10], velocities[10], currents[10].
// Data layout: grouped [pos0..9, vel0..9, cur0..9].
[[nodiscard]] inline bool DecodeAllMotorResponse(
    const uint8_t* buf, std::size_t len, uint8_t& cmd_out, uint8_t& mode_out,
    std::array<float, packets::kMotorDataCount>& positions,
    std::array<float, packets::kMotorDataCount>& velocities,
    std::array<float, packets::kMotorDataCount>& currents) noexcept {
  packets::AllMotorResponsePacket pkt{};
  if (!packets::DecodeAllMotorResponse(buf, len, pkt))
    return false;
  cmd_out = pkt.cmd;
  mode_out = pkt.mode;
  packets::ExtractAllMotorFloats(pkt, positions, velocities, currents);
  return true;
}

// Decode a bulk sensor response (259 bytes), extracting all fingertip data.
// Output: concatenated barometer[8]+tof[3] per finger (num_fingertips × 11 raw int32).
[[nodiscard]] inline bool DecodeAllSensorResponseRaw(const uint8_t* buf, std::size_t len,
                                                     uint8_t& cmd_out, uint8_t& mode_out,
                                                     int32_t* out, int num_fingertips) noexcept {
  packets::AllSensorResponsePacket pkt{};
  if (!packets::DecodeAllSensorResponse(buf, len, pkt))
    return false;
  cmd_out = pkt.cmd;
  mode_out = pkt.mode;
  packets::ExtractAllSensorValuesRaw(pkt, out, num_fingertips);
  return true;
}

// Legacy: Decode a motor response (without mode output).
[[nodiscard]] inline bool DecodeResponse(
    const uint8_t* buf, std::size_t len, uint8_t& cmd_out,
    std::array<float, packets::kMotorDataCount>& data_out) noexcept {
  uint8_t mode_unused;
  return DecodeMotorResponse(buf, len, cmd_out, mode_unused, data_out);
}

}  // namespace udp_hand_driver::codec

#endif  // UDP_HAND_DRIVER_UDP_HAND_CODEC_HPP_
