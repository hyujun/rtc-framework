#ifndef UDP_HAND_DRIVER_PROTOCOL_SENSOR_PROTOCOL_CANFD_HPP_
#define UDP_HAND_DRIVER_PROTOCOL_SENSOR_PROTOCOL_CANFD_HPP_

// SensorProtocol2: the proto2 (CAN FD) sensor seam (epic #895, H1-F04-2c).
//
// On this wire the "bulk sensor response" is the raw 192-byte RespSet that
// CanFdHandComm::ReadCycle copies into HandReadBuffers::sensor_raw on a sensor
// cycle; the joint triplet comes from the same set, so there is no separate
// sensor request and no motor-space read (HasMotorSpaceRead false, like 1b).
// Tactile values are decoded through hand_canfd_packets.hpp (signs and metres
// included) into the 1b force slots — sensor_force[f*6 + {fx, fy, fz, px, py,
// temp}] — so the existing 1b publish path carries fx fy fz unchanged; the
// contact flag goes to UdpHandState::sensor_contact[f] (D29: the one field
// proto2 adds to the sensor message). CreateSensorProtocol("2") builds it.
// Header-only: the controller's SensorProtocol seam is the only consumer.
//
// RT: DecodeAllSensors is noexcept and allocation-free (memcpy + arithmetic).

#include "udp_hand_driver/hand_canfd_packets.hpp"
#include "udp_hand_driver/protocol/sensor_protocol.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_state.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace udp_hand_driver {

class SensorProtocol2 final : public SensorProtocol {
 public:
  /// Force slots per fingertip, shared with 1b: fx fy fz px py temp.
  static constexpr std::size_t kValuesPerFingertip = 6;
  static_assert(kValuesPerFingertip == static_cast<std::size_t>(kP1bValuesPerFingertip),
                "proto2 reuses the 1b sensor_force layout");

  [[nodiscard]] std::size_t ResponseSize() const noexcept override {
    return sizeof(canfd::RespSet);  // 192
  }

  [[nodiscard]] bool HasMotorSpaceRead() const noexcept override { return false; }

  /// No MODE byte on this wire; the firmware works in motor space (W1).
  [[nodiscard]] packets::JointMode JointIoMode() const noexcept override {
    return packets::JointMode::kMotor;
  }

  [[nodiscard]] bool RunsSensorPostProcess() const noexcept override { return false; }

  /// `buf` is a RespSet (len must be 192). Fingertips beyond
  /// state.num_fingertips (clamped to the wire's 4) are left untouched.
  [[nodiscard]] bool DecodeAllSensors(const uint8_t* buf, std::size_t len,
                                      UdpHandState& state) noexcept override {
    if (buf == nullptr || len != sizeof(canfd::RespSet))
      return false;
    canfd::RespSet set{};
    std::memcpy(&set, buf, sizeof(set));
    canfd::DecodeTelemetry(set, telemetry_);

    const int requested = state.num_fingertips;
    const std::size_t n = (requested <= 0) ? std::size_t{0}
                                           : std::min(static_cast<std::size_t>(requested),
                                                      canfd::kWireFingertipCount);
    static_assert(canfd::kWireFingertipCount * kValuesPerFingertip <=
                      static_cast<std::size_t>(kMaxSensorForceValues),
                  "sensor_force holds every wire fingertip");
    static_assert(canfd::kWireFingertipCount <= static_cast<std::size_t>(kMaxFingertips),
                  "sensor_contact holds every wire fingertip");
    for (std::size_t f = 0; f < n; ++f) {
      const canfd::Tactile& t = telemetry_.tactile[f];
      float* slot = &state.sensor_force[f * kValuesPerFingertip];
      slot[0] = t.fx_n;
      slot[1] = t.fy_n;
      slot[2] = t.fz_n;
      slot[3] = t.px_m;
      slot[4] = t.py_m;
      slot[5] = t.temp_c;
      state.sensor_contact[f] = t.contact ? 1 : 0;
    }
    return true;
  }

  [[nodiscard]] bool VerifiesResponseMode() const noexcept override { return false; }

  [[nodiscard]] const char* Version() const noexcept override { return "2"; }

  /// The last decoded set in host units (pos / vel / cur / tactile incl.
  /// contact) — for tests and diagnostics.
  [[nodiscard]] const canfd::Telemetry& telemetry() const noexcept { return telemetry_; }

 private:
  canfd::Telemetry telemetry_{};  ///< scratch, reused every decode
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_PROTOCOL_SENSOR_PROTOCOL_CANFD_HPP_
