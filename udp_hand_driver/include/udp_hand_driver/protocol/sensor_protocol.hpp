#ifndef UDP_HAND_DRIVER_PROTOCOL_SENSOR_PROTOCOL_HPP_
#define UDP_HAND_DRIVER_PROTOCOL_SENSOR_PROTOCOL_HPP_

// SensorProtocol: version seam for the hand's bulk-sensor read path.
//
// The joint/motor read+write path is identical across firmware protocol
// versions; only the *sensor* response differs (wire size, decode, and
// post-processing). This abstract interface localizes that difference so the
// EventLoop never branches on a version string (ARCH-3): it queries polymorphic
// capabilities and dispatches decode through the injected protocol.
//
//   - 1a: 259 B bulk response (4 fingers x 16 int32 barometer/reserved/tof).
//         Decoded into UdpHandState::sensor_data(_raw); the barometer/ToF
//         post-processing pipeline (LPF, drift, F/T inference) runs afterwards.
//   - 1b: 99 B bulk response (3 B header + 4 fingers x 6 float32
//         [fx,fy,fz,Lx,Ly,Temp]). Decoded into UdpHandState::sensor_force;
//         no post-processing (force is pre-computed by firmware). Motor-space
//         (kMotor 0x10) reads are skipped entirely.
//
// RT safety: DecodeAllSensors runs on the EventLoop hot path and must be
// noexcept and allocation-free. Construction/selection happens off the RT path
// (on_configure) via CreateSensorProtocol().

#include "udp_hand_driver/udp_hand_packets.hpp"
#include "udp_hand_driver/udp_hand_state.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace udp_hand_driver {

class SensorProtocol {
 public:
  virtual ~SensorProtocol() = default;

  /// Expected byte size of the bulk-sensor (0x19) UDP response for this
  /// version. 1a = 259, 1b = 99.
  [[nodiscard]] virtual std::size_t ResponseSize() const noexcept = 0;

  /// Whether the driver performs motor-space (kMotor 0x10) reads. 1a = true;
  /// 1b = false (no motor_states published — see migration plan).
  [[nodiscard]] virtual bool HasMotorSpaceRead() const noexcept = 0;

  /// Joint-space I/O MODE byte the driver must send for the position write and
  /// the joint-space (pos/vel/cur) read. 1a firmware uses kJoint (gear-ratio
  /// mapped by firmware); 1b firmware only services joint state/command under
  /// kMotor (0x00) — a kJoint request returns no joint data there. The EventLoop
  /// routes the write + joint read through this instead of a hardcoded mode so
  /// the version difference stays in the seam (ARCH-3). The motor-space read
  /// keeps its own kMotor and is separately gated by HasMotorSpaceRead().
  [[nodiscard]] virtual packets::JointMode JointIoMode() const noexcept = 0;

  /// Whether the barometer/ToF post-processing pipeline (LPF, drift detection,
  /// F/T inference) runs after decode. 1a = true; 1b = false (force values are
  /// pre-computed by firmware). This is the single polymorphic gate the
  /// EventLoop uses to skip that block instead of a version-string branch.
  [[nodiscard]] virtual bool RunsSensorPostProcess() const noexcept = 0;

  /// Decode a validated bulk-sensor response payload into `state`. Called on the
  /// EventLoop hot path — noexcept, allocation-free. `buf`/`len` is the raw
  /// response (already length-checked against ResponseSize() by the caller).
  /// `state.num_fingertips` is read as an input (how many fingertips to decode,
  /// clamped to the wire count); it is not written. 1a fills sensor_data(_raw);
  /// 1b fills sensor_force. Returns false if malformed (too short / header
  /// mismatch).
  [[nodiscard]] virtual bool DecodeAllSensors(const uint8_t* buf, std::size_t len,
                                              UdpHandState& state) noexcept = 0;

  /// Whether the firmware echoes a meaningful MODE byte in the joint / set-mode
  /// responses (both versions do — MODE echoes the requested joint/sensor mode
  /// and is validated on receive). Kept as a capability for future firmware that
  /// may not echo MODE reliably. NOTE: an earlier revision set this false for 1b
  /// under a "MODE is don't-care" reading; that was a partial misdiagnosis — 1b
  /// dropped joint data because the driver requested kJoint while 1b only serves
  /// kMotor (see JointIoMode()), not (only) because MODE was garbage. The joint
  /// / set-mode paths echo MODE correctly on both versions, so both return true;
  /// the bulk-sensor path is gated separately (see VerifiesBulkSensorResponseMode()).
  [[nodiscard]] virtual bool VerifiesResponseMode() const noexcept = 0;

  /// Whether the firmware echoes a meaningful MODE byte specifically in the
  /// bulk-sensor (0x19) response. Defaults to VerifiesResponseMode() — firmware
  /// that echoes MODE at all usually echoes it on every response. 1b overrides
  /// this to false: its 0x19 bulk-sensor response fills the MODE byte with an
  /// arbitrary value even though its joint / set-mode responses echo MODE
  /// correctly, so a strict MODE check there drops every otherwise-valid sensor
  /// frame. Length + CMD (0x19) validation still guard the bulk-sensor path.
  [[nodiscard]] virtual bool VerifiesBulkSensorResponseMode() const noexcept {
    return VerifiesResponseMode();
  }

  /// Version tag ("1a"/"1b") for diagnostics/logging.
  [[nodiscard]] virtual const char* Version() const noexcept = 0;
};

/// Construct the sensor protocol for `version` ("1a" or "1b"). Returns nullptr
/// for an unsupported version — callers (on_configure, off the RT path) must
/// treat nullptr as a configuration failure rather than throwing.
[[nodiscard]] std::unique_ptr<SensorProtocol> CreateSensorProtocol(const std::string& version);

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_PROTOCOL_SENSOR_PROTOCOL_HPP_
