#include "udp_hand_driver/protocol/sensor_protocol.hpp"

#include "udp_hand_driver/udp_hand_codec.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace udp_hand_driver {
namespace {

// ── Proto_1a: barometer/ToF int32 pipeline (259 B bulk response) ─────────────
// Behaviour-preserving wrapper over the existing codec. Decode fills the int32
// sensor buffers; the EventLoop then runs the LPF/drift/F/T pipeline because
// RunsSensorPostProcess() is true.
class SensorProtocol1a final : public SensorProtocol {
 public:
  [[nodiscard]] std::size_t ResponseSize() const noexcept override {
    return packets::kAllSensorResponseSize;  // 259
  }

  [[nodiscard]] bool HasMotorSpaceRead() const noexcept override { return true; }

  [[nodiscard]] bool RunsSensorPostProcess() const noexcept override { return true; }

  // Caller sets state.num_fingertips before decode (a decode input, not filled
  // by decode). Only that many fingertips' worth of int32 values are written,
  // clamped to the wire's fingertip count so a mis-sized num_fingertips can
  // never index past the 259 B packet (remainder left as-is / zero).
  [[nodiscard]] bool DecodeAllSensors(const uint8_t* buf, std::size_t len,
                                      UdpHandState& state) noexcept override {
    const int requested = state.num_fingertips;
    if (requested <= 0)
      return false;
    const int n = std::min(requested, static_cast<int>(packets::kAllSensorFingertipCount));
    uint8_t cmd_out = 0;
    uint8_t mode_out = 0;
    if (!codec::DecodeAllSensorResponseRaw(buf, len, cmd_out, mode_out,
                                           state.sensor_data_raw.data(), n)) {
      return false;
    }
    if (cmd_out != static_cast<uint8_t>(packets::Command::kReadAllSensors))
      return false;
    // 1a keeps a filtered view (sensor_data) and a raw view (sensor_data_raw);
    // at decode time both hold the freshly decoded raw values. The EventLoop's
    // post-process filters sensor_data in place afterwards.
    state.sensor_data = state.sensor_data_raw;
    return true;
  }

  [[nodiscard]] const char* Version() const noexcept override { return "1a"; }
};

// ── Proto_1b: per-fingertip float32 force (99 B bulk response) ───────────────
// Decode fills state.sensor_force; no post-processing (RunsSensorPostProcess()
// false) and no motor-space read (HasMotorSpaceRead() false).
class SensorProtocol1b final : public SensorProtocol {
 public:
  [[nodiscard]] std::size_t ResponseSize() const noexcept override {
    return packets::kP1bSensorResponseSize;  // 99
  }

  [[nodiscard]] bool HasMotorSpaceRead() const noexcept override { return false; }

  [[nodiscard]] bool RunsSensorPostProcess() const noexcept override { return false; }

  // Caller sets state.num_fingertips before decode; clamped to the wire's fixed
  // fingertip count. Decodes all 6 float32 per fingertip (fx,fy,fz,Lx,Ly,Temp)
  // into sensor_force; downstream publishing selects which components to emit.
  [[nodiscard]] bool DecodeAllSensors(const uint8_t* buf, std::size_t len,
                                      UdpHandState& state) noexcept override {
    packets::P1bSensorResponsePacket pkt{};
    if (!packets::DecodeP1bSensorResponse(buf, len, pkt))
      return false;
    if (pkt.cmd != static_cast<uint8_t>(packets::Command::kReadAllSensors))
      return false;

    const int requested = state.num_fingertips;
    const std::size_t wire = packets::kP1bSensorFingertipCount;
    const std::size_t n =
        (requested <= 0) ? std::size_t{0} : std::min(static_cast<std::size_t>(requested), wire);
    constexpr std::size_t kVals = static_cast<std::size_t>(kP1bValuesPerFingertip);
    for (std::size_t f = 0; f < n; ++f) {
      for (std::size_t i = 0; i < kVals; ++i) {
        state.sensor_force[f * kVals + i] = pkt.data[f * kVals + i];
      }
    }
    return true;
  }

  [[nodiscard]] const char* Version() const noexcept override { return "1b"; }
};

}  // namespace

std::unique_ptr<SensorProtocol> CreateSensorProtocol(const std::string& version) {
  if (version == "1a")
    return std::make_unique<SensorProtocol1a>();
  if (version == "1b")
    return std::make_unique<SensorProtocol1b>();
  return nullptr;
}

}  // namespace udp_hand_driver
