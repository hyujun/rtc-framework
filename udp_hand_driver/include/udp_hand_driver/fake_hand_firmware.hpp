#ifndef UDP_HAND_DRIVER_FAKE_HAND_FIRMWARE_HPP_
#define UDP_HAND_DRIVER_FAKE_HAND_FIRMWARE_HPP_

// Full-SIL loopback firmware simulator for the hand UDP protocol.
//
// FakeHandFirmware is the *device side* of the wire protocol: it consumes the
// exact request datagrams the real udp_hand_node emits and synthesizes the
// exact response datagrams (correct sizes, CMD echo, MODE mirror) a real hand
// controller would send. Paired with an unmodified udp_hand_node
// (use_fake_hand=false, target_ip=127.0.0.1) it exercises the *entire* UDP
// transport path — send/recv, framing, echo/MODE validation, failure detection,
// decode, publish — with no hardware. This is complementary to the controller's
// own use_fake_hand (which bypasses the socket and mirrors commands through an
// LPF): that is loop-model SIL, this is network-level SIL.
//
// Design intent: this class is the *decoder's mirror image*. It is
// self-consistent with the driver's decode expectations, NOT a faithful copy of
// the real firmware — driver↔real-firmware agreement is only ever proven on real
// hardware (Phase 5 is ground truth). Loopback latency is ~µs, so this harness
// validates protocol/decode correctness, not timing.
//
// All response builders write into a caller-provided buffer (no allocation).
// The class is single-threaded: the UDP server loop in fake_hand_firmware_main
// owns one instance and calls BuildResponse() serially.
//
// ARCH note: udp_hand_driver is a hand-specific package (not rtc_*), so
// hand-domain constants live here directly. This is a peer simulator, not a
// second production implementation, so ARCH-3 (abstract-interface-first) does
// not apply.

#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <random>

namespace udp_hand_driver {

// CMD byte of a protocol command (usable in constexpr switch-case labels).
inline constexpr uint8_t FakeHandCmd(packets::Command c) noexcept {
  return static_cast<uint8_t>(c);
}

class FakeHandFirmware {
 public:
  struct Config {
    int num_fingertips = kDefaultNumFingertips;  ///< fingers with live sensor data (rest zeroed)
    bool proto_1b = false;                       ///< false = 1a (int32), true = 1b (float force)
    double lpf_time_constant_s = 0.1;            ///< joint LPF time constant τ [s]
    double effort_stiffness = 1.0;               ///< effort kp: kp·(target−pos)
    double effort_damping = 0.1;                 ///< effort kd: −kd·vel
    double step_rate_hz = 500.0;                 ///< fixed LPF step rate (dt = 1/step_rate_hz)
    int sensor_base_min = 100;                   ///< per-channel base ~ U[min, max] (init once)
    int sensor_base_max = 4000;
    double sensor_noise_stddev = 5.0;  ///< per-response additive N(0, stddev)
    uint32_t seed = 42;                ///< std::mt19937 seed (base + noise)
  };

  explicit FakeHandFirmware(const Config& cfg)
      : cfg_(cfg),
        dt_(CalcDt(cfg.step_rate_hz)),
        inv_dt_(CalcInvDt(cfg.step_rate_hz)),
        alpha_(CalcAlpha(cfg.lpf_time_constant_s, CalcDt(cfg.step_rate_hz))),
        rng_(cfg.seed) {
    InitSensorBases();
  }

  // Dispatch a request datagram to its response builder.
  //   req[0]=ID, req[1]=CMD (dispatch key), req[2]=MODE (mirrored into response).
  // Returns the number of bytes written into `out` (0 = drop / no response, e.g.
  // short header, insufficient capacity, or unknown command). A write command
  // (0x01) latches the new target and advances the LPF model one step; read
  // commands snapshot the current pos/vel/effort. Not const: mutates the joint
  // model (writes) and the RNG (every response draws sensor noise).
  std::size_t BuildResponse(const uint8_t* req, std::size_t len, uint8_t* out, std::size_t cap) {
    if (req == nullptr || out == nullptr || len < packets::kHeaderSize)
      return 0;

    const uint8_t cmd = req[1];
    const uint8_t mode = req[2];

    switch (cmd) {
      case FakeHandCmd(packets::Command::kWritePosition): {  // 0x01
        // Latch the commanded target (only if the full 43 B payload arrived),
        // then advance the model. The echo carries the current position; the
        // driver only checks echo_buf[1] == 0x01, but we return the full 43 B.
        if (len >= packets::kMotorPacketSize) {
          packets::MotorPacket in{};
          std::memcpy(&in, req, packets::kMotorPacketSize);
          for (std::size_t i = 0; i < kNumHandMotors; ++i)
            target_[i] = packets::Uint32ToFloat(in.data[i]);
        }
        StepModel();
        return BuildMotorPacket(cmd, mode, pos_, out, cap);
      }
      case FakeHandCmd(packets::Command::kReadAllMotors):  // 0x10
        return BuildAllMotorPacket(mode, out, cap);
      case FakeHandCmd(packets::Command::kReadPosition):  // 0x11
        return BuildMotorPacket(cmd, mode, pos_, out, cap);
      case FakeHandCmd(packets::Command::kReadVelocity):  // 0x12
        return BuildMotorPacket(cmd, mode, vel_, out, cap);
      case FakeHandCmd(packets::Command::kSetSensorMode):  // 0x04
        return BuildAck(mode, out, cap);
      case FakeHandCmd(packets::Command::kReadSensor0):  // 0x14
      case FakeHandCmd(packets::Command::kReadSensor1):  // 0x15
      case FakeHandCmd(packets::Command::kReadSensor2):  // 0x16
      case FakeHandCmd(packets::Command::kReadSensor3):  // 0x17
        return BuildFingerSensorPacket(
            cmd, mode, static_cast<int>(cmd - Cmd(packets::Command::kReadSensor0)), out, cap);
      case FakeHandCmd(packets::Command::kReadAllSensors):  // 0x19
        return cfg_.proto_1b ? BuildP1bSensorPacket(mode, out, cap)
                             : BuildAllSensorPacket(mode, out, cap);
      default:
        return 0;
    }
  }

  // Advance the first-order joint model one fixed step (dt = 1/step_rate_hz).
  // Identical formula to UdpHandController::FakeLpfStep (cross-referenced —
  // keep in sync):
  //   pos += alpha·(target − pos),  alpha = dt/(τ+dt)
  //   vel  = Δpos/dt
  //   effort = kp·(target − pos) − kd·vel
  void StepModel() noexcept {
    const float alpha = static_cast<float>(alpha_);
    const float inv_dt = static_cast<float>(inv_dt_);
    const float kp = static_cast<float>(cfg_.effort_stiffness);
    const float kd = static_cast<float>(cfg_.effort_damping);
    for (std::size_t i = 0; i < kNumHandMotors; ++i) {
      const float pos_old = pos_[i];
      const float pos_new = pos_old + alpha * (target_[i] - pos_old);
      const float vel = (pos_new - pos_old) * inv_dt;
      pos_[i] = pos_new;
      vel_[i] = vel;
      effort_[i] = kp * (target_[i] - pos_new) - kd * vel;
    }
  }

  // ── Accessors (unit tests / introspection) ─────────────────────────────────
  [[nodiscard]] const std::array<float, kNumHandMotors>& pos() const noexcept { return pos_; }

  [[nodiscard]] const std::array<float, kNumHandMotors>& vel() const noexcept { return vel_; }

  [[nodiscard]] const std::array<float, kNumHandMotors>& effort() const noexcept { return effort_; }

  [[nodiscard]] const std::array<float, kNumHandMotors>& target() const noexcept { return target_; }

  [[nodiscard]] double alpha() const noexcept { return alpha_; }

 private:
  static constexpr uint8_t Cmd(packets::Command c) noexcept { return static_cast<uint8_t>(c); }

  static double CalcDt(double rate) noexcept { return rate > 0.0 ? 1.0 / rate : 1.0 / 500.0; }

  static double CalcInvDt(double rate) noexcept { return rate > 0.0 ? rate : 500.0; }

  static double CalcAlpha(double tau, double dt) noexcept {
    const double denom = tau + dt;
    return denom > 0.0 ? dt / denom : 1.0;
  }

  void InitSensorBases() {
    const int lo = std::min(cfg_.sensor_base_min, cfg_.sensor_base_max);
    const int hi = std::max(cfg_.sensor_base_min, cfg_.sensor_base_max);
    std::uniform_int_distribution<int> ud(lo, hi);
    for (auto& b : base_1a_)
      b = static_cast<int32_t>(ud(rng_));
    for (auto& b : base_1b_)
      b = static_cast<int32_t>(ud(rng_));
  }

  // Per-response sensor draws: base (fixed at construction) + N(0, stddev).
  [[nodiscard]] int32_t SampleSensor1a(std::size_t channel) {
    std::normal_distribution<double> nd(0.0, cfg_.sensor_noise_stddev);
    return base_1a_[channel] + static_cast<int32_t>(std::lround(nd(rng_)));
  }

  [[nodiscard]] float SampleSensor1b(std::size_t channel) {
    std::normal_distribution<double> nd(0.0, cfg_.sensor_noise_stddev);
    return static_cast<float>(base_1b_[channel]) + static_cast<float>(nd(rng_));
  }

  [[nodiscard]] bool FingerActive(int finger) const noexcept {
    return finger >= 0 && finger < cfg_.num_fingertips;
  }

  // ── Response builders ──────────────────────────────────────────────────────

  // 43 B motor packet echoing `cmd`/`mode` with `data` as the float payload.
  std::size_t BuildMotorPacket(uint8_t cmd, uint8_t mode,
                               const std::array<float, kNumHandMotors>& data, uint8_t* out,
                               std::size_t cap) const {
    if (cap < packets::kMotorPacketSize)
      return 0;
    packets::MotorPacket pkt{};
    pkt.id = packets::kDeviceId;
    pkt.cmd = cmd;
    pkt.mode = mode;
    for (std::size_t i = 0; i < kNumHandMotors; ++i)
      pkt.data[i] = packets::FloatToUint32(data[i]);
    std::memcpy(out, &pkt, packets::kMotorPacketSize);
    return packets::kMotorPacketSize;
  }

  // 123 B bulk motor response: [pos0..9, vel0..9, cur0..9] (cur = effort).
  std::size_t BuildAllMotorPacket(uint8_t mode, uint8_t* out, std::size_t cap) const {
    if (cap < packets::kAllMotorResponseSize)
      return 0;
    packets::AllMotorResponsePacket pkt{};
    pkt.id = packets::kDeviceId;
    pkt.cmd = Cmd(packets::Command::kReadAllMotors);
    pkt.mode = mode;
    for (std::size_t i = 0; i < kNumHandMotors; ++i) {
      pkt.data[i] = packets::FloatToUint32(pos_[i]);
      pkt.data[packets::kMotorDataCount + i] = packets::FloatToUint32(vel_[i]);
      pkt.data[2 * packets::kMotorDataCount + i] = packets::FloatToUint32(effort_[i]);
    }
    std::memcpy(out, &pkt, packets::kAllMotorResponseSize);
    return packets::kAllMotorResponseSize;
  }

  // 67 B single-fingertip sensor response (16 int32: baro8 + reserved5 + tof3).
  std::size_t BuildFingerSensorPacket(uint8_t cmd, uint8_t mode, int finger, uint8_t* out,
                                      std::size_t cap) {
    if (cap < packets::kSensorResponseSize)
      return 0;
    packets::SensorResponsePacket pkt{};
    pkt.id = packets::kDeviceId;
    pkt.cmd = cmd;
    pkt.mode = mode;
    const bool active = FingerActive(finger);
    for (std::size_t j = 0; j < packets::kSensorResponseDataCount; ++j) {
      pkt.data[j] =
          active ? SampleSensor1a(static_cast<std::size_t>(finger) * kSensorDataPerPacket + j) : 0;
    }
    std::memcpy(out, &pkt, packets::kSensorResponseSize);
    return packets::kSensorResponseSize;
  }

  // 259 B bulk sensor response (1a): 4 fingers × 16 int32.
  std::size_t BuildAllSensorPacket(uint8_t mode, uint8_t* out, std::size_t cap) {
    if (cap < packets::kAllSensorResponseSize)
      return 0;
    packets::AllSensorResponsePacket pkt{};
    pkt.id = packets::kDeviceId;
    pkt.cmd = Cmd(packets::Command::kReadAllSensors);
    pkt.mode = mode;
    for (int f = 0; f < static_cast<int>(packets::kAllSensorFingertipCount); ++f) {
      const bool active = FingerActive(f);
      for (std::size_t j = 0; j < kSensorDataPerPacket; ++j) {
        const std::size_t k = static_cast<std::size_t>(f) * kSensorDataPerPacket + j;
        pkt.data[k] = active ? SampleSensor1a(k) : 0;
      }
    }
    std::memcpy(out, &pkt, packets::kAllSensorResponseSize);
    return packets::kAllSensorResponseSize;
  }

  // 99 B bulk sensor response (1b): 4 fingers × 6 float32 [fx,fy,fz,Lx,Ly,Temp].
  std::size_t BuildP1bSensorPacket(uint8_t mode, uint8_t* out, std::size_t cap) {
    if (cap < packets::kP1bSensorResponseSize)
      return 0;
    packets::P1bSensorResponsePacket pkt{};
    pkt.id = packets::kDeviceId;
    pkt.cmd = Cmd(packets::Command::kReadAllSensors);
    pkt.mode = mode;
    for (int f = 0; f < static_cast<int>(packets::kP1bSensorFingertipCount); ++f) {
      const bool active = FingerActive(f);
      for (std::size_t v = 0; v < packets::kP1bSensorFloatsPerFingertip; ++v) {
        const std::size_t k =
            static_cast<std::size_t>(f) * packets::kP1bSensorFloatsPerFingertip + v;
        pkt.data[k] = active ? SampleSensor1b(k) : 0.0f;
      }
    }
    std::memcpy(out, &pkt, packets::kP1bSensorResponseSize);
    return packets::kP1bSensorResponseSize;
  }

  // 3 B set-sensor-mode ack: echo CMD=0x04, mirror MODE.
  std::size_t BuildAck(uint8_t mode, uint8_t* out, std::size_t cap) const {
    if (cap < packets::kSensorRequestSize)
      return 0;
    packets::SensorRequestPacket pkt{};
    pkt.id = packets::kDeviceId;
    pkt.cmd = Cmd(packets::Command::kSetSensorMode);
    pkt.mode = mode;
    std::memcpy(out, &pkt, packets::kSensorRequestSize);
    return packets::kSensorRequestSize;
  }

  Config cfg_;
  double dt_;
  double inv_dt_;
  double alpha_;

  std::array<float, kNumHandMotors> target_{};
  std::array<float, kNumHandMotors> pos_{};
  std::array<float, kNumHandMotors> vel_{};
  std::array<float, kNumHandMotors> effort_{};

  static constexpr std::size_t kBase1aSize =
      static_cast<std::size_t>(kMaxFingertips) * kSensorDataPerPacket;  // 128
  static constexpr std::size_t kBase1bSize =
      static_cast<std::size_t>(kMaxFingertips) * kP1bValuesPerFingertip;  // 48
  std::array<int32_t, kBase1aSize> base_1a_{};
  std::array<int32_t, kBase1bSize> base_1b_{};

  std::mt19937 rng_;
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_FAKE_HAND_FIRMWARE_HPP_
