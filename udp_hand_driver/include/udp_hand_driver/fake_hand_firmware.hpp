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
// proto2 (CAN FD, epic #895 · H1-F04-2b): the same joint model answers the
// CAN FD wire through BuildCanFdResponse(), the MCU's mirror image for one
// hand (Config::hand_side → node id, D51). One received frame yields 0..3
// frames: a CMD Frame 0 / 1 is applied and its 64 bytes echoed verbatim
// (D41); a REQ (DLC 0 by D47 — any length is answered, a payload counted) is
// answered by the RESP burst idx 0, 1, 2 built from the joint model and the
// 1b tactile channels, with the Proto §4.3 sign flips pre-applied so the host
// decodes the same positive forces the 1b path yields; everything else (Frame
// 2 / 3, another node's target, error frames, bad lengths) is dropped and
// counted (D45). Broadcast (0xFF) is answered like the node's own id. The
// feedforward frame is kept for inspection only — the LPF model has no
// torque input, and the kt conversion lives in the driver (D50). All bytes
// come from hand_canfd_packets.hpp; the vcan socket shell is F04-2c, so this
// header still has no <linux/can.h>.
//
// All response builders write into a caller-provided buffer (no allocation).
// The class is single-threaded: the UDP server loop in fake_hand_firmware_main
// owns one instance and calls BuildResponse() serially; the CAN FD shell will
// own its own instance and call BuildCanFdResponse() the same way.
//
// ARCH note: udp_hand_driver is a hand-specific package (not rtc_*), so
// hand-domain constants live here directly. This is a peer simulator, not a
// second production implementation, so ARCH-3 (abstract-interface-first) does
// not apply.

#include "udp_hand_driver/fake_hand_lpf.hpp"
#include "udp_hand_driver/hand_canfd_packets.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <algorithm>
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
    double sensor_noise_stddev = 5.0;                     ///< per-response additive N(0, stddev)
    uint32_t seed = 42;                                   ///< std::mt19937 seed (base + noise)
    canfd::HandSide hand_side = canfd::HandSide::kRight;  ///< proto2 node (D51: right 1 · left 2)
  };

  /// One frame as the proto2 mirror emits it: the bare 29-bit identifier (the
  /// socket shell adds CAN_EFF_FLAG), the payload length and the payload.
  struct CanFdFrame {
    uint32_t can_id{0};
    std::size_t len{0};
    std::array<uint8_t, canfd::kFrameLen> data{};
  };

  /// What BuildCanFdResponse() saw besides the frames it answered. The first
  /// two are normal on a shared bus (the socket shell may pre-filter them);
  /// the rest are drops. Single writer; canfd_counters() hands out a copy.
  struct CanFdCounters {
    uint64_t peer_traffic{0};      ///< addressed to the host (another hand's echo / RESP)
    uint64_t req_with_payload{0};  ///< REQ answered although it carried data (D47 says DLC 0)
    uint64_t error_frames{0};      ///< CAN_ERR_FLAG set — rejected before the id is read
    uint64_t other_target{0};      ///< target another hand (neither this node, host nor broadcast)
    uint64_t unknown_type{0};      ///< msg type other than CMD / REQ (RESP, or an undefined value)
    uint64_t bad_length{0};        ///< CMD Frame 0 / 1 whose payload is not 64 bytes
    uint64_t cmd_ignored{0};       ///< CMD Frame 2, 3 or higher (D27 · D45: no model, no echo)
  };

  explicit FakeHandFirmware(const Config& cfg)
      : cfg_(cfg),
        dt_(CalcDt(cfg.step_rate_hz)),
        inv_dt_(CalcInvDt(cfg.step_rate_hz)),
        alpha_(CalcAlpha(cfg.lpf_time_constant_s, CalcDt(cfg.step_rate_hz))),
        node_(canfd::NodeIdOf(cfg.hand_side)),
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

  // proto2 mirror: dispatch one received CAN FD frame and write the frames the
  // hand would send into `out` (at most `cap` of them). Returns the number of
  // frames written; 0 means dropped — counted in canfd_counters(), except for
  // an insufficient `cap`, which is the caller's bug and leaves the model and
  // the RNG untouched. `can_id` is taken as the socket delivers it (EFF / ERR
  // flags still set) or as a bare 29-bit id; `data` holds `len` bytes.
  //   CMD Frame 0 (64 B) → target latched, model stepped once, 64 B echoed (D41)
  //   CMD Frame 1 (64 B) → kept as last_feedforward(), 64 B echoed (D50)
  //   CMD Frame 2 / 3 / … → counted, nothing else (D45)
  //   REQ (DLC 0, D47)    → RESP burst idx 0, 1, 2 — all three or none (D43);
  //                         a REQ with data is answered too, and counted
  // Not const: mutates the joint model (Frame 0) and the RNG (every burst).
  std::size_t BuildCanFdResponse(uint32_t can_id, const uint8_t* data, std::size_t len,
                                 CanFdFrame* out, std::size_t cap) {
    if (out == nullptr)
      return 0;
    if (canfd::IsErrorFrame(can_id)) {
      ++canfd_counters_.error_frames;
      return 0;
    }
    const uint32_t id = can_id & canfd::kExtIdMask;
    const uint8_t target = canfd::TargetOf(id);
    if (target == canfd::kNodeHost) {
      ++canfd_counters_.peer_traffic;
      return 0;
    }
    if (target != node_ && target != canfd::kNodeBroadcast) {
      ++canfd_counters_.other_target;
      return 0;
    }
    switch (canfd::MsgTypeOf(id)) {
      case static_cast<uint8_t>(canfd::MsgType::kCmd):
        return HandleCanFdCmd(canfd::FrameIndexOf(id), data, len, out, cap);
      case static_cast<uint8_t>(canfd::MsgType::kReq):
        if (len != 0)
          ++canfd_counters_.req_with_payload;
        return BuildCanFdRespBurst(out, cap);
      default:
        ++canfd_counters_.unknown_type;
        return 0;
    }
  }

  // Advance the first-order joint model one fixed step (dt = 1/step_rate_hz).
  // Delegates the per-joint math to the shared FakeLpfStep (fake_hand_lpf.hpp)
  // so this device-side model and UdpHandController's controller-side model
  // stay identical by construction:
  //   pos += alpha·(target − pos),  alpha = dt/(τ+dt)
  //   vel  = Δpos/dt
  //   effort = kp·(target − pos) − kd·vel
  void StepModel() noexcept {
    const float alpha = static_cast<float>(alpha_);
    const float inv_dt = static_cast<float>(inv_dt_);
    const float kp = static_cast<float>(cfg_.effort_stiffness);
    const float kd = static_cast<float>(cfg_.effort_damping);
    for (std::size_t i = 0; i < kNumHandMotors; ++i) {
      const FakeJointSample s = FakeLpfStep(target_[i], pos_[i], alpha, inv_dt, kp, kd);
      pos_[i] = s.pos;
      vel_[i] = s.vel;
      effort_[i] = s.effort;
    }
  }

  // ── Accessors (unit tests / introspection) ─────────────────────────────────
  [[nodiscard]] const std::array<float, kNumHandMotors>& pos() const noexcept { return pos_; }

  [[nodiscard]] const std::array<float, kNumHandMotors>& vel() const noexcept { return vel_; }

  [[nodiscard]] const std::array<float, kNumHandMotors>& effort() const noexcept { return effort_; }

  [[nodiscard]] const std::array<float, kNumHandMotors>& target() const noexcept { return target_; }

  [[nodiscard]] double alpha() const noexcept { return alpha_; }

  /// Payload of the last CMD Frame 1 (feedforward current, A, firmware motor
  /// order); zeros until one arrives. Inspection only — not fed to the model.
  [[nodiscard]] const std::array<float, kNumHandMotors>& last_feedforward() const noexcept {
    return last_feedforward_;
  }

  /// CMD Frame 1 frames applied so far.
  [[nodiscard]] uint64_t feedforward_count() const noexcept { return feedforward_count_; }

  [[nodiscard]] CanFdCounters canfd_counters() const noexcept { return canfd_counters_; }

 private:
  static_assert(canfd::kWireMotorCount == static_cast<std::size_t>(kNumHandMotors),
                "the joint model is copied motor-for-motor onto the proto2 wire");
  static_assert(canfd::kWireFingertipCount <= static_cast<std::size_t>(kMaxFingertips),
                "every wire fingertip has 1b base channels");

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
  // Integer channels (1a, proto2) share DrawCounts; 1b stays float.
  [[nodiscard]] long DrawCounts(int32_t base) {
    std::normal_distribution<double> nd(0.0, cfg_.sensor_noise_stddev);
    return static_cast<long>(base) + std::lround(nd(rng_));
  }

  [[nodiscard]] int32_t SampleSensor1a(std::size_t channel) {
    return static_cast<int32_t>(DrawCounts(base_1a_[channel]));
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

  // ── proto2 (CAN FD) builders ───────────────────────────────────────────────

  // One fingertip's raw tactile record. Synthesised in the wire's count
  // domain from the 1b base channels ([fx, fy, fz, Lx, Ly, Temp] — the same
  // channel order as TactileRaw): base + N(0, stddev), rounded and clamped to
  // int16. The host applies the Proto §4.3 sign flips on fy, fz and py, so
  // those raw counts are negated here and every decoded channel comes out
  // positive, as the 1b fake's do — a SIL consumer thresholding normal force
  // sees the same sign on both wires. With the default bases (100..4000
  // counts) the host decodes a thumb fx of 0.08..3 N. An active fingertip
  // reports contact; an inactive one is all zeros.
  [[nodiscard]] canfd::TactileRaw SampleTactile(canfd::Fingertip finger) {
    canfd::TactileRaw r{};
    const int f = static_cast<int>(finger);
    if (!FingerActive(f))
      return r;
    const std::size_t k0 = static_cast<std::size_t>(f) * kP1bValuesPerFingertip;
    r.fx = SampleSensorCanFd(k0 + 0, +1);
    r.fy = SampleSensorCanFd(k0 + 1, -1);
    r.fz = SampleSensorCanFd(k0 + 2, -1);
    r.px = SampleSensorCanFd(k0 + 3, +1);
    r.py = SampleSensorCanFd(k0 + 4, -1);
    r.temp = SampleSensorCanFd(k0 + 5, +1);
    r.contact = 1;
    return r;
  }

  // int16 wire count; `sign` −1 pre-applies a Proto §4.3 flip.
  [[nodiscard]] int16_t SampleSensorCanFd(std::size_t channel, long sign) {
    return static_cast<int16_t>(
        std::clamp<long>(sign * DrawCounts(base_1b_[channel]), INT16_MIN, INT16_MAX));
  }

  // CMD Frame `idx`: Frame 0 latches the target and steps the model, Frame 1
  // is kept; both are echoed verbatim. Higher indices are counted and ignored
  // (D45). A payload that is not 64 bytes is a bad frame, not a command.
  std::size_t HandleCanFdCmd(uint8_t idx, const uint8_t* data, std::size_t len, CanFdFrame* out,
                             std::size_t cap) {
    if (idx > static_cast<uint8_t>(canfd::CmdFrame::kFeedforward)) {
      ++canfd_counters_.cmd_ignored;
      return 0;
    }
    canfd::CmdMotorFrame in{};
    if (data == nullptr || !canfd::DecodeFrame(data, len, in)) {
      ++canfd_counters_.bad_length;
      return 0;
    }
    if (cap < 1)
      return 0;
    if (idx == static_cast<uint8_t>(canfd::CmdFrame::kPosition)) {
      for (std::size_t i = 0; i < kNumHandMotors; ++i)
        target_[i] = in.values[i];
      StepModel();
    } else {
      for (std::size_t i = 0; i < kNumHandMotors; ++i)
        last_feedforward_[i] = in.values[i];
      ++feedforward_count_;
    }
    out[0].can_id = canfd::EchoId(node_, idx);
    out[0].len = canfd::kFrameLen;
    std::memcpy(out[0].data.data(), data, canfd::kFrameLen);
    return 1;
  }

  // The RESP burst for one REQ: F0 pos + vel M0..M5, F1 vel M6..M9 + cur
  // (= effort) + thumb head, F2 thumb tail + index / middle / ring. Written
  // whole or not at all — a partial set is what the host discards (D43).
  // Tactile draws happen in fingertip order, so one seed gives one burst.
  std::size_t BuildCanFdRespBurst(CanFdFrame* out, std::size_t cap) {
    if (cap < canfd::kNumRespFrames)
      return 0;
    canfd::RespFrame0 f0{};
    canfd::RespFrame1 f1{};
    canfd::RespFrame2 f2{};
    for (std::size_t i = 0; i < canfd::kWireMotorCount; ++i) {
      f0.pos[i] = pos_[i];
      f1.cur[i] = effort_[i];
    }
    for (std::size_t i = 0; i < canfd::kVelLoCount; ++i)
      f0.vel_lo[i] = vel_[i];
    for (std::size_t i = 0; i < canfd::kVelHiCount; ++i)
      f1.vel_hi[i] = vel_[canfd::kVelLoCount + i];
    const canfd::TactileRaw thumb = SampleTactile(canfd::Fingertip::kThumb);
    f1.thumb_fx = thumb.fx;
    f1.thumb_fy = thumb.fy;
    f1.thumb_fz = thumb.fz;
    f1.thumb_px = thumb.px;
    f2.thumb_py = thumb.py;
    f2.thumb_temp = thumb.temp;
    f2.thumb_contact = thumb.contact;
    f2.index = SampleTactile(canfd::Fingertip::kIndex);
    f2.middle = SampleTactile(canfd::Fingertip::kMiddle);
    f2.ring = SampleTactile(canfd::Fingertip::kRing);
    EmitCanFd(out[0], canfd::RespId(node_, 0), f0);
    EmitCanFd(out[1], canfd::RespId(node_, 1), f1);
    EmitCanFd(out[2], canfd::RespId(node_, 2), f2);
    return canfd::kNumRespFrames;
  }

  template <typename Frame>
  static void EmitCanFd(CanFdFrame& dst, uint32_t id, const Frame& frame) noexcept {
    dst.can_id = id;
    dst.len = canfd::kFrameLen;
    canfd::SerializeFrame(frame, dst.data.data());
  }

  Config cfg_;
  double dt_;
  double inv_dt_;
  double alpha_;
  uint8_t node_;  ///< proto2 node id, from cfg_.hand_side

  std::array<float, kNumHandMotors> target_{};
  std::array<float, kNumHandMotors> pos_{};
  std::array<float, kNumHandMotors> vel_{};
  std::array<float, kNumHandMotors> effort_{};
  std::array<float, kNumHandMotors> last_feedforward_{};  ///< CMD Frame 1, kept only
  uint64_t feedforward_count_{0};
  CanFdCounters canfd_counters_{};

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
