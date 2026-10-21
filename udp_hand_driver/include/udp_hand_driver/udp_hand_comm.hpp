#ifndef UDP_HAND_DRIVER_UDP_HAND_COMM_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_COMM_HPP_

// UdpHandComm: HandComm over UDP for the proto1a / proto1b hands.
//
// A thin cycle-level wrapper around UdpHandTransport (which keeps its name and
// its socket / retry / stats behaviour unchanged — decision D1): the request
// sequence each cycle issues is exactly what UdpHandController issued directly
// before the seam, so proto1 behaviour, stats attribution and the trace span
// names are bit-for-bit the same. The feedforward torque has no proto1 wire
// field and is ignored.
//
// RT: every HandComm RT method here is noexcept and allocation-free — the
// command scratch buffers are members, the per-finger decode buffer too.

#include "rtc_base/tracing/trace_scope.hpp"
#include "udp_hand_driver/hand_comm.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"
#include "udp_hand_driver/udp_hand_transport.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace udp_hand_driver {

class UdpHandComm final : public HandComm {
 public:
  explicit UdpHandComm(std::string target_ip, int target_port, double recv_timeout_ms,
                       std::string local_ip = {}, std::string local_interface = {}) noexcept
      : transport_(std::move(target_ip), target_port, recv_timeout_ms, std::move(local_ip),
                   std::move(local_interface)) {}

  /// The wrapped transport (tests / forensics). Not for the RT cycle — the
  /// controller talks through the HandComm surface only.
  [[nodiscard]] UdpHandTransport& transport() noexcept { return transport_; }

  [[nodiscard]] const UdpHandTransport& transport() const noexcept { return transport_; }

  [[nodiscard]] packets::JointMode joint_io_mode() const noexcept { return joint_io_mode_; }

  // ── Non-RT ────────────────────────────────────────────────────────────────

  [[nodiscard]] bool Open() noexcept override { return transport_.Open(); }

  void Close() noexcept override { transport_.Close(); }

  [[nodiscard]] bool InitializeSensors() noexcept override {
    return transport_.InitializeSensors();
  }

  void set_verify_response_mode(bool v) noexcept override {
    transport_.set_verify_response_mode(v);
  }

  void set_verify_bulk_sensor_mode(bool v) noexcept override {
    transport_.set_verify_bulk_sensor_mode(v);
  }

  void set_joint_io_mode(packets::JointMode mode) noexcept override { joint_io_mode_ = mode; }

  void ResetCommStats() noexcept override { transport_.ResetCommStats(); }

  [[nodiscard]] UdpHandCommStats comm_stats() const noexcept override {
    return transport_.comm_stats();
  }

  [[nodiscard]] uint64_t recv_error_count() const noexcept override {
    return transport_.recv_error_count();
  }

  [[nodiscard]] double recv_timeout_ms() const noexcept override {
    return transport_.recv_timeout_ms();
  }

  [[nodiscard]] bool SupportsIndividualMode() const noexcept override { return true; }

  // ── RT ────────────────────────────────────────────────────────────────────

  int DrainStale() noexcept override { return transport_.DrainStaleDatagrams(); }

  // proto1 has no feedforward field: `ff_nm` is ignored (epic #895 D56 — the
  // CAN FD comm is the first consumer).
  [[nodiscard]] bool WriteCommandWithEcho(const HandMotorArray& pos,
                                          const HandMotorArray& /*ff_nm*/) noexcept override {
    return transport_.WritePositionWithEcho(pos, send_buf_, echo_buf_, joint_io_mode_);
  }

  void WriteCommandFireAndForget(const HandMotorArray& pos,
                                 const HandMotorArray& /*ff_nm*/) noexcept override {
    transport_.WritePositionFireAndForget(pos, joint_io_mode_);
  }

  // Bulk: 0x10 kMotor (if planned) → 0x10 joint_io_mode → 0x19 raw (if planned).
  void ReadCycle(const HandReadPlan& plan, HandReadBuffers& bufs,
                 HandReadOutcome& out) noexcept override {
    out = HandReadOutcome{};
    if (plan.motor_space_read) {
      RTC_TRACE_SCOPE("hand_read_motor");
      out.motor_space_ok = transport_.RequestAllMotorRead(
          bufs.motor_pos, bufs.motor_vel, bufs.motor_cur, packets::JointMode::kMotor,
          &out.received_motor_mode, RequestKind::kMotorRead);
    }
    out.phase_end[0] = HandReadOutcome::Clock::now();
    {
      RTC_TRACE_SCOPE("hand_read_joint");
      out.joint_space_ok =
          transport_.RequestAllMotorRead(bufs.joint_pos, bufs.joint_vel, bufs.joint_cur,
                                         joint_io_mode_, nullptr, RequestKind::kJointRead);
    }
    out.phase_end[1] = HandReadOutcome::Clock::now();
    if (plan.sensors) {
      RTC_TRACE_SCOPE("hand_read_sensor");
      out.sensor_recvd =
          transport_.RequestBulkSensorRaw(bufs.sensor_raw.data(), bufs.sensor_raw.size(),
                                          plan.sensor_response_size, packets::SensorMode::kRaw);
    }
    out.phase_end[2] = HandReadOutcome::Clock::now();
    out.phase_end[3] = out.phase_end[2];
  }

  // Individual: 0x11 kMotor → 0x11 joint_io_mode → 0x12 kMotor → 0x14+i per
  // finger (if planned). A finger that validated is copied into its slot of
  // bufs.sensor_values and flagged; a failed finger leaves its slot untouched.
  void ReadCycleIndividual(const HandReadPlan& plan, HandReadBuffers& bufs,
                           HandReadOutcome& out) noexcept override {
    out = HandReadOutcome{};
    {
      RTC_TRACE_SCOPE("hand_read_pos");
      out.pos_ok = transport_.RequestMotorRead(packets::Command::kReadPosition, bufs.motor_pos,
                                               packets::JointMode::kMotor, &out.received_motor_mode,
                                               RequestKind::kMotorRead);
    }
    out.phase_end[0] = HandReadOutcome::Clock::now();
    {
      RTC_TRACE_SCOPE("hand_read_joint_pos");
      out.joint_pos_ok =
          transport_.RequestMotorRead(packets::Command::kReadPosition, bufs.joint_pos,
                                      joint_io_mode_, nullptr, RequestKind::kJointRead);
    }
    out.phase_end[1] = HandReadOutcome::Clock::now();
    {
      RTC_TRACE_SCOPE("hand_read_vel");
      out.vel_ok =
          transport_.RequestMotorRead(packets::Command::kReadVelocity, bufs.motor_vel,
                                      packets::JointMode::kMotor, nullptr, RequestKind::kMotorRead);
    }
    out.phase_end[2] = HandReadOutcome::Clock::now();
    if (plan.sensors) {
      RTC_TRACE_SCOPE("hand_read_sensor");
      const int fingers = std::min(plan.num_fingertips, kMaxFingertips);
      for (int i = 0; i < fingers; ++i) {
        if (transport_.RequestSensorRead(packets::SensorCommand(i), finger_buf_,
                                         packets::SensorMode::kRaw)) {
          std::copy_n(finger_buf_.begin(), kSensorValuesPerFingertip,
                      bufs.sensor_values.begin() +
                          static_cast<std::ptrdiff_t>(i) * kSensorValuesPerFingertip);
          out.finger_ok_mask |= static_cast<uint8_t>(1u << static_cast<unsigned>(i));
        }
      }
    }
    out.phase_end[3] = HandReadOutcome::Clock::now();
  }

  [[nodiscard]] UdpHandCommStats& comm_stats_mut() noexcept override {
    return transport_.comm_stats_mut();
  }

  void PublishCommStats() noexcept override { transport_.PublishCommStats(); }

  [[nodiscard]] uint8_t last_unexpected_cmd() const noexcept override {
    return transport_.last_unexpected_cmd();
  }

  [[nodiscard]] uint16_t last_unexpected_len() const noexcept override {
    return transport_.last_unexpected_len();
  }

 private:
  UdpHandTransport transport_;
  packets::JointMode joint_io_mode_{packets::JointMode::kJoint};
  // Scratch reused every cycle (allocation-free RT path).
  std::array<uint8_t, packets::kMotorPacketSize> send_buf_{};
  std::array<uint8_t, packets::kMotorPacketSize> echo_buf_{};
  std::array<int32_t, kSensorValuesPerFingertip> finger_buf_{};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_COMM_HPP_
